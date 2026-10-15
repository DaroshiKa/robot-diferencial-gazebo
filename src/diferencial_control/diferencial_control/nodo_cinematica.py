#!/usr/bin/env python3
"""Nodo de cinemática de las llantas del robot diferencial.

  /joint_states --(w = [wD, wI])--> u = A w --> q_dot = J(theta) u --integración--> q
  q se publica en /odom_cinematica (sin TF: el TF odom->base_link ya lo publica DiffDrive)
  y se compara con /odom (plugin DiffDrive de Gazebo):

      e_pos   = || p_propia - p_DiffDrive ||                 [m]
      e_theta = wrap(theta_propia - theta_DiffDrive)         [rad]

  /error_odometria (std_msgs/Float64MultiArray) = [e_x, e_y, e_pos, e_theta, distancia_recorrida]

Parámetros ROS (deben coincidir con el xacro):
  radio_rueda (0.05), separacion_ruedas (0.30), joint_der, joint_izq,
  metodo ('punto_medio' | 'euler'), periodo_log [s] (1.0), csv (ruta; vacío = no guarda)

    ros2 run diferencial_control cinematica_diferencial --ros-args -p use_sim_time:=true
"""
import bisect
import csv
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from diferencial_control.cinematica import (directa_u, envolver, integrar,
                                            yaw_de_cuaternion)


def _t(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class CinematicaDiferencial(Node):
    def __init__(self):
        super().__init__('cinematica_diferencial')
        self.declare_parameter('radio_rueda', 0.05)
        self.declare_parameter('separacion_ruedas', 0.30)
        self.declare_parameter('joint_der', 'joint_rueda_der')
        self.declare_parameter('joint_izq', 'joint_rueda_izq')
        self.declare_parameter('metodo', 'punto_medio')
        self.declare_parameter('periodo_log', 1.0)
        self.declare_parameter('csv', '')

        self.r = self.get_parameter('radio_rueda').value
        self.b = self.get_parameter('separacion_ruedas').value
        self.j_der = self.get_parameter('joint_der').value
        self.j_izq = self.get_parameter('joint_izq').value
        self.metodo = self.get_parameter('metodo').value
        self.periodo_log = self.get_parameter('periodo_log').value
        ruta_csv = self.get_parameter('csv').value

        # pose propia
        self.q = None            # (x, y, theta) cuando ya se inicializó con el primer /odom
        self.t_ant = None
        self.dist = 0.0
        # historial para comparar en el instante del /odom (interpolación lineal)
        self.hist_t, self.hist_q = [], []

        # estadísticas del error
        self.n = 0
        self.suma_pos = 0.0
        self.max_pos = 0.0
        self.max_th = 0.0
        self.ult = (0.0, 0.0, 0.0, 0.0)
        self.t_log = None

        self.csv = None
        if ruta_csv:
            self.csv = open(ruta_csv, 'w', newline='')
            self.csv_w = csv.writer(self.csv)
            self.csv_w.writerow(['t', 'x_prop', 'y_prop', 'th_prop', 'x_gz', 'y_gz', 'th_gz',
                                 'e_x', 'e_y', 'e_pos', 'e_theta', 'dist'])

        self.create_subscription(JointState, '/joint_states', self.cb_joints, 50)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 50)
        self.pub_odom = self.create_publisher(Odometry, '/odom_cinematica', 10)
        self.pub_err = self.create_publisher(Float64MultiArray, '/error_odometria', 10)

        self.get_logger().info(
            f'r={self.r} m, b={self.b} m, integración={self.metodo}. '
            'Esperando el primer /odom para inicializar q(0)...')

    # ---- w -> u -> q_dot -> q ----
    def cb_joints(self, msg):
        if self.q is None:
            return
        try:
            w_der = msg.velocity[msg.name.index(self.j_der)]
            w_izq = msg.velocity[msg.name.index(self.j_izq)]
        except (ValueError, IndexError):
            return
        t = _t(msg.header.stamp)
        if self.t_ant is None:
            self.t_ant = t
            return
        dt = t - self.t_ant
        if dt <= 0.0:
            return
        self.t_ant = t

        v, omega = directa_u(w_der, w_izq, self.r, self.b)
        x, y, th = self.q
        x2, y2, th2 = integrar(x, y, th, v, omega, dt, self.metodo)
        self.dist += math.hypot(x2 - x, y2 - y)
        self.q = (x2, y2, th2)

        self.hist_t.append(t)
        self.hist_q.append((x2, y2, th2))
        if len(self.hist_t) > 2000:        # ~2 s a 1 kHz
            del self.hist_t[:1000], self.hist_q[:1000]

        od = Odometry()
        od.header.stamp = msg.header.stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = x2
        od.pose.pose.position.y = y2
        od.pose.pose.orientation.z = math.sin(th2 / 2.0)
        od.pose.pose.orientation.w = math.cos(th2 / 2.0)
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = omega
        self.pub_odom.publish(od)

    def _pose_en(self, t):
        """Pose propia interpolada en el instante t (None si no hay datos que lo cubran)."""
        if not self.hist_t or t < self.hist_t[0] or t > self.hist_t[-1]:
            return None
        i = bisect.bisect_left(self.hist_t, t)
        if i == 0:
            return self.hist_q[0]
        t0, t1 = self.hist_t[i - 1], self.hist_t[i]
        a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        (x0, y0, h0), (x1, y1, h1) = self.hist_q[i - 1], self.hist_q[i]
        return (x0 + a * (x1 - x0), y0 + a * (y1 - y0), envolver(h0 + a * envolver(h1 - h0)))

    # ---- comparación con el /odom de DiffDrive ----
    def cb_odom(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        th_gz = yaw_de_cuaternion(o.x, o.y, o.z, o.w)
        t = _t(msg.header.stamp)
        if self.q is None:
            self.q = (p.x, p.y, th_gz)       # q(0) = primera lectura de /odom
            self.get_logger().info(
                f'q(0) = ({p.x:.3f}, {p.y:.3f}, {th_gz:.3f}) tomada de /odom')
            return
        q = self._pose_en(t)
        if q is None:
            return
        e_x, e_y = q[0] - p.x, q[1] - p.y
        e_pos = math.hypot(e_x, e_y)
        e_th = envolver(q[2] - th_gz)

        self.n += 1
        self.suma_pos += e_pos
        self.max_pos = max(self.max_pos, e_pos)
        self.max_th = max(self.max_th, abs(e_th))
        self.ult = (e_x, e_y, e_pos, e_th)

        self.pub_err.publish(Float64MultiArray(data=[e_x, e_y, e_pos, e_th, self.dist]))
        if self.csv:
            self.csv_w.writerow([f'{t:.4f}', q[0], q[1], q[2], p.x, p.y, th_gz,
                                 e_x, e_y, e_pos, e_th, self.dist])
        if self.t_log is None or t - self.t_log >= self.periodo_log:
            self.t_log = t
            rel = 100.0 * e_pos / self.dist if self.dist > 0.05 else 0.0
            self.get_logger().info(
                f'dist={self.dist:6.2f} m | e_pos={e_pos * 1000:7.2f} mm ({rel:4.2f} % de lo recorrido) '
                f'| e_theta={math.degrees(e_th):7.3f}° | propia=({q[0]:.2f},{q[1]:.2f},{math.degrees(q[2]):.1f}°) '
                f'DiffDrive=({p.x:.2f},{p.y:.2f},{math.degrees(th_gz):.1f}°)')

    def resumen(self):
        if self.n == 0:
            return
        e_x, e_y, e_pos, e_th = self.ult
        media = self.suma_pos / self.n
        rel = 100.0 * e_pos / self.dist if self.dist > 0.05 else 0.0
        self.get_logger().info(
            f'RESUMEN: muestras={self.n}, distancia={self.dist:.2f} m | e_pos media={media * 1000:.2f} mm, '
            f'max={self.max_pos * 1000:.2f} mm, final={e_pos * 1000:.2f} mm ({rel:.2f} %) | '
            f'|e_theta| max={math.degrees(self.max_th):.3f}°, final={math.degrees(e_th):.3f}°')


def main():
    rclpy.init()
    nodo = CinematicaDiferencial()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.resumen()
        if nodo.csv:
            nodo.csv.close()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
