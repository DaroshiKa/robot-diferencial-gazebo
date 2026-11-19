#!/usr/bin/env python3
"""Nodo de cinemática de las llantas (skid-steer de 4 ruedas tratado como diferencial).

  /joint_states --(wD = media(del_der, tras_der), wI = media(del_izq, tras_izq))--> u = A w
      --> q_dot = J(theta) u --integración--> q

  q se publica en /odom_cinematica (marco world, sin TF: el TF odom->base_link lo publica DiffDrive)
  y se compara con la POSE REAL del simulador (/ground_truth, plugin OdometryPublisher):

      e_pos   = || p_estimada - p_real ||          [m]
      e_theta = wrap(theta_estimada - theta_real)  [rad]

  /error_odometria (std_msgs/Float64MultiArray) = [e_x, e_y, e_pos, e_theta, distancia_recorrida]

q(0) se toma de la primera lectura de la referencia (el robot arranca quieto en x = -0.60 m), de modo
que la estimación queda en el mismo marco (world) que la pose real y el error de cierre por vuelta
se calcula directamente (nodo vueltas).

Parámetros ROS:
  radio_rueda (0.0245), separacion_ruedas (0.134; con skid-steer ver README "vía efectiva"),
  joints_der, joints_izq (listas), metodo ('punto_medio' | 'euler'), referencia ('/ground_truth'),
  periodo_log [s] (2.0), csv (ruta; vacío = no guarda)
"""
import bisect
import csv
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from diferencial_control.util_ros import girar
from diferencial_control.cinematica import (directa_u, envolver, integrar,
                                            yaw_de_cuaternion)


def _t(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class CinematicaDiferencial(Node):
    def __init__(self):
        super().__init__('cinematica_diferencial')
        self.declare_parameter('radio_rueda', 0.0245)
        self.declare_parameter('separacion_ruedas', 0.134)
        self.declare_parameter('joints_der', ['joint_rueda_del_der', 'joint_rueda_tras_der'])
        self.declare_parameter('joints_izq', ['joint_rueda_del_izq', 'joint_rueda_tras_izq'])
        self.declare_parameter('metodo', 'punto_medio')
        self.declare_parameter('referencia', '/ground_truth')
        self.declare_parameter('periodo_log', 2.0)
        self.declare_parameter('csv', '')

        self.r = self.get_parameter('radio_rueda').value
        self.b = self.get_parameter('separacion_ruedas').value
        self.j_der = list(self.get_parameter('joints_der').value)
        self.j_izq = list(self.get_parameter('joints_izq').value)
        self.metodo = self.get_parameter('metodo').value
        self.periodo_log = self.get_parameter('periodo_log').value
        ruta_csv = self.get_parameter('csv').value
        topico_ref = self.get_parameter('referencia').value

        self.q = None
        self.t_ant = None
        self.dist = 0.0
        self.w_ult = (0.0, 0.0)
        self.hist_t, self.hist_q = [], []
        self.idx = None              # índices de las juntas en /joint_states (se cachean)
        self._n_nombres = 0

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
            self.csv_w.writerow(['t', 'x_est', 'y_est', 'th_est', 'x_real', 'y_real', 'th_real',
                                 'e_x', 'e_y', 'e_pos', 'e_theta', 'dist', 'wD', 'wI'])

        self.create_subscription(JointState, '/joint_states', self.cb_joints, 50)
        self.create_subscription(Odometry, topico_ref, self.cb_ref, 50)
        self.pub_odom = self.create_publisher(Odometry, '/odom_cinematica', 10)
        self.pub_err = self.create_publisher(Float64MultiArray, '/error_odometria', 10)

        self.get_logger().info(
            f'r={self.r} m, b={self.b} m, integración={self.metodo}, wD=media{self.j_der}, '
            f'wI=media{self.j_izq}. Esperando {topico_ref} para inicializar q(0)...')

    # ---- w -> u -> q_dot -> q ----
    def _velocidades(self, msg):
        if self.idx is None or len(msg.name) != self._n_nombres:
            try:
                self.idx = ([msg.name.index(j) for j in self.j_der],
                            [msg.name.index(j) for j in self.j_izq])
                self._n_nombres = len(msg.name)
            except ValueError:
                return None
        if len(msg.velocity) < len(msg.name):
            return None
        i_d, i_i = self.idx
        w_der = sum(msg.velocity[i] for i in i_d) / len(i_d)
        w_izq = sum(msg.velocity[i] for i in i_i) / len(i_i)
        return w_der, w_izq

    def cb_joints(self, msg):
        if self.q is None:
            return
        w = self._velocidades(msg)
        if w is None:
            return
        w_der, w_izq = w
        t = _t(msg.header.stamp)
        if self.t_ant is None:
            self.t_ant = t
            return
        dt = t - self.t_ant
        if dt <= 0.0:
            return
        self.t_ant = t
        self.w_ult = (w_der, w_izq)

        v, omega = directa_u(w_der, w_izq, self.r, self.b)
        x, y, th = self.q
        x2, y2, th2 = integrar(x, y, th, v, omega, dt, self.metodo)
        self.dist += math.hypot(x2 - x, y2 - y)
        self.q = (x2, y2, th2)

        self.hist_t.append(t)
        self.hist_q.append((x2, y2, th2))
        if len(self.hist_t) > 4000:
            del self.hist_t[:2000], self.hist_q[:2000]

        od = Odometry()
        od.header.stamp = msg.header.stamp
        od.header.frame_id = 'world'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = x2
        od.pose.pose.position.y = y2
        od.pose.pose.orientation.z = math.sin(th2 / 2.0)
        od.pose.pose.orientation.w = math.cos(th2 / 2.0)
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = omega
        self.pub_odom.publish(od)

    def _pose_en(self, t):
        if not self.hist_t or t < self.hist_t[0] or t > self.hist_t[-1]:
            return None
        i = bisect.bisect_left(self.hist_t, t)
        if i == 0:
            return self.hist_q[0]
        t0, t1 = self.hist_t[i - 1], self.hist_t[i]
        a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        (x0, y0, h0), (x1, y1, h1) = self.hist_q[i - 1], self.hist_q[i]
        return (x0 + a * (x1 - x0), y0 + a * (y1 - y0), envolver(h0 + a * envolver(h1 - h0)))

    # ---- comparación con la pose real ----
    def cb_ref(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        th_r = yaw_de_cuaternion(o.x, o.y, o.z, o.w)
        t = _t(msg.header.stamp)
        if self.q is None:
            self.q = (p.x, p.y, th_r)
            self.get_logger().info(f'q(0) = ({p.x:.3f}, {p.y:.3f}, {th_r:.3f}) tomada de la referencia')
            return
        q = self._pose_en(t)
        if q is None:
            return
        e_x, e_y = q[0] - p.x, q[1] - p.y
        e_pos = math.hypot(e_x, e_y)
        e_th = envolver(q[2] - th_r)

        self.n += 1
        self.suma_pos += e_pos
        self.max_pos = max(self.max_pos, e_pos)
        self.max_th = max(self.max_th, abs(e_th))
        self.ult = (e_x, e_y, e_pos, e_th)

        self.pub_err.publish(Float64MultiArray(data=[e_x, e_y, e_pos, e_th, self.dist]))
        if self.csv:
            self.csv_w.writerow([f'{t:.4f}', q[0], q[1], q[2], p.x, p.y, th_r,
                                 e_x, e_y, e_pos, e_th, self.dist, self.w_ult[0], self.w_ult[1]])
        if self.t_log is None or t - self.t_log >= self.periodo_log:
            self.t_log = t
            rel = 100.0 * e_pos / self.dist if self.dist > 0.05 else 0.0
            self.get_logger().info(
                f'dist={self.dist:6.2f} m | e_pos={e_pos * 100:6.2f} cm ({rel:5.2f} %) '
                f'| e_theta={math.degrees(e_th):7.2f}°')

    def resumen(self):
        if self.n == 0:
            return
        e_x, e_y, e_pos, e_th = self.ult
        media = self.suma_pos / self.n
        rel = 100.0 * e_pos / self.dist if self.dist > 0.05 else 0.0
        self.get_logger().info(
            f'RESUMEN: distancia={self.dist:.2f} m | e_pos media={media * 100:.2f} cm, '
            f'max={self.max_pos * 100:.2f} cm, final={e_pos * 100:.2f} cm ({rel:.2f} %) | '
            f'|e_theta| max={math.degrees(self.max_th):.2f}°, final={math.degrees(e_th):.2f}°')


def main():
    rclpy.init()
    nodo = CinematicaDiferencial()

    def al_salir():
        nodo.resumen()
        if nodo.csv:
            nodo.csv.close()
    girar(nodo, al_salir)


if __name__ == '__main__':
    main()
