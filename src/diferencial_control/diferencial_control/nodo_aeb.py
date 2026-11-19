#!/usr/bin/env python3
"""Nodo AEB: último eslabón antes de /cmd_vel. Tiene prioridad sobre CUALQUIER comando.

    /cmd_vel_mux --> [AEB: TTC con /scan y /odom] --> /cmd_vel (DiffDrive)

Con cada /scan calcula TTC_i = r_i / max(v cos(theta_i), 0) (ver aeb.py) y, si min TTC_i < umbral(v),
anula la componente de avance del comando (v = 0, con la desaceleración máxima del DiffDrive).
Mientras el comando insista en avanzar hacia el obstáculo el robot queda DETENIDO (v = 0, omega = 0);
girar en el sitio (v = 0) o retroceder (alejarse) sí se permite, para que el operador pueda salir.
El freno se suelta cuando el camino queda libre para avanzar a v_liberacion (p. ej. al retirar el
obstáculo), con histéresis y un tiempo mínimo de frenado.

Seguridad: si no llega /scan por más de timeout_scan s, se manda v = 0 (falla segura).

Topics de diagnóstico: /aeb/activo (Bool), /aeb/ttc_min (Float64), /aeb/umbral (Float64).
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64

from diferencial_control.util_ros import girar
from diferencial_control.aeb import AEB, velocidad_para_ttc


class NodoAEB(Node):
    def __init__(self):
        super().__init__('aeb')
        dp = self.declare_parameter
        dp('largo', 0.1986)          # contorno del robot (tabla del MicroROS)
        dp('ancho', 0.153)
        dp('lidar_x', 0.0)           # posición del LiDAR en base_link
        dp('margen_lateral', 0.015)
        dp('a_freno', 1.6)           # m/s² (80 % de a_max = 2.0: margen por deslizamiento)
        dp('t_latencia', 0.12)       # s (periodo del LiDAR 0.1 s + transporte)
        dp('t_margen', 0.10)         # s
        dp('histeresis', 1.3)
        dp('t_min_frenado', 0.5)
        dp('pasar_omega', False)     # True: deja girar aunque el comando pida avanzar
        dp('v_liberacion', 1.0)      # m/s (= v_max): el camino debe estar libre para avanzar a v_max
        dp('timeout_scan', 0.5)
        dp('frecuencia', 50.0)
        dp('csv', '')                # t, v, omega, ttc_min, umbral, frenando (para graficar)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        L, A = g('largo'), g('ancho')
        self.aeb = AEB(x_del=L / 2, x_tras=-L / 2, y_izq=A / 2, y_der=-A / 2, lidar_x=g('lidar_x'),
                       margen_lateral=g('margen_lateral'), a_freno=g('a_freno'),
                       t_latencia=g('t_latencia'), t_margen=g('t_margen'),
                       histeresis=g('histeresis'), t_min_frenado=g('t_min_frenado'),
                       v_liberacion=g('v_liberacion'))
        self.pasar_omega = g('pasar_omega')
        self.timeout_scan = g('timeout_scan')

        self.cmd = Twist()
        self.v_med = 0.0
        self.t_scan = None
        self.frenando = False
        self.sentido_freno = 1.0
        self.n_frenados = 0
        self.angulos = None

        self.csv = None
        if g('csv'):
            self.csv = open(g('csv'), 'w')
            self.csv.write('t,v_ttc,v_cmd,v_med,omega,ttc_min,umbral,frenando\n')
        self.create_subscription(Twist, '/cmd_vel_mux', self._cb_cmd, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._cb_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_activo = self.create_publisher(Bool, '/aeb/activo', 10)
        self.pub_ttc = self.create_publisher(Float64, '/aeb/ttc_min', 10)
        self.pub_umbral = self.create_publisher(Float64, '/aeb/umbral', 10)
        self.create_timer(1.0 / g('frecuencia'), self._publicar)
        self.get_logger().info(
            f'AEB activo: umbral(v) = |v|/(2·{g("a_freno")}) + {g("t_latencia")} + {g("t_margen")} s, '
            f'contorno {L:.4f} x {A:.3f} m, franja ±{self.aeb.semiancho:.3f} m')

    def _ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cb_cmd(self, msg):
        self.cmd = msg

    def _cb_odom(self, msg):
        self.v_med = msg.twist.twist.linear.x

    def _cb_scan(self, msg):
        n = len(msg.ranges)
        if self.angulos is None or len(self.angulos) != n:
            self.angulos = msg.angle_min + msg.angle_increment * np.arange(n)
        r = np.asarray(msg.ranges, dtype=float)
        r[(r < msg.range_min) | (r > msg.range_max)] = np.inf
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.t_scan = self._ahora()
        v = velocidad_para_ttc(self.cmd.linear.x, self.v_med)
        antes = self.frenando
        self.frenando, ttc, umbral = self.aeb.decidir(r, self.angulos, v, self.cmd.angular.z, t)
        if self.frenando and not antes:
            self.n_frenados += 1
            self.sentido_freno = 1.0 if v >= 0.0 else -1.0
            self.get_logger().warn(
                f'FRENADO #{self.n_frenados}: TTC = {ttc:.3f} s < umbral {umbral:.3f} s '
                f'(v = {v:.2f} m/s)')
            self._publicar()                     # actuar ya, sin esperar al temporizador
        elif antes and not self.frenando:
            self.get_logger().info('AEB liberado: camino libre')
        if self.csv:
            self.csv.write(f'{t:.3f},{v:.4f},{self.cmd.linear.x:.4f},{self.v_med:.4f},{self.cmd.angular.z:.4f},'
                           f'{min(ttc, 99.0):.4f},{umbral:.4f},{int(self.frenando)}\n')
        self.pub_activo.publish(Bool(data=self.frenando))
        self.pub_ttc.publish(Float64(data=min(ttc, 99.0)))
        self.pub_umbral.publish(Float64(data=umbral))

    def _publicar(self):
        out = Twist()
        out.linear.x = self.cmd.linear.x
        out.angular.z = self.cmd.angular.z
        sin_scan = self.t_scan is None or (self._ahora() - self.t_scan) > self.timeout_scan
        if sin_scan:
            out.linear.x = 0.0                                  # falla segura: sin LiDAR no se avanza
            out.angular.z = 0.0
        elif self.frenando and out.linear.x * self.sentido_freno > 0.0:
            # el comando insiste en ir hacia el obstáculo: se detiene el robot. Girar en el sitio
            # (v = 0) o alejarse (v en sentido contrario) sí se permite.
            out.linear.x = 0.0
            if not self.pasar_omega:
                out.angular.z = 0.0
        self.pub.publish(out)


def main():
    rclpy.init()
    n = NodoAEB()
    def al_salir():
        print(f'[aeb] frenados del AEB en la sesión: {n.n_frenados}', flush=True)
        if n.csv:
            n.csv.close()
    girar(n, al_salir)


if __name__ == '__main__':
    main()
