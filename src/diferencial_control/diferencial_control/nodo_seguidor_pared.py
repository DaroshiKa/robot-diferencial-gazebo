#!/usr/bin/env python3
"""Modo autónomo REACTIVO: seguimiento de pared con PID (ver seguidor_pared.py).

    /scan --> [alpha, D_t, D_t+1 con los rayos a y b] --> PID --> omega
                                          v según la curvatura ---> /cmd_vel_auto  (-> mux -> AEB)

No usa mapas, waypoints ni trayectorias: cada comando depende solo del barrido actual.

Diagnóstico para sintonizar (rqt_plot o ros2 topic echo):
  /seguidor/diag (Float64MultiArray) = [e, D_t, D_t+1, D_des, alpha, v, omega, P, I, D, ancho, d_frente]
"""
import math
from dataclasses import fields

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

from diferencial_control.util_ros import girar
from diferencial_control.seguidor_pared import ParametrosSeguidor, SeguidorPared


class NodoSeguidor(Node):
    def __init__(self):
        super().__init__('seguidor_pared')
        base = ParametrosSeguidor()
        for f in fields(ParametrosSeguidor):
            self.declare_parameter(f.name, getattr(base, f.name))
        p = ParametrosSeguidor(**{f.name: self.get_parameter(f.name).value
                                  for f in fields(ParametrosSeguidor)})
        self.declare_parameter('csv', '')
        ruta = self.get_parameter('csv').value
        self.csv = open(ruta, 'w') if ruta else None
        self.campos = ('e', 'd_t', 'd_t1', 'd_des', 'alpha', 'v', 'omega', 'p', 'i', 'd', 'ancho', 'd_frente')
        if self.csv:
            self.csv.write('t,' + ','.join(self.campos) + '\n')
        self.seg = SeguidorPared(p)
        self.v_med = 0.0
        self.create_subscription(LaserScan, '/scan', self._cb_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.pub_diag = self.create_publisher(Float64MultiArray, '/seguidor/diag', 10)
        th = math.radians(p.theta_deg)
        self.get_logger().info(
            f'seguimiento de pared {p.lado}: theta={p.theta_deg}°, D_nom={p.d_nom} m, l={p.l_anticipacion} m, '
            f'Kp={p.kp}, Ki={p.ki}, Kd={p.kd} | modelo: wn=sqrt(v Kp)={math.sqrt(p.v_max * p.kp):.2f} rad/s, '
            f'zeta=(l/2)sqrt(Kp/v)={p.l_anticipacion / 2 * math.sqrt(p.kp / p.v_max):.2f} a v={p.v_max} m/s | '
            f'rayos b={90 if p.lado == "izquierda" else -90}°, a={math.degrees(math.pi / 2 - th):.0f}°')

    def _cb_odom(self, msg):
        self.v_med = msg.twist.twist.linear.x

    def _cb_scan(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        r = [x if msg.range_min <= x <= msg.range_max else float('inf') for x in msg.ranges]
        v, w = self.seg.paso(r, msg.angle_min, msg.angle_increment, t, self.v_med)
        out = Twist()
        out.linear.x = v
        out.angular.z = w
        self.pub.publish(out)
        d = self.seg.diag
        datos = [float(d[k]) for k in self.campos]
        self.pub_diag.publish(Float64MultiArray(data=datos))
        if self.csv:
            self.csv.write(f'{t:.3f},' + ','.join(f'{x:.4f}' for x in datos) + '\n')


def main():
    rclpy.init()
    n = NodoSeguidor()
    def al_salir():
        n.pub.publish(Twist())                       # al salir, dejar el robot quieto
        if n.csv:
            n.csv.close()
    girar(n, al_salir)


if __name__ == '__main__':
    main()
