#!/usr/bin/env python3
"""Multiplexor de comandos: lleva el modo activo a /cmd_vel_mux (que luego pasa por el AEB).

    /cmd_vel_teleop (teleop_twist_keyboard) --+
                                              +--> mux --> /cmd_vel_mux --> AEB --> /cmd_vel
    /cmd_vel_auto   (seguidor de pared)     --+

  * modo = 'teleop' | 'auto' (parámetro; se puede cambiar en caliente con
    ros2 param set /mux_cmd modo auto).
  * teleop: se mantiene el último comando (teleop_twist_keyboard publica solo al pulsar una tecla,
    y así se comporta también DiffDrive); 'k' detiene el robot.
  * auto: si el seguidor deja de publicar por más de timeout_auto, se manda velocidad cero.
  * Se recortan v y omega a los límites del robot (1.0 m/s, 4.0 rad/s): ninguna fuente puede
    pedir más de lo permitido en la competencia.
  * Si parar_al_terminar = true y /carrera/terminada = true (nodo vueltas), en modo auto se detiene.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool

from diferencial_control.util_ros import girar


class Mux(Node):
    def __init__(self):
        super().__init__('mux_cmd')
        self.declare_parameter('modo', 'teleop')
        self.declare_parameter('timeout_auto', 0.5)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('w_max', 4.0)
        self.declare_parameter('frecuencia', 50.0)
        self.declare_parameter('parar_al_terminar', True)
        self.modo = self.get_parameter('modo').value
        self.timeout = self.get_parameter('timeout_auto').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.parar_fin = self.get_parameter('parar_al_terminar').value
        self.add_on_set_parameters_callback(self._cambio_param)

        self.ult = {'teleop': (Twist(), None), 'auto': (Twist(), None)}
        self.terminada = False
        self.create_subscription(Twist, '/cmd_vel_teleop', lambda m: self._cb('teleop', m), 10)
        self.create_subscription(Twist, '/cmd_vel_auto', lambda m: self._cb('auto', m), 10)
        self.create_subscription(Bool, '/carrera/terminada', self._cb_fin, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_mux', 10)
        self.create_timer(1.0 / self.get_parameter('frecuencia').value, self._publicar)
        self.get_logger().info(f'modo = {self.modo} (teleop: /cmd_vel_teleop, auto: /cmd_vel_auto)')

    def _cambio_param(self, params):
        for p in params:
            if p.name == 'modo':
                if p.value not in ('teleop', 'auto'):
                    return SetParametersResult(successful=False, reason="modo: 'teleop' | 'auto'")
                self.modo = p.value
                self.get_logger().info(f'modo -> {self.modo}')
        return SetParametersResult(successful=True)

    def _cb(self, fuente, msg):
        self.ult[fuente] = (msg, self.get_clock().now())

    def _cb_fin(self, msg):
        if msg.data and not self.terminada:
            self.get_logger().info('carrera terminada')
        self.terminada = msg.data

    def _publicar(self):
        msg, t = self.ult[self.modo]
        salida = Twist()
        if self.modo == 'teleop':
            salida.linear.x, salida.angular.z = msg.linear.x, msg.angular.z
        elif t is not None and not (self.terminada and self.parar_fin):
            edad = (self.get_clock().now() - t).nanoseconds * 1e-9
            if edad <= self.timeout:
                salida.linear.x, salida.angular.z = msg.linear.x, msg.angular.z
        salida.linear.x = max(-self.v_max, min(self.v_max, salida.linear.x))
        salida.angular.z = max(-self.w_max, min(self.w_max, salida.angular.z))
        self.pub.publish(salida)


def main():
    rclpy.init()
    girar(Mux())


if __name__ == '__main__':
    main()
