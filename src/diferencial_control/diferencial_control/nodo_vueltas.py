#!/usr/bin/env python3
"""Cronómetro de vueltas + error de cierre de la odometría por vuelta (para el pitch).

Entradas:
  /ground_truth     pose REAL (plugin OdometryPublisher)  -> cruces de la línea y referencia
  /odom_cinematica  pose ESTIMADA por el nodo de cinemática (promedio de ruedas por lado)
  /odom             odometría del DiffDrive (se lleva al marco world con la pose inicial)
  /scan             holgura mínima contorno-obstáculo en cada vuelta (aprox. de "contactos")
  /aeb/activo       frenados del AEB en cada vuelta

Salidas:
  /carrera/terminada (Bool, latched) tras n_vueltas vueltas (el mux puede detener el robot)
  /carrera/vueltas (Int32)
  <salida>/vueltas.csv     vuelta, t_cruce, tiempo, e_pos y e_theta (cinemática y DiffDrive), ...
  <salida>/trayectoria.csv t, pose real, estimada y DiffDrive (para graficar con analizar_odometria)

El tiempo es de simulación (/clock, use_sim_time): desde el 1.er cruce (salida) hasta el cruce
n_vueltas + 1.
"""
import csv
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32

from diferencial_control.util_ros import girar
from diferencial_control.aeb import distancia_contorno
from diferencial_control.cinematica import yaw_de_cuaternion
from diferencial_control.vueltas import ContadorVueltas, Historial, envolver, error_cierre


def _pose(msg):
    p, o = msg.pose.pose.position, msg.pose.pose.orientation
    return p.x, p.y, yaw_de_cuaternion(o.x, o.y, o.z, o.w)


def _t(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class NodoVueltas(Node):
    def __init__(self):
        super().__init__('vueltas')
        self.declare_parameter('x_linea', 0.0)
        self.declare_parameter('semiancho_linea', 0.6)
        self.declare_parameter('n_vueltas', 3)
        self.declare_parameter('salida', '')
        self.declare_parameter('etiqueta', '')
        self.declare_parameter('largo', 0.1986)
        self.declare_parameter('ancho', 0.153)
        self.declare_parameter('lidar_x', 0.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cont = ContadorVueltas(g('x_linea'), g('semiancho_linea'), g('n_vueltas'))
        self.L, self.A, self.lx = g('largo'), g('ancho'), g('lidar_x')

        salida = g('salida')
        if not salida:
            etiqueta = g('etiqueta') or 'corrida'
            salida = os.path.join(os.path.expanduser('~'), 'resultados_robot_diferencial',
                                  time.strftime('%Y%m%d_%H%M%S_') + etiqueta)
        os.makedirs(salida, exist_ok=True)
        self.salida = salida
        self.f_tray = open(os.path.join(salida, 'trayectoria.csv'), 'w', newline='')
        self.w_tray = csv.writer(self.f_tray)
        self.w_tray.writerow(['t', 'x_real', 'y_real', 'th_real', 'x_est', 'y_est', 'th_est',
                              'x_dd', 'y_dd', 'th_dd', 'vuelta'])
        self.f_v = open(os.path.join(salida, 'vueltas.csv'), 'w', newline='')
        self.w_v = csv.writer(self.f_v)
        self.w_v.writerow(['vuelta', 't_cruce', 'tiempo_vuelta', 'tiempo_acumulado',
                           'e_pos_cin', 'e_th_cin_deg', 'e_pos_dd', 'e_th_dd_deg',
                           'x_real', 'y_real', 'th_real_deg', 'x_est', 'y_est', 'th_est_deg',
                           'holgura_min_cm', 'frenados_aeb'])

        self.h_real, self.h_est, self.h_dd = Historial(), Historial(), Historial()
        self.T0 = None               # pose inicial real (para llevar /odom de DiffDrive a world)
        self.holgura = math.inf
        self.frenados = 0
        self.aeb_ant = False
        self.filas = []
        self.contorno = None
        self.t_ult_escritura = -1.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_fin = self.create_publisher(Bool, '/carrera/terminada', latched)
        self.pub_n = self.create_publisher(Int32, '/carrera/vueltas', 10)
        self.pub_fin.publish(Bool(data=False))
        # trayectorias para RViz (marco world): real, estimada (cinemática) y DiffDrive
        self.rutas = {k: Path() for k in ('real', 'estimada', 'diffdrive')}
        self.pub_rutas = {k: self.create_publisher(Path, f'/ruta/{k}', 1) for k in self.rutas}
        self.t_ult_ruta = -1.0
        self.create_timer(1.0, self._publicar_rutas)
        self.create_subscription(Odometry, '/ground_truth', self._cb_real, 50)
        self.create_subscription(Odometry, '/odom_cinematica', self._cb_est, 50)
        self.create_subscription(Odometry, '/odom', self._cb_dd, 50)
        self.create_subscription(LaserScan, '/scan', self._cb_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/aeb/activo', self._cb_aeb, 10)
        self.get_logger().info(f'línea de meta en x = {g("x_linea")} m; {g("n_vueltas")} vueltas; '
                               f'resultados en {salida}')

    # ---------------- entradas ----------------
    def _cb_est(self, msg):
        self.h_est.agregar(_t(msg.header.stamp), *_pose(msg))

    def _cb_dd(self, msg):
        if self.T0 is None:
            return
        xo, yo, tho = _pose(msg)
        x0, y0, th0 = self.T0
        c, s = math.cos(th0), math.sin(th0)
        self.h_dd.agregar(_t(msg.header.stamp), x0 + c * xo - s * yo, y0 + s * xo + c * yo,
                          envolver(th0 + tho))

    def _cb_scan(self, msg):
        n = len(msg.ranges)
        if self.contorno is None or len(self.contorno) != n:
            ang = msg.angle_min + msg.angle_increment * np.arange(n)
            self.contorno = distancia_contorno(ang, self.L / 2 - self.lx, -self.L / 2 - self.lx,
                                               self.A / 2, -self.A / 2)
        r = np.asarray(msg.ranges, dtype=float)
        ok = np.isfinite(r) & (r >= msg.range_min)
        if ok.any() and len(self.cont.cruces) >= 1:
            self.holgura = min(self.holgura, float(np.min(r[ok] - self.contorno[ok])))

    def _cb_aeb(self, msg):
        if msg.data and not self.aeb_ant and len(self.cont.cruces) >= 1:
            self.frenados += 1
        self.aeb_ant = msg.data

    def _cb_real(self, msg):
        t = _t(msg.header.stamp)
        x, y, th = _pose(msg)
        if self.T0 is None:
            self.T0 = (x, y, th)
        self.h_real.agregar(t, x, y, th)
        if t - self.t_ult_escritura >= 0.05:          # 20 Hz en el CSV
            self.t_ult_escritura = t
            e = self.h_est.en(t) or (math.nan,) * 3
            d = self.h_dd.en(t) or (math.nan,) * 3
            self.w_tray.writerow([f'{t:.3f}', x, y, th, *e, *d, self.cont.vueltas])
        if t - self.t_ult_ruta >= 0.2:
            self.t_ult_ruta = t
            for k, q in (('real', (x, y, th)), ('estimada', self.h_est.en(t)), ('diffdrive', self.h_dd.en(t))):
                if q is not None:
                    self._agregar_ruta(k, msg.header.stamp, q)
        if self.cont.terminada:
            return
        tc = self.cont.actualizar(t, x, y)
        if tc is not None:
            self._registrar_cruce(tc)

    def _agregar_ruta(self, k, stamp, q):
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp = stamp
        ps.pose.position.x, ps.pose.position.y = float(q[0]), float(q[1])
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(q[2] / 2), math.cos(q[2] / 2)
        ruta = self.rutas[k]
        ruta.poses.append(ps)
        if len(ruta.poses) > 6000:
            del ruta.poses[:1000]

    def _publicar_rutas(self):
        for k, ruta in self.rutas.items():
            ruta.header.frame_id = 'world'
            ruta.header.stamp = self.get_clock().now().to_msg()
            self.pub_rutas[k].publish(ruta)

    # ---------------- cruce ----------------
    def _registrar_cruce(self, tc):
        k = self.cont.vueltas
        real = self.h_real.en(tc)
        est = self.h_est.en(tc)
        dd = self.h_dd.en(tc)
        e_c = error_cierre(est, real) if est else (math.nan,) * 4
        e_d = error_cierre(dd, real) if dd else (math.nan,) * 4
        tiempos = self.cont.tiempos()
        t_v = tiempos[-1] if tiempos else 0.0
        t_acum = sum(tiempos)
        hol = self.holgura * 100 if math.isfinite(self.holgura) else math.nan
        fila = [k, round(tc, 3), round(t_v, 3), round(t_acum, 3),
                round(e_c[0], 4), round(math.degrees(e_c[1]), 2),
                round(e_d[0], 4), round(math.degrees(e_d[1]), 2),
                round(real[0], 3), round(real[1], 3), round(math.degrees(real[2]), 2),
                round(est[0], 3) if est else math.nan, round(est[1], 3) if est else math.nan,
                round(math.degrees(est[2]), 2) if est else math.nan,
                round(hol, 1), self.frenados]
        self.w_v.writerow(fila)
        self.f_v.flush()
        self.filas.append(fila)
        if k == 0:
            self.get_logger().info(f'SALIDA: cruce de la línea en t = {tc:.2f} s (cronómetro en marcha)')
        else:
            self.get_logger().info(
                f'VUELTA {k}: {t_v:.2f} s (total {t_acum:.2f} s) | cierre cinemática: '
                f'{e_c[0] * 100:.1f} cm, {math.degrees(e_c[1]):.1f}° | DiffDrive: {e_d[0] * 100:.1f} cm, '
                f'{math.degrees(e_d[1]):.1f}° | holgura mín {hol:.1f} cm | frenados AEB {self.frenados}')
        self.holgura = math.inf
        self.frenados = 0
        self.pub_n.publish(Int32(data=k))
        if self.cont.terminada:
            self.pub_fin.publish(Bool(data=True))
            self._resumen()

    def _resumen(self):
        tiempos = self.cont.tiempos()
        lineas = ['', '================ CARRERA TERMINADA (tiempo de simulación) ================',
                  ' vuelta | tiempo [s] | cierre cinem. [cm, °] | cierre DiffDrive [cm, °] | holgura [cm] | AEB']
        for f in self.filas[1:]:
            lineas.append(f' {f[0]:6d} | {f[2]:10.2f} | {f[4] * 100:9.1f} {f[5]:9.1f} | '
                          f'{f[6] * 100:10.1f} {f[7]:11.1f} | {f[14]:12.1f} | {f[15]:3d}')
        lineas.append(f' TOTAL  | {sum(tiempos):10.2f} |')
        lineas.append(f' resultados: {self.salida}')
        self.get_logger().info('\n'.join(lineas))
        self.f_tray.flush()

    def cerrar(self):
        for f in (self.f_tray, self.f_v):
            try:
                f.close()
            except Exception:
                pass


def main():
    rclpy.init()
    n = NodoVueltas()
    girar(n, n.cerrar)


if __name__ == '__main__':
    main()
