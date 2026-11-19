"""Frenado autónomo de emergencia (AEB) por tiempo a la colisión (TTC). Funciones puras, sin ROS.

Para cada rayo i del LiDAR (ángulo theta_i en el marco del robot, distancia r_i):

    r_dot_i = -v cos(theta_i)                (rapidez con que se acorta r_i si el obstáculo es fijo)
    TTC_i   = r_i / max(-r_dot_i, 0)         (infinito si el rayo no se acorta)

y se frena cuando  min_i TTC_i < umbral.

Dos refinamientos (justificados en el README) evitan frenados falsos sin cambiar la fórmula:

  1. r_i se mide desde el LiDAR, pero lo que choca es el CONTORNO del robot. Se usa
     r_i' = r_i - d_contorno(theta_i), la distancia del contorno rectangular al obstáculo a lo
     largo del mismo rayo (para un robot de 0.2 m de largo, eso son ~10 cm al frente).
  2. Solo cuentan los puntos que están dentro de la FRANJA que barrerá el robot (ancho del robot
     + margen), siguiendo el arco de radio R = v/omega del comando actual. Un muro paralelo
     (estrechamiento) o el muro exterior de una curva que el robot ya está tomando quedan fuera
     de la franja y no producen frenado; un obstáculo en la trayectoria sí.

Umbral dependiente de la velocidad (en lugar de un número fijo):

    umbral(v) = |v| / (2 a_freno) + t_latencia + t_margen

|v|/(2 a) es el TTC equivalente a la distancia de frenado v²/(2a) recorrida a velocidad v;
t_latencia cubre la edad del barrido (1/f_LiDAR = 0.1 s) y el transporte por el bridge; t_margen
deja holgura para el ruido (σ = 1 cm) y la discretización.
"""
import math

import numpy as np


def umbral_ttc(v, a_freno=1.6, t_latencia=0.12, t_margen=0.10):
    return abs(v) / (2.0 * a_freno) + t_latencia + t_margen


def distancia_contorno(theta, x_del, x_tras, y_izq, y_der):
    """Distancia desde el LiDAR hasta el borde del rectángulo del robot a lo largo de theta.

    El rectángulo se da en el marco del LiDAR: x_del > 0 (frente), x_tras < 0, y_izq > 0, y_der < 0.
    """
    c, s = np.cos(theta), np.sin(theta)
    with np.errstate(divide='ignore', invalid='ignore'):
        tx = np.where(c > 1e-9, x_del / c, np.where(c < -1e-9, x_tras / c, np.inf))
        ty = np.where(s > 1e-9, y_izq / s, np.where(s < -1e-9, y_der / s, np.inf))
    return np.minimum(tx, ty)


def en_franja(x, y, v, omega, semiancho, horizonte_ang=math.pi / 2):
    """True para los puntos (x, y) [marco base_link] que el robot barrería con el comando (v, omega).

    - |omega| pequeño: recta, franja |y| <= semiancho por delante (o por detrás si v < 0).
    - si no: arco de radio R = v/omega alrededor del CIR (0, R); el punto debe estar en el anillo
      |R| ± semiancho y a menos de horizonte_ang de recorrido angular por delante.
    """
    x = np.asarray(x)
    y = np.asarray(y)
    sentido = 1.0 if v >= 0 else -1.0
    if abs(omega) < 1e-3 or abs(v / omega) > 20.0:
        return (sentido * x > 0) & (np.abs(y) <= semiancho)
    R = v / omega                            # con signo: > 0 CIR a la izquierda
    dx, dy = x, y - R
    rho = np.hypot(dx, dy)
    dentro = np.abs(rho - abs(R)) <= semiancho
    # ángulo recorrido sobre el arco desde el robot (0, 0) hasta el punto, en el sentido de avance
    ang0 = math.atan2(-R, 0.0)
    ang = np.arctan2(dy, dx)
    giro = math.copysign(1.0, omega)           # todo el cuerpo rota alrededor del CIR con omega
    recorrido = np.mod(giro * (ang - ang0), 2 * math.pi)
    return dentro & (recorrido <= horizonte_ang)


class AEB:
    """Decisión de frenado con histéresis. Uso: decidir(...) en cada barrido."""

    def __init__(self, x_del, x_tras, y_izq, y_der, lidar_x=0.0, lidar_y=0.0, margen_lateral=0.015,
                 a_freno=1.6, t_latencia=0.12, t_margen=0.10, histeresis=1.3, t_min_frenado=0.5,
                 r_min=0.03, v_liberacion=1.0):
        # contorno del robot en el marco del LiDAR
        self.x_del, self.x_tras = x_del - lidar_x, x_tras - lidar_x
        self.y_izq, self.y_der = y_izq - lidar_y, y_der - lidar_y
        self.lidar_x, self.lidar_y = lidar_x, lidar_y
        self.semiancho = max(y_izq, -y_der) + margen_lateral
        self.a_freno, self.t_lat, self.t_mar = a_freno, t_latencia, t_margen
        self.histeresis = histeresis
        self.t_min = t_min_frenado
        self.r_min = r_min
        self.v_lib = v_liberacion
        self.frenando = False
        self.t_inicio = None
        self._cache = None

    def ttc(self, rangos, angulos, v, omega):
        """Devuelve (ttc_min, i_min, ttc_por_rayo)."""
        r = np.asarray(rangos, dtype=float)
        th = np.asarray(angulos, dtype=float)
        validos = np.isfinite(r) & (r >= self.r_min)
        clave = (len(th), float(th[0]), float(th[-1]))
        if self._cache is None or self._cache[0] != clave:
            d = distancia_contorno(th, self.x_del, self.x_tras, self.y_izq, self.y_der)
            self._cache = (clave, d, np.cos(th), np.sin(th))
        _, d_cont, c, s = self._cache
        r_ef = np.maximum(r - d_cont, 1e-3)                 # contorno -> obstáculo
        r_fin = np.where(validos, r, 1e3)                   # inf/nan -> lejos (sin NaN en x, y)
        r_ef = np.where(validos, r_ef, 1e3)
        x = self.lidar_x + r_fin * c                        # punto en base_link
        y = self.lidar_y + r_fin * s
        franja = en_franja(x, y, v, omega, self.semiancho)
        r_dot = -v * c
        with np.errstate(divide='ignore'):
            ttc = np.where(validos & franja & (-r_dot > 1e-6), r_ef / np.maximum(-r_dot, 1e-9), np.inf)
        i = int(np.argmin(ttc))
        return float(ttc[i]), i, ttc

    def decidir(self, rangos, angulos, v, omega, t):
        """v: velocidad para el TTC (ver nodo), omega: comando angular, t: tiempo [s].

        Devuelve (frenar, ttc_min, umbral)."""
        ttc_min, _, _ = self.ttc(rangos, angulos, v, omega)
        u = umbral_ttc(v, self.a_freno, self.t_lat, self.t_mar)
        if not self.frenando:
            if ttc_min < u:
                self.frenando = True
                self.t_inicio = t
        elif (t - self.t_inicio) >= self.t_min:
            # Liberación: el camino debe estar libre (con histéresis) para avanzar a v_liberacion,
            # no solo a la velocidad que pida el comando en ese momento. Si no, un comando lento
            # (el seguidor reduce v frente al obstáculo) liberaría el freno y el robot se
            # "arrastraría" hasta el obstáculo. Así el robot queda detenido mientras el obstáculo
            # siga delante, y continúa solo cuando se retira.
            if v != 0.0 and abs(v) < self.v_lib:
                v_l = math.copysign(self.v_lib, v)
                ttc_l, _, _ = self.ttc(rangos, angulos, v_l, omega * self.v_lib / abs(v))
                u_l = umbral_ttc(v_l, self.a_freno, self.t_lat, self.t_mar)
            else:
                ttc_l, u_l = ttc_min, u
            if ttc_l > self.histeresis * u_l:
                self.frenando = False
        return self.frenando, ttc_min, u


def velocidad_para_ttc(v_cmd, v_med):
    """Velocidad con la que se evalúa el TTC.

    Si el comando y la medida van en el mismo sentido se toma la de mayor magnitud (conservador:
    al arrancar manda el comando, al frenar manda la medida). Si el comando pide ir en sentido
    contrario (p. ej. retroceder para salir), se usa el comando: alejarse siempre se permite.
    Con el robot detenido por el AEB, v_med = 0 y el comando que insiste en avanzar mantiene el
    TTC bajo -> el frenado se sostiene sin oscilar (no hay ciclo frenar/soltar).
    """
    if v_cmd * v_med <= 0.0:
        return v_cmd
    return math.copysign(max(abs(v_cmd), abs(v_med)), v_cmd)
