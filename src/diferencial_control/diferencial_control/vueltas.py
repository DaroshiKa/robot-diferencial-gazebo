"""Detección de vueltas y error de cierre (funciones puras, sin ROS).

La línea de salida/meta es x = x_linea, |y| <= semiancho (perpendicular a la recta principal, que
se recorre en +x). Una vuelta se cuenta cuando la pose REAL cruza la línea de x < x_linea a
x >= x_linea. El instante de cruce se interpola linealmente entre las dos muestras.

Tiempo: desde el PRIMER cruce (salida; el robot arranca en x = -0.60, detrás de la línea) hasta el
cruce número n_vueltas + 1 (fin de la última vuelta). Todo en tiempo de simulación.

Error de cierre de la vuelta k: pose estimada (odometría) en el instante del cruce k frente a la
pose real en ese mismo instante:  e_pos = ||p_est - p_real||, e_theta = wrap(th_est - th_real).
"""
import bisect
import math


def envolver(a):
    return math.atan2(math.sin(a), math.cos(a))


class Historial:
    """Poses con sello de tiempo; interpolación lineal (ángulo envuelto)."""

    def __init__(self, max_len=20000):
        self.t, self.q = [], []
        self.max_len = max_len

    def agregar(self, t, x, y, th):
        if self.t and t <= self.t[-1]:
            return
        self.t.append(t)
        self.q.append((x, y, th))
        if len(self.t) > self.max_len:
            del self.t[:self.max_len // 2], self.q[:self.max_len // 2]

    def en(self, t):
        if not self.t:
            return None
        if t <= self.t[0]:
            return self.q[0]
        if t >= self.t[-1]:
            return self.q[-1]
        i = bisect.bisect_left(self.t, t)
        t0, t1 = self.t[i - 1], self.t[i]
        a = (t - t0) / (t1 - t0)
        (x0, y0, h0), (x1, y1, h1) = self.q[i - 1], self.q[i]
        return (x0 + a * (x1 - x0), y0 + a * (y1 - y0), envolver(h0 + a * envolver(h1 - h0)))


class ContadorVueltas:
    def __init__(self, x_linea=0.0, semiancho=0.6, n_vueltas=3):
        self.x_linea, self.semiancho, self.n_vueltas = x_linea, semiancho, n_vueltas
        self.ant = None
        self.cruces = []           # instantes de cruce [s]

    def actualizar(self, t, x, y):
        """Devuelve el instante del cruce si (t, x, y) completa un cruce válido, si no None."""
        cruce = None
        if self.ant is not None:
            t0, x0, y0 = self.ant
            if x0 < self.x_linea <= x and abs(y) <= self.semiancho and abs(y0) <= self.semiancho:
                a = (self.x_linea - x0) / (x - x0) if x != x0 else 0.0
                cruce = t0 + a * (t - t0)
                self.cruces.append(cruce)
        self.ant = (t, x, y)
        return cruce

    @property
    def vueltas(self):
        return max(0, len(self.cruces) - 1)

    @property
    def terminada(self):
        return self.vueltas >= self.n_vueltas

    def tiempos(self):
        return [b - a for a, b in zip(self.cruces[:-1], self.cruces[1:])]


def error_cierre(est, real):
    ex, ey = est[0] - real[0], est[1] - real[1]
    return math.hypot(ex, ey), envolver(est[2] - real[2]), ex, ey
