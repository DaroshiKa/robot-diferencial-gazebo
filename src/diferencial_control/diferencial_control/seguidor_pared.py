"""Seguimiento de pared con LiDAR y PID (funciones puras, sin ROS).

Geometría (dos rayos sobre la misma pared, enunciado §3.2-4). Para el muro IZQUIERDO:
    rayo b a +90° (perpendicular al robot) y rayo a a +90° - theta (adelantado theta respecto a b)

    alpha   = arctan( (a cos(theta) - b) / (a sin(theta)) )   ángulo entre el rumbo y la pared
                                                                (> 0: el robot se ALEJA de la pared)
    D_t     = b cos(alpha)                                      distancia actual a la pared
    D_t+1   = D_t + l sin(alpha)                                distancia proyectada a l metros
    e       = D_des - D_t+1

Con el muro a la izquierda, e > 0 significa "demasiado cerca": hay que girar a la DERECHA (omega < 0),
así que omega = -PID(e); para el muro derecho omega = +PID(e).

Modelo para sintonizar (pequeños ángulos, v constante, d = distancia al muro izquierdo):
    d_dot = v alpha,  alpha_dot = -omega,  omega = -Kp (D_des - d - l alpha)
    =>  d_ddot + Kp l d_dot + v Kp d = v Kp D_des
    =>  wn = sqrt(v Kp),  zeta = (l / 2) sqrt(Kp / v)
La distancia de anticipación l funciona como el término derivativo (l alpha ≈ l d_dot / v).

Distancia deseada adaptativa: D_des = min(D_nom, W/2), con W = D_izq + D_der el ancho del corredor
medido con los mismos dos rayos en cada lado. En el estrechamiento (W = 0.40 m) el robot se centra
solo; en el resto sigue el muro a D_nom.

Velocidad según la curvatura (más lento en curvas):
    v <= sqrt(a_lat / |kappa|) con kappa = omega / v  ->  v <= a_lat / |omega|  (aceleración lateral)
    v <= sqrt(2 a_frenado (d_frente - d_seguridad))                   (pared de frente = curva cerrada)
    v <= v_max
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ParametrosSeguidor:
    lado: str = 'izquierda'          # 'izquierda' | 'derecha'
    theta_deg: float = 45.0          # ángulo entre los rayos a y b
    d_nom: float = 0.40              # m, distancia nominal a la pared seguida
    l_anticipacion: float = 0.45     # m, distancia de anticipación l
    kp: float = 10.0                 # rad/s por m
    ki: float = 0.3                  # rad/s por m·s
    kd: float = 0.3                  # rad/s por m/s
    tau_d: float = 0.1               # s, filtro del término derivativo
    i_max: float = 0.3               # m·s, anti-windup (|integral| máx.)
    v_max: float = 1.0               # m/s (límite del robot)
    v_min: float = 0.15              # m/s
    w_max: float = 4.0               # rad/s (límite del robot)
    a_lat: float = 1.2               # m/s², aceleración lateral admisible
    a_frenado: float = 1.2           # m/s², para la velocidad según la pared de frente
    d_seguridad: float = 0.30        # m, distancia LiDAR-pared de frente a la que v -> v_min
    cono_frente_deg: float = 12.0    # ±, rayos usados para la distancia de frente
    ventana: int = 2                 # ± rayos alrededor de a y b (mediana -> rechaza ruido σ=1 cm)
    a_v: float = 2.0                 # m/s², rampa de la referencia de velocidad (= a_max del robot)
    adaptativa: bool = True          # D_des = min(D_nom, W/2)
    margen_centro: float = 0.0       # m, se resta a W/2 (0 = centrado exacto en el estrechamiento)
    vel_d_des: float = 50.0          # m/s, rapidez máx. de cambio de D_des (transición suave)
    e_max: float = 1.0               # m, saturación del error (evita volantazos ante saltos de D)
    usar_ancho_adelante: bool = True   # también mide el ancho con los rayos a (anticipa el estrechamiento)
    r_max_valido: float = 3.0        # m, rayos más largos se consideran "sin pared"


def _rango(rangos, ang_min, inc, ang, ventana, r_max):
    n = len(rangos)
    i = int(round((ang - ang_min) / inc)) % n
    idx = [(i + k) % n for k in range(-ventana, ventana + 1)]
    v = np.asarray([rangos[j] for j in idx], dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return r_max
    return float(np.median(v))


def geometria_pared(rangos, ang_min, inc, lado, theta, l, ventana=2, r_max=3.0):
    """Devuelve (alpha, D_t, D_t+1, a, b) para la pared del lado indicado (+1 izq, -1 der).

    Para el lado derecho se usa la misma fórmula con los ángulos reflejados, de modo que
    alpha > 0 siempre significa "alejándose de esa pared"."""
    s = 1.0 if lado > 0 else -1.0
    b = min(_rango(rangos, ang_min, inc, s * math.pi / 2, ventana, r_max), r_max)
    a = min(_rango(rangos, ang_min, inc, s * (math.pi / 2 - theta), ventana, r_max), r_max)
    alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
    d_t = b * math.cos(alpha)
    d_t1 = d_t + l * math.sin(alpha)
    return alpha, d_t, d_t1, a, b


def distancia_frente(rangos, ang_min, inc, cono, r_max=12.0):
    n = len(rangos)
    i0 = int(round((0.0 - ang_min) / inc)) % n
    k = int(math.ceil(cono / inc))
    v = np.asarray([rangos[(i0 + j) % n] for j in range(-k, k + 1)], dtype=float)
    v = v[np.isfinite(v)]
    return float(v.min()) if v.size else r_max


class PID:
    def __init__(self, kp, ki, kd, tau_d=0.1, i_max=1.0):
        self.kp, self.ki, self.kd, self.tau_d, self.i_max = kp, ki, kd, tau_d, i_max
        self.reiniciar()

    def reiniciar(self):
        self.i = 0.0
        self.d = 0.0
        self.e_ant = None

    def __call__(self, e, dt, saturado=False):
        if self.e_ant is None or dt <= 0.0:
            self.e_ant = e
            return self.kp * e + self.ki * self.i
        # derivada filtrada (primer orden) para no amplificar el ruido del LiDAR
        d_crudo = (e - self.e_ant) / dt
        a = dt / (self.tau_d + dt)
        self.d += a * (d_crudo - self.d)
        self.e_ant = e
        # anti-windup: integración condicional (no integra si la salida está saturada) + límite
        if not saturado:
            self.i = max(-self.i_max, min(self.i_max, self.i + e * dt))
        return self.kp * e + self.ki * self.i + self.kd * self.d


class SeguidorPared:
    def __init__(self, p: ParametrosSeguidor = None):
        self.p = p or ParametrosSeguidor()
        self.pid = PID(self.p.kp, self.p.ki, self.p.kd, self.p.tau_d, self.p.i_max)
        self.t_ant = None
        self.v_ref = 0.0
        self.saturado = False
        self.d_des = None
        self.diag = {}

    def paso(self, rangos, ang_min, inc, t, v_med=None):
        """Un barrido -> (v, omega). t en segundos (tiempo de simulación)."""
        p = self.p
        dt = 0.1 if self.t_ant is None else max(1e-3, t - self.t_ant)
        self.t_ant = t
        th = math.radians(p.theta_deg)
        s = 1 if p.lado == 'izquierda' else -1

        al, d_t, d_t1, a, b = geometria_pared(rangos, ang_min, inc, s, th, p.l_anticipacion,
                                              p.ventana, p.r_max_valido)
        d_obj = p.d_nom
        ancho = float('nan')
        if p.adaptativa:
            _, d_otro, d_otro_t1, a_otro, _ = geometria_pared(rangos, ang_min, inc, -s, th,
                                                              p.l_anticipacion, p.ventana,
                                                              p.r_max_valido)
            if p.usar_ancho_adelante:
                ancho = d_t1 + d_otro_t1          # ancho proyectado a l metros (ve venir el estrechamiento)
            else:
                ancho = d_t + d_otro              # ancho a la altura del robot
            d_obj = min(p.d_nom, ancho / 2.0 - p.margen_centro)
        # D_des con rapidez de cambio limitada (vel_d_des grande = sin límite)
        if self.d_des is None:
            self.d_des = d_obj
        paso = p.vel_d_des * dt
        self.d_des += max(-paso, min(paso, d_obj - self.d_des))
        d_des = self.d_des
        e = max(-p.e_max, min(p.e_max, d_des - d_t1))

        u = self.pid(e, dt, self.saturado)
        omega = -u if s > 0 else u
        omega_sat = max(-p.w_max, min(p.w_max, omega))
        self.saturado = omega_sat != omega
        omega = omega_sat

        # ---- velocidad según la curvatura ----
        d_fr = distancia_frente(rangos, ang_min, inc, math.radians(p.cono_frente_deg))
        v_curva = p.a_lat / abs(omega) if abs(omega) > 1e-6 else p.v_max
        v_frente = math.sqrt(max(0.0, 2.0 * p.a_frenado * (d_fr - p.d_seguridad)))
        v_obj = max(p.v_min, min(p.v_max, v_curva, v_frente))
        # rampa (la subida no puede superar a_v; la bajada es inmediata)
        if v_obj > self.v_ref:
            self.v_ref = min(v_obj, self.v_ref + p.a_v * dt)
        else:
            self.v_ref = v_obj
        self.diag = dict(alpha=al, d_t=d_t, ancho=ancho, d_t1=d_t1, d_des=d_des, e=e, a=a, b=b, d_frente=d_fr,
                         v_curva=v_curva, v_frente=v_frente, omega=omega, v=self.v_ref,
                         p=self.pid.kp * e, i=self.pid.ki * self.pid.i, d=self.pid.kd * self.pid.d)
        return self.v_ref, omega
