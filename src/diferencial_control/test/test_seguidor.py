"""Pruebas del seguimiento de pared (sin ROS)."""
import math

import numpy as np

from diferencial_control.seguidor_pared import ParametrosSeguidor, SeguidorPared, geometria_pared

N = 450
INC = 2 * math.pi / N
ANG0 = -math.pi
ANG = ANG0 + np.arange(N) * INC


def scan_corredor(d_izq, d_der, psi=0.0):
    """Corredor recto: muro izquierdo a d_izq, derecho a d_der; robot girado psi (rad, + izquierda)."""
    th = ANG + psi                               # dirección del rayo en el marco del corredor
    s = np.sin(th)
    r = np.full(N, np.inf)
    izq = s > 1e-6
    der = s < -1e-6
    r[izq] = d_izq / s[izq]
    r[der] = d_der / -s[der]
    return list(r)


def test_alpha_y_distancia_paralelo():
    r = scan_corredor(0.3, 0.5)
    al, d_t, d_t1, a, b = geometria_pared(r, ANG0, INC, +1, math.radians(45), 0.45, ventana=0)
    assert abs(al) < 0.01 and abs(d_t - 0.3) < 0.005 and abs(d_t1 - 0.3) < 0.005


def test_alpha_signo():
    # robot girado 10° hacia la izquierda (hacia el muro izquierdo): alpha < 0 (se ACERCA)
    r = scan_corredor(0.3, 0.5, psi=math.radians(10))
    al, d_t, d_t1, _, _ = geometria_pared(r, ANG0, INC, +1, math.radians(45), 0.45, ventana=0)
    assert al < 0 and abs(math.degrees(al) + 10) < 1.0
    assert abs(d_t - 0.3) < 0.01 and d_t1 < d_t                      # proyectada más cerca
    # el mismo giro visto desde el muro derecho: se aleja de él (alpha > 0)
    al_d, _, _, _, _ = geometria_pared(r, ANG0, INC, -1, math.radians(45), 0.45, ventana=0)
    assert al_d > 0


def test_corrige_hacia_el_centro():
    s = SeguidorPared(ParametrosSeguidor())
    # demasiado cerca del muro izquierdo en un corredor de 0.8 m -> gira a la derecha
    v, w = s.paso(scan_corredor(0.25, 0.55), ANG0, INC, 0.0)
    assert w < 0
    s = SeguidorPared(ParametrosSeguidor())
    v, w = s.paso(scan_corredor(0.55, 0.25), ANG0, INC, 0.0)
    assert w > 0


def test_centrado_en_estrechamiento():
    """W = 0.40 m -> D_des = 0.20 (centro) aunque D_nom = 0.40."""
    s = SeguidorPared(ParametrosSeguidor())
    for k in range(5):
        s.paso(scan_corredor(0.20, 0.20), ANG0, INC, 0.1 * k)
    assert abs(s.diag['d_des'] - 0.20) < 0.01 and abs(s.diag['omega']) < 0.05


def test_mas_lento_en_curva():
    p = ParametrosSeguidor()
    s = SeguidorPared(p)
    for k in range(30):                      # en recta alcanza v_max
        v, _ = s.paso(scan_corredor(0.4, 0.4), ANG0, INC, 0.1 * k)
    assert math.isclose(v, p.v_max)
    s2 = SeguidorPared(p)
    for k in range(30):                      # error grande -> |omega| grande -> v menor
        v2, w2 = s2.paso(scan_corredor(0.15, 0.65), ANG0, INC, 0.1 * k)
    assert v2 < p.v_max and abs(w2) > 0


def test_modelo_amortiguamiento():
    """zeta = (l/2) sqrt(Kp/v) y wn = sqrt(v Kp) con los valores por defecto (justificación del README)."""
    p = ParametrosSeguidor()
    zeta = p.l_anticipacion / 2 * math.sqrt(p.kp / p.v_max)
    wn = math.sqrt(p.v_max * p.kp)
    assert 0.6 < zeta < 0.9 and 2.0 < wn < 4.0
