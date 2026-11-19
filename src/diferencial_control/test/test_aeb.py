"""Pruebas del AEB (sin ROS):  cd diferencial_control && python3 -m pytest test -q"""
import math

import numpy as np

from diferencial_control.aeb import (AEB, distancia_contorno, en_franja, umbral_ttc,
                                     velocidad_para_ttc)

L, A = 0.1986, 0.153
ANG = -math.pi + np.arange(450) * 2 * math.pi / 450


def nuevo():
    return AEB(x_del=L / 2, x_tras=-L / 2, y_izq=A / 2, y_der=-A / 2)


def scan_muro_frontal(d):
    """Muro perpendicular a d metros del LiDAR (al frente)."""
    c = np.cos(ANG)
    r = np.where(c > 1e-6, d / np.maximum(c, 1e-9), np.inf)
    return r


def scan_corredor(semiancho):
    """Corredor recto infinito de ancho 2·semiancho, robot centrado y alineado."""
    s = np.abs(np.sin(ANG))
    return np.where(s > 1e-6, semiancho / np.maximum(s, 1e-9), np.inf)


def test_formula_ttc_rayo_frontal():
    a = nuevo()
    r = scan_muro_frontal(1.0)
    ttc, i, ttc_i = a.ttc(r, ANG, 0.5, 0.0)
    # rayo de 0°: r = 1.0, contorno = L/2  -> TTC = (1 - L/2) / 0.5
    assert math.isclose(ttc, (1.0 - L / 2) / 0.5, rel_tol=1e-6)
    assert abs(ANG[i]) < 1e-9
    # rayos hacia atrás no se acercan: TTC infinito
    assert np.all(np.isinf(ttc_i[np.cos(ANG) < 0]))


def test_retroceso_usa_rayos_traseros():
    a = nuevo()
    r = scan_muro_frontal(0.3)
    ttc, _, _ = a.ttc(r, ANG, -0.5, 0.0)
    assert math.isinf(ttc)                     # alejándose del muro: nunca choca


def test_contorno_rectangulo():
    d = distancia_contorno(np.array([0.0, math.pi / 2, math.pi, -math.pi / 2]), L / 2, -L / 2, A / 2, -A / 2)
    assert np.allclose(d, [L / 2, A / 2, L / 2, A / 2])


def test_sin_frenado_falso_en_estrechamiento():
    """Corredor de 0.40 m (estrechamiento) a 1 m/s: los muros paralelos no están en la franja."""
    a = nuevo()
    frena, ttc, _ = a.decidir(scan_corredor(0.20), ANG, 1.0, 0.0, 0.0)
    assert not frena and ttc > 10


def test_frena_frente_a_muro_y_mantiene():
    a = nuevo()
    v = 1.0
    u = umbral_ttc(v)
    d_disparo = L / 2 + 0.9 * u * v                 # TTC = 0.9 umbral
    frena, ttc, _ = a.decidir(scan_muro_frontal(d_disparo), ANG, v, 0.0, 0.0)
    assert frena and ttc < u
    # robot detenido a 15 cm; el comando sigue pidiendo avanzar lento (0.15): no se libera
    for k in range(1, 20):
        frena, _, _ = a.decidir(scan_muro_frontal(L / 2 + 0.15), ANG, velocidad_para_ttc(0.15, 0.0), 0.0, 0.1 * k)
        assert frena
    # se retira el obstáculo: se libera tras el tiempo mínimo
    frena, _, _ = a.decidir(np.full(450, np.inf), ANG, 0.15, 0.0, 3.0)
    assert not frena


def test_umbral_crece_con_v():
    assert umbral_ttc(1.0) > umbral_ttc(0.5) > umbral_ttc(0.0)
    # a 1 m/s: v/(2a) + latencia + margen
    assert math.isclose(umbral_ttc(1.0, 1.6, 0.12, 0.10), 1.0 / 3.2 + 0.22)


def test_franja_en_curva():
    # girando a la izquierda (R = 1 m): un punto sobre el arco está en la franja; uno recto al frente no
    assert en_franja(np.array([1.0]), np.array([1.0]), 1.0, 1.0, 0.09)[0]
    assert not en_franja(np.array([1.0]), np.array([0.0]), 1.0, 1.0, 0.09)[0]
    # recta
    assert en_franja(np.array([1.0]), np.array([0.05]), 1.0, 0.0, 0.09)[0]
    assert not en_franja(np.array([-1.0]), np.array([0.0]), 1.0, 0.0, 0.09)[0]


def test_velocidad_para_ttc():
    assert velocidad_para_ttc(0.5, 1.0) == 1.0         # frenando: manda la medida
    assert velocidad_para_ttc(1.0, 0.0) == 1.0         # arrancando: manda el comando
    assert velocidad_para_ttc(-0.3, 0.8) == -0.3       # pide retroceder: se evalúa hacia atrás
