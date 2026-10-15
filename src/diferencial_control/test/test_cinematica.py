"""Pruebas de la cinemática (sin ROS):  cd diferencial_control && python3 -m pytest test -q"""
import math

import numpy as np

from diferencial_control.cinematica import (directa_u, envolver, integrar, inversa_w,
                                            matriz_A, matriz_T)

R, B = 0.05, 0.30


def test_giro_en_sitio_y_recta():
    v, om = directa_u(10.0, 10.0, R, B)
    assert math.isclose(v, 0.5) and math.isclose(om, 0.0, abs_tol=1e-12)
    v, om = directa_u(5.0, -5.0, R, B)                    # derecha adelante, izquierda atrás
    assert math.isclose(v, 0.0, abs_tol=1e-12) and math.isclose(om, 2 * R * 5.0 / B)
    assert om > 0                                         # antihorario (REP-103)


def test_inversa_es_inversa():
    for v, om in [(0.3, 0.0), (0.0, 1.0), (0.4, -0.7)]:
        wd, wi = inversa_w(v, om, R, B)
        v2, om2 = directa_u(wd, wi, R, B)
        assert math.isclose(v, v2, abs_tol=1e-12) and math.isclose(om, om2, abs_tol=1e-12)
        # fórmulas cerradas
        assert math.isclose(wd, (v + om * B / 2) / R) and math.isclose(wi, (v - om * B / 2) / R)


def test_T_igual_J_por_A():
    th = 0.7
    q_dot = matriz_T(R, B, th) @ np.array([8.0, 6.0])
    v, om = directa_u(8.0, 6.0, R, B)
    assert np.allclose(q_dot, [v * math.cos(th), v * math.sin(th), om])
    assert np.linalg.matrix_rank(matriz_A(R, B)) == 2      # invertible (b != 0, r != 0)


def test_circulo_cierra():
    """v y omega constantes durante un periodo completo -> vuelve al origen."""
    v, om = 0.3, 0.6
    T = 2 * math.pi / om
    n = 20000
    for metodo, tol in (('punto_medio', 1e-6), ('euler', 5e-2)):
        x = y = th = 0.0
        for _ in range(n):
            x, y, th = integrar(x, y, th, v, om, T / n, metodo)
        assert math.hypot(x, y) < tol, (metodo, math.hypot(x, y))


def test_punto_medio_mejor_que_euler():
    v, om, n = 0.5, 1.0, 125
    dt = 2 * math.pi / om / n                              # exactamente un periodo
    res = {}
    for metodo in ('euler', 'punto_medio'):
        x = y = th = 0.0
        for _ in range(n):
            x, y, th = integrar(x, y, th, v, om, dt, metodo)
        res[metodo] = math.hypot(x, y)
    assert res['punto_medio'] < res['euler']


def test_envolver():
    assert math.isclose(envolver(3 * math.pi), math.pi) or math.isclose(envolver(3 * math.pi), -math.pi)
    assert math.isclose(envolver(0.1), 0.1)
