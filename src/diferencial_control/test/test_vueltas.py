"""Pruebas del cronómetro de vueltas y del error de cierre."""
import math

from diferencial_control.vueltas import ContadorVueltas, Historial, error_cierre


def test_cruces_y_tiempos():
    c = ContadorVueltas(0.0, 0.6, 3)
    t = 0.0
    tiempos_cruce = []
    for vuelta in range(4):
        for x in (-0.2, 0.2):                  # cruza de x<0 a x>=0
            tc = c.actualizar(t, x, 0.0)
            if tc is not None:
                tiempos_cruce.append(tc)
            t += 1.0
        t += 10.0
    assert len(tiempos_cruce) == 4 and c.vueltas == 3 and c.terminada
    assert all(math.isclose(a, 12.0) for a in c.tiempos())
    assert math.isclose(tiempos_cruce[0], 0.5)          # interpolado a mitad del paso


def test_no_cuenta_fuera_de_la_linea_ni_en_reversa():
    c = ContadorVueltas(0.0, 0.6, 3)
    c.actualizar(0.0, -0.1, 2.0)
    assert c.actualizar(1.0, 0.1, 2.0) is None          # |y| > semiancho (otro tramo de la pista)
    c.actualizar(2.0, 0.1, 0.0)
    assert c.actualizar(3.0, -0.1, 0.0) is None         # cruce en sentido contrario


def test_historial_interpola_y_error():
    h = Historial()
    h.agregar(0.0, 0.0, 0.0, math.radians(170))
    h.agregar(1.0, 1.0, 0.0, math.radians(-170))
    q = h.en(0.5)
    assert math.isclose(q[0], 0.5) and math.isclose(abs(q[2]), math.pi, abs_tol=1e-9)
    e_pos, e_th, _, _ = error_cierre((1.0, 1.0, 0.1), (0.0, 1.0, -0.1))
    assert math.isclose(e_pos, 1.0) and math.isclose(e_th, 0.2)
