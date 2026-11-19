"""Pruebas de la geometría de las pistas y del modelo:  cd src/robot_diferencial && python3 -m pytest tools -q"""
import os
import re
import shutil
import struct
import subprocess

import numpy as np
import pytest

import pistas

AQUI = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(AQUI)


@pytest.mark.parametrize('nombre', ['propia', 'competencia'])
def test_cierra_antihoraria_y_requisitos(nombre):
    p = pistas.obtener(nombre)
    r = pistas.verificar(p, verbose=False)
    assert r['cierre'] < 1e-3 and abs(r['giro'] - 360) < 1e-6
    assert r['ancho_min'] >= 0.40 - 1e-9          # ancho mínimo 0.40 m
    assert p.muro_alto >= 0.25                      # muros de al menos 0.25 m
    assert r['separacion_muros'] > 0.05


def test_pista_propia_tiene_cinco_cambios_de_seccion():
    p = pistas.obtener('propia')
    tipos = []
    for t in p.tramos:
        if t[0] == 'arco':
            R, ang = t[1], t[2]
            tipos.append('horquilla' if abs(ang) >= 180 else 'cerrada' if R <= 0.8 else
                         'amplia' if R >= 2.0 else 'media' if R >= 1.4 else 'chicana')
    assert {'amplia', 'chicana', 'horquilla', 'cerrada', 'media'} <= set(tipos)
    assert p.estrechamientos and p.estrechamientos[0].ancho_min == 0.40


def test_replica_competencia_mide_37_2_m():
    c = pistas.linea_central(pistas.obtener('competencia'))
    assert abs(c['L'] - 37.2) < 0.05


@pytest.mark.parametrize('nombre', ['pista_propia', 'pista_competencia_replica'])
def test_stl_en_metros_y_estanco(nombre):
    ruta = os.path.join(PKG, 'models', nombre, 'meshes', nombre + '.stl')
    d = open(ruta, 'rb').read()
    n = struct.unpack('<I', d[80:84])[0]
    v = np.frombuffer(d[84:], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))['v'].reshape(-1, 3, 3)
    assert len(v) == n
    ext = v.reshape(-1, 3).max(0) - v.reshape(-1, 3).min(0)
    assert 5 < ext[0] < 30 and 4 < ext[1] < 30 and 0.25 <= ext[2] <= 0.5   # metros, no mm
    aristas = {}
    for t in np.round(v.astype(float), 5):
        for i, j in ((0, 1), (1, 2), (2, 0)):
            k = tuple(sorted((tuple(t[i]), tuple(t[j]))))
            aristas[k] = aristas.get(k, 0) + 1
    assert all(c == 2 for c in aristas.values())                           # malla cerrada


@pytest.mark.skipif(shutil.which('xacro') is None, reason='xacro no disponible')
def test_modelo_microros():
    urdf = subprocess.run(['xacro', os.path.join(PKG, 'urdf', 'diferencial.urdf.xacro')],
                          capture_output=True, text=True, check=True).stdout
    masas = [float(m) for m in re.findall(r'<mass value="([0-9.eE+-]+)"', urdf)]
    assert abs(sum(masas) - 0.906) < 1e-6
    assert urdf.count('<left_joint>') == 2 and urdf.count('<right_joint>') == 2
    assert '<samples>450</samples>' in urdf and '<max_linear_velocity>1.0<' in urdf
    assert '<max_angular_velocity>4.0<' in urdf and '<max_linear_acceleration>2.0<' in urdf
    assert '<stddev>0.01</stddev>' in urdf and '<min>0.03</min>' in urdf
