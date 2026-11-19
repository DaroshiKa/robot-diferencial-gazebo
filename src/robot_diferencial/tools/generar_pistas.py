#!/usr/bin/env python3
"""Genera las pistas como CAD (STEP) + malla STL en METROS + modelo/world de Gazebo.

Uso (desde src/robot_diferencial):
    python3 tools/generar_pistas.py            # pista propia + réplica de competencia
    python3 tools/generar_pistas.py --sin-step # solo STL/SDF (si no hay CadQuery)

Salidas por pista <p> (p = pista_propia, pista_competencia_replica):
    cad/<p>.step                  sólido de los muros (CadQuery / OpenCascade) para abrir en Fusion/SolidWorks
    cad/<p>_planta.dxf            planta 2D (caras de muro, eje y línea de meta)
    models/<p>/meshes/<p>.stl     malla de los muros, en metros (visual + colisión en Gazebo)
    models/<p>/model.sdf, model.config
    worlds/<p>.sdf                world: suelo + pista (include) + línea de salida/meta + plugins
    docs/<p>.png                  vista superior con las secciones
    config/<p>_eje.csv            línea central (s, x, y, ancho) para análisis (NO la usa el control)

El diseño (tramos, radios, ancho, estrechamiento) está en tools/pistas.py.
"""
import argparse
import math
import os
import struct
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(AQUI)
sys.path.insert(0, AQUI)
import pistas  # noqa: E402

NOMBRES = {'propia': 'pista_propia', 'competencia': 'pista_competencia_replica'}


# ----------------------------------------- geometría -----------------------------------------
def rdp_indices(pts, tol):
    """Ramer-Douglas-Peucker sobre polilínea abierta; devuelve índices conservados."""
    keep = np.zeros(len(pts), bool)
    keep[0] = keep[-1] = True
    pila = [(0, len(pts) - 1)]
    while pila:
        i, j = pila.pop()
        if j <= i + 1:
            continue
        a, b = pts[i], pts[j]
        ab = b - a
        n = math.hypot(*ab)
        seg = pts[i + 1:j]
        if n < 1e-12:
            d = np.hypot(*(seg - a).T)
        else:
            d = np.abs(ab[0] * (seg[:, 1] - a[1]) - ab[1] * (seg[:, 0] - a[0])) / n
        k = int(np.argmax(d))
        if d[k] > tol:
            m = i + 1 + k
            keep[m] = True
            pila += [(i, m), (m, j)]
    return np.flatnonzero(keep)


def anillo_muro(p, lado, tol=0.0015):
    """Cara interna y trasera (mismos índices) de un muro cerrado. lado = +1 izq, -1 der."""
    m = pistas.muros(p)
    c = m['c']
    w = m['w']
    nx, ny = m['nx'], m['ny']
    cara = np.stack([c['x'] + lado * nx * w / 2, c['y'] + lado * ny * w / 2], 1)
    tras = np.stack([c['x'] + lado * nx * (w / 2 + p.muro_espesor),
                     c['y'] + lado * ny * (w / 2 + p.muro_espesor)], 1)
    # simplificar con la cara interna (es la que ve el LiDAR) y con la trasera; unión de índices
    n = len(cara)
    idx = set()
    for poli in (cara, tras):
        ext = np.vstack([poli, poli[:1]])
        idx |= set(rdp_indices(ext, tol).tolist())
    idx = np.array(sorted(i for i in idx if i < n))
    return cara[idx], tras[idx]


def triangulos_anillo(cara, tras, alto):
    """Sólido cerrado (estanco) entre dos polilíneas cerradas con la misma cantidad de puntos."""
    n = len(cara)
    tris = []
    for i in range(n):
        j = (i + 1) % n
        a0, a1 = np.append(cara[i], 0), np.append(cara[j], 0)
        b0, b1 = np.append(tras[i], 0), np.append(tras[j], 0)
        A0, A1, B0, B1 = a0 + [0, 0, alto], a1 + [0, 0, alto], b0 + [0, 0, alto], b1 + [0, 0, alto]
        tris += [(A0, A1, B1), (A0, B1, B0)]           # tapa
        tris += [(a0, b1, a1), (a0, b0, b1)]           # fondo
        tris += [(a0, a1, A1), (a0, A1, A0)]           # cara interna
        tris += [(b0, B1, b1), (b0, B0, B1)]           # cara trasera
    T = np.array(tris, dtype=float)
    # orientar normales hacia afuera del sólido de forma consistente: se comprueba con el volumen
    vol = np.sum(np.einsum('ij,ij->i', T[:, 0], np.cross(T[:, 1], T[:, 2]))) / 6.0
    if vol < 0:
        T = T[:, [0, 2, 1]]
    return T


def escribir_stl(ruta, T, nombre='pista'):
    nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    with open(ruta, 'wb') as f:
        f.write(nombre.encode()[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(T)))
        for k in range(len(T)):
            f.write(struct.pack('<12fH', *nrm[k], *T[k, 0], *T[k, 1], *T[k, 2], 0))


def escribir_step(ruta, anillos, alto):
    import cadquery as cq
    solido = None
    for cara, tras in anillos:
        # contorno = cara interna (ida) + trasera (vuelta) no sirve para un anillo cerrado;
        # se extruye la región entre las dos curvas cerradas: exterior - interior
        def wire(pts):
            return cq.Workplane('XY').polyline([tuple(map(float, q)) for q in pts]).close()
        a = wire(cara).extrude(alto)
        b = wire(tras).extrude(alto)
        va, vb = a.val().Volume(), b.val().Volume()
        anillo = (b.cut(a) if vb > va else a.cut(b))
        solido = anillo if solido is None else solido.union(anillo)
    cq.exporters.export(solido, ruta)


def escribir_dxf(ruta, p, anillos):
    try:
        import ezdxf
    except ImportError:
        return False
    doc = ezdxf.new()
    doc.units = ezdxf.units.M
    msp = doc.modelspace()
    for cara, tras in anillos:
        msp.add_lwpolyline([tuple(q) for q in cara], close=True, dxfattribs={'layer': 'MURO_CARA'})
        msp.add_lwpolyline([tuple(q) for q in tras], close=True, dxfattribs={'layer': 'MURO_TRASERA'})
    c = pistas.linea_central(p)
    msp.add_lwpolyline(list(zip(c['x'][::5], c['y'][::5])), close=True, dxfattribs={'layer': 'EJE'})
    msp.add_line((0, -p.ancho / 2), (0, p.ancho / 2), dxfattribs={'layer': 'META'})
    doc.saveas(ruta)
    return True


# ----------------------------------------- Gazebo -----------------------------------------
def model_sdf(nombre, alto):
    return f"""<?xml version="1.0"?>
<!-- Generado por tools/generar_pistas.py (no editar a mano) -->
<sdf version="1.10">
  <model name="{nombre}">
    <static>true</static>
    <link name="muros">
      <collision name="colision">
        <geometry><mesh><uri>model://{nombre}/meshes/{nombre}.stl</uri></mesh></geometry>
        <surface><friction><ode><mu>0.3</mu><mu2>0.3</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://{nombre}/meshes/{nombre}.stl</uri></mesh></geometry>
        <material>
          <ambient>0.85 0.85 0.82 1</ambient>
          <diffuse>0.85 0.85 0.82 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def model_config(nombre, desc):
    return f"""<?xml version="1.0"?>
<model>
  <name>{nombre}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <author><name>Jaime Landinez, Juan David Botero</name></author>
  <description>{desc}</description>
</model>
"""


def world_sdf(nombre, p, c):
    xmin, xmax = c['x'].min() - 2, c['x'].max() + 2
    ymin, ymax = c['y'].min() - 2, c['y'].max() + 2
    cx, cy, sx, sy = (xmin + xmax) / 2, (ymin + ymax) / 2, xmax - xmin, ymax - ymin
    w0 = p.ancho
    # línea de salida/meta en x = 0: franja a cuadros (solo visual, sin colisión)
    cuadros = []
    nq = 8
    for q in range(nq):
        for k in range(2):
            col = '0.95 0.95 0.95' if (q + k) % 2 == 0 else '0.05 0.05 0.05'
            yy = -w0 / 2 + (q + 0.5) * w0 / nq
            cuadros.append(
                f'<visual name="meta_{q}_{k}"><pose>{-0.025 + 0.05 * k:.3f} {yy:.4f} 0.0015 0 0 0</pose>'
                f'<geometry><box><size>0.05 {w0 / nq:.4f} 0.002</size></box></geometry>'
                f'<material><ambient>{col} 1</ambient><diffuse>{col} 1</diffuse></material></visual>')
    cuadros = '\n        '.join(cuadros)
    return f"""<?xml version="1.0"?>
<!-- Generado por tools/generar_pistas.py (no editar a mano). Pista: {p.nombre}, {c['L']:.2f} m por vuelta,
     antihoraria. Línea de salida/meta en x = 0; el robot arranca en x = -0.60, y = 0, yaw = 0. -->
<sdf version="1.10">
  <world name="{nombre}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.8 0.85 1</background>
    </scene>

    <light type="directional" name="sol">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="suelo">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{sx:.2f} {sy:.2f}</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{sx:.2f} {sy:.2f}</size></plane></geometry>
          <material><ambient>0.3 0.3 0.32 1</ambient><diffuse>0.3 0.3 0.32 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Muros: malla STL (en metros) con colisión -->
    <include>
      <uri>model://{nombre}</uri>
      <name>{nombre}</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <model name="linea_meta">
      <static>true</static>
      <link name="link">
        {cuadros}
      </link>
    </model>
  </world>
</sdf>
"""


def figura(ruta, p, anillos, c):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 6.5))
    for cara, tras in anillos:
        cc, tt = np.vstack([cara, cara[:1]]), np.vstack([tras, tras[:1]])
        ax.fill(np.r_[cc[:, 0], tt[::-1, 0]], np.r_[cc[:, 1], tt[::-1, 1]], color='0.25', lw=0)
        ax.plot(*np.vstack([cara, cara[:1]]).T, color='0.15', lw=1.2)
    ax.plot(c['x'], c['y'], ls='--', color='0.6', lw=0.8)
    ax.plot([0, 0], [-p.ancho / 2, p.ancho / 2], color='tab:red', lw=3)
    ax.annotate('', xy=(0.6, -p.ancho * 0.75), xytext=(-0.6, -p.ancho * 0.75),
                arrowprops=dict(arrowstyle='->', color='tab:red'))
    ax.plot(-0.6, 0, 'o', color='tab:red', ms=4)
    for etiqueta, i in p.secciones:
        s = pistas.s_inicio_tramo(p, i)
        k = int(np.argmin(np.abs(c['s'] - s)))
        ax.plot(c['x'][k], c['y'][k], 'o', color='tab:blue', ms=4)
        ax.annotate(etiqueta, (c['x'][k], c['y'][k]), xytext=(6, 6), textcoords='offset points',
                    color='tab:blue', fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'{p.nombre}: {c["L"]:.2f} m por vuelta, ancho {p.ancho:.2f} m '
                 f'(mín. {min(e.ancho_min for e in p.estrechamientos):.2f} m), muros {p.muro_alto:.2f} m')
    fig.tight_layout()
    fig.savefig(ruta, dpi=110)
    plt.close(fig)


def generar(clave, con_step=True):
    p = pistas.obtener(clave)
    res = pistas.verificar(p)
    nombre = NOMBRES[clave]
    anillos = [anillo_muro(p, +1), anillo_muro(p, -1)]
    c = pistas.linea_central(p)

    dir_model = os.path.join(PKG, 'models', nombre)
    os.makedirs(os.path.join(dir_model, 'meshes'), exist_ok=True)
    for d in ('worlds', 'docs', 'config', 'cad'):
        os.makedirs(os.path.join(PKG, d), exist_ok=True)

    T = np.concatenate([triangulos_anillo(a, b, p.muro_alto) for a, b in anillos])
    escribir_stl(os.path.join(dir_model, 'meshes', nombre + '.stl'), T, nombre)
    desc = ('Muros de la pista propia (diseño del equipo)' if clave == 'propia' else
            'RÉPLICA para pruebas de la pista de competencia (la oficial la entrega el profesor)')
    with open(os.path.join(dir_model, 'model.sdf'), 'w') as f:
        f.write(model_sdf(nombre, p.muro_alto))
    with open(os.path.join(dir_model, 'model.config'), 'w') as f:
        f.write(model_config(nombre, desc))
    with open(os.path.join(PKG, 'worlds', nombre + '.sdf'), 'w') as f:
        f.write(world_sdf(nombre, p, c))
    w = pistas.perfil_ancho(p, c['s'])
    np.savetxt(os.path.join(PKG, 'config', nombre + '_eje.csv'),
               np.column_stack([c['s'], c['x'], c['y'], w]), delimiter=',', fmt='%.4f',
               header='s_m,x_m,y_m,ancho_m', comments='')
    figura(os.path.join(PKG, 'docs', nombre + '.png'), p, anillos, c)
    escribir_dxf(os.path.join(PKG, 'cad', nombre + '_planta.dxf'), p, anillos)
    if con_step:
        try:
            escribir_step(os.path.join(PKG, 'cad', nombre + '.step'), anillos, p.muro_alto)
        except ImportError:
            print('  (CadQuery no instalado: se omite el STEP; pip install cadquery)')
    print(f'  -> {nombre}: {len(T)} triángulos, {len(anillos[0][0])}+{len(anillos[1][0])} vértices de muro')
    return res


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--sin-step', action='store_true')
    ap.add_argument('pistas', nargs='*', default=['propia', 'competencia'])
    a = ap.parse_args()
    for k in a.pistas:
        generar(k, not a.sin_step)
