#!/usr/bin/env python3
"""Genera worlds/pista.sdf a partir de la línea central de la pista de referencia.

Uso (desde la raíz del paquete robot_diferencial):
    python3 tools/generar_pista.py

Entradas : config/pista_centerline_px.csv  (línea central en píxeles de docs/pista_referencia.png,
           obtenida por binarización + esqueletización (scikit-image) del trazo negro de la imagen)
Salidas  : worlds/pista.sdf, config/pista_centerline_m.csv, docs/pista_preview.png

Decisiones de diseño (se justifican en el README / Notion):
  * ESCALA   : 0.080 m/px -> el circuito mide ~28 x 19 m y el lazo ~78 m.
  * ANCHO    : 1.20 m (3.5 veces el ancho del robot, 0.34 m) en tramo normal.
  * ESTRECH. : 0.80 m (2.3 veces el ancho del robot) en la recta superior, con transiciones suaves.
  * OBSTÁCULOS: cilindros y cajas en zigzag que dejan >= 0.75 m libres (2.2x el ancho del robot).
  * SPAWN    : sobre la línea Start/Finish, orientado según la tangente de la pista.
"""
import math
import os
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(AQUI)

# ------------------------- parámetros -------------------------
ESCALA = 0.080            # m / px
ANCHO = 1.20              # m, ancho nominal del corredor
ANCHO_ESTRECHO = 0.80     # m, ancho en el estrechamiento
MURO_ESPESOR = 0.08       # m
MURO_ALTO = 0.40          # m  (el LiDAR esta a 0.20 m y debe verlos)
DS = 0.05                 # m, paso de remuestreo
ROBOT_ANCHO = 0.34        # m, solo para las verificaciones
ROBOT_LARGO = 0.42        # m, solo para las verificaciones
PX_SF = (204.0, 113.0)    # marca Start/Finish en la imagen
PX_ESTRECH = (205.0, 31.0)    # centro del estrechamiento (recta superior)
L_PLATEAU = 2.0           # m con ancho minimo
L_TRANSICION = 1.2        # m de transicion a cada lado

# obstaculos: (punto en la imagen, forma, desplazamiento lateral [m, + izquierda], color RGB)
OBSTACULOS = [
    ((268, 126), 'cil', +0.28, (0.85, 0.10, 0.10)),
    ((330, 137), 'caja', -0.27, (0.95, 0.75, 0.10)),
    ((362, 80), 'cil', +0.27, (0.10, 0.35, 0.90)),
    ((330, 22), 'caja', -0.27, (0.10, 0.65, 0.25)),
    ((140, 36), 'cil', +0.28, (0.85, 0.10, 0.10)),
    ((60, 112), 'caja', -0.27, (0.60, 0.20, 0.80)),
    ((32, 185), 'cil', +0.28, (0.95, 0.45, 0.05)),
]


# ------------------------- geometría -------------------------
def periodico_suave(p, sigma_m):
    """Suavizado gaussiano periódico (sigma en metros, p remuestreado a DS)."""
    n = len(p)
    k = int(4 * sigma_m / DS)
    w = np.exp(-0.5 * (np.arange(-k, k + 1) * DS / sigma_m) ** 2)
    w /= w.sum()
    out = np.zeros_like(p)
    for c in range(2):
        ext = np.concatenate([p[-k:, c], p[:, c], p[:k, c]])
        out[:, c] = np.convolve(ext, w, mode='valid')[:n]
    return out


def remuestrear(p, ds):
    q = np.vstack([p, p[:1]])
    seg = np.hypot(*np.diff(q, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    L = s[-1]
    n = int(round(L / ds))
    ss = np.linspace(0, L, n, endpoint=False)
    return np.stack([np.interp(ss, s, q[:, 0]), np.interp(ss, s, q[:, 1])], 1), L / n


def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


def rdp(pts, tol):
    """Ramer-Douglas-Peucker para polilinea abierta."""
    if len(pts) < 3:
        return pts
    a, b = pts[0], pts[-1]
    ab = b - a
    nrm = np.hypot(*ab)
    d = (np.abs(ab[0] * (pts[:, 1] - a[1]) - ab[1] * (pts[:, 0] - a[0])) / nrm
         if nrm > 1e-12 else np.hypot(*(pts - a).T))
    i = int(np.argmax(d))
    if d[i] > tol:
        return np.vstack([rdp(pts[:i + 1], tol)[:-1], rdp(pts[i:], tol)])
    return np.vstack([a, b])


def rdp_cerrada(pts, tol, max_len):
    n = len(pts)
    m = n // 2
    a = rdp(np.vstack([pts[:m + 1]]), tol)
    b = rdp(np.vstack([pts[m:], pts[:1]]), tol)
    poly = np.vstack([a[:-1], b[:-1]])
    # limitar longitud maxima de segmento
    out = []
    for i in range(len(poly)):
        p0, p1 = poly[i], poly[(i + 1) % len(poly)]
        k = max(1, int(math.ceil(np.hypot(*(p1 - p0)) / max_len)))
        for j in range(k):
            out.append(p0 + (p1 - p0) * j / k)
    return np.array(out)


def construir():
    px = np.loadtxt(os.path.join(PKG, 'config', 'pista_centerline_px.csv'), delimiter=',', skiprows=1)
    # px -> metros (eje y hacia arriba)
    m = np.stack([px[:, 0] * ESCALA, -px[:, 1] * ESCALA], 1)
    m, _ = remuestrear(m, DS)
    m = periodico_suave(m, 0.25)
    m, ds = remuestrear(m, DS)

    sf_m = np.array([PX_SF[0] * ESCALA, -PX_SF[1] * ESCALA])
    i_sf = int(np.argmin(np.hypot(*(m - sf_m).T)))
    m = np.roll(m, -i_sf, axis=0)             # el indice 0 es Start/Finish
    # sentido de marcha: hacia +x en la recta principal (de izquierda a derecha en la imagen)
    if m[10, 0] - m[0, 0] < 0:
        m = np.vstack([m[:1], m[1:][::-1]])
    t0 = m[8] - m[-8]
    ang = math.atan2(t0[1], t0[0])
    c, s = math.cos(-ang), math.sin(-ang)
    R = np.array([[c, -s], [s, c]])
    m = (m - m[0]) @ R.T                       # origen en Start/Finish, +x = tangente
    n = len(m)

    # tangente, normal izquierda y curvatura
    d = (np.roll(m, -1, axis=0) - np.roll(m, 1, axis=0)) / (2 * DS)
    d /= np.hypot(*d.T)[:, None]
    nor = np.stack([-d[:, 1], d[:, 0]], 1)
    th = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    kappa = np.gradient(th, DS)
    s_arc = np.arange(n) * DS
    L = n * DS

    # perfil de ancho
    def arco_de_px(pp):
        q = np.array([pp[0] * ESCALA, -pp[1] * ESCALA])
        q = (q - (sf_m)) @ R.T                 # mismo origen/rotacion que arriba
        return int(np.argmin(np.hypot(*(m - q).T)))
    i_e = arco_de_px(PX_ESTRECH)
    dist = np.abs(s_arc - s_arc[i_e])
    dist = np.minimum(dist, L - dist)
    w = ANCHO - (ANCHO - ANCHO_ESTRECHO) * (1 - smoothstep((dist - L_PLATEAU / 2) / L_TRANSICION))

    izq = m + nor * (w / 2)[:, None]
    der = m - nor * (w / 2)[:, None]
    return dict(m=m, nor=nor, kappa=kappa, w=w, izq=izq, der=der, L=L, i_e=i_e,
                s_arc=s_arc, arco_de_px=arco_de_px, ang=ang)


def verificar(g):
    m, kappa, w, L = g['m'], g['kappa'], g['w'], g['L']
    rmin = 1.0 / np.max(np.abs(kappa))
    print(f'longitud del lazo        : {L:.1f} m')
    print(f'radio de curvatura minimo: {rmin:.2f} m (debe ser > w/2 = {ANCHO / 2:.2f} m)')
    if rmin < ANCHO / 2 + 0.05:
        sys.exit('ERROR: el muro interior se cruzaria en la curva mas cerrada; aumente ESCALA')
    # distancia entre muros de tramos no adyacentes (que no se solapen corredores vecinos)
    from scipy.spatial.distance import cdist
    D = cdist(m, m)
    idx = np.arange(len(m))
    arc = np.abs(idx[:, None] - idx[None, :]) * DS
    arc = np.minimum(arc, L - arc)
    D[arc < 6.0] = 1e9
    dmin = D.min()
    print(f'distancia minima entre ejes de tramos no adyacentes: {dmin:.2f} m '
          f'(corredores {ANCHO:.2f} m + 2 muros {2 * MURO_ESPESOR:.2f} m -> separacion libre {dmin - ANCHO - MURO_ESPESOR:.2f} m)')
    if dmin < ANCHO + 2 * MURO_ESPESOR + 0.2:
        sys.exit('ERROR: dos tramos de la pista quedan demasiado cerca')
    # barrido de la huella del robot sobre la linea central: ningun vertice dentro de un muro
    caja = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]]) * np.array([ROBOT_LARGO / 2, ROBOT_ANCHO / 2])
    holg = 9.9
    th = np.arctan2(np.gradient(m[:, 1]), np.gradient(m[:, 0]))
    for i in range(0, len(m), 2):
        cth, sth = math.cos(th[i]), math.sin(th[i])
        esq = m[i] + caja @ np.array([[cth, sth], [-sth, cth]])
        for e in esq:
            j = np.argmin(np.hypot(*(m - e).T))
            off = np.dot(e - m[j], g['nor'][j])
            holg = min(holg, w[j] / 2 - abs(off))
    print(f'holgura minima de la huella del robot a los muros (sobre el eje): {holg * 100:.1f} cm')
    if holg < 0.05:
        sys.exit('ERROR: la huella del robot toca un muro')


# ------------------------- SDF -------------------------
def box_xml(nombre, cx, cy, yaw, largo, ancho, alto, color, colision=True, z=None):
    z = alto / 2 if z is None else z
    pose = f'{cx:.4f} {cy:.4f} {z:.4f} 0 0 {yaw:.5f}'
    geo = f'<geometry><box><size>{largo:.4f} {ancho:.4f} {alto:.4f}</size></box></geometry>'
    mat = (f'<material><ambient>{color[0]} {color[1]} {color[2]} 1</ambient>'
           f'<diffuse>{color[0]} {color[1]} {color[2]} 1</diffuse></material>')
    s = ''
    if colision:
        s += f'<collision name="c_{nombre}"><pose>{pose}</pose>{geo}</collision>\n        '
    s += f'<visual name="v_{nombre}"><pose>{pose}</pose>{geo}{mat}</visual>'
    return s


def muros_xml(poly, prefijo, color):
    out = []
    n = len(poly)
    for i in range(n):
        p0, p1 = poly[i], poly[(i + 1) % n]
        v = p1 - p0
        ln = np.hypot(*v)
        yaw = math.atan2(v[1], v[0])
        c = (p0 + p1) / 2
        out.append(box_xml(f'{prefijo}{i}', c[0], c[1], yaw, ln + MURO_ESPESOR, MURO_ESPESOR,
                           MURO_ALTO, color))
    return out


def escribir_sdf(g):
    m, nor, kappa, w = g['m'], g['nor'], g['kappa'], g['w']
    cmuro = (0.88, 0.88, 0.85)
    izq = rdp_cerrada(g['izq'], 0.012, 0.60)
    der = rdp_cerrada(g['der'], 0.012, 0.60)
    muros = muros_xml(izq, 'i', cmuro) + muros_xml(der, 'd', cmuro)
    print(f'segmentos de muro: {len(izq)} (izq) + {len(der)} (der)')

    obs = []
    obs_info = []
    for k, (pp, forma, lat, col) in enumerate(OBSTACULOS):
        j = g['arco_de_px'](pp)
        dist = abs(g['s_arc'][j] - g['s_arc'][g['i_e']])
        dist = min(dist, g['L'] - dist)
        curv = np.max(np.abs(kappa[max(0, j - 20):j + 20]))
        if g['s_arc'][j] < 3.0 or g['s_arc'][j] > g['L'] - 3.0:
            print(f'  obstaculo {k} omitido (cerca del spawn)'); continue
        if dist < 4.0:
            print(f'  obstaculo {k} omitido (cerca del estrechamiento)'); continue
        if curv > 1 / 1.6:
            print(f'  obstaculo {k} en curva cerrada (r={1 / curv:.2f} m), se mueve a la recta mas cercana')
            for dj in range(1, 400):
                for jj in (j + dj, j - dj):
                    jj %= len(m)
                    if np.max(np.abs(kappa[max(0, jj - 20):jj + 20])) < 1 / 3.0:
                        j = jj; break
                else:
                    continue
                break
        x, y = m[j] + nor[j] * lat
        yaw = math.atan2(nor[j][1], nor[j][0]) + math.pi / 2
        libre = w[j] / 2 + (-lat if lat < 0 else lat)   # no se usa; la holgura se calcula abajo
        if forma == 'cil':
            r = 0.15
            ext = r
            obs.append(
                f'<collision name="c_obs{k}"><pose>{x:.3f} {y:.3f} 0.25 0 0 0</pose><geometry><cylinder>'
                f'<radius>{r}</radius><length>0.50</length></cylinder></geometry></collision>\n        '
                f'<visual name="v_obs{k}"><pose>{x:.3f} {y:.3f} 0.25 0 0 0</pose><geometry><cylinder>'
                f'<radius>{r}</radius><length>0.50</length></cylinder></geometry><material>'
                f'<ambient>{col[0]} {col[1]} {col[2]} 1</ambient><diffuse>{col[0]} {col[1]} {col[2]} 1</diffuse>'
                f'</material></visual>')
        else:
            lg, an = 0.30, 0.30
            ext = 0.15 * math.sqrt(2)
            obs.append(box_xml(f'obs{k}', x, y, yaw, lg, an, 0.50, col))
        paso = w[j] - (abs(lat) + ext)
        obs_info.append((k, forma, float(g['s_arc'][j]), float(x), float(y), float(paso)))
        print(f'  obstaculo {k}: {forma} a s={g["s_arc"][j]:.1f} m, paso libre {paso:.2f} m')
        if paso < 0.65:
            sys.exit('ERROR: obstaculo bloquea el paso')

    # marcas de pista (solo visuales): linea de salida con cuadros y flecha de spawn
    marcas = []
    ancho0 = w[0]
    nq = 8
    for q in range(nq):
        for r_ in range(2):
            col = (0.95, 0.95, 0.95) if (q + r_) % 2 == 0 else (0.05, 0.05, 0.05)
            yy = -ancho0 / 2 + (q + 0.5) * ancho0 / nq
            marcas.append(box_xml(f'sf{q}_{r_}', 0.0 + 0.10 * r_ - 0.05, yy, 0.0, 0.10, ancho0 / nq, 0.002,
                                  col, colision=False, z=0.001))
    # linea central discontinua (solo visual)
    for i in range(0, len(m), 12):
        p0, p1 = m[i], m[(i + 5) % len(m)]
        v = p1 - p0
        marcas.append(box_xml(f'ln{i}', *((p0 + p1) / 2), math.atan2(v[1], v[0]), np.hypot(*v), 0.03,
                              0.002, (0.95, 0.85, 0.2), colision=False, z=0.001))

    xmin, ymin = m.min(0) - 3.0
    xmax, ymax = m.max(0) + 3.0
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    sx, sy = xmax - xmin, ymax - ymin

    sdf = f"""<?xml version="1.0"?>
<!-- Mundo generado por tools/generar_pista.py - NO EDITAR A MANO (regenerar con el script) -->
<sdf version="1.10">
  <world name="pista">
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

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- Suelo: asfalto con friccion alta (la cinematica asume rodadura sin deslizamiento) -->
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
          <material>
            <ambient>0.25 0.25 0.27 1</ambient>
            <diffuse>0.25 0.25 0.27 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Muros del circuito (colision + visual), generados a ambos lados de la linea central -->
    <model name="muros">
      <static>true</static>
      <link name="link">
        {chr(10).join('        ' + b if i else b for i, b in enumerate(muros))}
      </link>
    </model>

    <!-- Obstaculos (zigzag): dejan >= 0.65 m libres -->
    <model name="obstaculos">
      <static>true</static>
      <link name="link">
        {chr(10).join('        ' + b if i else b for i, b in enumerate(obs))}
      </link>
    </model>

    <!-- Marcas visuales: linea de salida y eje de la pista (sin colision) -->
    <model name="marcas">
      <static>true</static>
      <link name="link">
        {chr(10).join('        ' + b if i else b for i, b in enumerate(marcas))}
      </link>
    </model>
  </world>
</sdf>
"""
    with open(os.path.join(PKG, 'worlds', 'pista.sdf'), 'w') as f:
        f.write(sdf)
    return obs_info


def guardar_csv_y_preview(g, obs_info):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    m = g['m']
    np.savetxt(os.path.join(PKG, 'config', 'pista_centerline_m.csv'),
               np.column_stack([np.arange(len(m)) * DS, m, g['w']]), delimiter=',', fmt='%.4f',
               header='s_m,x_m,y_m,ancho_m (marco world; spawn en (0,0) con yaw 0)', comments='')
    fig, ax = plt.subplots(figsize=(11, 6.5))
    for lado, col in ((g['izq'], 'k'), (g['der'], 'k')):
        ax.plot(*np.vstack([lado, lado[:1]]).T, col, lw=1.6)
    ax.plot(*m.T, color='goldenrod', lw=0.8, ls='--')
    for (k, forma, s_, x, y, paso) in obs_info:
        ax.plot(x, y, 'o' if forma == 'cil' else 's', ms=8, label=None)
    ax.plot(0, 0, 'g>', ms=12)
    ax.annotate('spawn / Start-Finish', (0, 0), (0.5, -3.2), arrowprops=dict(arrowstyle='->'))
    ax.annotate('estrechamiento 0.8 m', m[g['i_e']], m[g['i_e']] + [0, 2.0], arrowprops=dict(arrowstyle='->'))
    ax.set_aspect('equal'); ax.grid(alpha=.3)
    ax.set_title(f'Pista generada: lazo {g["L"]:.1f} m, ancho {ANCHO} m (estrechamiento {ANCHO_ESTRECHO} m)')
    ax.set_xlabel('x [m] (world)'); ax.set_ylabel('y [m] (world)')
    fig.tight_layout()
    fig.savefig(os.path.join(PKG, 'docs', 'pista_preview.png'), dpi=110)


if __name__ == '__main__':
    g = construir()
    verificar(g)
    info = escribir_sdf(g)
    guardar_csv_y_preview(g, info)
    print('OK -> worlds/pista.sdf, config/pista_centerline_m.csv, docs/pista_preview.png')
