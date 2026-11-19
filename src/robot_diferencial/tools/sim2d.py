#!/usr/bin/env python3
"""Simulador 2D rápido (sin ROS ni Gazebo) para SINTONIZAR el seguidor de pared y el AEB.

Modela lo que importa para el control:
  * LiDAR MS200: 450 rayos en 360°, 10 Hz, 0.03-12 m, ruido gaussiano σ = 1 cm (trazado de rayos
    contra las caras de los muros de tools/pistas.py y contra obstáculos opcionales)
  * cadena de comandos: seguidor (o guion de teleop) -> AEB -> DiffDrive
  * DiffDrive: límites v ≤ 1 m/s, |ω| ≤ 4 rad/s, a ≤ 2 m/s², α ≤ 29.9 rad/s²; latencia de comando
  * skid-steer: el giro real es ω_cmd / χ (χ = 1 ideal; χ > 1 por deslizamiento lateral)
  * colisión: contorno 0.1986 × 0.153 m contra los muros (cuenta contactos como en la competencia)
  * vueltas: cruce de la línea x = 0 hacia +x; tiempo de simulación

No reemplaza a Gazebo (no hay dinámica de contacto); sirve para barrer ganancias en segundos.
La validación final se hace en Gazebo (tools/prueba_gazebo.sh).

    python3 tools/sim2d.py --pista competencia --vueltas 3 --figura /tmp/c.png
    python3 tools/sim2d.py --pista propia --chi 1.3
    python3 tools/sim2d.py --pista competencia --obstaculo 4.0   # prueba del AEB en la recta
"""
import argparse
import math
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
sys.path.insert(0, os.path.join(AQUI, '..', '..', 'diferencial_control'))
import pistas  # noqa: E402
from diferencial_control.aeb import AEB, velocidad_para_ttc  # noqa: E402
from diferencial_control.seguidor_pared import ParametrosSeguidor, SeguidorPared  # noqa: E402

LARGO, ANCHO = 0.1986, 0.153
LIDAR_X = 0.0
N_RAYOS = 450
V_MAX, W_MAX, A_MAX = 1.0, 4.0, 2.0
ALFA_MAX = 2 * A_MAX / 0.134


class Mundo:
    def __init__(self, nombre, obstaculos=()):
        self.p = pistas.obtener(nombre)
        m = pistas.muros(self.p)
        self.c, self.w = m['c'], m['w']
        segs = []
        for poli in (m['izq'], m['der']):
            idx = np.arange(0, len(poli), 2)
            q = poli[idx]
            segs.append(np.stack([q, np.roll(q, -1, axis=0)], 1))
        for (cx, cy, lx, ly) in obstaculos:      # cajas alineadas con los ejes
            esq = np.array([[cx - lx / 2, cy - ly / 2], [cx + lx / 2, cy - ly / 2],
                            [cx + lx / 2, cy + ly / 2], [cx - lx / 2, cy + ly / 2]])
            segs.append(np.stack([esq, np.roll(esq, -1, axis=0)], 1))
        self.obstaculos = obstaculos
        S = np.concatenate(segs)
        self.P0, self.D = S[:, 0], S[:, 1] - S[:, 0]
        self.arbol = cKDTree(np.stack([self.c['x'], self.c['y']], 1))
        self.nx, self.ny = -np.sin(self.c['yaw']), np.cos(self.c['yaw'])

    def lidar(self, x, y, yaw, rng, sigma=0.01):
        ang = -math.pi + np.arange(N_RAYOS) * 2 * math.pi / N_RAYOS
        ox, oy = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
        dx, dy = np.cos(ang + yaw), np.sin(ang + yaw)
        # solo segmentos a menos de 12 m
        cerca = np.hypot(self.P0[:, 0] - ox, self.P0[:, 1] - oy) < 12.5
        P0, D = self.P0[cerca], self.D[cerca]
        wx, wy = P0[:, 0] - ox, P0[:, 1] - oy                          # (S,)
        den = dx[:, None] * D[None, :, 1] - dy[:, None] * D[None, :, 0]  # (R,S)
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (wx[None] * D[None, :, 1] - wy[None] * D[None, :, 0]) / den
            u = (wx[None] * dy[:, None] - wy[None] * dx[:, None]) / den
        ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        t = np.where(ok, t, np.inf)
        r = t.min(1)
        r = r + rng.normal(0, sigma, size=r.shape)
        r = np.where(r < 0.03, np.inf, r)
        r = np.where(r > 12.0, np.inf, r)
        return ang, r

    def colision(self, x, y, yaw):
        """Holgura mínima (m) del contorno del robot a los muros; < 0 = contacto."""
        u = np.linspace(-0.5, 0.5, 11)
        bx = np.concatenate([u * LARGO, u * LARGO, np.full(11, LARGO / 2), np.full(11, -LARGO / 2)])
        by = np.concatenate([np.full(11, ANCHO / 2), np.full(11, -ANCHO / 2), u * ANCHO, u * ANCHO])
        c, s = math.cos(yaw), math.sin(yaw)
        px, py = x + c * bx - s * by, y + s * bx + c * by
        _, j = self.arbol.query(np.stack([px, py], 1))
        off = (px - self.c['x'][j]) * self.nx[j] + (py - self.c['y'][j]) * self.ny[j]
        h = float(np.min(self.w[j] / 2 - np.abs(off)))
        for (cx, cy, lx, ly) in self.obstaculos:
            dxo = np.maximum(np.abs(px - cx) - lx / 2, 0)
            dyo = np.maximum(np.abs(py - cy) - ly / 2, 0)
            dentro = (np.abs(px - cx) < lx / 2) & (np.abs(py - cy) < ly / 2)
            d = np.where(dentro, -0.01, np.hypot(dxo, dyo))
            h = min(h, float(d.min()))
        return h


def simular(pista='competencia', vueltas=3, params=None, chi=1.0, obstaculos=(), t_max=200.0,
            semilla=0, latencia=0.03, guion=None, aeb_activo=True, verbose=False, registrar=False):
    rng = np.random.default_rng(semilla)
    mundo = Mundo(pista, obstaculos)
    seg = SeguidorPared(params or ParametrosSeguidor())
    aeb = AEB(x_del=LARGO / 2, x_tras=-LARGO / 2, y_izq=ANCHO / 2, y_der=-ANCHO / 2, lidar_x=LIDAR_X)
    dt = 0.002
    x, y, yaw, v, w = -0.6, 0.0, 0.0, 0.0, 0.0
    cmd_pend = []                  # (t_aplica, v, w)
    v_c = w_c = 0.0
    t = 0.0
    t_scan = 0.0
    cruces = []
    contactos = 0
    en_contacto = False
    h_min = 9.9
    frenados = 0
    sentido = 1.0
    log = []
    detenido_desde = None
    max_detenido = 0.0
    while t < t_max:
        if t >= t_scan - 1e-9:
            t_scan += 0.1
            ang, r = mundo.lidar(x, y, yaw, rng)
            if guion is not None:
                vc, wc = guion(t, x, y, yaw)
            else:
                vc, wc = seg.paso(r, ang[0], ang[1] - ang[0], t, v)
            vc = max(-V_MAX, min(V_MAX, vc))
            wc = max(-W_MAX, min(W_MAX, wc))
            frena = False
            if aeb_activo:
                vt = velocidad_para_ttc(vc, v)
                antes = aeb.frenando
                frena, ttc, umb = aeb.decidir(r, ang, vt, wc, t)
                if frena and not antes:
                    sentido = 1.0 if vt >= 0 else -1.0
                if frena and vc * sentido > 0:           # misma lógica que nodo_aeb
                    vc, wc = 0.0, 0.0
            frenados += int(frena)
            cmd_pend.append((t + latencia, vc, wc))
            if registrar:
                log.append((t, x, y, yaw, v, w, vc, wc, int(frena), seg.diag.get('e', 0.0)))
        while cmd_pend and cmd_pend[0][0] <= t:
            _, v_c, w_c = cmd_pend.pop(0)
        # DiffDrive: límites de aceleración
        v += max(-A_MAX * dt, min(A_MAX * dt, v_c - v))
        w += max(-ALFA_MAX * dt, min(ALFA_MAX * dt, w_c - w))
        w_real = w / chi
        xa = x
        yaw += w_real * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        t += dt
        # vueltas
        if xa < 0.0 <= x and abs(y) < 0.6:
            cruces.append(t)
            if verbose:
                print(f'  cruce {len(cruces)} t={t:.2f}')
            if len(cruces) == vueltas + 1:
                break
        # contactos (cada 10 ms)
        if int(t / dt) % 5 == 0:
            h = mundo.colision(x, y, yaw)
            h_min = min(h_min, h)
            if h < 0 and not en_contacto:
                contactos += 1
            en_contacto = h < 0
        if abs(v) < 0.02:
            detenido_desde = t if detenido_desde is None else detenido_desde
            max_detenido = max(max_detenido, t - detenido_desde)
        else:
            detenido_desde = None
    tiempos = np.diff(cruces) if len(cruces) > 1 else []
    return dict(cruces=cruces, tiempos=[float(x) for x in tiempos], total=float(np.sum(tiempos)) if len(tiempos) else None,
                vueltas=len(tiempos), contactos=contactos, holgura_min=h_min, frenados=frenados,
                max_detenido=max_detenido, log=np.array(log) if registrar else None, mundo=mundo,
                pose_final=(x, y, yaw))


def figura(res, ruta, titulo=''):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    m = res['mundo']
    mm = pistas.muros(m.p)
    L = res['log']
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(*mm['izq'].T, 'k', lw=1)
    ax.plot(*mm['der'].T, 'k', lw=1)
    sc = ax.scatter(L[:, 1], L[:, 2], c=L[:, 4], s=3, cmap='viridis', vmin=0, vmax=1)
    fr = L[:, 8] > 0
    ax.plot(L[fr, 1], L[fr, 2], 'rx', ms=4)
    for (cx, cy, lx, ly) in m.obstaculos:
        ax.add_patch(plt.Rectangle((cx - lx / 2, cy - ly / 2), lx, ly, color='tab:red'))
    plt.colorbar(sc, label='v [m/s]')
    ax.set_aspect('equal')
    ax.set_title(titulo)
    fig.tight_layout()
    fig.savefig(ruta, dpi=90)
    plt.close(fig)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--pista', default='competencia')
    ap.add_argument('--vueltas', type=int, default=3)
    ap.add_argument('--chi', type=float, default=1.0)
    ap.add_argument('--obstaculo', type=float, default=None, help='x del obstáculo en la recta principal')
    ap.add_argument('--figura', default=None)
    ap.add_argument('--semilla', type=int, default=0)
    a = ap.parse_args()
    obs = [(a.obstaculo, 0.0, 0.2, 0.2)] if a.obstaculo is not None else []
    r = simular(a.pista, a.vueltas, chi=a.chi, obstaculos=obs, semilla=a.semilla,
                registrar=a.figura is not None, verbose=True, t_max=60 if obs else 300)
    print(f"vueltas={r['vueltas']} tiempos={[round(x, 2) for x in r['tiempos']]} total={r['total']} "
          f"contactos={r['contactos']} holgura_min={r['holgura_min'] * 100:.1f} cm frenados_AEB={r['frenados']} "
          f"pose_final={tuple(round(q, 3) for q in r['pose_final'])}")
    if a.figura:
        figura(r, a.figura, f'{a.pista} chi={a.chi}')
