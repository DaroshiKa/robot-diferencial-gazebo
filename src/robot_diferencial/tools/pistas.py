#!/usr/bin/env python3
"""Definición geométrica de las pistas (fuente única de verdad).

La usan:
  * tools/generar_pistas.py  -> STL/STEP en metros + worlds/*.sdf + figuras
  * tools/sim2d.py           -> simulador 2D para sintonizar AEB y seguimiento de pared
  * las pruebas (test_pistas.py)

Una pista es una lista de tramos sobre la LÍNEA CENTRAL, recorrida en sentido antihorario:
    ('recta', L)            recta de longitud L [m]
    ('arco', R, ang_deg)    arco de radio R [m] (sobre la línea central); ang > 0 = izquierda
y un perfil de ANCHO w(s) (por defecto constante) con estrechamientos de transición lineal.

Convención: la línea de salida/meta es x = 0 (perpendicular a la recta principal, que va
en +x por y = 0). El robot arranca en x = -0.60 m, y = 0, yaw = 0 (detrás de la línea).
"""
import math
from dataclasses import dataclass, field

import numpy as np

DS = 0.01   # m, paso de muestreo de la línea central


@dataclass
class Estrechamiento:
    s_ini: float          # m, abscisa curvilínea donde empieza la transición de entrada
    l_transicion: float   # m, longitud de cada transición lineal
    l_plano: float        # m, longitud con el ancho mínimo
    ancho_min: float      # m


@dataclass
class Pista:
    nombre: str
    inicio: tuple                 # (x, y, yaw) del primer punto de la línea central
    tramos: list
    ancho: float                  # m, ancho nominal entre caras internas de los muros
    estrechamientos: list = field(default_factory=list)
    secciones: list = field(default_factory=list)   # (etiqueta, índice de tramo inicial) para figuras
    muro_alto: float = 0.30
    muro_espesor: float = 0.05


# ---------------------------------------------------------------------------------------------
# PISTA PROPIA (diseño del equipo). 6 secciones / 5 cambios de sección + retorno:
#   S1 recta principal (salida/meta) -> S2 curva amplia 90° (R 2.0) -> S3 chicana (R 1.2: -35°,
#   +70°, -35°) -> S4 horquilla 180° (R 0.6) -> S5 curva cerrada a la DERECHA 90° (R 0.8)
#   -> S6 estrechamiento a 0.40 m (2.0 m) -> retorno con dos curvas medias de 90° (R 1.4).
# La curva a la derecha obliga al controlador a seguir el muro izquierdo cuando es el EXTERIOR.
# ---------------------------------------------------------------------------------------------
_X0_PROPIA = -2.0
PROPIA = Pista(
    nombre='propia',
    inicio=(_X0_PROPIA, 0.0, 0.0),
    tramos=[
        ('recta', None),                # S1: desde x=-2.0 (meta en x=0); se calcula para cerrar
        ('arco', 2.0, 90.0),            # S2: curva amplia
        ('recta', 0.8),
        ('arco', 1.2, -35.0),           # S3: chicana
        ('arco', 1.2, 70.0),
        ('arco', 1.2, -35.0),
        ('recta', 0.6),
        ('arco', 0.6, 180.0),           # S4: horquilla
        ('recta', 0.5),
        ('arco', 0.8, -90.0),           # S5: curva cerrada a la derecha
        ('recta', 6.2),                 # S6: recta con estrechamiento
        ('arco', 1.4, 90.0),            # retorno
        ('recta', None),                # (se calcula para cerrar el circuito)
        ('arco', 1.4, 90.0),
    ],
    ancho=0.80,
    estrechamientos=[],                 # se fija abajo (depende de s)
    secciones=[('S1', 0), ('S2', 1), ('S3', 3), ('S4', 7), ('S5', 9), ('S6', 10), ('S7', 11)],
    muro_alto=0.30,
)

# ---------------------------------------------------------------------------------------------
# RÉPLICA de la pista de competencia (Fig. 1 del enunciado) SOLO PARA PRUEBAS: la oficial es el
# paquete pista_competencia del profesor. Medida sobre la figura: 37.2 m por vuelta.
# ---------------------------------------------------------------------------------------------
_R_CH = 1.6
_AV_CH = 4 * _R_CH * math.sin(math.radians(40.0))   # avance de la chicana (+40, -80, +40)
COMPETENCIA = Pista(
    nombre='competencia_replica',
    inicio=(-1.8, 0.0, 0.0),
    tramos=[
        ('recta', 10.8),                # S1 (9.0 m de x=0 a x=9) + 1.8 m antes de la meta
        ('arco', 2.5, 180.0),           # S2
        ('recta', 1.0),
        ('arco', _R_CH, 40.0),          # S3 chicana
        ('arco', _R_CH, -80.0),
        ('arco', _R_CH, 40.0),
        ('recta', 10.6 - _AV_CH),       # incluye S4 (estrechamiento)
        ('arco', 1.0, 90.0),            # S5
        ('recta', 2.2),
        ('arco', 1.8, 90.0),            # S6
    ],
    ancho=0.80,
    estrechamientos=[],
    secciones=[('S1', 0), ('S2', 1), ('S3', 3), ('S4', 6), ('S5', 7), ('S6', 9)],
    muro_alto=0.25,
)


# ------------------------------------------ geometría ------------------------------------------
def _avance_arco(R, ang):
    a = math.radians(ang)
    # desplazamiento en el marco del inicio del arco
    return R * math.sin(abs(a)), math.copysign(R * (1 - math.cos(a)), a), a


def resolver_cierre(p: Pista):
    """Calcula la longitud de la(s) recta(s) con L=None para que el circuito cierre en x e y."""
    huecos = [i for i, t in enumerate(p.tramos) if t[0] == 'recta' and t[1] is None]
    if not huecos:
        return p
    # posición final con L=0 en los huecos; luego se resuelve lineal en las longitudes
    x, y, h = p.inicio
    dirs = []
    for i, t in enumerate(p.tramos):
        if t[0] == 'recta':
            if t[1] is None:
                dirs.append((math.cos(h), math.sin(h)))
            else:
                x += t[1] * math.cos(h); y += t[1] * math.sin(h)
        else:
            ax, ay, a = _avance_arco(t[1], t[2])
            x += ax * math.cos(h) - ay * math.sin(h)
            y += ax * math.sin(h) + ay * math.cos(h)
            h += a
    falta = np.array([p.inicio[0] - x, p.inicio[1] - y])
    D = np.array(dirs).T
    if len(huecos) == 1:
        L = float(np.dot(falta, D[:, 0]))
        if np.linalg.norm(falta - L * D[:, 0]) > 1e-6:
            raise ValueError(f'{p.nombre}: una sola recta libre no cierra el circuito (residuo '
                             f'{np.linalg.norm(falta - L * D[:, 0]):.3f} m)')
        Ls = [L]
    else:
        Ls = np.linalg.lstsq(D, falta, rcond=None)[0]
    for i, L in zip(huecos, Ls):
        if L < 0:
            raise ValueError(f'{p.nombre}: la recta {i} quedaría con longitud negativa ({L:.2f} m)')
        p.tramos[i] = ('recta', float(L))
    return p


def linea_central(p: Pista, ds=DS):
    """Devuelve dict con s, x, y, yaw, kappa (muestreo uniforme ~ds, lazo cerrado sin repetir el 1.er punto)."""
    x, y, h = p.inicio
    X, Y, H, K = [x], [y], [h], []
    for t in p.tramos:
        if t[0] == 'recta':
            n = max(1, int(round(t[1] / ds)))
            for _ in range(n):
                x += t[1] / n * math.cos(h); y += t[1] / n * math.sin(h)
                X.append(x); Y.append(y); H.append(h); K.append(0.0)
        else:
            R, a = t[1], math.radians(t[2])
            n = max(1, int(round(R * abs(a) / ds)))
            dh = a / n
            for _ in range(n):
                hm = h + dh / 2
                c = 2 * R * math.sin(abs(dh) / 2)
                x += c * math.cos(hm); y += c * math.sin(hm); h += dh
                X.append(x); Y.append(y); H.append(h); K.append(math.copysign(1.0 / R, a))
    X, Y, H = np.array(X[:-1]), np.array(Y[:-1]), np.array(H[:-1])
    K = np.array(K)
    seg = np.hypot(np.diff(np.append(X, X[0])), np.diff(np.append(Y, Y[0])))
    s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    cierre = math.hypot(X[0] - x, Y[0] - y)
    return dict(s=s, x=X, y=Y, yaw=H, kappa=K, L=float(np.sum(seg)), cierre=cierre,
                giro_total=math.degrees(h - p.inicio[2]))


def perfil_ancho(p: Pista, s):
    w = np.full_like(s, p.ancho, dtype=float)
    for e in p.estrechamientos:
        a = e.s_ini
        b = a + e.l_transicion
        c = b + e.l_plano
        d = c + e.l_transicion
        f = np.ones_like(s)
        f = np.where((s >= a) & (s < b), 1 - (s - a) / e.l_transicion, f)
        f = np.where((s >= b) & (s <= c), 0.0, f)
        f = np.where((s > c) & (s <= d), (s - c) / e.l_transicion, f)
        w = np.minimum(w, e.ancho_min + (p.ancho - e.ancho_min) * f)
    return w


def s_inicio_tramo(p: Pista, idx):
    s = 0.0
    for t in p.tramos[:idx]:
        s += t[1] if t[0] == 'recta' else t[1] * abs(math.radians(t[2]))
    return s


def muros(p: Pista, ds=DS):
    """Polilíneas cerradas de las CARAS INTERNAS de los muros izquierdo (interior en antihorario)
    y derecho, más la línea central muestreada."""
    c = linea_central(p, ds)
    w = perfil_ancho(p, c['s'])
    nx, ny = -np.sin(c['yaw']), np.cos(c['yaw'])
    izq = np.stack([c['x'] + nx * w / 2, c['y'] + ny * w / 2], 1)
    der = np.stack([c['x'] - nx * w / 2, c['y'] - ny * w / 2], 1)
    return dict(c=c, w=w, izq=izq, der=der, nx=nx, ny=ny)


def preparar(p: Pista):
    """Cierra el circuito y fija los estrechamientos que dependen de s."""
    resolver_cierre(p)
    if p.nombre == 'propia' and not p.estrechamientos:
        s6 = s_inicio_tramo(p, 10)
        # recta de 6.2 m: 1.2 m recto + 1.0 transición + 2.0 a 0.40 m + 1.0 transición + 1.0 recto
        p.estrechamientos = [Estrechamiento(s6 + 1.2, 1.0, 2.0, 0.40)]
    if p.nombre == 'competencia_replica' and not p.estrechamientos:
        # en la recta superior (va en -x por y = 5): plano de x = 1.89 a x = -0.61, transiciones de 1 m
        s_rec = s_inicio_tramo(p, 6)
        x_ini_recta = 8.0 - _AV_CH
        p.estrechamientos = [Estrechamiento(s_rec + (x_ini_recta - 2.89), 1.0, 2.5, 0.40)]
    return p


PISTAS = {'propia': PROPIA, 'competencia': COMPETENCIA}


def obtener(nombre):
    return preparar(PISTAS[nombre])


def verificar(p: Pista, robot_ancho=0.153, robot_largo=0.1986, verbose=True):
    """Comprobaciones geométricas. Devuelve dict con métricas; lanza ValueError si algo falla."""
    m = muros(p)
    c = m['c']
    res = dict(L=c['L'], cierre=c['cierre'], giro=c['giro_total'],
               ancho_min=float(m['w'].min()))
    if c['cierre'] > 1e-3:
        raise ValueError(f'el circuito no cierra ({c["cierre"]:.4f} m)')
    if abs(res['giro'] - 360.0) > 1e-6:
        raise ValueError(f'giro total {res["giro"]:.2f}° != 360° (debe ser antihorario)')
    # radio de los muros interiores (no puede invertirse el offset)
    kmax = np.max(np.abs(c['kappa']))
    res['r_min_eje'] = 1 / kmax
    res['r_min_muro'] = 1 / kmax - p.ancho / 2 - p.muro_espesor
    if res['r_min_muro'] <= 0.02:
        raise ValueError('una curva es tan cerrada que el muro interior se auto-intersecta')
    # separación entre tramos no adyacentes (los muros de dos corredores no se tocan)
    from scipy.spatial import cKDTree
    pts = np.stack([c['x'], c['y']], 1)
    arbol = cKDTree(pts)
    n = len(pts)
    dmin = 1e9
    w = m['w']
    for i in range(0, n, 3):
        vec = arbol.query_ball_point(pts[i], r=3.0)
        for j in vec:
            darc = abs(c['s'][i] - c['s'][j])
            darc = min(darc, c['L'] - darc)
            if darc < 3.0:
                continue
            d = math.hypot(*(pts[i] - pts[j])) - (w[i] + w[j]) / 2 - 2 * p.muro_espesor
            dmin = min(dmin, d)
    res['separacion_muros'] = dmin
    if dmin < 0.05:
        raise ValueError(f'dos tramos quedan demasiado cerca (holgura entre muros {dmin:.3f} m)')
    res['holgura_estrecho'] = (res['ancho_min'] - robot_ancho) / 2
    if verbose:
        print(f'[{p.nombre}] L = {res["L"]:.2f} m | ancho {p.ancho:.2f} m (mín {res["ancho_min"]:.2f} m, '
              f'holgura lateral del robot {res["holgura_estrecho"] * 100:.1f} cm) | R mín eje '
              f'{res["r_min_eje"]:.2f} m (muro {res["r_min_muro"]:.2f} m) | separación entre muros '
              f'de tramos vecinos {dmin:.2f} m | cierre {res["cierre"] * 1000:.3f} mm')
    return res


if __name__ == '__main__':
    for k in PISTAS:
        p = obtener(k)
        verificar(p)
        print('  tramos:', [(t[0], round(t[1], 3)) + tuple(t[2:]) for t in p.tramos])
