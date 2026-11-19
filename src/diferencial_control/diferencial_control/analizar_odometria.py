#!/usr/bin/env python3
"""Análisis de la odometría en varias vueltas (figuras y tabla para el pitch).

    ros2 run diferencial_control analizar_odometria <carpeta_resultados>
    (o: python3 -m diferencial_control.analizar_odometria <carpeta>)

Lee <carpeta>/vueltas.csv, trayectoria.csv y cinematica.csv (los escriben los nodos vueltas y
cinematica_diferencial) y genera:

  odometria_trayectorias.png   real vs estimada (cinemática) vs DiffDrive, con los cruces de la meta
  odometria_error.png          error de posición y de orientación frente a la distancia recorrida
  odometria_cierre.png         error de cierre en cada vuelta (barras)
  aeb.png                      mín TTC vs umbral y velocidad (si existe aeb.csv)
  seguidor.png                 D_t+1, D_des, error, omega y v del seguimiento de pared (si existe seguidor.csv)
  resumen.txt                  tiempos por vuelta, errores de cierre, vía efectiva

VÍA EFECTIVA (skid-steer): con la vía geométrica b, w = r (wD - wI) / b sobrestima el giro porque
las ruedas deslizan lateralmente al girar. Se estima el factor chi = (giro según ruedas) / (giro real)
con todo el recorrido y se re-integra la odometría con b_ef = chi·b para comparar.
(Mandow et al., 2007; Martínez et al., 2005: modelo cinemático de skid-steer por ICR.)
"""
import argparse
import csv
import math
import os
import sys

import numpy as np


def leer_csv(ruta):
    with open(ruta) as f:
        filas = list(csv.reader(f))
    cab = filas[0]
    datos = np.array([[float(x) if x not in ('', 'nan') else np.nan for x in fila] for fila in filas[1:]
                      if len(fila) == len(cab)], dtype=float)
    return {c: datos[:, i] for i, c in enumerate(cab)} if len(datos) else {c: np.array([]) for c in cab}


def envolver(a):
    return np.arctan2(np.sin(a), np.cos(a))


def reintegrar(cin, r, b):
    """Integra (punto medio) con las velocidades de rueda guardadas y la vía b."""
    t, wD, wI = cin['t'], cin['wD'], cin['wI']
    x, y, th = cin['x_real'][0], cin['y_real'][0], cin['th_real'][0]
    X, Y, TH = [x], [y], [th]
    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        v = r * (wD[k] + wI[k]) / 2
        w = r * (wD[k] - wI[k]) / b
        thm = th + 0.5 * w * dt
        x += v * math.cos(thm) * dt
        y += v * math.sin(thm) * dt
        th += w * dt
        X.append(x), Y.append(y), TH.append(th)
    return np.array(X), np.array(Y), envolver(np.array(TH))


def cierre_en(t_c, t, x, y, th, cin):
    xr = np.interp(t_c, cin['t'], cin['x_real'])
    yr = np.interp(t_c, cin['t'], cin['y_real'])
    thr = np.interp(t_c, cin['t'], np.unwrap(cin['th_real']))
    xe = np.interp(t_c, t, x)
    ye = np.interp(t_c, t, y)
    the = np.interp(t_c, t, np.unwrap(th))
    return math.hypot(xe - xr, ye - yr), math.degrees(envolver(the - thr))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('carpeta')
    ap.add_argument('--r', type=float, default=0.0245)
    ap.add_argument('--b', type=float, default=0.134)
    a = ap.parse_args(argv)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    d = a.carpeta
    vu = leer_csv(os.path.join(d, 'vueltas.csv'))
    tr = leer_csv(os.path.join(d, 'trayectoria.csv'))
    ruta_cin = os.path.join(d, 'cinematica.csv')
    cin = leer_csv(ruta_cin) if os.path.exists(ruta_cin) else None
    lineas = [f'Resultados: {os.path.abspath(d)}', '']

    cruces = vu['t_cruce'] if len(vu['vuelta']) else np.array([])
    # ---------------- vía efectiva ----------------
    chi = None
    if cin is not None and len(cin['t']) > 10:
        # calibración con la PRIMERA vuelta (de la salida al 1.er cruce); las vueltas 2 y 3 quedan
        # como validación "fuera de muestra"
        t_fin = cruces[1] if len(cruces) > 1 else cin['t'][-1]
        t_ini = cruces[0] if len(cruces) > 0 else cin['t'][0]
        m = (cin['t'] >= t_ini) & (cin['t'] <= t_fin)
        th_real = np.unwrap(cin['th_real'][m])
        dt = np.diff(cin['t'][m])
        w_ruedas = a.r * (cin['wD'][m][1:] - cin['wI'][m][1:]) / a.b
        w_real = np.diff(th_real) / np.maximum(dt, 1e-6)
        if np.sum(np.abs(w_real * dt)) > 1.0:
            # mínimos cuadrados de  w_real = w_ruedas / chi   ->  1/chi = <w_r, w_real> / <w_r, w_r>
            chi = float(np.sum(w_ruedas * w_ruedas * dt) / np.sum(w_ruedas * w_real * dt))
            lineas.append(f'Vía efectiva (calibrada con la vuelta 1, mínimos cuadrados de omega): '
                          f'chi = omega_ruedas/omega_real = {chi:.3f} -> b_ef = chi·b = {chi * a.b:.4f} m '
                          f'(b geométrica = {a.b} m)')
            lineas.append('')

    # ---------------- tabla por vuelta ----------------
    lineas.append(' vuelta | tiempo [s] | cierre cinemática b   | cierre DiffDrive      | cierre con b_ef')
    lineas.append('        |            |  [cm]      [°]        |  [cm]      [°]        |  [cm]      [°]')
    xr_ef = None
    if chi is not None:
        xr_ef = reintegrar(cin, a.r, chi * a.b)
    for k in range(len(vu['vuelta'])):
        if vu['vuelta'][k] == 0:
            continue
        ef = ''
        if xr_ef is not None:
            e_p, e_t = cierre_en(vu['t_cruce'][k], cin['t'], *xr_ef, cin)
            ef = f'{e_p * 100:7.1f} {e_t:8.1f}'
        lineas.append(f' {int(vu["vuelta"][k]):6d} | {vu["tiempo_vuelta"][k]:10.2f} | '
                      f'{vu["e_pos_cin"][k] * 100:7.1f} {vu["e_th_cin_deg"][k]:8.1f}      | '
                      f'{vu["e_pos_dd"][k] * 100:7.1f} {vu["e_th_dd_deg"][k]:8.1f}      | {ef}')
    if len(vu['vuelta']) > 1:
        lineas.append(f' TOTAL  | {vu["tiempo_acumulado"][-1]:10.2f} |')
    texto = '\n'.join(lineas)
    print(texto)
    with open(os.path.join(d, 'resumen.txt'), 'w') as f:
        f.write(texto + '\n')

    # ---------------- figuras ----------------
    fig, ax = plt.subplots(figsize=(11, 6.5))
    ax.plot(tr['x_real'], tr['y_real'], 'k-', lw=1.6, label='real (ground truth)')
    ax.plot(tr['x_est'], tr['y_est'], '-', color='tab:red', lw=1, label=f'cinemática, b = {a.b} m')
    ax.plot(tr['x_dd'], tr['y_dd'], '--', color='tab:blue', lw=1, label='DiffDrive /odom')
    if xr_ef is not None:
        ax.plot(xr_ef[0], xr_ef[1], '-', color='tab:green', lw=1, label=f'cinemática, b_ef = {chi * a.b:.3f} m')
    if len(vu['vuelta']):
        ax.plot(vu['x_real'], vu['y_real'], 'ko', ms=5)
        ax.plot(vu['x_est'], vu['y_est'], 'o', color='tab:red', ms=5)
    ax.axvline(0, color='0.6', lw=0.8, ls=':')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=9)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Trayectoria estimada vs real (círculos: cruces de la línea de meta)')
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'odometria_trayectorias.png'), dpi=110)
    plt.close(fig)

    if cin is not None and len(cin['t']):
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        a1.plot(cin['dist'], cin['e_pos'] * 100, color='tab:red', label=f'b = {a.b} m')
        a2.plot(cin['dist'], np.degrees(np.unwrap(cin['e_theta'])), color='tab:red')
        if xr_ef is not None:
            e = np.hypot(xr_ef[0] - cin['x_real'], xr_ef[1] - cin['y_real'])
            et = np.degrees(envolver(xr_ef[2] - cin['th_real']))
            a1.plot(cin['dist'], e * 100, color='tab:green', label=f'b_ef = {chi * a.b:.3f} m')
            a2.plot(cin['dist'], et, color='tab:green')
        a1.set_ylabel('error de posición [cm]')
        a2.set_ylabel('error de orientación [°]')
        a2.set_xlabel('distancia recorrida [m]')
        a1.legend()
        for axx in (a1, a2):
            axx.grid(alpha=0.3)
        fig.suptitle('Crecimiento del error de odometría')
        fig.tight_layout()
        fig.savefig(os.path.join(d, 'odometria_error.png'), dpi=110)
        plt.close(fig)

    ks = [k for k in range(len(vu['vuelta'])) if vu['vuelta'][k] > 0]
    if ks:
        fig, ax = plt.subplots(figsize=(7, 4))
        n = np.array([vu['vuelta'][k] for k in ks])
        ax.bar(n - 0.2, [vu['e_pos_cin'][k] * 100 for k in ks], 0.4, label=f'cinemática b = {a.b} m',
               color='tab:red')
        ax.bar(n + 0.2, [vu['e_pos_dd'][k] * 100 for k in ks], 0.4, label='DiffDrive', color='tab:blue')
        ax.set_xlabel('vuelta')
        ax.set_ylabel('error de cierre de posición [cm]')
        ax.set_xticks(n)
        ax.legend()
        ax.grid(alpha=0.3, axis='y')
        fig.tight_layout()
        fig.savefig(os.path.join(d, 'odometria_cierre.png'), dpi=110)
        plt.close(fig)
    graficar_aeb(d, plt)
    graficar_seguidor(d, plt, cruces)
    print(f'figuras en {os.path.abspath(d)}')
    return 0


def graficar_aeb(d, plt):
    ruta = os.path.join(d, 'aeb.csv')
    if not os.path.exists(ruta):
        return
    a = leer_csv(ruta)
    if not len(a['t']):
        return
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 5.5), sharex=True)
    a1.semilogy(a['t'], np.clip(a['ttc_min'], 1e-2, 99), color='tab:blue', lw=1, label='mín TTC_i')
    a1.semilogy(a['t'], a['umbral'], color='tab:red', lw=1.2, label='umbral(v)')
    f = a['frenando'] > 0
    a1.fill_between(a['t'], 1e-2, 99, where=f, color='tab:red', alpha=0.15, label='AEB frenando')
    a1.set_ylim(0.05, 50)
    a1.set_ylabel('TTC [s]')
    a1.legend(loc='upper right', fontsize=8)
    a2.plot(a['t'], a['v_cmd'], color='0.5', lw=1, label='v comandada (entrada al AEB)')
    a2.plot(a['t'], a['v_med'], color='k', lw=1.2, label='v medida (/odom)')
    a2.set_ylabel('v [m/s]')
    a2.set_xlabel('t [s] (simulación)')
    a2.legend(loc='upper right', fontsize=8)
    for ax in (a1, a2):
        ax.grid(alpha=0.3)
    fig.suptitle('AEB: tiempo a la colisión frente al umbral')
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'aeb.png'), dpi=110)
    plt.close(fig)


def graficar_seguidor(d, plt, cruces):
    ruta = os.path.join(d, 'seguidor.csv')
    if not os.path.exists(ruta):
        return
    s = leer_csv(ruta)
    if not len(s['t']):
        return
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    a1.plot(s['t'], s['d_t1'], lw=1, label='D_t+1 (proyectada)')
    a1.plot(s['t'], s['d_des'], lw=1.2, color='k', label='D_des = min(D_nom, W/2)')
    a1.set_ylabel('distancia [m]')
    a1.legend(loc='upper right', fontsize=8)
    a2.plot(s['t'], s['e'], lw=1, color='tab:red', label='e = D_des - D_t+1')
    a2.plot(s['t'], s['omega'], lw=0.8, color='tab:green', alpha=0.7, label='omega [rad/s]')
    a2.set_ylabel('e [m] / omega')
    a2.legend(loc='upper right', fontsize=8)
    a3.plot(s['t'], s['v'], lw=1, color='tab:purple', label='v comandada')
    a3.set_ylabel('v [m/s]')
    a3.set_xlabel('t [s] (simulación)')
    for ax in (a1, a2, a3):
        ax.grid(alpha=0.3)
        for tc in cruces:
            ax.axvline(tc, color='0.6', ls=':', lw=0.8)
    e = s['e'][s['v'] > 0.5]
    if len(e):
        a2.set_title(f'error: media {np.mean(e) * 100:.1f} cm, RMS {np.sqrt(np.mean(e ** 2)) * 100:.1f} cm, '
                     f'máx |e| {np.max(np.abs(e)) * 100:.1f} cm', fontsize=9)
    fig.suptitle('Seguimiento de pared (líneas punteadas: cruces de la meta)')
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'seguidor.png'), dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    sys.exit(main())
