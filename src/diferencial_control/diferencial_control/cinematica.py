"""Cinemática de un robot de tracción diferencial (funciones puras, sin ROS).

Notación (Siegwart, Nourbakhsh y Scaramuzza, "Introduction to Autonomous Mobile Robots", cap. 3):

    w  = [wD, wI]^T      velocidades angulares de las llantas derecha e izquierda [rad/s]
    u  = [v, omega]^T    velocidad lineal y angular del robot, en el marco del robot
    q  = [x, y, theta]^T pose en el marco odom;  q_dot = [x_dot, y_dot, theta_dot]^T

Cadena  w --(A)--> u --(J(theta))--> q_dot, con r = radio de llanta y b = separación entre llantas:

    v     = r (wD + wI) / 2          A = [[ r/2,  r/2],
    omega = r (wD - wI) / b              [ r/b, -r/b]]

              [cos(theta)  0]
    J(theta)= [sin(theta)  0]          q_dot = J(theta) u = J(theta) A w
              [    0       1]

Inversa (la que usa el plugin DiffDrive al recibir /cmd_vel):  w = A^-1 u
    wD = (v + omega b / 2) / r ,  wI = (v - omega b / 2) / r
"""
import math

import numpy as np


def matriz_A(r, b):
    """u = A w   (2x2)."""
    return np.array([[r / 2.0, r / 2.0],
                     [r / b, -r / b]], dtype=float)


def matriz_J(theta):
    """q_dot = J(theta) u   (3x2)."""
    return np.array([[math.cos(theta), 0.0],
                     [math.sin(theta), 0.0],
                     [0.0, 1.0]], dtype=float)


def matriz_T(r, b, theta):
    """q_dot = T w con T = J(theta) A   (3x2)."""
    return matriz_J(theta) @ matriz_A(r, b)


def directa_u(w_der, w_izq, r, b):
    """w -> u = [v, omega]."""
    u = matriz_A(r, b) @ np.array([w_der, w_izq], dtype=float)
    return float(u[0]), float(u[1])


def inversa_w(v, omega, r, b):
    """u -> w = [wD, wI]."""
    w = np.linalg.solve(matriz_A(r, b), np.array([v, omega], dtype=float))
    return float(w[0]), float(w[1])


def envolver(a):
    """Ángulo en [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def integrar(x, y, theta, v, omega, dt, metodo='punto_medio'):
    """Integra q_dot = J(theta) u durante dt.

    'euler'       : q += q_dot(theta_k) dt                       (orden 1)
    'punto_medio' : usa theta_k + omega dt/2 para la dirección   (orden 2, = Runge-Kutta 2)
    """
    if metodo == 'euler':
        th = theta
    elif metodo == 'punto_medio':
        th = theta + 0.5 * omega * dt
    else:
        raise ValueError(f'método desconocido: {metodo}')
    x += v * math.cos(th) * dt
    y += v * math.sin(th) * dt
    theta = envolver(theta + omega * dt)
    return x, y, theta


def yaw_de_cuaternion(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
