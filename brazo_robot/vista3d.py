"""Dibujo del robot en 3D a partir de los ángulos de sus servos, para el simulador y la app.

Es la postura ORDENADA (cinemática directa con L1, L2, L3, H y MAPEO), no una medida del robot real.
Los puntos se pasan de los ejes del robot a los tuyos según el montaje, y la vista se orienta como tu
brazo en la webcam (que se ve en espejo): tu "afuera" queda a la derecha y "hacia la cámara" hacia
quien mira. Al costado se dibuja la pared de la caja; de pie, la mesa.
"""

from math import cos, radians, sin

import cv2
import numpy as np

from . import config
from .cinematica import postura_desde_servos
from .config import GARRA_MAX_GRADOS, H, L1, L2, MAPEO

VERDE, NARANJA, GRIS, AMBAR, ROJO = (0, 220, 0), (255, 160, 0), (200, 200, 200), (0, 200, 255), (60, 60, 255)
NOMBRES = ("base", "hombro", "codo", "muneca")


def parametros_vista():
    """Lo que hace falta para orientar el dibujo. El servidor lo manda a la app con su calibración."""
    return {"montaje": config.MONTAJE, "base": config.ARTICULAR_BASE,
            "signo": config.ARTICULAR_SIGNO_PROFUNDIDAD, "espejo": config.ARTICULAR_ESPEJO}


def a_mundo(p, vista):
    """Punto en ejes del robot -> (afuera, adelante, arriba) de la persona.

    vista None: modos mano y brazo, en los que la mano a la izquierda de la pantalla es Y+ del robot
    y hacia la webcam es X+.
    """
    if vista is None:
        return (-p[1], p[0], p[2])
    b0 = radians((vista["base"] - MAPEO["base"][0]) / MAPEO["base"][1])
    x = p[0] * cos(b0) + p[1] * sin(b0)                 # ejes del robot con la base en ARTICULAR_BASE
    y = -p[0] * sin(b0) + p[1] * cos(b0)
    s = vista["signo"]
    if vista["montaje"] == "costado":
        return (p[2], s * y, -x)
    lado = -1.0 if vista["espejo"] else 1.0
    return (lado * x, s * y, p[2])


def puntos_robot(servos):
    """Base, hombro, codo, muñeca y punto de agarre en 3D (mm, ejes del robot)."""
    base, puntos = postura_desde_servos(servos[:4])
    b = radians(base)
    return [(0.0, 0.0, 0.0)] + [(r * cos(b), r * sin(b), z) for r, z in puntos]


class Camara3D:
    """Proyección ortográfica en tus ejes: azimut alrededor de la vertical y elevación (desde arriba)."""

    def __init__(self, azimut=30.0, elevacion=20.0):
        a, e = radians(azimut), radians(elevacion)
        d, v = np.array((1.0, 0.0, 0.0)), np.array((0.0, 1.0, 0.0))     # afuera -> derecha, adelante -> hacia ti
        self.derecha = d * cos(a) + v * sin(a)
        frente = -d * sin(a) + v * cos(a)
        z = np.array((0.0, 0.0, 1.0))
        self.arriba = z * cos(e) - frente * sin(e)
        self.hacia_ti = frente * cos(e) + z * sin(e)

    def proyectar(self, p):
        p = np.asarray(p, float)
        return float(p @ self.derecha), float(p @ self.arriba), float(p @ self.hacia_ti)


def dibujar_robot(puntos, garra, ancho=640, alto=480, vista=None, azimut=30.0, elevacion=20.0, topes=(), titulo=""):
    """Imagen BGR del robot.

    puntos: base, hombro, codo, muñeca y punto de agarre en ejes del robot (puntos_robot); garra: grados
    del servo; vista: parametros_vista() en modo articular, None en los modos mano y brazo; topes:
    articulaciones en su tope (se marcan en rojo).
    """
    img = np.full((alto, ancho, 3), 32, np.uint8)
    camara = Camara3D(azimut, elevacion)
    costado = vista is not None and vista["montaje"] == "costado"
    alcance = L1 + L2 + 140.0
    escala = 0.42 * min(ancho, alto) / alcance
    centro = (ancho * 0.5, alto * (0.5 if costado else 0.72))

    def px(p_robot):
        x, y, _ = camara.proyectar(a_mundo(p_robot, vista))
        return int(centro[0] + x * escala), int(centro[1] - y * escala)

    # Superficie z = 0 del robot: la mesa (de pie) o la pared de la caja (costado), cada 100 mm
    for k in range(-3, 4):
        c = 100.0 * k
        cv2.line(img, px((c, -300, 0)), px((c, 300, 0)), (55, 55, 55), 1)
        cv2.line(img, px((-300, c, 0)), px((300, c, 0)), (55, 55, 55), 1)
    if vista is not None:                    # plano de referencia: el de la base en ARTICULAR_BASE
        b0 = radians((vista["base"] - MAPEO["base"][0]) / MAPEO["base"][1])
        e = np.array((cos(b0), sin(b0), 0.0))
        esquinas = [tuple(e * r + np.array((0.0, 0.0, z))) for r in (-300.0, 300.0) for z in (0.0, 320.0)]
        cv2.polylines(img, [np.int32([px(esquinas[0]), px(esquinas[1]), px(esquinas[3]), px(esquinas[2])])],
                      True, (90, 70, 70), 1)
    cv2.line(img, px((0, 0, 0)), px((0, 0, H)), (110, 110, 110), 10)          # columna de la base
    texto = "cuadricula: pared de la caja" if costado else "cuadricula: mesa"
    cv2.putText(img, texto, (12, alto - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (130, 130, 130), 1)

    tramos = ((1, 2, VERDE), (2, 3, NARANJA), (3, 4, GRIS))                   # brazo, antebrazo, garra
    for a, b, color in sorted(tramos, key=lambda t: camara.proyectar(a_mundo(puntos[t[0]], vista))[2]):
        cv2.line(img, px(puntos[a]), px(puntos[b]), color, 6)
    for i, nombre in enumerate(NOMBRES):     # base, hombro, codo y muñeca: puntos 0..3
        color = ROJO if nombre in topes else (255, 255, 255)
        cv2.circle(img, px(puntos[i]), 7, color, -1)
    cv2.circle(img, px(puntos[4]), 6 + int(9 * garra / GARRA_MAX_GRADOS), AMBAR, 2)   # más grande = más abierta

    # Vista desde arriba (esquina)
    lado = 110
    x0, y0 = ancho - lado - 10, 10
    cv2.rectangle(img, (x0, y0), (x0 + lado, y0 + lado), (50, 50, 50), -1)
    arriba = Camara3D(0.0, 90.0)
    c2 = (x0 + lado // 2, y0 + lado // 2)
    e2 = 0.45 * lado / (L1 + L2 + 120.0)

    def px2(p_robot):
        x, y, _ = arriba.proyectar(a_mundo(p_robot, vista))
        return int(c2[0] + x * e2), int(c2[1] - y * e2)

    cv2.line(img, px2(puntos[1]), px2(puntos[4]), VERDE, 2)
    cv2.circle(img, px2(puntos[0]), 4, (150, 150, 150), -1)
    cv2.putText(img, "desde arriba", (x0 + 4, y0 + lado - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRIS, 1)

    if titulo:
        cv2.putText(img, titulo, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(img, titulo, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    return img
