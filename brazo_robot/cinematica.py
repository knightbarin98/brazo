"""Cinemática inversa de el punto de agarre de la garra."""

from math import acos, atan2, cos, degrees, hypot, pi, radians, sin, sqrt

import numpy as np

from . import config
from .ajustes import AJUSTES, numero_en_rango
from .config import H, L1, L2, MAPEO, Z_SEGURO


class Inalcanzable(Exception):
    pass


def ik(x, y, z, inclinacion=-90.):
    """IK de la garra, compensando el desplazamiento de L3 al inclinar.

    -90° reproduce la geometría anterior; el ángulo no es el valor del servo.
    """
    theta = radians(numero_en_rango(inclinacion, -120, -60, "Inclinación"))
    r = hypot(x, y) - config.L3 * cos(theta)
    zw = z - H - config.L3 * sin(theta)
    if r <= 0:
        raise Inalcanzable("Inclinación incompatible con ese radio")
    d = hypot(r, zw)
    if d > L1 + L2 - 1 or d < abs(L1 - L2) + 1:
        raise Inalcanzable(f"({x:.0f}, {y:.0f}, {z:.0f}) fuera de alcance")

    base = degrees(atan2(y, x))
    a = atan2(zw, r)
    b = acos(max(-1, min(1, (L1**2 + d**2 - L2**2) / (2 * L1 * d))))
    sh = a + b
    el = acos(max(-1, min(1, (L1**2 + L2**2 - d**2) / (2 * L1 * L2))))
    l2 = sh - (pi - el)
    wr = theta - l2

    geo = {"base": base, "hombro": degrees(sh), "codo": degrees(l2 - sh), "muneca": degrees(wr)}
    servo = []
    for k in ("base", "hombro", "codo", "muneca"):
        cero, signo = MAPEO[k]
        v = cero + signo * geo[k]
        if v < 0 or v > 180:
            raise Inalcanzable(f"({x:.0f}, {y:.0f}, {z:.0f}): servo {k} fuera de rango ({v:.0f}°)")
        servo.append(v)
    return servo


def radio_max_teorico():
    zw = config.Z_RECOGIDA + config.L3 - H
    return sqrt((L1 + L2 - 1) ** 2 - zw**2)


def z_transito(x, y, z_bajo, z_alto=None, inclinacion=None):
    """Altura más alta (<= z_alto) desde la que se puede bajar en vertical hasta z_bajo.

    Algunos puntos lejanos son alcanzables abajo pero no a Z_SEGURO (la muñeca
    saldría de 0..180°). v12 rechazaba esos puntos; aquí se baja la altura de
    aproximación sin forzar ni recortar ángulos.
    """
    z_alto = Z_SEGURO if z_alto is None else z_alto
    inclinacion = AJUSTES["inclinacion"] if inclinacion is None else inclinacion
    ik(x, y, z_bajo, inclinacion)                 # el punto bajo DEBE ser alcanzable
    tope = z_bajo
    for z in np.arange(z_bajo + 1, z_alto + 1e-6, 1.0):
        try:
            ik(x, y, float(z), inclinacion)
        except Inalcanzable:
            break
        tope = float(z)
    if z_alto > z_bajo and tope - z_bajo < min(10.0, z_alto - z_bajo):
        raise Inalcanzable(f"({x:.0f}, {y:.0f}): no hay ni 10 mm de altura libre sobre Z={z_bajo:.0f}")
    return min(tope, z_alto)


def firma_geometria():
    return [L1, L2, config.L3, H, *[v for par in MAPEO.values() for v in par]]


def firma_postura():
    return {"geometria": firma_geometria(), "z": float(config.Z_RECOGIDA),
            "inclinacion": float(AJUSTES["inclinacion"])}
