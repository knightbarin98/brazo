"""Dirección del brazo con las líneas de la imagen, con una y dos cámaras (cámaras sintéticas)."""

from math import cos, radians, sin

import numpy as np
import pytest

from brazo_robot.seguimiento.mapeo import angulo_entre
from brazo_robot.seguimiento.vistas import LARGOS_DEFECTO, Lado, Orientaciones, Vista, brazo_desde_vistas, proporciones

LADO = Lado(11, 13, 15, 12, 23, 24, (17, 19))
TORSO = 0.5                                    # m de caderas a hombros
TAM = (640, 480)


def giro(eje, grados):
    eje = np.asarray(eje, float) / np.linalg.norm(eje)
    k = np.array(((0, -eje[2], eje[1]), (eje[2], 0, -eje[0]), (-eje[1], eje[0], 0)))
    a = radians(grados)
    return np.eye(3) + sin(a) * k + (1 - cos(a)) * k @ k


def direccion(flexion=0.0, abduccion=0.0):
    """(afuera, adelante, arriba) desde colgando: flexión (adelante) y luego abducción (afuera)."""
    f, a = radians(flexion), radians(abduccion)
    return np.array((sin(a), cos(a) * sin(f), -cos(a) * cos(f)))


def cuerpo(u, f, h):
    """Puntos de la persona en ejes del torso (m)."""
    p = {23: np.array((0.1, 0, 0)), 24: np.array((-0.1, 0, 0)), 11: np.array((0.18, 0, TORSO)),
         12: np.array((-0.18, 0, TORSO)), 0: np.array((0, 0.1, TORSO + 0.15)),
         14: np.array((-0.2, 0, 0.2)), 16: np.array((-0.2, 0.05, 0.0)), 18: np.array((-0.2, 0.08, -0.05)),
         20: np.array((-0.2, 0.06, -0.06))}
    p[13] = p[11] + LARGOS_DEFECTO["brazo"] * TORSO * np.asarray(u) / np.linalg.norm(u)
    p[15] = p[13] + LARGOS_DEFECTO["antebrazo"] * TORSO * np.asarray(f) / np.linalg.norm(f)
    mano = p[15] + LARGOS_DEFECTO["mano"] * TORSO * np.asarray(h) / np.linalg.norm(h)
    p[17], p[19] = mano + np.array((0, 0.01, 0)), mano - np.array((0, 0.01, 0))
    return p


# Ejes de cada cámara (x a la derecha de la imagen, y hacia abajo, z alejándose) en ejes del torso.
# De frente: mira a la persona desde delante y abajo (inclinada 25° hacia arriba).
FRENTE = giro((1, 0, 0), -25) @ np.array(((-1, 0, 0), (0, 0, -1), (0, -1, 0))).T
# Lateral: desde el costado del brazo que se sigue, también desde abajo.
LATERAL = giro((0, 1, 0), 20) @ np.array(((0, 1, 0), (0, 0, -1), (-1, 0, 0))).T


def ver(puntos, ejes, rng, profundidad=0.08, imagen=0.002):
    """Vista de una cámara: 3D con mucho error de profundidad, imagen casi exacta (proyección débil)."""
    mundo, img = {}, {}
    for i, p in puntos.items():
        c = ejes.T @ p
        img[i] = (0.5 + 0.55 * c[0] * 480 / 640 + rng.normal(0, imagen), 0.55 + 0.55 * c[1] + rng.normal(0, imagen), 0.95)
        mundo[i] = (c[0], c[1], c[2] + rng.normal(0, profundidad), 0.95)
    return Vista(mundo, img, TAM)


POSTURAS = [(f, a) for f in range(-80, 81, 20) for a in range(0, 86, 15)]


def medir(u, f, h, rng, lateral=True, profundidad=0.08, segundos=4.0, cambiar=False, otro=None):
    """Como en vivo: unos segundos moviendo el brazo (la orientación de las cámaras se asienta) y luego la postura."""
    orientaciones, t = Orientaciones(), 0.0
    for _ in range(int(segundos * 30)):
        t += 1 / 30
        d = rng.normal(size=3)
        p = cuerpo(d, d, d)
        lat = ver(p, LATERAL, rng, profundidad) if lateral else None
        brazo_desde_vistas(ver(p, FRENTE, rng, profundidad), lat, LADO, orientaciones=orientaciones, t=t)
    p = cuerpo(u, f, h)
    for _ in range(15):                                     # medio segundo quieto en la postura
        t += 1 / 30
        lat = ver(otro if otro is not None else p, LATERAL, rng, profundidad) if lateral else None
        if lat is not None and cambiar:
            lat = lat.cambiada()
        r = brazo_desde_vistas(ver(p, FRENTE, rng, profundidad), lat, LADO, orientaciones=orientaciones, t=t)
    return r


@pytest.mark.parametrize("flexion, abduccion", POSTURAS)
def test_dos_camaras_dan_la_direccion_del_brazo(flexion, abduccion):
    rng = np.random.default_rng(abs(flexion) * 100 + abduccion)
    u = direccion(flexion, abduccion)
    r = medir(u, u, u, rng, profundidad=0.05)          # 5 cm de error de profundidad en cada punto
    assert r is not None
    (brazo, antebrazo, mano), info = r
    assert info["lateral"]
    assert angulo_entre(brazo, u) < 10.0
    assert angulo_entre(antebrazo, u) < 10.0


def test_brazo_horizontal_al_costado_se_lee_horizontal():
    # el caso de tu captura: con el 3D de Pose se leía ~40°; con las líneas de la imagen, 85°
    rng = np.random.default_rng(1)
    u = direccion(0, 85)
    (brazo, _, _), _ = medir(u, u, u, rng, lateral=False, profundidad=0.08)
    assert angulo_entre(brazo, u) < 8.0


@pytest.mark.parametrize("flexion, abduccion", [(0, 0), (0, 45), (0, 80), (30, 30), (60, 10), (-40, 20), (45, 60)])
def test_una_camara_con_el_largo_del_brazo(flexion, abduccion):
    rng = np.random.default_rng(flexion + 7 * abduccion)
    u = direccion(flexion, abduccion)
    (brazo, _, _), info = medir(u, u, u, rng, lateral=False, profundidad=0.03)
    assert not info["lateral"]
    assert angulo_entre(brazo, u) < 12.0


def test_mano_caida_respecto_al_antebrazo():
    rng = np.random.default_rng(2)
    u = direccion(0, 80)
    h = direccion(0, 10)                                    # la mano cuelga
    (brazo, antebrazo, mano), _ = medir(u, u, h, rng, profundidad=0.05)
    assert angulo_entre(antebrazo, mano) == pytest.approx(angulo_entre(u, h), abs=8.0)


def test_lateral_con_izquierda_y_derecha_cambiadas():
    rng = np.random.default_rng(3)
    u = direccion(50, 20)
    (brazo, _, _), info = medir(u, u, u, rng, cambiar=True)
    assert info["lateral"] and info["lados_cambiados"]
    assert angulo_entre(brazo, u) < 10.0


@pytest.mark.parametrize("otro", [(130, 0), (90, 0)])
def test_una_lateral_que_ve_otra_cosa_no_estropea_el_brazo(otro):
    # La lateral ve a alguien con el brazo en otra postura: o se descarta, o sólo se usa lo que encaja.
    # (Límite conocido: si esa otra postura es compatible con las dos imágenes y con el largo del brazo,
    # dos vistas no bastan para notarlo; por eso la lateral debe verte sólo a ti.)
    rng = np.random.default_rng(4)
    u, d = direccion(0, 30), direccion(*otro)
    (brazo, _, _), info = medir(u, u, u, rng, otro=cuerpo(d, d, d))
    assert angulo_entre(brazo, u) < 10.0


def test_proporciones_colgando_son_los_largos():
    rng = np.random.default_rng(5)
    u = direccion()
    largos = proporciones(ver(cuerpo(u, u, u), FRENTE, rng), LADO)
    for nombre in ("brazo", "antebrazo"):
        assert largos[nombre] == pytest.approx(LARGOS_DEFECTO[nombre], rel=0.05)


def test_sin_caderas_no_hay_ejes():
    rng = np.random.default_rng(6)
    u = direccion()
    v = ver(cuerpo(u, u, u), FRENTE, rng)
    del v.imagen[23]
    assert brazo_desde_vistas(v, None, LADO) is None
