"""Dibujo de la app del operador, sin ventanas ni cámara."""

import numpy as np

from brazo_robot.cinematica import ik, postura_desde_servos
from brazo_robot.remoto import app


def estado_de(x, y, z, garra=50.0):
    base, puntos = postura_desde_servos(ik(x, y, z))
    return {"estado": "SIGUIENDO", "pose": [x, y, z], "garra": garra, "base": base, "esqueleto": puntos}


def test_panel_del_brazo_sin_datos():
    img = app._panel_brazo({})
    assert img.shape == (app.ALTO_PANEL, 640, 3)


def test_panel_del_brazo_cambia_con_la_postura():
    a = app._panel_brazo(estado_de(200, 0, 30))
    b = app._panel_brazo(estado_de(180, 100, 50))
    assert a.shape == b.shape == (app.ALTO_PANEL, 640, 3)
    assert np.any(a != b)


class ConexionFalsa:
    conectado, motivo, retardo_ms = True, "", 12.0

    def __init__(self, estado):
        self.estado = estado


def test_componer_la_ventana_completa():
    estado = estado_de(200, 0, 30)
    estado.update(aviso="Siguiendo. Abre y cierra la mano.", inclinacion=-90, factor=1.0, garra_sigue=True)
    botones = []
    img = app._componer(app._panel_brazo(estado), app._panel_webcam(np.zeros((480, 640, 3), np.uint8), None),
                        ConexionFalsa(estado), 25.0, "", botones)
    assert img.shape[0] == app.ALTO_PANEL + app.ALTO_ESTADO + app.ALTO_BOTONES
    assert [b[4] for b in botones] == ["activar", "pausar", "garra", "home", "salir"]
