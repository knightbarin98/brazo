import time

import numpy as np
import pytest

from brazo_robot import cli
from brazo_robot.robot import Brazo
from brazo_robot.vision import camara as modulo_camara
from brazo_robot.vision.camara import Camara, fuente_de_texto


class CapFalsa:
    def __init__(self, fuente):
        self.fuente, self.abierta, self.fallar = fuente, fuente != "mala", False

    def isOpened(self):
        return self.abierta

    def set(self, *a):
        return True

    def get(self, p):
        return 0

    def read(self):
        time.sleep(0.001)
        return (False, None) if self.fallar else (True, np.zeros((4, 4, 3), np.uint8))

    def release(self):
        self.abierta = False


@pytest.fixture(autouse=True)
def cap_falsa(monkeypatch):
    monkeypatch.setattr(modulo_camara.cv2, "VideoCapture", CapFalsa)


def test_fuente_de_texto():
    assert fuente_de_texto("2") == 2
    assert fuente_de_texto("/dev/v4l/by-id/x") == "/dev/v4l/by-id/x"


def test_camara_que_no_abre_indica_cual():
    with pytest.raises(RuntimeError, match="persona"):
        Camara("mala", "persona")


def test_hilo_entrega_fotogramas_nuevos_y_se_detiene():
    c = Camara(0)
    c.iniciar_hilo()
    fin = time.time() + 2
    while c.ultimo()[1] is None and time.time() < fin:
        time.sleep(0.005)
    n, f = c.ultimo()
    assert f is not None and n >= 1
    c.detener_hilo()
    assert c.ultimo()[1] is None
    assert c.leer().shape == (4, 4, 3)     # sigue usable sin el hilo


def test_hilo_avisa_si_la_camara_muere():
    c = Camara(0)
    c.cap.fallar = True
    c.iniciar_hilo()
    fin = time.time() + 5
    while c.viva and time.time() < fin:
        time.sleep(0.01)
    assert not c.viva
    c.cerrar()


def test_simulado_mover_continuo_no_espera(monkeypatch):
    brazo = Brazo(simular=True)
    monkeypatch.setattr(time, "sleep", lambda s: pytest.fail("no debe esperar"))
    brazo.mover_continuo([90, 90, 90, 90, 1])
    assert brazo.ultimo == [90.0, 90.0, 90.0, 90.0, 1.0]


def test_argumentos_de_la_consola():
    a = cli.leer_argumentos(["--camara-robot", "/dev/video4", "--simular"])
    assert (a.camara_robot, a.simular) == ("/dev/video4", True)


def test_detectar_webcam_toma_la_primera_que_da_imagen(monkeypatch):
    class Cap(CapFalsa):
        def isOpened(self):
            return self.fuente >= 2
    monkeypatch.setattr(modulo_camara.cv2, "VideoCapture", Cap)
    assert modulo_camara.detectar_webcam() == 2


def test_detectar_webcam_sin_camaras(monkeypatch):
    class Cap(CapFalsa):
        def isOpened(self):
            return False
    monkeypatch.setattr(modulo_camara.cv2, "VideoCapture", Cap)
    with pytest.raises(RuntimeError, match="ninguna webcam"):
        modulo_camara.detectar_webcam()


def test_salir_devuelve_false():
    assert cli.procesar_orden("salir", None, None) is False
