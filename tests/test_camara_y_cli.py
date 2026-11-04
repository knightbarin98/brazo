import time

import numpy as np
import pytest

from brazo_robot import cli, config
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
    assert brazo.mover_continuo([90, 90, 90, 90, 40]) is True
    assert brazo.ultimo == [90.0, 90.0, 90.0, 90.0, 40.0]


class SerialGrabador:
    """Registra lo enviado; responde lo que diga 'respuestas' (por defecto OK)."""

    def __init__(self):
        self.escrito, self.respuestas, self.limpiezas = [], [], 0
        self.timeout = None

    def write(self, b):
        self.escrito.append(b.decode().strip())

    def readline(self):
        return self.respuestas.pop(0) if self.respuestas else b"OK\n"

    def reset_input_buffer(self):
        self.limpiezas += 1


@pytest.fixture
def brazo_grabador():
    b = Brazo(simular=True)
    b.ser = SerialGrabador()
    return b


def test_garra_se_envia_en_grados_y_con_tope(brazo_grabador):
    brazo_grabador.mover_continuo([90, 90, 90, 90, 250])
    assert brazo_grabador.ser.escrito[-1] == "S,90.0,90.0,90.0,90.0,100.0"


def test_servos_conservan_el_rango_completo(brazo_grabador):
    brazo_grabador.mover_continuo([200, -5, 90, 90, 10])
    assert brazo_grabador.ser.escrito[-1] == "S,180.0,0.0,90.0,90.0,10.0"


def test_ok_tardio_no_se_confunde_y_timeout_rapido(brazo_grabador):
    limpiezas = brazo_grabador.ser.limpiezas
    brazo_grabador.mover_continuo([90, 90, 90, 90, 10])
    assert brazo_grabador.ser.limpiezas == limpiezas + 1
    assert brazo_grabador.ser.timeout == config.SERIAL_TIMEOUT_RAPIDO


def test_un_fallo_se_tolera_y_varios_detienen(brazo_grabador):
    b = brazo_grabador
    previo = list(b.ultimo)
    b.ser.respuestas = [b""] * config.SERIAL_FALLOS_MAX
    for _ in range(config.SERIAL_FALLOS_MAX - 1):
        assert b.mover_continuo([1, 2, 3, 4, 5]) is False
    assert b.ultimo == previo
    with pytest.raises(RuntimeError, match="no confirmó"):
        b.mover_continuo([1, 2, 3, 4, 5])


def test_un_ok_reinicia_los_fallos(brazo_grabador):
    b = brazo_grabador
    b.ser.respuestas = [b"", b"OK\n"]
    assert b.mover_continuo([1, 2, 3, 4, 5]) is False
    assert b.mover_continuo([1, 2, 3, 4, 5]) is True
    assert b.fallos == 0


def test_tras_seguimiento_las_ordenes_normales_recuperan_el_timeout(brazo_grabador, monkeypatch):
    monkeypatch.setattr(time, "sleep", lambda s: None)
    brazo_grabador.mover_continuo([90, 90, 90, 90, 10])
    brazo_grabador.mover([90, 90, 90, 90, 10])
    assert brazo_grabador.ser.timeout == config.SERIAL_TIMEOUT


def test_home_abre_por_defecto_y_puede_conservar_la_garra(brazo_grabador, monkeypatch):
    monkeypatch.setattr(time, "sleep", lambda s: None)
    brazo_grabador.home()
    assert brazo_grabador.ser.escrito[-1].endswith(",100.0")
    brazo_grabador.home(30.0)
    assert brazo_grabador.ser.escrito[-1].endswith(",30.0")


@pytest.mark.parametrize("orden, grados", [("garra cerrar", 0.0), ("garra abrir", 100.0), ("garra 40", 40.0)])
def test_ordenes_de_garra(brazo_grabador, monkeypatch, orden, grados):
    monkeypatch.setattr(time, "sleep", lambda s: None)
    assert cli.procesar_orden(orden, brazo_grabador, None) is True
    assert brazo_grabador.ultimo[4] == grados


def test_orden_de_garra_fuera_de_rango(brazo_grabador):
    with pytest.raises(ValueError):
        cli.procesar_orden("garra 150", brazo_grabador, None)


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
