"""Servidor y cliente reales hablando por TCP en esta máquina, con cámara y Arduino simulados."""

import threading
import time

import numpy as np
import pytest

from brazo_robot import config
from brazo_robot.remoto.cliente import ConexionRobot
from brazo_robot.remoto.protocolo import mano_a_mensaje
from brazo_robot.remoto.servidor import ServidorRobot
from brazo_robot.robot import Brazo
from brazo_robot.seguimiento.control import PAUSADO, SIGUIENDO
from brazo_robot.seguimiento.mapeo import Mano

TOKEN = "token-de-prueba"


class CamaraFalsa:
    def __init__(self):
        self.viva, self.n = True, 0

    def iniciar_hilo(self):
        pass

    def detener_hilo(self):
        pass

    def ultimo(self):
        if not self.viva:
            return self.n, None
        self.n += 1
        return self.n, np.zeros((48, 64, 3), np.uint8)


class VisionFalsa:
    Hinv0 = None

    def __init__(self):
        self.camara = CamaraFalsa()

    def _overlay(self, img):
        pass


def mano(x=0.5, y=0.5, escala=0.2, pinza=1.0):
    return Mano(x, y, escala, pinza, [])


def esperar(condicion, segundos=4.0):
    fin = time.monotonic() + segundos
    while time.monotonic() < fin:
        if condicion():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture
def servidor(monkeypatch):
    monkeypatch.setattr(config, "Z_RECOGIDA", 0.0)
    monkeypatch.setattr(Brazo, "mover", Brazo.mover_continuo)    # sin las esperas del movimiento real
    s = ServidorRobot(Brazo(simular=True), VisionFalsa(), TOKEN, "127.0.0.1", 0)
    h = threading.Thread(target=s.ejecutar, daemon=True)
    h.start()
    yield s
    s.detener()
    h.join(2)


@pytest.fixture
def cliente(servidor):
    c = ConexionRobot("127.0.0.1", servidor.puerto, TOKEN)
    c.iniciar()
    assert esperar(lambda: c.conectado and c.estado)
    yield c
    c.cerrar()


def siguiendo(servidor, c):
    """Lleva el brazo a seguir la mano, como haría el operador."""
    c.enviar({"tipo": "activar"})                       # primera vez: al punto de partida
    assert esperar(lambda: servidor.control.pose is not None)
    c.enviar(mano_a_mensaje(mano()))
    time.sleep(0.05)
    c.enviar({"tipo": "activar"})
    assert esperar(lambda: servidor.control.estado == SIGUIENDO)


def test_el_operador_recibe_estado_y_video(cliente):
    assert cliente.estado["estado"] == PAUSADO
    assert esperar(lambda: cliente.ultimo_video()[0] is not None)
    assert cliente.ultimo_video()[0].shape[:2] == (48, 64)


def test_token_incorrecto_no_entra_y_no_reintenta(servidor):
    c = ConexionRobot("127.0.0.1", servidor.puerto, "malo")
    c.iniciar()
    assert esperar(lambda: c.error_fatal)
    assert "Token" in c.error_fatal and not c.conectado
    c.cerrar()


def test_segundo_operador_rechazado_y_entra_al_liberarse(servidor, cliente):
    otro = ConexionRobot("127.0.0.1", servidor.puerto, TOKEN)
    otro.iniciar()
    assert esperar(lambda: "otro operador" in otro.motivo)
    assert not otro.conectado and otro.error_fatal is None
    cliente.cerrar()
    assert esperar(lambda: otro.conectado, 6.0)
    otro.cerrar()


def test_la_mano_mueve_el_brazo(servidor, cliente):
    siguiendo(servidor, cliente)
    y0 = servidor.control.pose[1]
    for i in range(30):
        cliente.enviar(mano_a_mensaje(mano(x=0.5 - i * 0.005)))
        time.sleep(0.03)
    assert servidor.control.pose[1] > y0 + 5


def test_dejar_de_recibir_datos_detiene_el_brazo(servidor, cliente):
    # La app sigue conectada pero no llegan manos: como una red muy lenta.
    siguiendo(servidor, cliente)
    assert esperar(lambda: servidor.control.estado == PAUSADO, config.SEGUIR_TIMEOUT_MANO + 1.0)


def test_mano_nula_detiene_el_brazo(servidor, cliente):
    siguiendo(servidor, cliente)
    fin = time.monotonic() + config.SEGUIR_TIMEOUT_MANO + 1.0
    while time.monotonic() < fin and servidor.control.estado == SIGUIENDO:
        cliente.enviar(mano_a_mensaje(None))
        time.sleep(0.03)
    assert servidor.control.estado == PAUSADO


def test_cerrar_la_app_detiene_el_brazo_al_instante(servidor, cliente):
    siguiendo(servidor, cliente)
    cliente.enviar(mano_a_mensaje(mano()))
    cliente._cerrar_sockets()                           # como si se cayera la conexión
    assert esperar(lambda: servidor.control.estado == PAUSADO, 0.3)
    assert "desconectado" in servidor.control.aviso


def test_reconexion_queda_en_pausa(servidor, cliente):
    siguiendo(servidor, cliente)
    cliente._cerrar_sockets()
    assert esperar(lambda: not cliente.conectado)
    assert esperar(lambda: cliente.conectado and cliente.estado, 6.0)
    assert cliente.estado["estado"] == PAUSADO


def test_perder_el_video_detiene_el_brazo_y_reconecta(servidor, cliente):
    siguiendo(servidor, cliente)
    video = cliente._video
    video.shutdown(2)                                   # sólo se corta el video
    assert esperar(lambda: servidor.control.estado == PAUSADO, 1.0)
    assert esperar(lambda: cliente.conectado and cliente._video is not None and cliente._video is not video, 6.0)


def test_datos_invalidos_no_rompen_ni_mueven(servidor, cliente):
    siguiendo(servidor, cliente)
    pose = servidor.control.pose
    cliente.enviar({"tipo": "mano", "mano": {"x": float("nan"), "y": 0, "escala": 1, "pinza": 1}})
    cliente.enviar({"tipo": "mano", "mano": {"x": "hola"}})
    cliente.enviar({"tipo": "desconocido"})
    time.sleep(0.1)
    assert servidor.control.pose == pose
    assert cliente.conectado


def test_orden_vieja_se_descarta(servidor):
    with servidor._cerrojo:
        servidor._eventos.append(("iman", time.monotonic() - 5.0))
    time.sleep(0.1)
    assert not servidor.control.iman_encendido


def test_camara_del_robot_perdida_detiene_y_no_deja_activar(servidor, cliente):
    siguiendo(servidor, cliente)
    servidor.vision.camara.viva = False
    assert esperar(lambda: servidor.control.estado == PAUSADO, 0.5)
    cliente.enviar(mano_a_mensaje(mano()))
    cliente.enviar({"tipo": "activar"})
    time.sleep(0.1)
    assert servidor.control.estado == PAUSADO


def test_error_del_arduino_pausa_y_el_servidor_sigue(servidor, cliente):
    siguiendo(servidor, cliente)

    def falla(texto):
        raise RuntimeError("Arduino no confirmó")
    servidor.brazo._cmd = falla
    for i in range(10):
        cliente.enviar(mano_a_mensaje(mano(x=0.5 - i * 0.01)))
        time.sleep(0.03)
    assert esperar(lambda: servidor.control.estado == PAUSADO, 1.0)
    assert "Arduino" in servidor.control.aviso
    assert cliente.conectado
