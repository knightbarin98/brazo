import json
import os
import socket
import stat
import threading

import pytest

from brazo_robot.remoto import protocolo as p
from brazo_robot.seguimiento.mapeo import Mano


@pytest.fixture
def par():
    a, b = socket.socketpair()
    a.settimeout(2)
    b.settimeout(2)
    yield a, b
    a.close()
    b.close()


def test_marco_ida_y_vuelta(par):
    a, b = par
    p.enviar_marco(a, b"hola")
    p.enviar_marco(a, b"")
    assert p.recibir_marco(b, 100) == b"hola"
    assert p.recibir_marco(b, 100) == b""


def test_marco_demasiado_grande_se_rechaza(par):
    a, b = par
    p.enviar_marco(a, b"x" * 200)
    with pytest.raises(ConnectionError, match="demasiado grande"):
        p.recibir_marco(b, 100)


def test_conexion_cerrada_a_mitad():
    a, b = socket.socketpair()
    a.sendall(b"\x00\x00\x00\x10abc")
    a.close()
    with pytest.raises(ConnectionError):
        p.recibir_marco(b, 100)
    b.close()


@pytest.mark.parametrize("basura", [b"no es json", b"[1, 2]"])
def test_json_invalido_cierra(par, basura):
    a, b = par
    p.enviar_marco(a, basura)
    with pytest.raises(ConnectionError, match="inválido"):
        p.recibir_json(b)


def correr_saludo(par, token_s, token_c, canal=p.CANAL_CONTROL):
    a, b = par
    resultado = {}

    def servidor():
        try:
            resultado["canal"] = p.saludo_servidor(a, token_s)
            p.aceptar(a)
        except p.ErrorAutenticacion as e:
            p.rechazar(a, str(e), definitivo=True)

    h = threading.Thread(target=servidor)
    h.start()
    try:
        p.saludo_cliente(b, token_c, canal)
    finally:
        h.join()
    return resultado


def test_saludo_correcto(par):
    assert correr_saludo(par, "secreto", "secreto", p.CANAL_VIDEO)["canal"] == p.CANAL_VIDEO


def test_token_incorrecto_es_definitivo(par):
    with pytest.raises(p.ErrorAutenticacion, match="Token incorrecto"):
        correr_saludo(par, "secreto", "otro")


def test_el_token_no_viaja_por_la_red(par):
    a, b = par

    def cliente():
        with pytest.raises(p.ErrorAutenticacion):
            p.saludo_cliente(b, "mi-secreto", p.CANAL_CONTROL)

    h = threading.Thread(target=cliente)
    h.start()
    p.enviar_json(a, {"version": p.VERSION, "nonce": "abc"})
    respuesta = p.recibir_marco(a, 4096)
    p.rechazar(a, "fin", definitivo=True)
    h.join(2)
    assert b"mi-secreto" not in respuesta


def test_la_firma_de_un_canal_no_sirve_para_otro():
    assert p._firma("t", "n", p.CANAL_VIDEO) != p._firma("t", "n", p.CANAL_CONTROL)


def test_version_distinta_es_definitiva(par):
    a, b = par
    p.enviar_json(a, {"version": 999, "nonce": "x"})
    with pytest.raises(p.ErrorAutenticacion, match="versiones"):
        p.saludo_cliente(b, "t", p.CANAL_CONTROL)


def test_rechazo_temporal_se_puede_reintentar(par):
    a, b = par
    p.enviar_json(a, {"version": p.VERSION, "nonce": "x"})
    p.rechazar(a, "ocupado")
    with pytest.raises(p.Rechazado, match="ocupado"):
        p.saludo_cliente(b, "t", p.CANAL_CONTROL)


def test_token_se_crea_privado_y_se_reutiliza(tmp_path):
    ruta = str(tmp_path / "token.txt")
    token = p.token_del_servidor(ruta)
    assert len(token) >= 16
    assert p.token_del_servidor(ruta) == token
    assert stat.S_IMODE(os.stat(ruta).st_mode) == 0o600


def test_mano_ida_y_vuelta():
    m = p.mano_de_mensaje(json.loads(json.dumps(p.mano_a_mensaje(Mano(0.1, 0.2, 0.3, 0.4, [(0, 0)]))))["mano"])
    assert (m.x, m.y, m.escala, m.pinza) == (0.1, 0.2, 0.3, 0.4)
    assert p.mano_a_mensaje(None)["mano"] is None
    assert p.mano_de_mensaje(None) is None


@pytest.mark.parametrize("malo", [
    {"x": float("nan"), "y": 0.5, "escala": 0.2, "pinza": 1},
    {"x": 0.5, "y": float("inf"), "escala": 0.2, "pinza": 1},
    {"x": 0.5, "y": 0.5, "escala": 0, "pinza": 1},        # log(0) en el mapeo
    {"x": 0.5, "y": 0.5, "escala": -1, "pinza": 1},
    {"x": 50, "y": 0.5, "escala": 0.2, "pinza": 1},
    {"x": 0.5, "y": 0.5, "escala": 0.2, "pinza": -1},
])
def test_mano_invalida_se_rechaza(malo):
    with pytest.raises(ValueError):
        p.mano_de_mensaje(malo)


@pytest.mark.parametrize("malo", [{"x": 0.5}, {"x": "a", "y": 0, "escala": 1, "pinza": 1}])
def test_mano_incompleta_o_con_texto_falla(malo):
    with pytest.raises((KeyError, ValueError)):
        p.mano_de_mensaje(malo)
