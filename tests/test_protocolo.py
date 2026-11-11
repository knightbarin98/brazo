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
    assert correr_saludo(par, "secreto", "secreto")["canal"] == p.CANAL_CONTROL


def test_canal_desconocido_se_rechaza(par):
    with pytest.raises(p.ErrorAutenticacion, match="canal"):
        correr_saludo(par, "secreto", "secreto", "video")


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
    assert p._firma("t", "n", "otro") != p._firma("t", "n", p.CANAL_CONTROL)


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


def lectura(**cambios):
    datos = {"modo": "mano", "x": 0.5, "y": 0.5, "vector": [-1.5, -0.5, -0.5], "apertura": 0.5, "cabeceo": 10.0,
             "largo": None}
    datos.update(cambios)
    return datos


def test_mano_ida_y_vuelta():
    original = Mano("brazo", 0.1, 0.2, (0.3, 0.2, -0.1), 0.4, 15.0, 0.55, puntos=[(0, 0)])
    msj = json.loads(json.dumps(p.mano_a_mensaje(original, 12.5)))
    m = p.mano_de_mensaje(msj["mano"])
    assert (m.modo, m.x, m.y, m.vector, m.apertura, m.cabeceo, m.largo) == ("brazo", 0.1, 0.2, (0.3, 0.2, -0.1), 0.4, 15.0, 0.55)
    assert msj["t"] == 12.5
    assert p.mano_a_mensaje(None)["mano"] is None
    assert p.mano_de_mensaje(None) is None


def test_mano_sin_apertura_ni_cabeceo_es_valida():
    m = p.mano_de_mensaje(lectura(apertura=None, cabeceo=None))
    assert m.apertura is None and m.cabeceo is None


@pytest.mark.parametrize("cambios", [
    {"x": float("nan")}, {"y": float("inf")}, {"x": 50}, {"vector": [0, 0]}, {"vector": "abc"},
    {"vector": [float("nan"), 0, 0]}, {"vector": [99, 0, 0]}, {"apertura": 1.5}, {"apertura": -0.1},
    {"cabeceo": 400}, {"modo": "otro"}, {"modo": "brazo", "largo": None}, {"modo": "brazo", "largo": 0},
])
def test_mano_invalida_se_rechaza(cambios):
    with pytest.raises(ValueError):
        p.mano_de_mensaje(lectura(**cambios))


@pytest.mark.parametrize("malo", [{"x": 0.5}, {"modo": "mano", "x": "a", "y": 0, "vector": [0, 0, 0]}])
def test_mano_incompleta_o_con_texto_falla(malo):
    with pytest.raises((KeyError, ValueError)):
        p.mano_de_mensaje(malo)


@pytest.mark.parametrize("anfitrion, esperado", [("127.0.0.1", True), ("localhost", True), ("192.0.2.9", False)])
def test_es_esta_maquina(anfitrion, esperado):
    assert p.es_esta_maquina(anfitrion) is esperado


def test_la_ip_local_propia_cuenta_como_esta_maquina(monkeypatch):
    monkeypatch.setattr(p, "direccion_local", lambda: "192.168.1.50")
    assert p.es_esta_maquina("192.168.1.50")
    assert not p.es_esta_maquina("192.168.1.51")


def test_leer_token_no_lo_crea(tmp_path):
    ruta = str(tmp_path / "t.txt")
    assert p.leer_token(ruta) is None
    assert not os.path.exists(ruta)
