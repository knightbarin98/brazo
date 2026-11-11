"""Protocolo Raspberry <-> app: marcos sobre TCP, autenticación y mensajes."""

import hashlib
import hmac
import json
import math
import os
import secrets
import socket
import struct

from ..config import REMOTO_TOKEN_ARCHIVO
from ..seguimiento.mapeo import MODO_BRAZO, MODO_MANO, Mano

VERSION = 3
CANAL_CONTROL = "control"
MAX_CONTROL = 16 * 1024            # bytes por mensaje


class ErrorAutenticacion(Exception):
    """Token o versión incorrectos: reintentar no sirve."""


class Rechazado(ConnectionError):
    """El servidor no acepta la conexión por ahora (p. ej. ya hay un operador)."""


def enviar_marco(sock, datos):
    sock.sendall(struct.pack("!I", len(datos)) + datos)


def _recibir_exacto(sock, n):
    partes = []
    while n:
        parte = sock.recv(min(n, 65536))
        if not parte:
            raise ConnectionError("conexión cerrada")
        partes.append(parte)
        n -= len(parte)
    return b"".join(partes)


def recibir_marco(sock, maximo):
    n = struct.unpack("!I", _recibir_exacto(sock, 4))[0]
    if n > maximo:
        raise ConnectionError(f"mensaje demasiado grande ({n} bytes)")
    return _recibir_exacto(sock, n)


def enviar_json(sock, datos):
    enviar_marco(sock, json.dumps(datos).encode())


def recibir_json(sock, maximo=MAX_CONTROL):
    try:
        datos = json.loads(recibir_marco(sock, maximo))
    except ValueError as e:
        raise ConnectionError(f"mensaje inválido: {e}")
    if not isinstance(datos, dict):
        raise ConnectionError("mensaje inválido")
    return datos


# ---------------------------------------------------------------- autenticación
def _firma(token, nonce, canal):
    return hmac.new(token.encode(), f"{nonce}:{canal}".encode(), hashlib.sha256).hexdigest()


def saludo_servidor(sock, token):
    """Comprueba al cliente sin que el token viaje por la red. Devuelve el canal pedido."""
    nonce = secrets.token_hex(16)
    enviar_json(sock, {"version": VERSION, "nonce": nonce})
    r = recibir_json(sock)
    canal = r.get("canal")
    if canal != CANAL_CONTROL:
        raise ErrorAutenticacion("canal desconocido")
    firma = r.get("firma")
    if not isinstance(firma, str) or not hmac.compare_digest(firma, _firma(token, nonce, canal)):
        raise ErrorAutenticacion("Token incorrecto")
    return canal


def rechazar(sock, motivo, definitivo=False):
    enviar_json(sock, {"ok": False, "motivo": motivo, "definitivo": definitivo})


def aceptar(sock):
    enviar_json(sock, {"ok": True})


def saludo_cliente(sock, token, canal):
    h = recibir_json(sock)
    if h.get("version") != VERSION:
        raise ErrorAutenticacion("La app y la Raspberry tienen versiones distintas del programa.")
    enviar_json(sock, {"canal": canal, "firma": _firma(token, str(h.get("nonce")), canal)})
    r = recibir_json(sock)
    if not r.get("ok"):
        motivo = str(r.get("motivo", "conexión rechazada"))
        raise ErrorAutenticacion(motivo) if r.get("definitivo") else Rechazado(motivo)


def token_del_servidor(ruta=REMOTO_TOKEN_ARCHIVO):
    """Lee el token de la Raspberry o crea uno la primera vez (sólo legible por su dueño)."""
    token = leer_token(ruta)
    if token:
        return token
    token = secrets.token_urlsafe(16)
    descriptor = os.open(ruta, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(descriptor, "w") as f:
        f.write(token + "\n")
    return token


def leer_token(ruta=REMOTO_TOKEN_ARCHIVO):
    if not os.path.exists(ruta):
        return None
    with open(ruta) as f:
        return f.read().strip() or None


def direccion_local():
    """La IP de esta máquina en la red, o None. No se envía ningún paquete."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
        except OSError:
            return None


def es_esta_maquina(anfitrion):
    """True si la dirección apunta a esta misma máquina (127.0.0.1, localhost o su IP local)."""
    if anfitrion == "localhost" or anfitrion.startswith("127."):
        return True
    return anfitrion == direccion_local()


# ---------------------------------------------------------------- lecturas de la persona
def mano_a_mensaje(mano, t=None):
    """t: reloj del operador; el servidor lo devuelve para medir el retardo de la red."""
    datos = None
    if mano is not None:
        datos = {"modo": mano.modo, "x": mano.x, "y": mano.y, "vector": list(mano.vector),
                 "apertura": mano.apertura, "cabeceo": mano.cabeceo, "largo": mano.largo}
    return {"tipo": "mano", "mano": datos, "t": t}


def _numero(valor, minimo, maximo, nombre):
    v = float(valor)
    if not math.isfinite(v) or not minimo <= v <= maximo:
        raise ValueError(f"{nombre} fuera de rango")
    return v


def mano_de_mensaje(datos):
    """Lectura validada, o None si no hay. ValueError si los números no tienen sentido.

    Es la entrada de la red hacia el brazo: un NaN o un infinito no debe llegar al control.
    """
    if datos is None:
        return None
    modo = datos["modo"]
    if modo not in (MODO_MANO, MODO_BRAZO):
        raise ValueError("modo desconocido")
    x, y = _numero(datos["x"], -1.0, 2.0, "x"), _numero(datos["y"], -1.0, 2.0, "y")
    vector = datos["vector"]
    if not isinstance(vector, list) or len(vector) != 3:
        raise ValueError("vector inválido")
    vector = tuple(_numero(v, -20.0, 20.0, "vector") for v in vector)
    apertura = datos.get("apertura")
    apertura = None if apertura is None else _numero(apertura, 0.0, 1.0, "apertura")
    cabeceo = datos.get("cabeceo")
    cabeceo = None if cabeceo is None else _numero(cabeceo, -180.0, 180.0, "cabeceo")
    largo = datos.get("largo")
    largo = None if largo is None else _numero(largo, 0.05, 5.0, "largo")
    if modo == MODO_BRAZO and largo is None:
        raise ValueError("falta el largo del brazo")
    return Mano(modo, x, y, vector, apertura, cabeceo, largo)
