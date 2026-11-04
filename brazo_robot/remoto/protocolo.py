"""Protocolo Raspberry <-> app: marcos sobre TCP, autenticación y mensajes."""

import hashlib
import hmac
import json
import math
import os
import secrets
import struct

from ..config import REMOTO_TOKEN_ARCHIVO
from ..seguimiento.mapeo import Mano

VERSION = 1
CANAL_CONTROL, CANAL_VIDEO = "control", "video"
MAX_CONTROL = 16 * 1024            # bytes por mensaje de control
MAX_VIDEO = 8 * 1024 * 1024        # bytes por fotograma JPEG


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
    if canal not in (CANAL_CONTROL, CANAL_VIDEO):
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
    if os.path.exists(ruta):
        with open(ruta) as f:
            token = f.read().strip()
        if token:
            return token
    token = secrets.token_urlsafe(16)
    descriptor = os.open(ruta, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(descriptor, "w") as f:
        f.write(token + "\n")
    return token


# ---------------------------------------------------------------- mensajes de la mano
def mano_a_mensaje(mano):
    datos = None if mano is None else {"x": mano.x, "y": mano.y, "escala": mano.escala, "pinza": mano.pinza}
    return {"tipo": "mano", "mano": datos}


def mano_de_mensaje(datos):
    """Mano validada, o None si no hay mano. ValueError si los números no tienen sentido.

    Es la entrada de la red hacia el brazo: un NaN o un infinito no debe llegar al control.
    """
    if datos is None:
        return None
    x, y, escala, pinza = (float(datos[k]) for k in ("x", "y", "escala", "pinza"))
    if not all(map(math.isfinite, (x, y, escala, pinza))):
        raise ValueError("valores no finitos")
    if not (-1.0 <= x <= 2.0 and -1.0 <= y <= 2.0 and 1e-4 <= escala <= 10.0 and 0.0 <= pinza <= 1000.0):
        raise ValueError("valores fuera de rango")
    return Mano(x, y, escala, pinza, [])
