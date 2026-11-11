"""Lado operador: conexión con la Raspberry, con reconexión automática."""

import socket
import threading
import time

from ..config import REMOTO_TIMEOUT_LATIDO
from .protocolo import CANAL_CONTROL, ErrorAutenticacion, enviar_json, recibir_json, saludo_cliente


class ConexionRobot:
    def __init__(self, anfitrion, puerto, token):
        self.anfitrion, self.puerto, self.token = anfitrion, puerto, token
        self.conectado = False
        self.motivo = "Conectando..."
        self.error_fatal = None
        self.estado = {}
        self.retardo_ms = None
        self._control = None
        self._cerrojo_envio = threading.Lock()
        self._parar = threading.Event()
        self._hilo = None

    def iniciar(self):
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()

    def cerrar(self):
        self._parar.set()
        self._cerrar_socket()
        if self._hilo is not None:
            self._hilo.join(timeout=2.0)

    def enviar(self, mensaje):
        """False si no hay conexión. Nunca detiene la app por un error de red."""
        with self._cerrojo_envio:
            control = self._control
            if not self.conectado or control is None:
                return False
            try:
                enviar_json(control, mensaje)
                return True
            except OSError as e:
                self.motivo = f"Conexión perdida: {e}"
                self._cerrar_socket()
                return False

    def _bucle(self):
        while not self._parar.is_set():
            try:
                self._sesion()
            except ErrorAutenticacion as e:
                self.error_fatal = str(e)
                return
            except (OSError, ConnectionError, ValueError) as e:
                self.motivo = f"Sin conexión con {self.anfitrion}: {e}"
            self.conectado = False
            self.estado = {}
            self._cerrar_socket()
            self._parar.wait(2.0)

    def _sesion(self):
        sock = socket.create_connection((self.anfitrion, self.puerto), timeout=3.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(REMOTO_TIMEOUT_LATIDO)
        try:
            saludo_cliente(sock, self.token, CANAL_CONTROL)
        except Exception:
            sock.close()
            raise
        # Copia local: otro hilo puede cerrar la conexión y dejar self._control en None.
        self._control = control = sock
        self.conectado = True
        self.motivo = ""
        while not self._parar.is_set():
            msj = recibir_json(control)
            if msj.get("tipo") == "estado":
                self.estado = msj
                if isinstance(msj.get("eco"), (int, float)) and isinstance(msj.get("eco_edad"), (int, float)):
                    self.retardo_ms = max(0.0, (time.monotonic() - msj["eco"] - msj["eco_edad"]) * 1000)

    def _cerrar_socket(self):
        control, self._control = self._control, None
        if control is not None:
            try:
                control.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            control.close()
