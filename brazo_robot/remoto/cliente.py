"""Lado operador: conexión con la Raspberry, con reconexión automática."""

import socket
import threading
import time

import cv2
import numpy as np

from ..config import REMOTO_TIMEOUT_LATIDO
from .protocolo import (CANAL_CONTROL, CANAL_VIDEO, MAX_VIDEO, ErrorAutenticacion, enviar_json, recibir_json,
                        recibir_marco, saludo_cliente)


class ConexionRobot:
    def __init__(self, anfitrion, puerto, token):
        self.anfitrion, self.puerto, self.token = anfitrion, puerto, token
        self.conectado = False
        self.motivo = "Conectando..."
        self.error_fatal = None
        self.estado = {}
        self.retardo_ms = None
        self._control = self._video = None
        self._cerrojo_envio = threading.Lock()
        self._cerrojo_video = threading.Lock()
        self._frame, self._t_frame = None, 0.0
        self._parar = threading.Event()
        self._hilo = None

    def iniciar(self):
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()

    def cerrar(self):
        self._parar.set()
        self._cerrar_sockets()
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
                self._cerrar_sockets()
                return False

    def ultimo_video(self):
        """(imagen o None, segundos desde que llegó)."""
        with self._cerrojo_video:
            return self._frame, time.monotonic() - self._t_frame

    # ---------- hilos ----------
    def _conectar(self, canal):
        sock = socket.create_connection((self.anfitrion, self.puerto), timeout=3.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(REMOTO_TIMEOUT_LATIDO)
        try:
            saludo_cliente(sock, self.token, canal)
        except Exception:
            sock.close()
            raise
        return sock

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
            self._cerrar_sockets()
            self._parar.wait(2.0)

    def _sesion(self):
        # Copias locales: otro hilo puede cerrar la conexión y dejar self._control en None.
        control = self._control = self._conectar(CANAL_CONTROL)
        video = self._video = self._conectar(CANAL_VIDEO)
        video.settimeout(None)            # el video puede pararse; la conexión viva la vigila el control
        threading.Thread(target=self._recibir_video, args=(video,), daemon=True).start()
        self.conectado = True
        self.motivo = ""
        while not self._parar.is_set():
            msj = recibir_json(control)
            if msj.get("tipo") == "estado":
                self.estado = msj
                if isinstance(msj.get("eco"), (int, float)) and isinstance(msj.get("eco_edad"), (int, float)):
                    self.retardo_ms = max(0.0, (time.monotonic() - msj["eco"] - msj["eco_edad"]) * 1000)

    def _recibir_video(self, sock):
        try:
            while not self._parar.is_set():
                datos = recibir_marco(sock, MAX_VIDEO)
                f = cv2.imdecode(np.frombuffer(datos, np.uint8), cv2.IMREAD_COLOR)
                if f is not None:
                    with self._cerrojo_video:
                        self._frame, self._t_frame = f, time.monotonic()
        except (OSError, ConnectionError):
            if self._video is sock:       # sin video no se maneja: se reconecta la sesión completa
                self._cerrar_sockets()

    def _cerrar_sockets(self):
        control, video = self._control, self._video
        self._control = self._video = None
        for s in (control, video):
            if s is not None:
                try:
                    s.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                s.close()
