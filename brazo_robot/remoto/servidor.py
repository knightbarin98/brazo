"""Lado Raspberry: controla el brazo, transmite la cámara y atiende al operador."""

import socket
import threading
import time
from collections import deque

import cv2
import numpy as np

from ..ajustes import AJUSTES
from ..config import (REMOTO_ESCUCHAR, REMOTO_EVENTO_MAX_EDAD, REMOTO_MANO_MAX_EDAD, REMOTO_PUERTO,
                      REMOTO_TIMEOUT_LATIDO, REMOTO_VIDEO_CALIDAD, REMOTO_VIDEO_FPS)
from ..seguimiento.control import ControlSeguimiento
from .protocolo import (CANAL_CONTROL, ErrorAutenticacion, aceptar, enviar_json, enviar_marco, mano_de_mensaje,
                        rechazar, recibir_json, saludo_servidor)


class _Sesion:
    """Un operador: su conexión de control y la de video. Si cae una, se cierran las dos."""

    def __init__(self, sock):
        self.control, self.video = sock, None
        self.activa = True

    def cerrar(self):
        self.activa = False
        for s in (self.control, self.video):
            if s is not None:
                try:
                    s.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                s.close()


class ServidorRobot:
    """El brazo y sus protecciones viven aquí, no en la app: si la red o la app fallan, se detiene.

    Sólo el hilo principal (ejecutar) habla con el Arduino; los hilos de red dejan datos para él.
    """

    def __init__(self, brazo, vision, token, escuchar=REMOTO_ESCUCHAR, puerto=REMOTO_PUERTO):
        self.brazo, self.vision, self.token = brazo, vision, token
        self.control = ControlSeguimiento(brazo, AJUSTES["inclinacion"])
        self._cerrojo = threading.Lock()
        self._sesion = None
        self._mano, self._t_mano, self._mano_nueva = None, -1e9, False
        self._eventos = deque()
        self._estado = {}
        self._eco, self._t_eco = None, 0.0
        self._ordenes_previas, self._t_ordenes, self._ordenes_s = 0, time.monotonic(), 0.0
        self._parar = threading.Event()
        self._oyente = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._oyente.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._oyente.bind((escuchar, puerto))
        self._oyente.listen(4)
        self._oyente.settimeout(0.5)
        self.puerto = self._oyente.getsockname()[1]

    # ---------- bucle de control (hilo principal) ----------
    def ejecutar(self):
        self.vision.camara.iniciar_hilo()
        threading.Thread(target=self._aceptar, daemon=True).start()
        try:
            while not self._parar.is_set():
                self.ciclo(time.monotonic())
                time.sleep(0.01)
        finally:
            self.detener()

    def detener(self):
        self._parar.set()
        with self._cerrojo:
            sesion, self._sesion = self._sesion, None
        if sesion is not None:
            sesion.cerrar()
        self._oyente.close()
        self.vision.camara.detener_hilo()

    def ciclo(self, t):
        try:
            if not self.vision.camara.viva:
                self.control.pausar("Se perdió la cámara del robot: brazo detenido.")
            self._atender_eventos(t)
            with self._cerrojo:
                mano = self._mano if self._mano_nueva else None
                self._mano_nueva = False
            self.control.actualizar(mano, t)
        except (RuntimeError, OSError) as e:      # Arduino desconectado u orden sin confirmar
            print("Error del brazo:", e)
            self.control.pausar(f"Error del brazo: {e}.")
        self._publicar_estado()

    def _atender_eventos(self, t):
        while True:
            with self._cerrojo:
                if not self._eventos:
                    return
                tipo, llegada = self._eventos.popleft()
                mano = self._mano if t - self._t_mano <= REMOTO_MANO_MAX_EDAD else None
            if tipo == "desconectado":
                self.control.pausar("Operador desconectado: brazo detenido.")
            elif t - llegada > REMOTO_EVENTO_MAX_EDAD:
                continue                          # esperó durante un movimiento: ya no es lo que se pidió
            elif tipo == "pausar":
                self.control.pausar("En pausa: puedes recolocar la mano.")
            elif tipo == "activar":
                if self.vision.camara.viva:
                    self.control.activar(mano, t)
            elif tipo == "garra":
                self.control.alternar_garra()
            elif tipo == "home":
                self.control.ir_a_home()
            elif tipo == "factor_mas":
                self.control.ajustar_factor(+1)
            elif tipo == "factor_menos":
                self.control.ajustar_factor(-1)

    def _publicar_estado(self):
        c, ahora = self.control, time.monotonic()
        if ahora - self._t_ordenes >= 1.0:
            self._ordenes_s = (c.ordenes - self._ordenes_previas) / (ahora - self._t_ordenes)
            self._ordenes_previas, self._t_ordenes = c.ordenes, ahora
        with self._cerrojo:
            self._estado = {"tipo": "estado", "estado": c.estado, "aviso": c.aviso,
                            "pose": None if c.pose is None else [round(v, 1) for v in c.pose],
                            "garra": round(c.garra, 1), "garra_sigue": bool(c.seguidor_garra.sigue),
                            "inclinacion": round(c.inclinacion, 1), "factor": c.factor,
                            "camara": bool(self.vision.camara.viva), "ordenes_s": round(self._ordenes_s, 1)}

    # ---------- red (hilos secundarios) ----------
    def _aceptar(self):
        while not self._parar.is_set():
            try:
                sock, direccion = self._oyente.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            threading.Thread(target=self._atender, args=(sock, direccion), daemon=True).start()

    def _atender(self, sock, direccion):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(REMOTO_TIMEOUT_LATIDO)
        try:
            canal = saludo_servidor(sock, self.token)
            if canal == CANAL_CONTROL:
                self._sesion_control(sock, direccion)
            else:
                self._sesion_video(sock)
        except ErrorAutenticacion as e:
            print(f"Conexión rechazada desde {direccion[0]}: {e}")
            time.sleep(1.0)                       # frena los intentos de adivinar el token
            self._rechazar_y_cerrar(sock, str(e), definitivo=True)
        except (OSError, ConnectionError):
            sock.close()

    @staticmethod
    def _rechazar_y_cerrar(sock, motivo, definitivo=False):
        try:
            rechazar(sock, motivo, definitivo)
        except OSError:
            pass
        sock.close()

    def _sesion_control(self, sock, direccion):
        with self._cerrojo:
            ocupado = self._sesion is not None
            if not ocupado:
                sesion = self._sesion = _Sesion(sock)
        if ocupado:
            self._rechazar_y_cerrar(sock, "Ya hay otro operador conectado.")
            return
        print(f"Operador conectado desde {direccion[0]}.")
        try:
            aceptar(sock)
            threading.Thread(target=self._enviar_estado, args=(sesion,), daemon=True).start()
            while sesion.activa:
                self._recibir_mensaje(recibir_json(sock))
        except (OSError, ConnectionError) as e:
            print(f"Operador desconectado: {e}")
        finally:
            with self._cerrojo:
                if self._sesion is sesion:
                    self._sesion = None
                self._mano, self._mano_nueva = None, False
                self._eventos.append(("desconectado", time.monotonic()))
            sesion.cerrar()

    def _recibir_mensaje(self, msj):
        tipo, ahora = msj.get("tipo"), time.monotonic()
        if tipo == "mano":
            try:
                mano = mano_de_mensaje(msj.get("mano"))
            except (KeyError, TypeError, ValueError) as e:
                print("Datos de mano ignorados:", e)
                return
            with self._cerrojo:
                self._mano, self._mano_nueva = mano, mano is not None
                if mano is not None:
                    self._t_mano = ahora
                if isinstance(msj.get("t"), (int, float)):
                    self._eco, self._t_eco = msj["t"], ahora
        elif tipo in ("activar", "pausar", "garra", "home", "factor_mas", "factor_menos"):
            with self._cerrojo:
                self._eventos.append((tipo, ahora))

    def _enviar_estado(self, sesion):
        try:
            while sesion.activa:
                with self._cerrojo:
                    estado = dict(self._estado)
                    if self._eco is not None:
                        estado["eco"], estado["eco_edad"] = self._eco, time.monotonic() - self._t_eco
                if estado:
                    enviar_json(sesion.control, estado)
                time.sleep(0.1)
        except OSError:
            pass                                  # el lector de control se encarga del cierre

    def _sesion_video(self, sock):
        with self._cerrojo:
            sesion = self._sesion
            libre = sesion is not None and sesion.video is None
            if libre:
                sesion.video = sock
        if not libre:
            self._rechazar_y_cerrar(sock, "El video requiere una sesión de control activa.")
            return
        visto, periodo, siguiente = -1, 1.0 / REMOTO_VIDEO_FPS, 0.0
        calidad = [cv2.IMWRITE_JPEG_QUALITY, REMOTO_VIDEO_CALIDAD]
        try:
            aceptar(sock)
            while sesion.activa and not self._parar.is_set():
                n, f = self.vision.camara.ultimo()
                ahora = time.monotonic()
                if f is None or n == visto or ahora < siguiente:
                    time.sleep(0.005)
                    continue
                visto, siguiente = n, ahora + periodo
                ok, jpeg = cv2.imencode(".jpg", self._dibujar(f), calidad)
                if ok:
                    # Si la red va lenta esto tarda y se saltan fotogramas: siempre se manda el último.
                    enviar_marco(sock, jpeg.tobytes())
        finally:
            sesion.cerrar()                       # sin video no se maneja: el brazo pasa a pausa

    def _dibujar(self, f):
        img = f.copy()
        self.vision._overlay(img)
        pose = self.control.pose
        if pose is not None and self.vision.Hinv0 is not None:
            cx, cy = self.vision.corregir_xy(*pose[:2])
            real = (2 * pose[0] - cx, 2 * pose[1] - cy)   # inversa aproximada de la corrección
            p = np.int32(np.round(self.vision.a_px([real])[0]))
            cv2.drawMarker(img, (int(p[0]), int(p[1])), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
        return img
