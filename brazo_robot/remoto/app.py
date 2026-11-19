"""App del operador: la única cámara es su webcam; sus movimientos se traducen al brazo.

Funciona igual en otra computadora o en la misma Raspberry: sólo cambia la dirección del servidor.
No hay video del robot: el operador debe ver el brazo directamente. El panel izquierdo dibuja la
postura que se le ordena, calculada con los ángulos enviados a los servos.
"""

import threading
import time
import unicodedata
from math import cos, radians, sin

import cv2
import numpy as np

from ..config import ARTICULAR_PROFUNDIDAD_DUDOSA, GEMELO_CALIBRACION_SEGUNDOS, WEBCAM_RESOLUCION
from ..seguimiento.control import ACTIVOS, SIGUIENDO
from ..seguimiento.mano import (CONEXIONES, DetectorBrazo, DetectorMano, cargar_calibracion_mano,
                                guardar_calibracion_mano)
from ..seguimiento.mapeo import MODO_ARTICULAR, MODO_BRAZO, calibracion_gemelo
from ..ui import leer_tecla, texto_con_borde, ventana_cerrada
from ..vision.camara import Camara
from .cliente import ConexionRobot
from .protocolo import mano_a_mensaje
from ..vista3d import dibujar_robot

VENTANA = "Brazo - operador"
VERDE, AMBAR, ROJO, NARANJA = (0, 220, 0), (0, 200, 255), (60, 60, 255), (255, 160, 0)
ALTO_PANEL, ALTO_ESTADO, ALTO_BOTONES = 480, 78, 44
CONTROLES = [("ACTIVAR", "activar"), ("PAUSAR", "pausar"), ("GARRA", "garra"), ("HOME", "home"), ("SALIR", "salir")]


def _sin_acentos(texto):
    """putText de OpenCV sólo dibuja ASCII: una letra acentuada saldría como '?'."""
    return unicodedata.normalize("NFD", texto).encode("ascii", "ignore").decode()


def _ajustar(img, alto):
    return cv2.resize(img, (int(img.shape[1] * alto / img.shape[0]), alto))


class _Deteccion(threading.Thread):
    """Detecta y envía en su propio hilo: la ventana no se traba aunque el modelo sea lento."""

    def __init__(self, webcam, detector, conexion):
        super().__init__(daemon=True)
        self.webcam, self.detector, self.conexion = webcam, detector, conexion
        self.cerrojo = threading.Lock()
        self.activo = True
        self.error = None
        self.frame, self.mano, self.hz = None, None, 0.0
        self.n = 0                                   # lecturas hechas: para saber si hay una nueva

    def run(self):
        visto, cuenta, t_hz, ultimo_envio = -1, 0, time.monotonic(), 0.0
        try:
            while self.activo:
                ahora = time.monotonic()
                n, f = self.webcam.ultimo()
                if f is None or n == visto:
                    if ahora - ultimo_envio > 1.0:   # webcam parada: la conexión sigue viva y el brazo se detiene solo
                        self.conexion.enviar({"tipo": "latido"})
                        ultimo_envio = ahora
                    time.sleep(0.003)
                    continue
                visto = n
                f = cv2.flip(f, 1)                   # como un espejo: tu izquierda queda a la izquierda
                mano = self.detector.detectar(f, ahora)
                self.conexion.enviar(mano_a_mensaje(mano, time.monotonic()))
                ultimo_envio = ahora
                cuenta += 1
                if ahora - t_hz >= 1.0:
                    self.hz, cuenta, t_hz = cuenta / (ahora - t_hz), 0, ahora
                with self.cerrojo:
                    self.frame, self.mano, self.n = f, mano, self.n + 1
        except Exception as e:                       # cualquier fallo debe llegar a la ventana, no perderse
            self.error = e

    def ultimo(self):
        with self.cerrojo:
            return self.frame, self.mano

    def ultimo_numerado(self):
        with self.cerrojo:
            return self.n, self.frame, self.mano

    def parar(self):
        self.activo = False
        self.join(timeout=2.0)


def _panel_brazo(estado):
    """Postura ordenada al brazo, en 3D, orientada como tu brazo en la webcam."""
    if not estado.get("esqueleto"):
        img = np.full((ALTO_PANEL, 640, 3), 32, np.uint8)
        texto_con_borde(img, "Esperando al robot...", (210, 240), 0.7, AMBAR)
        return img
    b = radians(estado.get("base", 0.0))
    puntos = [(0.0, 0.0, 0.0)] + [(r * cos(b), r * sin(b), z) for r, z in estado["esqueleto"]]
    vista = estado.get("vista") if estado.get("articular") else None
    img = dibujar_robot(puntos, estado.get("garra", 0.0), 640, ALTO_PANEL, vista,
                        topes=estado.get("topes", ()), titulo="Postura ordenada")
    texto_con_borde(img, "Mira el brazo real: esto es lo que se le ordena", (20, ALTO_PANEL - 14), 0.5, AMBAR)
    return img


def _panel_webcam(frame, mano):
    if frame is None:
        img = np.full((ALTO_PANEL, 640, 3), 40, np.uint8)
        texto_con_borde(img, "Cargando webcam...", (210, 240), 0.7, AMBAR)
        return img
    img = _ajustar(frame, ALTO_PANEL)
    h, w = img.shape[:2]
    if mano is not None:
        if mano.puntos_brazo:
            pts = [(int(x * w), int(y * h)) for x, y in mano.puntos_brazo]
            cv2.line(img, pts[0], pts[1], VERDE, 3)
            cv2.line(img, pts[1], pts[2], NARANJA, 3)
            for p in pts:
                cv2.circle(img, p, 6, (255, 255, 255), -1)
        if mano.puntos:
            pts = [(int(x * w), int(y * h)) for x, y in mano.puntos]
            for a, b in CONEXIONES:
                cv2.line(img, pts[a], pts[b], VERDE, 2)
    aviso = "no te veo" if mano is None else ("no veo tu mano" if mano.apertura is None else "")
    texto_con_borde(img, "Tu webcam" + (f" - {aviso}" if aviso else ""), (8, 26), 0.6,
                    AMBAR if aviso else (255, 255, 255))
    if mano is not None and mano.medidas.get("flexion") is not None:
        m = mano.medidas
        texto_con_borde(img, f"codo {m['flexion']:.0f}  azimut {m['azimut']:.0f}  elevacion {m['elevacion']:.0f}",
                        (8, h - 12), 0.5)
        if m["hacia_camara"] > ARTICULAR_PROFUNDIDAD_DUDOSA and not m.get("lateral"):
            texto_con_borde(img, "Brazo hacia la camara: profundidad poco fiable", (8, h - 34), 0.5, AMBAR)
    elif mano is not None and mano.angulos is not None:
        nombres = ("giro", "hombro", "codo", "mano")
        texto = "  ".join(f"{n} {a:.0f}" for n, a in zip(nombres, mano.angulos) if a is not None)
        texto_con_borde(img, texto, (8, h - 12), 0.5)
    if mano is not None and mano.apertura is not None:
        x2 = 8 + int(mano.apertura * 150)
        cv2.rectangle(img, (8, 38), (158, 50), (70, 70, 70), -1)
        cv2.rectangle(img, (8, 38), (x2, 50), VERDE, -1)
    return img


def _lineas_estado(conexion, hz_deteccion, aviso_local):
    e = conexion.estado
    red = f"{conexion.retardo_ms:.0f} ms" if conexion.retardo_ms is not None else "?"
    medidas = f"red {red} | deteccion {hz_deteccion:.0f} Hz | ordenes {e.get('ordenes_s', 0):.0f}/s"
    if not (conexion.conectado and e):
        return ROJO, "DESCONECTADO", conexion.motivo, medidas
    color = VERDE if e.get("estado") == SIGUIENDO else AMBAR
    pose = "(%.0f, %.0f, %.0f) mm" % tuple(e["pose"]) if e.get("pose") else "sin posicion"
    sigue = "sigue tu mano" if e.get("garra_sigue") else "fija"
    linea1 = (f"{e.get('estado')} | {pose} | incl {e.get('inclinacion', 0):.0f} | "
              f"garra {e.get('garra', 0):.0f} ({sigue}) | x{e.get('factor', 1):.1f}")
    return color, linea1, aviso_local or e.get("aviso", ""), medidas


def _componer(brazo, webcam, conexion, hz_deteccion, aviso_local, botones):
    arriba = np.hstack([brazo, webcam])
    ancho = arriba.shape[1]
    img = np.zeros((ALTO_PANEL + ALTO_ESTADO + ALTO_BOTONES, ancho, 3), np.uint8)
    img[:ALTO_PANEL] = arriba
    color, linea1, linea2, medidas = _lineas_estado(conexion, hz_deteccion, aviso_local)
    texto_con_borde(img, linea1, (8, ALTO_PANEL + 22), 0.55, color)
    texto_con_borde(img, _sin_acentos(linea2)[:120], (8, ALTO_PANEL + 45), 0.45)
    texto_con_borde(img, medidas + " | C calibrar (brazo colgando) | O/P apertura de tu mano | +/- ganancia",
                    (8, ALTO_PANEL + 67), 0.42, (190, 190, 190))
    botones.clear()
    y1 = ALTO_PANEL + ALTO_ESTADO
    for i, (etiqueta, accion) in enumerate(CONTROLES):
        x1, x2 = i * ancho // len(CONTROLES), (i + 1) * ancho // len(CONTROLES) - 3
        botones.append((x1, y1, x2, y1 + ALTO_BOTONES, accion))
        cv2.rectangle(img, (x1, y1), (x2, y1 + ALTO_BOTONES), (65, 65, 65), -1)
        cv2.putText(img, etiqueta, (x1 + 12, y1 + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return img


def _accion_de_tecla(k, activo):
    if k == 32:
        return "pausar" if activo else "activar"
    return {ord('g'): "garra", ord('G'): "garra", ord('h'): "home", ord('H'): "home",
            ord('+'): "factor_mas", ord('='): "factor_mas", ord('-'): "factor_menos", ord('_'): "factor_menos"}.get(k)


def crear_detector_brazo(cal, brazo, articular, fuente_lateral=None):
    """DetectorBrazo, con la segunda webcam (a un costado) si se indicó."""
    lateral = None
    if fuente_lateral is not None:
        from ..seguimiento.lateral import CamaraLateral
        print(f"Webcam lateral: {fuente_lateral}")
        lateral = CamaraLateral(fuente_lateral)
    try:
        return DetectorBrazo(cal, brazo, articular=articular, lateral=lateral)
    except Exception:
        if lateral is not None:
            lateral.cerrar()
        raise


def _poner_lateral(img, detector):
    """Miniatura de la webcam lateral abajo a la derecha, con su estado: para ver si te encuadra."""
    lateral = getattr(detector, "lateral", None)
    if lateral is None:
        return img
    mini = lateral.miniatura(180)
    h, w = img.shape[:2]
    if mini is not None:
        mh, mw = mini.shape[:2]
        y0, x0 = h - mh - 8, w - mw - 8
        img[y0:y0 + mh, x0:x0 + mw] = mini
        cv2.rectangle(img, (x0 - 1, y0 - 1), (x0 + mw, y0 + mh), (200, 200, 200), 1)
    texto_con_borde(img, detector.estado_lateral or "lateral: iniciando", (w - 330, h - (mini.shape[0] if mini is not None else 0) - 16),
                    0.45, VERDE if detector.fusionado else AMBAR)
    return img


def ejecutar_app(anfitrion, puerto, token, fuente_webcam, modo="mano", brazo="derecho", fuente_lateral=None):
    print(f"Cargando el detector (modo {modo})...")
    cal = cargar_calibracion_mano()
    if modo in (MODO_BRAZO, MODO_ARTICULAR):
        detector = crear_detector_brazo(cal, brazo, modo == MODO_ARTICULAR, fuente_lateral)
    else:
        if fuente_lateral is not None:
            print("La webcam lateral sólo se usa en los modos articular y brazo: se ignora.")
        detector = DetectorMano(cal)
    try:
        webcam = Camara(fuente_webcam, "webcam", WEBCAM_RESOLUCION, mjpg=True)
    except RuntimeError:
        detector.cerrar()
        raise
    conexion = ConexionRobot(anfitrion, puerto, token)
    deteccion = _Deteccion(webcam, detector, conexion)
    pendientes, botones = [], []

    def clic(evento, x, y, flags, param):
        if evento == cv2.EVENT_LBUTTONDOWN:
            for x1, y1, x2, y2, accion in botones:
                if x1 <= x < x2 and y1 <= y < y2:
                    pendientes.append(accion)

    print(f"Conectando con {anfitrion}:{puerto}. ESPACIO activa/pausa | G garra | H home | C calibrar tu brazo "
          "(relajado, colgando) | O/P apertura | +/- ganancia | ESC salir.")
    try:
        cv2.namedWindow(VENTANA, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(VENTANA, clic)
        webcam.iniciar_hilo()
        conexion.iniciar()
        deteccion.start()
        visto, aviso_cal, t_aviso_cal = [False], "", 0.0
        calibrando, ultimo_n = None, -1
        while True:
            if conexion.error_fatal:
                raise RuntimeError(conexion.error_fatal)
            if not webcam.viva:
                raise RuntimeError("Se perdió la señal de la webcam.")
            if deteccion.error is not None:
                raise RuntimeError(f"Falló la detección: {deteccion.error}")
            n, f, mano = deteccion.ultimo_numerado()
            activo = conexion.estado.get("estado") in ACTIVOS
            ahora = time.monotonic()
            if calibrando is not None:
                if n != ultimo_n and mano is not None and mano.medidas.get("crudo"):
                    calibrando["muestras"].append(mano.medidas["crudo"])
                if ahora >= calibrando["fin"]:
                    try:
                        cal["gemelo"] = calibracion_gemelo(calibrando["muestras"])
                        guardar_calibracion_mano(cal)
                        aviso_cal = f"Calibrado con {len(calibrando['muestras'])} lecturas: el robot vuelve al inicio"
                    except ValueError:
                        aviso_cal = "No se pudo calibrar: no te vi el brazo."
                    calibrando, t_aviso_cal = None, ahora
                    conexion.enviar({"tipo": "home"})       # al estado inicial, como tu brazo colgando
            ultimo_n = n
            if calibrando is not None:
                aviso = f"Brazo relajado colgando, recto y quieto... {calibrando['fin'] - ahora:.1f}"
            else:
                aviso = aviso_cal if ahora - t_aviso_cal < 3.0 else ""
            cv2.imshow(VENTANA, _componer(_panel_brazo(conexion.estado), _poner_lateral(_panel_webcam(f, mano), detector), conexion,
                                          deteccion.hz, aviso, botones))
            k = leer_tecla(15)
            if k == 27 or ventana_cerrada(VENTANA, visto):
                break
            if k in (ord('c'), ord('C')):
                if modo != MODO_ARTICULAR:
                    aviso_cal, t_aviso_cal = "La calibracion del brazo es del modo articular.", time.monotonic()
                else:
                    conexion.enviar({"tipo": "pausar"})
                    calibrando = {"fin": time.monotonic() + GEMELO_CALIBRACION_SEGUNDOS, "muestras": []}
                continue
            if k in (ord('o'), ord('O'), ord('p'), ord('P')):
                clave = "abierta" if k in (ord('o'), ord('O')) else "cerrada"
                if mano is not None and mano.ratio is not None:
                    cal[clave] = mano.ratio
                    guardar_calibracion_mano(cal)
                    aviso_cal, t_aviso_cal = f"Guardada mano {clave} = {mano.ratio:.2f}", time.monotonic()
                else:
                    aviso_cal, t_aviso_cal = "No veo tu mano para calibrar.", time.monotonic()
                continue
            accion = _accion_de_tecla(k, activo) if k != -1 else None
            if accion is None and pendientes:
                accion = pendientes.pop(0)
            if accion == "salir":
                break
            if accion is not None:
                conexion.enviar({"tipo": accion})
    finally:
        conexion.enviar({"tipo": "pausar"})
        deteccion.parar()
        conexion.cerrar()
        webcam.cerrar()
        detector.cerrar()
        cv2.destroyWindow(VENTANA)
        print("App cerrada: el brazo queda en pausa donde estaba.")
