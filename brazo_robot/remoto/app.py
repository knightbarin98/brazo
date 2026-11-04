"""App del operador: su webcam guía el brazo y ve la cámara del robot."""

import time
import unicodedata

import cv2
import numpy as np

from ..config import REMOTO_VIDEO_MAX_EDAD, WEBCAM_RESOLUCION
from ..seguimiento.control import SIGUIENDO
from ..seguimiento.mano import CONEXIONES, DetectorMano
from ..ui import leer_tecla, texto_con_borde, ventana_cerrada
from ..vision.camara import Camara
from .cliente import ConexionRobot
from .protocolo import mano_a_mensaje

VENTANA = "Brazo remoto"
VERDE, AMBAR, ROJO = (0, 220, 0), (0, 200, 255), (60, 60, 255)
ALTO_PANEL, ALTO_ESTADO, ALTO_BOTONES = 480, 56, 44
CONTROLES = [("ACTIVAR", "activar"), ("PAUSAR", "pausar"), ("IMAN", "iman"), ("HOME", "home"), ("SALIR", "salir")]


def _sin_acentos(texto):
    """putText de OpenCV sólo dibuja ASCII: una letra acentuada saldría como '?'."""
    return unicodedata.normalize("NFD", texto).encode("ascii", "ignore").decode()


def _ajustar(img, alto):
    return cv2.resize(img, (int(img.shape[1] * alto / img.shape[0]), alto))


def _panel_robot(frame, edad):
    if frame is None:
        img = np.full((ALTO_PANEL, 640, 3), 40, np.uint8)
        texto_con_borde(img, "Esperando video del robot...", (150, 240), 0.7, AMBAR)
        return img
    img = _ajustar(frame, ALTO_PANEL)
    texto_con_borde(img, "Camara del robot", (8, 26), 0.6)
    if edad > REMOTO_VIDEO_MAX_EDAD:
        texto_con_borde(img, f"SIN VIDEO ({edad:.0f} s)", (8, 60), 1.0, ROJO)
    return img


def _panel_webcam(frame, mano):
    img = _ajustar(frame, ALTO_PANEL)
    h, w = img.shape[:2]
    if mano is not None:
        pts = [(int(x * w), int(y * h)) for x, y in mano.puntos]
        for a, b in CONEXIONES:
            cv2.line(img, pts[a], pts[b], VERDE, 2)
        for p in pts:
            cv2.circle(img, p, 3, (255, 255, 255), -1)
    texto_con_borde(img, "Tu webcam" + ("" if mano is not None else " - no veo tu mano"), (8, 26), 0.6,
                    (255, 255, 255) if mano is not None else AMBAR)
    return img


def _componer(robot, webcam, conexion, aviso_local, botones):
    arriba = np.hstack([robot, webcam])
    ancho = arriba.shape[1]
    img = np.zeros((ALTO_PANEL + ALTO_ESTADO + ALTO_BOTONES, ancho, 3), np.uint8)
    img[:ALTO_PANEL] = arriba
    e = conexion.estado
    if conexion.conectado and e:
        color = VERDE if e.get("estado") == SIGUIENDO else AMBAR
        pose = "(%.0f, %.0f, %.0f) mm" % tuple(e["pose"]) if e.get("pose") else "sin posicion de seguimiento"
        linea1 = f"{e.get('estado')} | {pose} | iman {'ON' if e.get('iman') else 'off'}"
        linea2 = aviso_local or e.get("aviso", "")
    else:
        color, linea1, linea2 = ROJO, "DESCONECTADO", conexion.motivo
    texto_con_borde(img, linea1, (8, ALTO_PANEL + 22), 0.6, color)
    texto_con_borde(img, _sin_acentos(linea2)[:110], (8, ALTO_PANEL + 46), 0.45)
    botones.clear()
    y1 = ALTO_PANEL + ALTO_ESTADO
    for i, (etiqueta, accion) in enumerate(CONTROLES):
        x1, x2 = i * ancho // len(CONTROLES), (i + 1) * ancho // len(CONTROLES) - 3
        botones.append((x1, y1, x2, y1 + ALTO_BOTONES, accion))
        cv2.rectangle(img, (x1, y1), (x2, y1 + ALTO_BOTONES), (65, 65, 65), -1)
        cv2.putText(img, etiqueta, (x1 + 12, y1 + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return img


def ejecutar_app(anfitrion, puerto, token, fuente_webcam):
    print("Cargando el detector de manos...")
    detector = DetectorMano()
    try:
        webcam = Camara(fuente_webcam, "webcam", WEBCAM_RESOLUCION)
    except RuntimeError:
        detector.cerrar()
        raise
    conexion = ConexionRobot(anfitrion, puerto, token)
    pendientes, botones = [], []

    def clic(evento, x, y, flags, param):
        if evento == cv2.EVENT_LBUTTONDOWN:
            for x1, y1, x2, y2, accion in botones:
                if x1 <= x < x2 and y1 <= y < y2:
                    pendientes.append(accion)

    print(f"Conectando con {anfitrion}:{puerto}. ESPACIO activa/pausa | I imán | H home | ESC salir.")
    try:
        cv2.namedWindow(VENTANA, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(VENTANA, clic)
        webcam.iniciar_hilo()
        conexion.iniciar()
        visto, visto_n, mano, ultimo_envio = [False], -1, None, 0.0
        panel_webcam = np.zeros((ALTO_PANEL, 640, 3), np.uint8)
        while True:
            if conexion.error_fatal:
                raise RuntimeError(conexion.error_fatal)
            if not webcam.viva:
                raise RuntimeError("Se perdió la señal de la webcam.")
            ahora = time.monotonic()
            n, f = webcam.ultimo()
            if f is not None and n != visto_n:
                visto_n = n
                f = cv2.flip(f, 1)            # como un espejo: tu izquierda queda a la izquierda
                mano = detector.detectar(f, ahora)
                conexion.enviar(mano_a_mensaje(mano))
                ultimo_envio = ahora
                panel_webcam = _panel_webcam(f, mano)
            elif ahora - ultimo_envio > 1.0:  # webcam parada: la conexión sigue viva y el brazo se detiene solo
                conexion.enviar({"tipo": "latido"})
                ultimo_envio = ahora

            video, edad = conexion.ultimo_video()
            video_ok = video is not None and edad <= REMOTO_VIDEO_MAX_EDAD
            siguiendo = conexion.estado.get("estado") == SIGUIENDO
            if siguiendo and not video_ok:    # no se maneja a ciegas
                conexion.enviar({"tipo": "pausar"})
            aviso_local = "" if video_ok else "Sin video del robot: no se puede activar."
            cv2.imshow(VENTANA, _componer(_panel_robot(video, edad), panel_webcam, conexion, aviso_local, botones))

            k = leer_tecla(5)
            if k == 27 or ventana_cerrada(VENTANA, visto):
                break
            accion = None
            if k == 32:
                accion = "pausar" if siguiendo else "activar"
            elif k in (ord('i'), ord('I')):
                accion = "iman"
            elif k in (ord('h'), ord('H')):
                accion = "home"
            elif pendientes:
                accion = pendientes.pop(0)
            if accion == "salir":
                break
            if accion is not None and (accion != "activar" or video_ok):
                conexion.enviar({"tipo": accion})
    finally:
        conexion.enviar({"tipo": "pausar"})
        conexion.cerrar()
        webcam.cerrar()
        detector.cerrar()
        cv2.destroyWindow(VENTANA)
        print("App cerrada: el brazo queda en pausa donde estaba.")
