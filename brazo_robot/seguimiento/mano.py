"""Lectura de la persona con MediaPipe. Corre donde esté la webcam del operador."""

import hashlib
import json
import os
import urllib.request

import cv2
import numpy as np

from ..config import (ARTICULACIONES, MANO_POSE, MANO_RATIO_ABIERTA, MANO_RATIO_CERRADA, MODELO_MANO, MODELO_MANO_SHA256,
                      MODELO_MANO_URL, MODELO_POSE, MODELO_POSE_SHA256, MODELO_POSE_URL, POSE_VISIBILIDAD_MIN,
                      SEGUIMIENTO_ARCHIVO)
from .mapeo import (MEDIO_BASE, MODO_ARTICULAR, MUNECA, a_robot, angulos_articulares, apertura_de_ratio,
                    cabeceo_mano, medir_brazo, medir_mano, ratio_apertura)

# Pares de puntos que forman el esqueleto de la mano
CONEXIONES = ((0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
              (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17))


def asegurar_modelo(ruta, url, sha256):
    """Descarga el modelo la primera vez y comprueba que llegó íntegro."""
    if os.path.exists(ruta):
        return ruta
    print(f"Descargando el modelo en {ruta} ...")
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    temporal = ruta + ".tmp"
    try:
        urllib.request.urlretrieve(url, temporal)
        with open(temporal, "rb") as f:
            if hashlib.sha256(f.read()).hexdigest() != sha256:
                raise RuntimeError(f"El modelo descargado de {url} está dañado; no se usará.")
        os.replace(temporal, ruta)
    finally:
        if os.path.exists(temporal):
            os.remove(temporal)
    return ruta


def cargar_calibracion_mano():
    cal = {"abierta": MANO_RATIO_ABIERTA, "cerrada": MANO_RATIO_CERRADA}
    if os.path.exists(SEGUIMIENTO_ARCHIVO):
        with open(SEGUIMIENTO_ARCHIVO) as f:
            cal.update(json.load(f))
    return cal


def guardar_calibracion_mano(cal):
    with open(SEGUIMIENTO_ARCHIVO, "w") as f:
        json.dump(cal, f)


def _mediapipe():
    # Se importa aquí: tarda en cargar y la consola y el servidor no lo necesitan.
    try:
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision
    except ImportError as e:
        raise RuntimeError(f"No se pudo cargar MediaPipe ({e}). Revisa el entorno de conda.")
    return mp, BaseOptions, vision


def _imagen(mp, bgr):
    return mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))


def _crear_manos(BaseOptions, vision, modo, confianza):
    opciones = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=asegurar_modelo(MODELO_MANO, MODELO_MANO_URL, MODELO_MANO_SHA256)),
        running_mode=modo, num_hands=1, min_hand_detection_confidence=confianza,
        min_hand_presence_confidence=confianza, min_tracking_confidence=confianza)
    return vision.HandLandmarker.create_from_options(opciones)


class DetectorMano:
    """Modo 'mano': sólo la mano. Ligero; basta con que la mano se vea de cerca."""

    def __init__(self, calibracion):
        self.cal = calibracion
        self._mp, BaseOptions, vision = _mediapipe()
        self._detector = _crear_manos(BaseOptions, vision, vision.RunningMode.VIDEO, 0.6)
        self._ms = -1

    def detectar(self, frame, t):
        """Lectura del fotograma BGR (ya en espejo) o None. t en segundos, siempre creciente."""
        self._ms = max(int(t * 1000), self._ms + 1)
        r = self._detector.detect_for_video(_imagen(self._mp, frame), self._ms)
        if not r.hand_landmarks:
            return None
        mundo = [(p.x, p.y, p.z) for p in r.hand_world_landmarks[0]]
        return medir_mano([(p.x, p.y) for p in r.hand_landmarks[0]], frame.shape[1] / frame.shape[0], mundo,
                          self.cal["abierta"], self.cal["cerrada"])

    def cerrar(self):
        self._detector.close()


class DetectorBrazo:
    """Modo 'brazo': mide en metros con Pose, sin depender de la distancia a la cámara.

    Hombro, codo y muñeca deben verse en cuadro. La mano se busca en un recorte junto a la
    muñeca: tiene más detalle y es la mano de ese brazo, no la de otra persona. Sólo se lee
    una persona y un brazo; el otro se ignora.

    articular=True: además de los puntos, cada articulación se traduce al motor equivalente.
    """

    def __init__(self, calibracion, brazo="derecho", articular=False):
        self.cal = calibracion
        self.indices = ARTICULACIONES[brazo]
        self.indices_mano = MANO_POSE[brazo]
        self.articular = articular
        self._giro = None
        self._mp, BaseOptions, vision = _mediapipe()
        ruta = asegurar_modelo(MODELO_POSE, MODELO_POSE_URL, MODELO_POSE_SHA256)
        self._pose = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=ruta), running_mode=vision.RunningMode.VIDEO, num_poses=1,
            min_pose_detection_confidence=0.5, min_pose_presence_confidence=0.5, min_tracking_confidence=0.5))
        # Modo imagen: el recorte se mueve, así que no hay video continuo que seguir.
        self._manos = _crear_manos(BaseOptions, vision, vision.RunningMode.IMAGE, 0.4)
        self._ms = -1

    @staticmethod
    def _recorte(ancho, alto, codo, muneca):
        """Cuadrado alrededor de la mano, estimado prolongando codo -> muñeca (en píxeles)."""
        dx, dy = muneca[0] - codo[0], muneca[1] - codo[1]
        largo = max((dx * dx + dy * dy) ** 0.5, 30.0)
        lado = int(max(128, min(1.6 * largo, 360, min(ancho, alto))))
        x0 = int(max(0, min(muneca[0] + 0.3 * dx - lado / 2, ancho - lado)))
        y0 = int(max(0, min(muneca[1] + 0.3 * dy - lado / 2, alto - lado)))
        return x0, y0, lado

    def detectar(self, frame, t):
        self._ms = max(int(t * 1000), self._ms + 1)
        r = self._pose.detect_for_video(_imagen(self._mp, frame), self._ms)
        if not r.pose_world_landmarks:
            return None
        mundo, img = r.pose_world_landmarks[0], r.pose_landmarks[0]
        if min(mundo[i].visibility for i in self.indices) < POSE_VISIBILIDAD_MIN:
            return None
        hombro, codo, muneca = (a_robot((mundo[i].x, mundo[i].y, mundo[i].z)) for i in self.indices)
        norm = [(img[i].x, img[i].y) for i in self.indices]
        lectura = medir_brazo(hombro, codo, muneca, norm[2])
        lectura.puntos_brazo = norm
        alto, ancho = frame.shape[:2]
        direccion = None
        if min(ancho, alto) >= 128:
            direccion = self._leer_mano(frame, lectura, ancho, alto, norm)
        if self.articular:
            if direccion is None and min(mundo[i].visibility for i in self.indices_mano) >= POSE_VISIBILIDAD_MIN:
                # Sin la mano detallada: de la muñeca al punto medio entre meñique e índice de Pose
                menique, indice = (a_robot((mundo[i].x, mundo[i].y, mundo[i].z)) for i in self.indices_mano)
                direccion = tuple((a + b) / 2 - m for a, b, m in zip(menique, indice, muneca))
            lectura.modo = MODO_ARTICULAR
            lectura.angulos = angulos_articulares(hombro, codo, muneca, direccion, self._giro)
            self._giro = lectura.angulos[0]
        return lectura

    def _leer_mano(self, frame, lectura, ancho, alto, norm):
        """Apertura e inclinación de la mano. Devuelve la dirección muñeca -> nudillos, o None."""
        px_codo = (norm[1][0] * ancho, norm[1][1] * alto)
        px_muneca = (norm[2][0] * ancho, norm[2][1] * alto)
        x0, y0, lado = self._recorte(ancho, alto, px_codo, px_muneca)
        r = self._manos.detect(_imagen(self._mp, frame[y0:y0 + lado, x0:x0 + lado]))
        if not r.hand_landmarks:
            return None
        mundo = [(p.x, p.y, p.z) for p in r.hand_world_landmarks[0]]
        lectura.ratio = ratio_apertura(mundo)
        lectura.apertura = apertura_de_ratio(lectura.ratio, self.cal["abierta"], self.cal["cerrada"])
        lectura.cabeceo = cabeceo_mano(mundo)
        lectura.puntos = [((x0 + p.x * lado) / ancho, (y0 + p.y * lado) / alto) for p in r.hand_landmarks[0]]
        return a_robot([mundo[MEDIO_BASE][k] - mundo[MUNECA][k] for k in range(3)])

    def cerrar(self):
        self._pose.close()
        self._manos.close()
