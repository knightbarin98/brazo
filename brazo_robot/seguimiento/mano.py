"""Detección de la mano con MediaPipe Hand Landmarker."""

import hashlib
import os
import urllib.request

import cv2

from ..config import MODELO_MANO, MODELO_MANO_SHA256, MODELO_MANO_URL
from .mapeo import medir_mano

# Pares de puntos que forman el esqueleto de la mano
CONEXIONES = ((0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
              (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17))


def asegurar_modelo(ruta=MODELO_MANO):
    """Descarga el modelo la primera vez (unos 8 MB) y comprueba que llegó íntegro."""
    if os.path.exists(ruta):
        return ruta
    print(f"Descargando el modelo de manos en {ruta} ...")
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    temporal = ruta + ".tmp"
    try:
        urllib.request.urlretrieve(MODELO_MANO_URL, temporal)
        with open(temporal, "rb") as f:
            if hashlib.sha256(f.read()).hexdigest() != MODELO_MANO_SHA256:
                raise RuntimeError("El modelo de manos descargado está dañado; no se usará.")
        os.replace(temporal, ruta)
    finally:
        if os.path.exists(temporal):
            os.remove(temporal)
    return ruta


class DetectorMano:
    def __init__(self):
        # Se importa aquí: tarda en cargar y sólo hace falta para 'seguir'.
        try:
            import mediapipe as mp
            from mediapipe.tasks.python import BaseOptions
            from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode
        except ImportError as e:
            raise RuntimeError(f"No se pudo cargar MediaPipe ({e}). Revisa: conda env update -f environment.yml")
        self._mp = mp
        opciones = HandLandmarkerOptions(base_options=BaseOptions(model_asset_path=asegurar_modelo()),
                                         running_mode=RunningMode.VIDEO, num_hands=1,
                                         min_hand_detection_confidence=0.6, min_tracking_confidence=0.6)
        self._detector = HandLandmarker.create_from_options(opciones)
        self._ms = -1

    def detectar(self, frame, t):
        """Devuelve la Mano del fotograma BGR o None. t en segundos, siempre creciente."""
        self._ms = max(int(t * 1000), self._ms + 1)
        imagen = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        r = self._detector.detect_for_video(imagen, self._ms)
        if not r.hand_landmarks:
            return None
        return medir_mano([(p.x, p.y) for p in r.hand_landmarks[0]], frame.shape[1] / frame.shape[0])

    def cerrar(self):
        self._detector.close()
