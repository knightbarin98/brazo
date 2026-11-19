"""Segunda webcam, a un costado de la persona: ve de lado lo que la de frente ve de canto.

Lee la cámara y detecta la pose en su propio hilo; entrega la lectura más reciente (puntos de la
imagen y estimación 3D). Cómo se combina con la de frente está en vistas.py: con las líneas del brazo
que se ven en cada imagen, no con la profundidad que adivina Pose.
"""

import threading
import time

import cv2

from ..config import LATERAL_MAX_EDAD, MODELO_POSE, MODELO_POSE_SHA256, MODELO_POSE_URL, WEBCAM_RESOLUCION


class CamaraLateral(threading.Thread):
    """Lee la webcam lateral y detecta la pose en su propio hilo. Da la lectura más reciente."""

    def __init__(self, fuente, resolucion=WEBCAM_RESOLUCION):
        super().__init__(daemon=True)
        from ..vision.camara import Camara
        from .mano import _imagen, _mediapipe, asegurar_modelo
        self.camara = Camara(fuente, "webcam lateral", resolucion, mjpg=True)
        try:
            self._mp, BaseOptions, vision = _mediapipe()
            ruta = asegurar_modelo(MODELO_POSE, MODELO_POSE_URL, MODELO_POSE_SHA256)
            self._pose = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=ruta), running_mode=vision.RunningMode.VIDEO,
                num_poses=1, min_pose_detection_confidence=0.5, min_pose_presence_confidence=0.5,
                min_tracking_confidence=0.5))
        except Exception:
            self.camara.cerrar()
            raise
        self._imagen = _imagen
        self.cerrojo = threading.Lock()
        self.activo = True
        self.error = None
        # (t, {i: (x, y, z, vis)} 3D, {i: (x, y, vis)} imagen normalizada, (ancho, alto))
        self.lectura = None
        self.frame, self.hz = None, 0.0
        self.brazo = (11, 13, 15)           # lo que dibuja la miniatura (lo fija DetectorBrazo)
        self._ms = -1

    def run(self):
        self.camara.iniciar_hilo()
        visto, cuenta, t_hz = -1, 0, time.monotonic()
        try:
            while self.activo:
                n, f = self.camara.ultimo()
                if f is None or n == visto:
                    time.sleep(0.003)
                    continue
                visto, t = n, time.monotonic()
                f = cv2.flip(f, 1)          # igual que la de frente: las dos ven el mismo brazo como 'derecho'
                self._ms = max(int(t * 1000), self._ms + 1)
                r = self._pose.detect_for_video(self._imagen(self._mp, f), self._ms)
                lectura = None
                if r.pose_world_landmarks:
                    mundo, img = r.pose_world_landmarks[0], r.pose_landmarks[0]
                    lectura = (t, {i: (p.x, p.y, p.z, p.visibility) for i, p in enumerate(mundo)},
                               {i: (p.x, p.y, p.visibility) for i, p in enumerate(img)},
                               (f.shape[1], f.shape[0]))
                cuenta += 1
                if t - t_hz >= 1.0:
                    self.hz, cuenta, t_hz = cuenta / (t - t_hz), 0, t
                with self.cerrojo:
                    self.frame, self.lectura = f, lectura
        except Exception as e:
            self.error = e

    def ultimo(self, t, max_edad=LATERAL_MAX_EDAD):
        """La lectura más reciente si no es más vieja que max_edad respecto a t; si no, None."""
        with self.cerrojo:
            lectura = self.lectura
        if lectura is None or abs(t - lectura[0]) > max_edad:
            return None
        return lectura

    def miniatura(self, ancho=200):
        """La imagen lateral pequeña con el brazo que se sigue dibujado, como en la de frente."""
        with self.cerrojo:
            f, lectura = self.frame, self.lectura
        if f is None:
            return None
        alto = int(f.shape[0] * ancho / f.shape[1])
        img = cv2.resize(f, (ancho, alto))
        if lectura is not None:
            pts = [(int(lectura[2][i][0] * ancho), int(lectura[2][i][1] * alto)) for i in self.brazo]
            cv2.line(img, pts[0], pts[1], (0, 220, 0), 3)
            cv2.line(img, pts[1], pts[2], (255, 160, 0), 3)
            for p in pts:
                cv2.circle(img, p, 4, (255, 255, 255), -1)
        return img

    def cerrar(self):
        self.activo = False
        if self.is_alive():
            self.join(timeout=2.0)
        self.camara.cerrar()
        self._pose.close()
