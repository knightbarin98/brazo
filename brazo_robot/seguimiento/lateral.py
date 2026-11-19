"""Segunda webcam, a un costado de la persona: ve de lado lo que la de frente ve de canto.

Lee la cámara y detecta la pose en su propio hilo; entrega la lectura más reciente (puntos de la
imagen y estimación 3D). Cómo se combina con la de frente está en vistas.py: con las líneas del brazo
que se ven en cada imagen, no con la profundidad que adivina Pose.
"""

import threading
import time

import cv2

from ..config import LATERAL_MAX_EDAD, MODELO_POSE, MODELO_POSE_SHA256, MODELO_POSE_URL, WEBCAM_RESOLUCION

AUTO, NO = "auto", "no"      # --webcam-lateral: buscar una segunda webcam, o usar sólo una


def frontalidad(frame):
    """Qué tan de frente te ve esa imagen: ancho de hombros / alto del torso (mayor = de frente), o None.

    De frente los hombros se ven separados; de perfil, casi uno encima del otro.
    """
    from .mano import _imagen, _mediapipe, asegurar_modelo
    mp, BaseOptions, vision = _mediapipe()
    ruta = asegurar_modelo(MODELO_POSE, MODELO_POSE_URL, MODELO_POSE_SHA256)
    with vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=ruta), running_mode=vision.RunningMode.IMAGE, num_poses=1)) as pose:
        r = pose.detect(_imagen(mp, frame))
    if not r.pose_landmarks:
        return None
    p, (alto, ancho) = r.pose_landmarks[0], frame.shape[:2]
    if min(p[i].visibility for i in (11, 12)) < 0.5:
        return None
    hombros = abs(p[11].x - p[12].x) * ancho
    torso = abs((p[11].y + p[12].y) / 2 - (p[23].y + p[24].y) / 2) * alto
    return hombros / max(torso, 1.0)


def fuente_lateral(texto):
    """--webcam-lateral: 'auto', 'no', un índice o una ruta."""
    if texto is None:
        return AUTO
    if isinstance(texto, str):
        texto = texto.strip()
        if texto.lower() in (AUTO, NO):
            return texto.lower()
        return int(texto) if texto.isdigit() else texto
    return texto


def elegir_webcams(webcam=None, lateral=AUTO, buscar=None, frente_de=None):
    """(webcam de frente, webcam lateral o None).

    webcam: la de frente, o None para buscarla. lateral: AUTO (buscar una segunda), NO (sólo una) o
    una fuente. Con las dos por buscar, la de frente es la que te ve más de frente (frontalidad); si no
    te ve ninguna al arrancar, el orden de los índices. buscar y frente_de: para las pruebas.
    """
    if buscar is None:
        from ..vision.camara import detectar_webcams as buscar
    frente_de = frente_de or _mas_de_frente
    if lateral not in (AUTO, NO) and webcam is not None:
        return webcam, lateral
    if lateral == NO:
        return (webcam if webcam is not None else _primera(buscar)), None
    reales = buscar()
    if webcam is not None:
        otras = [i for i in reales if i != webcam]
        return webcam, (otras[0] if otras else None)
    if lateral != AUTO:
        otras = [i for i in reales if i != lateral]
        if not otras:
            raise RuntimeError("No se detectó otra webcam para usar de frente.")
        return otras[0], lateral
    if not reales:
        from ..vision.camara import detectar_webcam
        return detectar_webcam(), None
    if len(reales) == 1:
        return reales[0], None
    return frente_de(reales[0], reales[1])


def _primera(buscar):
    reales = buscar()
    if reales:
        return reales[0]
    from ..vision.camara import detectar_webcam
    return detectar_webcam()


def _mas_de_frente(a, b):
    """(frente, lateral) entre las webcams a y b: la que te ve con los hombros más separados."""
    from ..vision.camara import Camara
    valores = {}
    for indice in (a, b):
        try:
            cam = Camara(indice, "webcam", WEBCAM_RESOLUCION, mjpg=True)
            try:
                valores[indice] = frontalidad(cv2.flip(cam.leer_fresco(), 1))
            finally:
                cam.cerrar()
        except RuntimeError:
            valores[indice] = None
    if valores[a] is None or valores[b] is None:
        print(f"No te veo en las dos webcams al arrancar: uso la {a} de frente y la {b} de lado. "
              f"Si están al revés: --webcam {b} --webcam-lateral {a}")
        return a, b
    return (a, b) if valores[a] >= valores[b] else (b, a)


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
