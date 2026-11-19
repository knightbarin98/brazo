"""Lectura de la persona con MediaPipe. Corre donde esté la webcam del operador."""

import hashlib
import json
import os
import threading
import urllib.request

import cv2
import numpy as np

from ..config import (ARTICULACIONES, CADERAS, HOMBRO_OTRO, MANO_MAX_EDAD, MANO_POSE, MANO_RATIO_ABIERTA, MANO_RATIO_CERRADA, MODELO_MANO, MODELO_MANO_SHA256,
                      MODELO_MANO_URL, MODELO_POSE, MODELO_POSE_SHA256, MODELO_POSE_URL, POSE_VISIBILIDAD_MIN,
                      SEGUIMIENTO_ARCHIVO)
from .vistas import METODO_VISTAS, Lado, Orientaciones, Vista, brazo_desde_vistas, proporciones
from .mapeo import (MEDIO_BASE, MODO_ARTICULAR, MUNECA, a_robot, angulos_articulares, angulos_hombro, aplicar_calibracion,
                    apertura_de_ratio, cabeceo_mano, en_torso, marco_torso, medidas_brazo3d, medir_brazo, medir_mano,
                    ratio_apertura)

BORDE = 0.015                       # fracción de la imagen: una mano que llega aquí está cortada


def _dentro(x, y):
    return BORDE < x < 1.0 - BORDE and BORDE < y < 1.0 - BORDE

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


class _ManoEnHilo(threading.Thread):
    """El modelo de la mano corre aparte, en paralelo con Pose.

    Juntos en serie tardan ~260 ms por cuadro en una Raspberry 4 (~3 lecturas/s): el brazo no
    se podía seguir en tiempo real. Así Pose no espera a la mano; cada lectura usa la mano más
    reciente, si no es más vieja que MANO_MAX_EDAD. Sólo se procesa el último recorte pedido.
    """

    def __init__(self, mp, detector):
        super().__init__(daemon=True)
        self._mp, self._detector = mp, detector
        self._cond = threading.Condition()
        self._pedido = self._resultado = None
        self._activo = True
        self.start()

    def pedir(self, recorte, x0, y0, lado, ancho, alto, t):
        with self._cond:
            self._pedido = (recorte, x0, y0, lado, ancho, alto, t)
            self._cond.notify()

    def ultimo(self, t):
        with self._cond:
            r = self._resultado
        return r if r is not None and t - r["t"] <= MANO_MAX_EDAD else None

    def run(self):
        while True:
            with self._cond:
                while self._activo and self._pedido is None:
                    self._cond.wait()
                if not self._activo:
                    return
                (recorte, x0, y0, lado, ancho, alto, t), self._pedido = self._pedido, None
            r = self._detector.detect(_imagen(self._mp, recorte))
            resultado = None
            if r.hand_landmarks:
                mundo = [(p.x, p.y, p.z) for p in r.hand_world_landmarks[0]]
                resultado = {
                    "t": t, "ratio": ratio_apertura(mundo), "cabeceo": cabeceo_mano(mundo),
                    "puntos": [((x0 + p.x * lado) / ancho, (y0 + p.y * lado) / alto) for p in r.hand_landmarks[0]],
                    "direccion": a_robot([mundo[MEDIO_BASE][k] - mundo[MUNECA][k] for k in range(3)]),
                    "direccion_mundo": tuple(mundo[MEDIO_BASE][k] - mundo[MUNECA][k] for k in range(3)),
                }
            with self._cond:
                if resultado is not None or (self._resultado is not None and t - self._resultado["t"] > MANO_MAX_EDAD):
                    self._resultado = resultado

    def cerrar(self):
        with self._cond:
            self._activo = False
            self._cond.notify()
        self.join(timeout=2.0)
        self._detector.close()


class DetectorBrazo:
    """Modo 'brazo': mide en metros con Pose, sin depender de la distancia a la cámara.

    Hombro, codo y muñeca deben verse en cuadro. La mano se busca en un recorte junto a la
    muñeca: tiene más detalle y es la mano de ese brazo, no la de otra persona. Sólo se lee
    una persona y un brazo; el otro se ignora.

    articular=True: además, tu brazo en 3D respecto al torso (brazo3d), con las líneas del brazo que
    se ven en la imagen (vistas.py) y, si la hay, la cámara lateral.
    lateral: CamaraLateral (segunda webcam, a un costado).
    """

    def __init__(self, calibracion, brazo="derecho", articular=False, lateral=None):
        self.cal = calibracion
        self.lateral = lateral
        self.estado_lateral, self.fusionado = "", False
        self.mano_cortada = False
        self.indices = ARTICULACIONES[brazo]
        self.indices_mano = MANO_POSE[brazo]
        self.hombro_otro = HOMBRO_OTRO[brazo]
        cadera = CADERAS[0] if brazo == "derecho" else CADERAS[1]
        self.lado = Lado(*self.indices, self.hombro_otro, cadera, CADERAS[1] if cadera == CADERAS[0] else CADERAS[0],
                         self.indices_mano)
        self.orientaciones = Orientaciones()
        if lateral is not None:
            lateral.brazo = self.indices
        self.articular = articular
        self.afuera = -1 if brazo == "derecho" else +1   # imagen en espejo: el brazo derecho sale hacia Y-
        self._mp, BaseOptions, vision = _mediapipe()
        ruta = asegurar_modelo(MODELO_POSE, MODELO_POSE_URL, MODELO_POSE_SHA256)
        self._pose = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=ruta), running_mode=vision.RunningMode.VIDEO, num_poses=1,
            min_pose_detection_confidence=0.5, min_pose_presence_confidence=0.5, min_tracking_confidence=0.5))
        # Modo imagen: el recorte se mueve, así que no hay video continuo que seguir.
        self._manos = _ManoEnHilo(self._mp, _crear_manos(BaseOptions, vision, vision.RunningMode.IMAGE, 0.4))
        self._ms = -1
        if lateral is not None and not lateral.is_alive():
            lateral.start()

    def _vista_lateral(self, t):
        """La lectura de la cámara lateral como Vista, o None (y por qué, en estado_lateral)."""
        if self.lateral is None:
            return None
        if self.lateral.error is not None or not self.lateral.camara.viva:
            self.estado_lateral = "lateral: sin senal"
            return None
        lectura = self.lateral.ultimo(t)
        if lectura is None:
            self.estado_lateral = "lateral: no te ve"
            return None
        return Vista(lectura[1], lectura[2], lectura[3])

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
        self.mano_cortada = False
        self._fuente_mano = ""
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
        direccion = direccion_mundo = None
        if min(ancho, alto) >= 128:
            direccion, direccion_mundo = self._leer_mano(frame, lectura, ancho, alto, norm, t)
            self._fuente_mano = "mano" if direccion is not None else ("cortada" if self.mano_cortada else "")
        if self.articular:
            if (direccion is None and not self.mano_cortada
                    and min(mundo[i].visibility for i in self.indices_mano) >= POSE_VISIBILIDAD_MIN
                    and all(_dentro(img[i].x, img[i].y) for i in self.indices_mano)):
                # Sin la mano detallada: de la muñeca al punto medio entre meñique e índice de Pose
                crudos = [(mundo[i].x, mundo[i].y, mundo[i].z) for i in self.indices_mano]
                m = (mundo[self.indices[2]].x, mundo[self.indices[2]].y, mundo[self.indices[2]].z)
                direccion_mundo = tuple((a + b) / 2 - c for a, b, c in zip(*crudos, m))
                direccion = a_robot(direccion_mundo)
                self._fuente_mano = "pose"
            lectura.modo = MODO_ARTICULAR
            lectura.angulos = angulos_articulares(hombro, codo, muneca, direccion, self.afuera)
            mano_px = None
            if self._fuente_mano == "mano" and lectura.puntos:
                mano_px = tuple((lectura.puntos[k][0] * ancho, lectura.puntos[k][1] * alto) for k in (0, 9))
            frente = Vista({i: (p.x, p.y, p.z, p.visibility) for i, p in enumerate(mundo)},
                           {i: (p.x, p.y, p.visibility) for i, p in enumerate(img)}, (ancho, alto), mano_px)
            self._medir_3d(lectura, mundo, direccion_mundo, frente, t)
        return lectura

    def _en_torso(self, mundo, direccion_mano):
        """Respaldo sin las líneas de la imagen (sin caderas en cuadro): con la estimación 3D de Pose."""
        if mundo[self.hombro_otro].visibility < POSE_VISIBILIDAD_MIN:
            return None
        punto = lambda i: (mundo[i].x, mundo[i].y, mundo[i].z)   # noqa: E731
        caderas = None
        if min(mundo[i].visibility for i in CADERAS) >= POSE_VISIBILIDAD_MIN:
            caderas = tuple((a + b) / 2 for a, b in zip(*(punto(i) for i in CADERAS)))
        hombro, codo, muneca = (punto(i) for i in self.indices)
        try:
            marco = marco_torso(hombro, punto(self.hombro_otro), caderas)
        except ValueError:
            return None
        return (en_torso(tuple(c - h for c, h in zip(codo, hombro)), marco),
                en_torso(tuple(m - c for m, c in zip(muneca, codo)), marco),
                None if direccion_mano is None else en_torso(direccion_mano, marco))

    def _medir_3d(self, lectura, mundo, direccion_mano, frente, t):
        """Brazo, antebrazo y mano en los ejes del torso, ya corregidos con la calibración (brazo colgando).

        Con las líneas de la imagen de una o dos cámaras (vistas.py); sin caderas en cuadro, con la
        estimación 3D de Pose. medidas lleva también lo necesario para calibrar ('calibrar') y, con la
        lateral, tu hombro medido sólo con la de frente (para comparar en el registro del simulador).
        """
        cal = self.cal.get("gemelo") or {}
        de_vistas = cal.get("metodo") == METODO_VISTAS
        largos = cal.get("largos") if de_vistas else None
        lateral = self._vista_lateral(t)
        r = brazo_desde_vistas(frente, lateral, self.lado, largos, self.orientaciones, t)
        extra, metodo = {}, METODO_VISTAS
        if r is not None:
            crudo, info = r
            self.fusionado = info["lateral"]
            if lateral is not None:
                self.estado_lateral = ("lateral: combinando las 2 camaras" if info["lateral"] else
                                       "lateral: no encaja con la de frente" if info["residuo"] is not None else
                                       "lateral: no ve tu brazo")
            # calibrar: el largo de cada segmento (colgando, todo en el plano de la imagen) y la lectura
            # hecha con esos largos, para medir lo que aparenta el codo estando recto
            largos_ahora = proporciones(frente, self.lado)
            if largos_ahora:
                con_largos = brazo_desde_vistas(frente, lateral, self.lado, largos_ahora)
                if con_largos is not None:
                    extra["calibrar"] = {"largos": largos_ahora, "crudo": con_largos[0]}
            if info["lateral"]:
                solo = brazo_desde_vistas(frente, None, self.lado, largos)
                if solo is not None:
                    f, a = angulos_hombro(aplicar_calibracion(*solo[0][:2], None, cal if de_vistas else None)[0])
                    extra.update(flexion_hombro_frente=f, abduccion_hombro_frente=a)
            usar_cal = cal if de_vistas else None
        else:
            self.fusionado = False
            crudo = self._en_torso(mundo, direccion_mano)
            if crudo is None:
                return
            metodo, usar_cal = "pose3d", (cal if cal and not de_vistas else None)
        lectura.brazo3d = aplicar_calibracion(*crudo, usar_cal)
        flexion_h, abduccion_h = angulos_hombro(lectura.brazo3d[0])
        lectura.medidas = dict(medidas_brazo3d(*lectura.brazo3d[:2]), crudo=crudo, caderas=r is not None,
                               lateral=self.fusionado, flexion_hombro=flexion_h, abduccion_hombro=abduccion_h,
                               mano_fuente=self._fuente_mano, metodo=metodo, calibrado=de_vistas, **extra)

    def _leer_mano(self, frame, lectura, ancho, alto, norm, t):
        """Pide la mano de este cuadro y aplica la más reciente.

        Devuelve la dirección muñeca -> nudillos en (adelante, izquierda, arriba) y en ejes de MediaPipe, o (None, None).
        """
        px_codo = (norm[1][0] * ancho, norm[1][1] * alto)
        px_muneca = (norm[2][0] * ancho, norm[2][1] * alto)
        x0, y0, lado = self._recorte(ancho, alto, px_codo, px_muneca)
        self._manos.pedir(frame[y0:y0 + lado, x0:x0 + lado].copy(), x0, y0, lado, ancho, alto, t)
        mano = self._manos.ultimo(t)
        self.mano_cortada = mano is not None and not all(_dentro(x, y) for x, y in mano["puntos"])
        if mano is None or self.mano_cortada:
            return None, None              # una mano cortada por el borde da una dirección sin sentido
        lectura.ratio, lectura.cabeceo, lectura.puntos = mano["ratio"], mano["cabeceo"], mano["puntos"]
        lectura.apertura = apertura_de_ratio(lectura.ratio, self.cal["abierta"], self.cal["cerrada"])
        return mano["direccion"], mano["direccion_mundo"]

    def cerrar(self):
        self._manos.cerrar()
        self._pose.close()
        if self.lateral is not None:
            self.lateral.cerrar()
