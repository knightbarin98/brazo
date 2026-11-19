"""Segunda webcam, a un costado de la persona: mejora la profundidad del brazo.

Pose da los puntos en 3D (metros, centrados en la cadera) en los ejes de cada cámara: la imagen
(x, y) es fiable y la profundidad (z) no. La cámara de frente ve mal justo lo que la lateral ve bien
(hacia adelante y hacia atrás), y al revés. Así que:

1. Alineación: el giro R (y el desplazamiento) entre los ejes de la lateral y los de la de frente se
   calcula solo, con los mismos puntos vistos por las dos (hombros, codos, muñecas y caderas), por
   mínimos cuadrados (Kabsch) sobre las lecturas de los últimos segundos. No hace falta medir dónde
   está la cámara, y si alguien la mueve se reajusta sola.
2. Fusión: cada punto se combina pesando cada cámara según lo que ve bien: poco peso en su dirección
   de profundidad (LATERAL_RELACION_PROFUNDIDAD veces peor) y según su visibilidad. El resultado
   queda en los ejes de la cámara de frente, y lo demás (marco del torso, gemelo) no cambia.

Vista de lado, Pose a veces cambia izquierda y derecha: de cada lectura lateral se usa la asignación
(tal cual o con los lados intercambiados) que mejor coincide con la de frente.
Un punto lateral que no encaja con el de la de frente (otra persona, un error de Pose) no se combina: queda el de frente. Si no encaja la mitad o más, se descarta la lectura lateral entera, y
tras muchas seguidas (la cámara se movió) el giro se vuelve a calcular. La discrepancia se mide en
desviaciones de cada cámara (lo que una ve en profundidad casi no cuenta): el ruido de profundidad no
la dispara y un brazo cambiado de lado, que se ve en la imagen, sí.
"""

import threading
import time

import cv2
import numpy as np

from ..config import (LATERAL_DISTANCIA_MAX, LATERAL_ERROR_IMAGEN, LATERAL_MAX_EDAD, LATERAL_MEMORIA,
                      LATERAL_MUESTRAS_MIN, LATERAL_PUNTOS, LATERAL_RELACION_PROFUNDIDAD, MODELO_POSE,
                      MODELO_POSE_SHA256, MODELO_POSE_URL, POSE_VISIBILIDAD_MIN, WEBCAM_RESOLUCION)

DESCARTES_PARA_REINICIAR = 30      # lecturas laterales descartadas seguidas: el giro calculado ya no sirve
# Pares izquierda/derecha de Pose: vista de lado, a veces los confunde todos a la vez.
PARES_LADOS = ((1, 4), (2, 5), (3, 6), (7, 8), (9, 10), (11, 12), (13, 14), (15, 16), (17, 18), (19, 20),
               (21, 22), (23, 24), (25, 26), (27, 28), (29, 30), (31, 32))


def cambiar_lados(puntos):
    """Los mismos puntos con izquierda y derecha intercambiadas."""
    salida = dict(puntos)
    for a, b in PARES_LADOS:
        if a in puntos and b in puntos:
            salida[a], salida[b] = puntos[b], puntos[a]
    return salida


def _peso(visibilidad):
    return max(float(visibilidad), 0.05)


def informacion(eje_profundidad, visibilidad, relacion=LATERAL_RELACION_PROFUNDIDAD):
    """Matriz de información (inversa de la covarianza) de un punto visto por una cámara."""
    e = np.asarray(eje_profundidad, float)
    return _peso(visibilidad) * (np.eye(3) + (1.0 / relacion ** 2 - 1.0) * np.outer(e, e))


class FusionLateral:
    """Giro entre las dos cámaras y combinación de sus puntos. Sin cámaras: se prueba con números."""

    def __init__(self):
        self.reiniciar()

    def reiniciar(self):
        self.suma_w = 0.0
        self.suma_p = np.zeros(3)          # puntos de la lateral
        self.suma_q = np.zeros(3)          # puntos de la de frente
        self.suma_pq = np.zeros((3, 3))
        self.muestras = 0
        self.t = None
        self.R, self.T, self._inversa = None, None, None
        self._iniciales = []               # lecturas hasta tener el primer giro
        self.descartes = 0
        self.residuo = None

    @property
    def lista(self):
        return self.R is not None

    def _comunes(self, frente, lateral, indices):
        return [i for i in indices if i in frente and i in lateral
                and frente[i][3] >= POSE_VISIBILIDAD_MIN and lateral[i][3] >= POSE_VISIBILIDAD_MIN]

    def alinear_lateral(self, p):
        return self.R @ np.asarray(p[:3], float) + self.T

    def distancia(self, pf, pl):
        """Discrepancia de un punto entre las dos cámaras ya alineadas, en desviaciones (Mahalanobis con el
        error de cada una: LATERAL_ERROR_IMAGEN en su imagen y RELACION veces más en su profundidad)."""
        if self._inversa is None:
            k2 = LATERAL_RELACION_PROFUNDIDAD ** 2
            ez, el = np.array((0.0, 0.0, 1.0)), self.R[:, 2]
            cov = LATERAL_ERROR_IMAGEN ** 2 * (2 * np.eye(3) + (k2 - 1) * (np.outer(ez, ez) + np.outer(el, el)))
            self._inversa = np.linalg.inv(cov)
        e = self.alinear_lateral(pl) - np.asarray(pf[:3], float)
        return float(np.sqrt(e @ self._inversa @ e))

    def orientar(self, frente, lateral):
        """La lectura lateral tal cual o con izquierda y derecha cambiadas: la que más coincide con la de frente."""
        if not self.lista:
            return lateral
        comunes = self._comunes(frente, lateral, LATERAL_PUNTOS)
        if len(comunes) < 4:
            return lateral
        cambiada = cambiar_lados(lateral)

        def discrepancia(l):
            return float(np.median([self.distancia(frente[i], l[i]) for i in comunes]))
        return cambiada if discrepancia(cambiada) < discrepancia(lateral) else lateral

    def actualizar(self, frente, lateral, t):
        """Suma un par de lecturas al cálculo del giro, sólo con los puntos en que las dos coinciden.

        False (lectura descartada) si discrepan la mitad de los puntos o más.
        """
        if not self.lista:
            comunes = self._comunes(frente, lateral, LATERAL_PUNTOS)
            if len(comunes) < 4:
                return False
            self._iniciales.append((frente, lateral, t))
            self.muestras = len(self._iniciales)
            if self.muestras >= LATERAL_MUESTRAS_MIN:
                self._primer_giro()
            return True
        lateral = self.orientar(frente, lateral)
        comunes = self._comunes(frente, lateral, LATERAL_PUNTOS)
        if len(comunes) < 4:
            return False
        distancias = {i: self.distancia(frente[i], lateral[i]) for i in comunes}
        self.residuo = float(np.median(list(distancias.values())))
        buenos = [i for i in comunes if distancias[i] <= LATERAL_DISTANCIA_MAX]
        if len(buenos) <= len(comunes) / 2:
            self.descartes += 1
            if self.descartes >= DESCARTES_PARA_REINICIAR:
                self.reiniciar()               # la cámara se movió o el primer cálculo salió mal: de nuevo
            return False
        self.descartes = 0
        self._sumar(frente, lateral, buenos, t)
        self.muestras += 1
        self._kabsch()
        return True

    def _sumar(self, frente, lateral, comunes, t):
        olvido = 1.0 if self.t is None else 0.5 ** (max(t - self.t, 0.0) / LATERAL_MEMORIA)
        self.t = t
        self.suma_w *= olvido
        self.suma_p *= olvido
        self.suma_q *= olvido
        self.suma_pq *= olvido
        for i in comunes:
            w = min(_peso(frente[i][3]), _peso(lateral[i][3]))
            p, q = np.asarray(lateral[i][:3], float), np.asarray(frente[i][:3], float)
            self.suma_w += w
            self.suma_p += w * p
            self.suma_q += w * q
            self.suma_pq += w * np.outer(p, q)

    def _primer_giro(self):
        """El primer giro, con las lecturas guardadas tal cual y con los lados cambiados: el que mejor encaja."""
        mejor = None
        for cambiar in (False, True):
            self.suma_w, self.suma_p, self.suma_q, self.suma_pq, self.t = 0.0, np.zeros(3), np.zeros(3), np.zeros((3, 3)), None
            pares = [(f, cambiar_lados(l) if cambiar else l, t) for f, l, t in self._iniciales]
            for f, l, t in pares:
                self._sumar(f, l, self._comunes(f, l, LATERAL_PUNTOS), t)
            self._kabsch()
            error = float(np.median([self.distancia(f[i], l[i]) for f, l, _ in pares
                                     for i in self._comunes(f, l, LATERAL_PUNTOS)]))
            if mejor is None or error < mejor[0]:
                mejor = (error, self.suma_w, self.suma_p.copy(), self.suma_q.copy(), self.suma_pq.copy(), self.t)
        _, self.suma_w, self.suma_p, self.suma_q, self.suma_pq, self.t = mejor
        self._kabsch()
        self._iniciales = []

    def _kabsch(self):
        mp, mq = self.suma_p / self.suma_w, self.suma_q / self.suma_w
        H = self.suma_pq / self.suma_w - np.outer(mp, mq)
        U, _, Vt = np.linalg.svd(H)
        d = 1.0 if np.linalg.det(Vt.T @ U.T) >= 0 else -1.0     # giro, nunca reflejo
        self.R = Vt.T @ np.diag((1.0, 1.0, d)) @ U.T
        self.T = mq - self.R @ mp
        self._inversa = None

    def fusionar(self, frente, lateral):
        """Puntos combinados {índice: (x, y, z, visibilidad)} en los ejes de la de frente, o None."""
        if not self.lista:
            return None
        lateral = self.orientar(frente, lateral)
        eje_frente, eje_lateral = np.array((0.0, 0.0, 1.0)), self.R[:, 2]   # dirección de profundidad de cada una
        salida = dict(frente)
        for i, pf in frente.items():
            pl = lateral.get(i)
            if pl is None or pl[3] < POSE_VISIBILIDAD_MIN or self.distancia(pf, pl) > LATERAL_DISTANCIA_MAX:
                continue                       # ese punto, sólo con la de frente
            i_f, i_l = informacion(eje_frente, pf[3]), informacion(eje_lateral, pl[3])
            x = np.linalg.solve(i_f + i_l, i_f @ np.asarray(pf[:3], float) + i_l @ self.alinear_lateral(pl))
            salida[i] = (float(x[0]), float(x[1]), float(x[2]), max(pf[3], pl[3]))
        return salida


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
        self.lectura = None                 # (t, {índice: (x, y, z, vis)}, [(x, y) normalizados])
        self.frame, self.hz = None, 0.0
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
                               [(p.x, p.y) for p in img])
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
        """La imagen lateral pequeña con el brazo dibujado, para ver si te encuadra."""
        with self.cerrojo:
            f, lectura = self.frame, self.lectura
        if f is None:
            return None
        alto = int(f.shape[0] * ancho / f.shape[1])
        img = cv2.resize(f, (ancho, alto))
        if lectura is not None:
            pts = [(int(x * ancho), int(y * alto)) for x, y in lectura[2]]
            for a, b in ((11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (11, 23), (12, 24), (23, 24)):
                cv2.line(img, pts[a], pts[b], (0, 220, 0), 2)
        return img

    def cerrar(self):
        self.activo = False
        if self.is_alive():
            self.join(timeout=2.0)
        self.camara.cerrar()
        self._pose.close()
