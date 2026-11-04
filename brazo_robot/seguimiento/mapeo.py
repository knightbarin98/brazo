"""Del movimiento de la mano al punto objetivo del brazo. Sin cámara ni hardware."""

from dataclasses import dataclass
from math import hypot, log, pi

# Índices de MediaPipe Hands
MUNECA, PULGAR_PUNTA, INDICE_BASE, INDICE_PUNTA, MEDIO_BASE, ANULAR_BASE, MENIQUE_BASE = 0, 4, 5, 8, 9, 13, 17
PALMA = (MUNECA, INDICE_BASE, MEDIO_BASE, ANULAR_BASE, MENIQUE_BASE)


@dataclass
class Mano:
    x: float        # centro de la palma, 0..1 a lo ancho de la imagen
    y: float        # 0..1 hacia abajo
    escala: float   # tamaño de la palma en altos de imagen; crece al acercarse a la cámara
    pinza: float    # distancia pulgar-índice / tamaño de la palma
    puntos: list    # los 21 puntos (x, y) normalizados, para dibujar


def medir_mano(puntos, aspecto):
    """puntos: 21 pares (x, y) normalizados; aspecto: ancho / alto de la imagen."""
    def dist(a, b):
        return hypot((puntos[a][0] - puntos[b][0]) * aspecto, puntos[a][1] - puntos[b][1])

    # Al girar la mano se acorta el largo o el ancho de la palma, rara vez los dos.
    escala = max(dist(MUNECA, MEDIO_BASE), 1.3 * dist(INDICE_BASE, MENIQUE_BASE), 1e-6)
    x = sum(puntos[i][0] for i in PALMA) / len(PALMA)
    y = sum(puntos[i][1] for i in PALMA) / len(PALMA)
    return Mano(x, y, escala, dist(PULGAR_PUNTA, INDICE_PUNTA) / escala, list(puntos))


class FiltroUnEuro:
    """Suaviza mucho cuando la mano está quieta y casi nada cuando se mueve rápido."""

    def __init__(self, corte_min, beta, corte_derivada=1.0):
        self.corte_min, self.beta, self.corte_derivada = corte_min, beta, corte_derivada
        self.valor = self.derivada = self.t = None

    @staticmethod
    def _alfa(corte, dt):
        return 1.0 / (1.0 + 1.0 / (2 * pi * corte * dt))

    def __call__(self, valor, t):
        if self.t is None:
            self.valor, self.derivada, self.t = valor, 0.0, t
            return valor
        dt = t - self.t
        if dt <= 0:
            return self.valor
        d = (valor - self.valor) / dt
        self.derivada += self._alfa(self.corte_derivada, dt) * (d - self.derivada)
        corte = self.corte_min + self.beta * abs(self.derivada)
        self.valor += self._alfa(corte, dt) * (valor - self.valor)
        self.t = t
        return self.valor


class MapeoRelativo:
    """Mueve el objetivo según lo que la mano se desplaza desde que se enganchó.

    Es relativo como un ratón: al enganchar no hay salto, y al pausar y volver a
    enganchar la persona puede recolocar la mano sin que el brazo se mueva.
    """

    def __init__(self, ganancias, limites, filtro_xy, filtro_escala):
        self.ganancias, self.limites = ganancias, limites
        self.filtro_xy, self.filtro_escala = filtro_xy, filtro_escala
        self.ref_mano = self.ref_pose = None

    def enganchar(self, mano, pose, t):
        self.ref_mano = (mano.x, mano.y, log(mano.escala))
        self.ref_pose = tuple(pose)
        self.filtros = (FiltroUnEuro(*self.filtro_xy), FiltroUnEuro(*self.filtro_xy),
                        FiltroUnEuro(*self.filtro_escala))
        for filtro, valor in zip(self.filtros, self.ref_mano):
            filtro(valor, t)

    def objetivo(self, mano, t):
        mx, my, me = (f(v, t) for f, v in zip(self.filtros, (mano.x, mano.y, log(mano.escala))))
        gx, gy, gz = self.ganancias
        punto = (self.ref_pose[0] + gx * (me - self.ref_mano[2]),
                 self.ref_pose[1] - gy * (mx - self.ref_mano[0]),   # imagen: x crece a la derecha
                 self.ref_pose[2] - gz * (my - self.ref_mano[1]))   # imagen: y crece hacia abajo
        return tuple(min(hi, max(lo, v)) for v, (lo, hi) in zip(punto, self.limites))


def limitar_paso(actual, objetivo, maximo):
    """Avanza de actual hacia objetivo, como mucho 'maximo' mm en línea recta."""
    delta = [o - a for a, o in zip(actual, objetivo)]
    largo = sum(d * d for d in delta) ** 0.5
    if largo <= maximo:
        return tuple(objetivo)
    return tuple(a + d * maximo / largo for a, d in zip(actual, delta))


class Pinza:
    """Detecta cada gesto de pinza una sola vez, con histéresis y tiempo mínimo sostenido."""

    def __init__(self, cerrar, abrir, tiempo):
        self.cerrar, self.abrir, self.tiempo = cerrar, abrir, tiempo
        self.reiniciar(1.0)

    def reiniciar(self, ratio):
        # Una mano que ya está en pinza al enganchar debe abrirse antes de contar.
        self.armada, self.desde = ratio >= self.cerrar, None

    def actualizar(self, ratio, t):
        """True sólo en el instante en que se confirma una pinza nueva."""
        if ratio >= self.abrir:
            self.armada, self.desde = True, None
        elif ratio > self.cerrar:
            self.desde = None
        elif self.armada:
            self.desde = t if self.desde is None else self.desde
            if t - self.desde >= self.tiempo:
                self.armada, self.desde = False, None
                return True
        return False


