"""Del movimiento de la persona al punto objetivo del brazo. Sin cámara ni hardware."""

from dataclasses import dataclass, field
from math import atan2, degrees, hypot, log, pi
from typing import Optional

# Índices de MediaPipe Hands
MUNECA, PULGAR_PUNTA, INDICE_BASE, MEDIO_BASE, ANULAR_BASE, MENIQUE_BASE = 0, 4, 5, 9, 13, 17
PUNTAS_DEDOS = (8, 12, 16, 20)
PALMA = (MUNECA, INDICE_BASE, MEDIO_BASE, ANULAR_BASE, MENIQUE_BASE)
MODO_MANO, MODO_BRAZO = "mano", "brazo"


@dataclass
class Mano:
    """Una lectura de la persona. 'vector' es (adelante, izquierda, arriba) en unidades del modo."""
    modo: str
    x: float                            # posición en la imagen, 0..1: sirve para detectar saltos
    y: float
    vector: tuple
    apertura: Optional[float] = None    # 0 = cerrada .. 1 = abierta; None si no se ve la mano
    cabeceo: Optional[float] = None     # elevación de la mano, grados
    largo: Optional[float] = None       # modo brazo: hombro-codo + codo-muñeca, en metros
    ratio: Optional[float] = None       # apertura sin calibrar: sólo para calibrar en la app
    puntos: list = field(default_factory=list)         # 21 puntos de la mano, para dibujar
    puntos_brazo: list = field(default_factory=list)   # hombro, codo y muñeca, para dibujar


def a_robot(p):
    """Coordenadas de mundo de MediaPipe sobre la imagen en espejo -> (adelante, izquierda, arriba)."""
    return (-p[2], -p[0], -p[1])


def _dist3(a, b):
    return sum((u - v) ** 2 for u, v in zip(a, b)) ** 0.5


def ratio_apertura(mundo):
    """Pulgar <-> promedio de las otras 4 puntas / largo de la palma, con los 21 puntos 3D.

    Al ser 3D y usar cuatro dedos, depende poco de cómo esté girada la mano.
    """
    palma = _dist3(mundo[MUNECA], mundo[MEDIO_BASE])
    if palma < 1e-6:
        return None
    dedos = [sum(mundo[i][k] for i in PUNTAS_DEDOS) / len(PUNTAS_DEDOS) for k in range(3)]
    return _dist3(mundo[PULGAR_PUNTA], dedos) / palma


def apertura_de_ratio(ratio, abierta, cerrada):
    if ratio is None or abs(abierta - cerrada) < 1e-3:
        return None
    return min(1.0, max(0.0, (ratio - cerrada) / (abierta - cerrada)))


def cabeceo_mano(mundo):
    """Elevación de muñeca -> nudillo medio: +90 apunta arriba, -90 abajo."""
    adelante, izquierda, arriba = a_robot([mundo[MEDIO_BASE][k] - mundo[MUNECA][k] for k in range(3)])
    return degrees(atan2(arriba, hypot(adelante, izquierda)))


def medir_mano(puntos, aspecto, mundo=None, abierta=1.10, cerrada=0.15):
    """Modo mano. puntos: 21 pares (x, y) normalizados; aspecto: ancho / alto; mundo: 21 puntos 3D."""
    def dist(a, b):
        return hypot((puntos[a][0] - puntos[b][0]) * aspecto, puntos[a][1] - puntos[b][1])

    # Al girar la mano se acorta el largo o el ancho de la palma, rara vez los dos.
    escala = max(dist(MUNECA, MEDIO_BASE), 1.3 * dist(INDICE_BASE, MENIQUE_BASE), 1e-6)
    x = sum(puntos[i][0] for i in PALMA) / len(PALMA)
    y = sum(puntos[i][1] for i in PALMA) / len(PALMA)
    ratio = ratio_apertura(mundo) if mundo is not None else None
    return Mano(MODO_MANO, x, y, (log(escala), -x, -y), apertura_de_ratio(ratio, abierta, cerrada),
                cabeceo_mano(mundo) if mundo is not None else None, ratio=ratio, puntos=list(puntos))


def medir_brazo(hombro, codo, muneca, px_muneca):
    """Modo brazo. Puntos 3D ya en (adelante, izquierda, arriba), en metros."""
    vector = tuple(m - h for m, h in zip(muneca, hombro))
    return Mano(MODO_BRAZO, px_muneca[0], px_muneca[1], vector,
                largo=_dist3(hombro, codo) + _dist3(codo, muneca))


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
    """Mueve el objetivo según lo que la persona se desplaza desde que se enganchó.

    Es relativo como un ratón: al enganchar no hay salto, y al pausar y volver a
    enganchar la persona puede recolocarse sin que el brazo se mueva.
    """

    def __init__(self, limites, filtros_por_eje):
        self.limites, self.filtros_por_eje = limites, filtros_por_eje
        self.factor = 1.0
        self.ref = self.ref_pose = self.mm = self.ultimo = None

    def enganchar(self, vector, pose, mm_por_unidad, t):
        self.ref, self.ref_pose, self.mm = tuple(vector), tuple(pose), tuple(mm_por_unidad)
        self.filtros = [FiltroUnEuro(*p) for p in self.filtros_por_eje]
        self.ultimo = tuple(f(v, t) for f, v in zip(self.filtros, vector))

    def objetivo(self, vector, t):
        self.ultimo = tuple(f(v, t) for f, v in zip(self.filtros, vector))
        punto = (p + self.factor * m * (u - r)
                 for p, m, u, r in zip(self.ref_pose, self.mm, self.ultimo, self.ref))
        return tuple(min(hi, max(lo, v)) for v, (lo, hi) in zip(punto, self.limites))

    def cambiar_factor(self, factor, pose):
        """Cambia la ganancia sin que el objetivo salte: la posición actual es la nueva referencia."""
        self.factor = factor
        if self.ref is not None:
            self.ref, self.ref_pose = self.ultimo, tuple(pose)


def limitar_paso(actual, objetivo, maximo):
    """Avanza de actual hacia objetivo, como mucho 'maximo' en línea recta."""
    delta = [o - a for a, o in zip(actual, objetivo)]
    largo = sum(d * d for d in delta) ** 0.5
    if largo <= maximo:
        return tuple(objetivo)
    return tuple(a + d * maximo / largo for a, d in zip(actual, delta))


def proyectar_alcanzable(centro, objetivo, alcanzable, iteraciones=8):
    """El punto alcanzable más lejano en la recta centro -> objetivo, o None si el centro no lo es.

    Se mide desde el centro y no desde la posición actual: así, con el objetivo fuera de
    alcance, el brazo se desliza por el borde en vez de quedarse trabado.
    """
    if alcanzable(objetivo):
        return tuple(objetivo)
    if not alcanzable(centro):
        return None
    dentro, fuera = 0.0, 1.0
    for _ in range(iteraciones):
        t = (dentro + fuera) / 2
        if alcanzable(tuple(c + t * (o - c) for c, o in zip(centro, objetivo))):
            dentro = t
        else:
            fuera = t
    return tuple(c + dentro * (o - c) for c, o in zip(centro, objetivo))


class SeguidorGarra:
    """La garra sigue la apertura de la mano, pero sólo después de que la mano la "alcance".

    Si al activar la garra sostiene una pieza y la mano está abierta, seguirla sin más la
    soltaría. No se mueve hasta que la apertura de la mano pase cerca del ángulo actual.
    """

    def __init__(self, tolerancia, filtro):
        self.tolerancia, self.parametros = tolerancia, filtro
        self.soltar()

    def soltar(self):
        self.sigue, self.lado, self.filtro = False, 0.0, FiltroUnEuro(*self.parametros)

    def actualizar(self, apertura, t, actual, abierta, cerrada):
        """Ángulo que debe tener la garra, o None si por ahora no debe moverse."""
        if apertura is None:
            return None
        valor = self.filtro(apertura, t)
        angulo = cerrada + valor * (abierta - cerrada)
        if not self.sigue:
            diferencia = angulo - actual
            cruzo = self.lado * diferencia < 0
            self.lado = diferencia
            if abs(diferencia) > self.tolerancia and not cruzo:
                return None
            self.sigue = True
        return angulo
