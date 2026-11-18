"""Del movimiento de la persona al punto objetivo del brazo. Sin cámara ni hardware."""

from dataclasses import dataclass, field
from math import acos, atan2, cos, degrees, hypot, log, pi, radians, sin
from statistics import median
from typing import Optional

# Índices de MediaPipe Hands
MUNECA, PULGAR_PUNTA, INDICE_BASE, MEDIO_BASE, ANULAR_BASE, MENIQUE_BASE = 0, 4, 5, 9, 13, 17
PUNTAS_DEDOS = (8, 12, 16, 20)
PALMA = (MUNECA, INDICE_BASE, MEDIO_BASE, ANULAR_BASE, MENIQUE_BASE)
MODO_MANO, MODO_BRAZO, MODO_ARTICULAR = "mano", "brazo", "articular"
MODOS = (MODO_MANO, MODO_BRAZO, MODO_ARTICULAR)


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
    angulos: Optional[tuple] = None     # modo articular: (None, hombro, codo, mano) en el plano frontal
    brazo3d: Optional[tuple] = None     # modo articular en 3D: (brazo, antebrazo, mano|None), ejes del torso
    medidas: dict = field(default_factory=dict)        # sólo para mostrar y calibrar en la app; no viaja


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


def envolver(grados):
    """A (-180, 180]."""
    return -((180.0 - grados) % 360.0 - 180.0)


def angulos_articulares(hombro, codo, muneca, direccion_mano=None, afuera=-1):
    """Brazo de la persona -> ángulos del robot, en el plano que ve la cámara frontal (grados).

    Puntos en (adelante, izquierda, arriba); la profundidad no se usa: una sola cámara la estima mal
    y el robot, montado en el lateral de la caja, se mueve en un plano fijo como el de tu brazo.
    En ese plano, 'afuera' es el lado hacia donde sale el brazo del cuerpo (derecho: -1, izquierdo: +1)
    y los ángulos se miden desde la horizontal hacia afuera, positivos hacia arriba:
      hombro: dirección del brazo (hombro -> codo): 0 = horizontal hacia afuera, 90 = arriba, -90 = colgando.
      codo:   giro del antebrazo respecto al brazo; positivo si se dobla hacia arriba.
      mano:   dirección de la mano (muñeca -> nudillos) en el mismo plano; None sin mano. El control
              la convierte en ángulo de muñeca con los ángulos que el robot sí alcanza.
    Devuelve (None, hombro, codo, mano): la base no se mueve en este modo.
    """
    def direccion(v):
        return degrees(atan2(v[2], afuera * v[1]))

    sh = direccion([c - h for c, h in zip(codo, hombro)])
    l2 = direccion([m - c for m, c in zip(muneca, codo)])
    mano = None if direccion_mano is None else direccion(direccion_mano)
    return (None, sh, envolver(l2 - sh), mano)


# ---------------------------------------------------------------- brazo en 3D respecto al torso
def _resta(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _punto(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cruz(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unitario(v):
    n = _punto(v, v) ** 0.5
    if n < 1e-9:
        raise ValueError("vector nulo")
    return tuple(x / n for x in v)


def marco_torso(hombro, hombro_otro, caderas=None):
    """Ejes unitarios del torso (afuera, adelante, arriba), en coordenadas de mundo de MediaPipe.

    MediaPipe: x a la derecha de la imagen, y hacia abajo, z alejándose de la cámara.
    afuera:   del otro hombro a este, hacia donde sale el brazo.
    arriba:   de las caderas al centro de los hombros; sin caderas en cuadro, la vertical de la cámara.
    adelante: perpendicular a los dos, hacia la cámara (la persona la mira de frente).
    Medir en estos ejes hace que girar el cuerpo frente a la cámara no cambie la lectura del brazo.
    """
    afuera = _unitario(_resta(hombro, hombro_otro))
    if caderas is not None:
        centro = tuple((a + b) / 2 for a, b in zip(hombro, hombro_otro))
        vertical = _resta(centro, caderas)
    else:
        vertical = (0.0, -1.0, 0.0)
    arriba = _unitario(_resta(vertical, tuple(_punto(vertical, afuera) * a for a in afuera)))
    adelante = _cruz(arriba, afuera)
    if adelante[2] > 0:                   # hacia la cámara es z negativa
        adelante = tuple(-a for a in adelante)
    return afuera, adelante, arriba


def en_torso(v, marco):
    """Vector de mundo -> (afuera, adelante, arriba)."""
    return tuple(_punto(v, eje) for eje in marco)


def angulo_entre(a, b):
    """Ángulo entre dos vectores, en grados (0 = misma dirección)."""
    c = _punto(a, b) / max((_punto(a, a) * _punto(b, b)) ** 0.5, 1e-12)
    return degrees(acos(max(-1.0, min(1.0, c))))


def medidas_brazo3d(brazo, antebrazo):
    """Lo que la persona hace, en grados: flexión del codo, azimut y elevación del brazo completo,
    y qué fracción del brazo apunta hacia la cámara (alta = profundidad poco fiable)."""
    d = tuple(a + b for a, b in zip(brazo, antebrazo))
    largo = max(_punto(d, d) ** 0.5, 1e-9)
    return {"flexion": angulo_entre(brazo, antebrazo),
            "azimut": degrees(atan2(d[1], d[0])),
            "elevacion": degrees(atan2(d[2], hypot(d[0], d[1]))),
            "hacia_camara": abs(d[1]) / largo}


def _rotar(v, eje, angulo):
    """Rodrigues: v girado 'angulo' radianes alrededor del eje unitario."""
    c, s = cos(angulo), sin(angulo)
    k = _cruz(eje, v)
    kv = _punto(eje, v)
    return tuple(v[i] * c + k[i] * s + eje[i] * kv * (1 - c) for i in range(3))


def _rotacion_hacia(a, b):
    """Función que gira cualquier vector como el giro mínimo que lleva la dirección a a la b."""
    a, b = _unitario(a), _unitario(b)
    eje = _cruz(a, b)
    s = _punto(eje, eje) ** 0.5
    if s < 1e-9:
        return (lambda v: v) if _punto(a, b) > 0 else (lambda v: (-v[0], -v[1], v[2]))
    eje, angulo = tuple(x / s for x in eje), atan2(s, _punto(a, b))
    return lambda v: _rotar(v, eje, angulo)


def _cambiar_angulo(base, v, nuevo):
    """v girado en el plano (base, v) para que forme 'nuevo' grados con base, conservando su largo."""
    n = _cruz(base, v)
    if _punto(n, n) < 1e-12:
        return v
    largo = _punto(v, v) ** 0.5
    u = _unitario(base)
    perp = _unitario(_cruz(_unitario(n), u))
    r = radians(nuevo)
    return tuple(largo * (cos(r) * a + sin(r) * b) for a, b in zip(u, perp))


def _reescalar(angulo, cero):
    """La flexión que se veía estando recto pasa a ser 0; 180 sigue siendo 180."""
    cero = min(max(cero, 0.0), 90.0)
    return max(0.0, (angulo - cero) * 180.0 / (180.0 - cero))


def _enderezar(brazo, antebrazo, mano, flexion, mano_cero):
    antebrazo = _cambiar_angulo(brazo, antebrazo, _reescalar(angulo_entre(brazo, antebrazo), flexion))
    if mano is not None:
        mano = _cambiar_angulo(antebrazo, mano, _reescalar(angulo_entre(antebrazo, mano), mano_cero))
    return brazo, antebrazo, mano


def calibracion_gemelo(muestras):
    """Con el brazo extendido hacia afuera, horizontal y quieto (en T): qué mide la cámara de más.

    muestras: lecturas (brazo, antebrazo, mano|None) sin calibrar. Devuelve la flexión que se ve con
    el codo recto, el ángulo mano-antebrazo con la mano recta y la dirección del brazo ya enderezado.
    """
    if not muestras:
        raise ValueError("sin lecturas para calibrar")
    flexion = median(angulo_entre(u, f) for u, f, _ in muestras)
    manos = [angulo_entre(f, h) for _, f, h in muestras if h is not None]
    mano = median(manos) if manos else 0.0
    # La dirección se mide después de enderezar: enderezar el codo mueve la dirección del brazo completo.
    direcciones = []
    for u, f, h in muestras:
        u, f, _ = _enderezar(u, f, h, flexion, mano)
        direcciones.append(_unitario(tuple(a + b for a, b in zip(u, f))))
    direccion = _unitario(tuple(median(d[k] for d in direcciones) for k in range(3)))
    return {"direccion": list(direccion), "flexion": flexion, "mano": mano}


def aplicar_calibracion(brazo, antebrazo, mano, cal):
    """Corrige una lectura con la calibración en T: la flexión (del codo y de la mano) que se veía
    estando recto pasa a ser 0, reescalada hasta 180, y la dirección medida pasa a ser 'afuera y horizontal'."""
    if not cal:
        return brazo, antebrazo, mano
    brazo, antebrazo, mano = _enderezar(brazo, antebrazo, mano, cal["flexion"], cal["mano"])
    girar = _rotacion_hacia(cal["direccion"], (1.0, 0.0, 0.0))
    return girar(brazo), girar(antebrazo), None if mano is None else girar(mano)


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
