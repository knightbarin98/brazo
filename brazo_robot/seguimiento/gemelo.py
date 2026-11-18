"""Gemelo del brazo: tu brazo en 3D (ejes del torso) -> ángulos de los servos, con la cinemática real.

Sin cámara ni hardware. Lo usan igual el servidor (robot real) y el simulador.

El robot tiene 4 articulaciones que cuentan para la postura: base (gira sobre la vertical), hombro,
codo y muñeca (las tres giran en el plano vertical que elige la base). Tu brazo tiene más: tu hombro
gira en tres ejes y tu codo puede quedar fuera de ese plano. Lo que se conserva, mientras los topes lo
permiten:
  - la dirección de tu brazo completo (hombro -> muñeca): azimut con la base, elevación con el hombro;
  - la flexión del codo, medida en 3D;
  - la dirección de tu mano, proyectada en el plano del robot, con la muñeca.
El codo del robot sólo se dobla hacia un lado (servo 0..180 con MAPEO (0, -1)): si el tuyo queda del
otro lado de la línea hombro-muñeca, el robot pone el codo en espejo respecto a esa línea, con la
misma flexión y la misma dirección de la mano.
"""

from math import acos, atan2, cos, degrees, hypot, radians, sin, sqrt
from statistics import median

from .. import config
from ..cinematica import ARTICULACIONES_SERVO, servos_desde_geo
from ..config import (ARTICULAR_DESFASE, ARTICULAR_FILTRO, ARTICULAR_FILTRO_BASE, ARTICULAR_LIMITES,
                      ARTICULAR_UMBRAL_HORIZONTAL, L1, L2, MAPEO)
from .mapeo import FiltroUnEuro, angulo_entre


def _geo_de_servos(servos):
    return [(s - MAPEO[k][0]) / MAPEO[k][1] for k, s in zip(ARTICULACIONES_SERVO, servos)]


def _rango_geo(nombre):
    """Límites de ARTICULAR_LIMITES (servo) pasados a ángulo geométrico."""
    cero, signo = MAPEO[nombre]
    a, b = ((v - cero) / signo for v in ARTICULAR_LIMITES[nombre])
    return min(a, b), max(a, b)


def _desenrollar(angulo, referencia, periodo=360.0):
    """El equivalente de 'angulo' (± periodo·k) más cercano a 'referencia': sin saltos al dar la vuelta."""
    return angulo + periodo * round((referencia - angulo) / periodo)


def a_ejes_robot(v):
    """(afuera, adelante, arriba) de la persona -> (X, Y, Z) del robot con la base en ARTICULAR_BASE.

    Al costado (MONTAJE "costado"): la Z del robot sale de la caja como tu 'afuera'; su X es tu
    'abajo' (el plano por el que el brazo cuelga) e Y tu 'hacia la cámara', con el signo calibrado.
    De pie: X es el plano de la caja; con ARTICULAR_ESPEJO el frente del robot mira hacia tu cuerpo,
    así que tu 'afuera' es su X negativa. Z es tu 'arriba'.
    """
    signo = config.ARTICULAR_SIGNO_PROFUNDIDAD
    if config.MONTAJE == "costado":
        return (-v[2], signo * v[1], v[0])
    lado = -1.0 if config.ARTICULAR_ESPEJO else 1.0
    return (lado * v[0], signo * v[1], v[2])


def de_ejes_robot(p):
    """Inversa de a_ejes_robot: (X, Y, Z) del robot (base en ARTICULAR_BASE) -> (afuera, adelante, arriba)."""
    signo = config.ARTICULAR_SIGNO_PROFUNDIDAD
    if config.MONTAJE == "costado":
        return (p[2], signo * p[1], -p[0])
    lado = -1.0 if config.ARTICULAR_ESPEJO else 1.0
    return (lado * p[0], signo * p[1], p[2])


def base_de_referencia():
    """Ángulo geométrico de la base en ARTICULAR_BASE."""
    return (config.ARTICULAR_BASE - MAPEO["base"][0]) / MAPEO["base"][1]


def apertura_hombro_codo(flexion):
    """Ángulo (grados) entre el brazo del robot y la línea hombro -> muñeca, para esa flexión del codo."""
    f = radians(min(max(flexion, 0.0), 180.0))
    d = sqrt(max(L1 * L1 + L2 * L2 + 2 * L1 * L2 * cos(f), 1e-9))
    return degrees(acos(max(-1.0, min(1.0, (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)))))


def flexion_para_apertura(apertura, flexion_preferida):
    """Flexión del codo del robot con la que su brazo forma 'apertura' grados con la línea hombro -> muñeca.

    Para L1 > L2 hay dos flexiones posibles (la apertura sube y vuelve a bajar al doblar): se elige la más
    cercana a tu flexión. Si la apertura no se puede conseguir, la flexión que más se acerca.
    """
    candidatas = [k * 0.5 for k in range(361)]
    return min(candidatas, key=lambda f: (round(abs(apertura_hombro_codo(f) - apertura), 1), abs(f - flexion_preferida)))


HISTERESIS_PLANO = 15.0   # grados de mejora que hacen falta para cambiar al giro opuesto de la base


class SeguidorGemelo:
    """Lecturas en 3D -> servos objetivo, con filtro por articulación y topes sin saltos.

    Cada ángulo se sigue sin envolver, como en el modo plano: al pasar por una zona que el robot no
    alcanza se queda en el tope por donde salió. El plano del robot se sigue con periodo de 180°
    (girar 180° y doblar el hombro hacia atrás deja el brazo en el mismo plano): se elige el
    equivalente más cercano a donde ya está la base.
    """

    def __init__(self, muestras, t, servos_actuales, plano_fijo=False):
        """plano_fijo: la base se queda en ARTICULAR_BASE y lo demás se proyecta en ese plano (sin profundidad)."""
        self.rangos = [_rango_geo(k) for k in ARTICULACIONES_SERVO]
        self.b0 = base_de_referencia()
        self.plano_fijo = plano_fijo
        geo = _geo_de_servos(servos_actuales)
        self.plano = 0.0 if plano_fijo else geo[0] - self.b0   # plano actual, sin envolver
        self.crudo = [None, None, None, None]
        self.filtros = [FiltroUnEuro(*ARTICULAR_FILTRO_BASE)] + [FiltroUnEuro(*ARTICULAR_FILTRO) for _ in range(3)]
        self.acercando = True
        self.info = {}
        brazos = [m.brazo3d for m in muestras]
        mediana = tuple(tuple(median(b[i][k] for b in brazos) for k in range(3)) for i in range(2))
        manos = [b[2] for b in brazos if b[2] is not None]
        mano = tuple(median(h[k] for h in manos) for k in range(3)) if manos else None
        self._calcular((mediana[0], mediana[1], mano), t, geo)   # siembra filtros y referencias

    def _seguir(self, k, angulo, t, periodo=360.0):
        if self.crudo[k] is not None:
            angulo = _desenrollar(angulo, self.crudo[k], periodo)
        self.crudo[k] = angulo
        return self.filtros[k](angulo, t)

    def _recortar(self, k, valor):
        lo, hi = self.rangos[k]
        return min(hi, max(lo, valor))

    def _elegir_plano(self, bruto, hombro_en):
        """Giro de la base (sin envolver) para el plano de tu brazo.

        El mismo plano se alcanza con dos giros separados 180°: en uno el brazo del robot apunta hacia
        su frente y en el otro hacia atrás, y sólo a veces los dos caben en los topes de la base y del
        hombro. Se elige el que cabe; ante un empate, el de siempre, para no saltar de uno a otro.
        """
        actual = _desenrollar(bruto, self.plano, 180.0)
        lo_b, hi_b = (v - self.b0 for v in self.rangos[0])
        lo_h, hi_h = self.rangos[1]

        def coste(giro):
            alcanzable = min(hi_b, max(lo_b, giro))
            _, hombro = hombro_en(alcanzable)
            fuera = abs(giro - alcanzable) + abs(hombro - min(hi_h, max(lo_h, hombro)))
            return fuera + (0.0 if abs(giro - self.plano) < 90.0 else HISTERESIS_PLANO)

        return min((actual, actual - 180.0, actual + 180.0), key=coste)

    def _calcular(self, brazo3d, t, geo_actual):
        u, f, h = (None if v is None else a_ejes_robot(v) for v in brazo3d)
        d = tuple(a + b for a, b in zip(u, f))
        largo = sqrt(sum(x * x for x in d))
        pedido, geo = list(geo_actual), list(geo_actual)
        lado = -1.0 if (config.MONTAJE != "costado" and config.ARTICULAR_ESPEJO) else 1.0

        flexion = angulo_entre(u, f)
        apertura = apertura_hombro_codo(flexion)
        ref_hombro = self.crudo[1] if self.crudo[1] is not None else sum(self.rangos[1]) / 2

        def hombro_en(giro_grados):
            """Hombro que deja la muñeca del robot en la dirección de tu brazo, con la base en ese giro."""
            g = radians(giro_grados)
            elevacion = degrees(atan2(d[2], d[0] * cos(g) + d[1] * sin(g)))
            return elevacion, _desenrollar(elevacion + apertura, ref_hombro)

        # Base: el plano vertical que contiene tu brazo. Casi vertical, ese plano no existe: se conserva.
        vertical = hypot(d[0], d[1]) <= ARTICULAR_UMBRAL_HORIZONTAL * largo
        if not vertical and not self.plano_fijo:
            self.plano = self._elegir_plano(degrees(atan2(d[1], d[0])), hombro_en)
        plano = self._seguir(0, self.plano, t, 180.0)
        pedido[0] = self.b0 + plano
        geo[0] = self._recortar(0, pedido[0])
        giro = geo[0] - self.b0                  # plano que la base sí alcanza; lo demás se proyecta
        radial = (cos(radians(giro)), sin(radians(giro)))

        # Hombro: la muñeca del robot en la dirección de tu brazo completo, dentro de ese plano.
        elevacion, hombro = hombro_en(giro)
        hombro = self._seguir(1, hombro, t)
        pedido[1] = hombro + lado * ARTICULAR_DESFASE["hombro"]
        # Codo: tu flexión en 3D; el del robot se dobla hacia la geometría negativa (MAPEO codo (0, -1)).
        codo = self._seguir(2, -flexion, t)
        pedido[2] = codo + lado * ARTICULAR_DESFASE["codo"]
        geo[1] = self._recortar(1, pedido[1])
        codo_ajustado = False
        if abs(geo[1] - pedido[1]) > 0.5:
            # Hombro en su tope: con tu flexión la muñeca ya no apuntaría como la tuya. Se prioriza la
            # dirección de la muñeca (donde queda la garra): el codo se dobla lo que haga falta para ella.
            flexion_robot = flexion_para_apertura(geo[1] - pedido[1] + apertura, flexion)
            pedido[2] = -flexion_robot + lado * ARTICULAR_DESFASE["codo"]
            codo_ajustado = abs(flexion_robot - flexion) > 1.0
        geo[2] = self._recortar(2, pedido[2])

        # Muñeca: la garra apunta como tu mano en el plano, con hombro y codo ya recortados.
        if h is not None:
            mano = degrees(atan2(h[2], h[0] * radial[0] + h[1] * radial[1]))
            mano = self._seguir(3, mano, t)
            pedido[3] = mano - geo[1] - geo[2] + ARTICULAR_DESFASE["muneca"]
            pedido[3] = _desenrollar(pedido[3], geo_actual[3])
            geo[3] = self._recortar(3, pedido[3])

        nombres = ("base", "hombro", "codo", "muneca")
        self.info = {"flexion": flexion, "plano": plano, "elevacion": elevacion, "brazo_vertical": vertical,
                     "codo_ajustado": codo_ajustado,
                     "topes": [n for n, g, p in zip(nombres, geo, pedido) if abs(g - p) > 0.5]}
        return geo, pedido

    def objetivo(self, mano, t, servos_actuales):
        """Servos pedidos y si alguna articulación quedó en su tope. Sin mano, la muñeca no se mueve."""
        geo, pedido = self._calcular(mano.brazo3d, t, _geo_de_servos(servos_actuales))
        recortado = bool(self.info["topes"])  # misma tolerancia (0.5°) que la lista de topes
        return servos_desde_geo(dict(zip(ARTICULACIONES_SERVO, geo))), recortado
