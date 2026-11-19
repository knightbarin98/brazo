"""Cinemática inversa de el punto de agarre de la garra."""

from math import acos, atan2, cos, degrees, hypot, pi, radians, sin, sqrt

import numpy as np

from . import config
from .ajustes import AJUSTES, numero_en_rango
from .config import H, L1, L2, MAPEO, Z_SEGURO


class Inalcanzable(Exception):
    pass


def ik(x, y, z, inclinacion=-90.):
    """IK de la garra, compensando el desplazamiento de L3 al inclinar.

    -90° reproduce la geometría anterior; el ángulo no es el valor del servo.
    """
    theta = radians(numero_en_rango(inclinacion, -120, -60, "Inclinación"))
    r = hypot(x, y) - config.L3 * cos(theta)
    zw = z - H - config.L3 * sin(theta)
    if r <= 0:
        raise Inalcanzable("Inclinación incompatible con ese radio")
    d = hypot(r, zw)
    if d > L1 + L2 - 1 or d < abs(L1 - L2) + 1:
        raise Inalcanzable(f"({x:.0f}, {y:.0f}, {z:.0f}) fuera de alcance")

    base = degrees(atan2(y, x))
    a = atan2(zw, r)
    b = acos(max(-1, min(1, (L1**2 + d**2 - L2**2) / (2 * L1 * d))))
    sh = a + b
    el = acos(max(-1, min(1, (L1**2 + L2**2 - d**2) / (2 * L1 * L2))))
    l2 = sh - (pi - el)
    wr = theta - l2

    geo = {"base": base, "hombro": degrees(sh), "codo": degrees(l2 - sh), "muneca": degrees(wr)}
    servo = []
    for k in ("base", "hombro", "codo", "muneca"):
        cero, signo = MAPEO[k]
        v = cero + signo * geo[k]
        if v < 0 or v > 180:
            raise Inalcanzable(f"({x:.0f}, {y:.0f}, {z:.0f}): servo {k} fuera de rango ({v:.0f}°)")
        servo.append(v)
    return servo


def radio_max_teorico():
    zw = config.Z_RECOGIDA + config.L3 - H
    return sqrt((L1 + L2 - 1) ** 2 - zw**2)


def z_transito(x, y, z_bajo, z_alto=None, inclinacion=None):
    """Altura más alta (<= z_alto) desde la que se puede bajar en vertical hasta z_bajo.

    Algunos puntos lejanos son alcanzables abajo pero no a Z_SEGURO (la muñeca
    saldría de 0..180°). v12 rechazaba esos puntos; aquí se baja la altura de
    aproximación sin forzar ni recortar ángulos.
    """
    z_alto = Z_SEGURO if z_alto is None else z_alto
    inclinacion = AJUSTES["inclinacion"] if inclinacion is None else inclinacion
    ik(x, y, z_bajo, inclinacion)                 # el punto bajo DEBE ser alcanzable
    tope = z_bajo
    for z in np.arange(z_bajo + 1, z_alto + 1e-6, 1.0):
        try:
            ik(x, y, float(z), inclinacion)
        except Inalcanzable:
            break
        tope = float(z)
    if z_alto > z_bajo and tope - z_bajo < min(10.0, z_alto - z_bajo):
        raise Inalcanzable(f"({x:.0f}, {y:.0f}): no hay ni 10 mm de altura libre sobre Z={z_bajo:.0f}")
    return min(tope, z_alto)


def firma_geometria():
    return [L1, L2, config.L3, H, *[v for par in MAPEO.values() for v in par]]


def firma_postura():
    return {"geometria": firma_geometria(), "z": float(config.Z_RECOGIDA),
            "inclinacion": float(AJUSTES["inclinacion"]), "montaje": config.MONTAJE}


ARTICULACIONES_SERVO = ("base", "hombro", "codo", "muneca")


def servos_desde_geo(geo):
    """Ángulos geométricos (dict) -> ángulos de servo, sin recortar."""
    return [MAPEO[k][0] + MAPEO[k][1] * geo[k] for k in ARTICULACIONES_SERVO]


def piso_seguro():
    """Z mínima (ejes del robot) de codo, muñeca y garra: la mesa (de pie) o la pared de la caja (costado).

    Al costado, una altura de recogida negativa (de una calibración con el robot de pie) metería el
    brazo en la pared: allí el piso nunca baja de 0.
    """
    return max(config.Z_RECOGIDA, 0.0) if config.MONTAJE == "costado" else config.Z_RECOGIDA


def postura_segura(servos):
    """Que ni la garra ni la muñeca ni el codo pasen del piso, golpeen la columna de la base ni, al
    costado, bajen más allá de ARTICULAR_LIMITE_ABAJO."""
    radio, alto = config.ARTICULAR_MARGEN_BASE
    base, (_, *puntos) = postura_desde_servos(servos)
    piso = piso_seguro()
    abajo = config.ARTICULAR_LIMITE_ABAJO if config.MONTAJE == "costado" else None
    # Al costado, "abajo" es la dirección del brazo con la base en ARTICULAR_BASE
    giro = radians(base - (config.ARTICULAR_BASE - MAPEO["base"][0]) / MAPEO["base"][1])
    for r, z in puntos:
        if z < piso or (abs(r) < radio and z < H + alto):
            return False
        if abajo is not None and r * cos(giro) > abajo:
            return False
    return True


def postura_desde_servos(servos):
    """Ángulo de la base y puntos del brazo de perfil (r, z) en mm: hombro, codo, muñeca y garra.

    Es la inversa de ik(): sirve para dibujar la postura ordenada sin cámara.
    """
    geo = {k: (s - MAPEO[k][0]) / MAPEO[k][1] for k, s in zip(("base", "hombro", "codo", "muneca"), servos)}
    sh = radians(geo["hombro"])
    l2 = sh + radians(geo["codo"])
    theta = l2 + radians(geo["muneca"])
    hombro = (0.0, float(H))
    codo = (hombro[0] + L1 * cos(sh), hombro[1] + L1 * sin(sh))
    muneca = (codo[0] + L2 * cos(l2), codo[1] + L2 * sin(l2))
    garra = (muneca[0] + config.L3 * cos(theta), muneca[1] + config.L3 * sin(theta))
    return geo["base"], [hombro, codo, muneca, garra]


def postura_inicial():
    """Servos del estado inicial, al que el robot vuelve al arrancar, con 'home' y al calibrar.

    Al costado: el brazo colgando recto en su plano de referencia, con la garra hacia abajo, como tu
    brazo relajado junto al cuerpo. Depende de 'calibrar montaje' (ARTICULAR_BASE y ceros de MAPEO).
    Si colgando a plomo toca algo (una Z de recogida por encima de H, una mesa en ARTICULAR_LIMITE_ABAJO)
    se separa el hombro de la pared lo justo. De pie, o si nada cabe: HOME de config.
    """
    if config.MONTAJE == "costado":
        base = (config.ARTICULAR_BASE - MAPEO["base"][0]) / MAPEO["base"][1]
        for hombro in range(0, 46, 5):
            servos = servos_desde_geo({"base": base, "hombro": float(hombro), "codo": 0.0, "muneca": 0.0})
            if any(s < -10.0 or s > 190.0 for s in servos):
                break
            # Con el codo recto su servo queda cerca del tope (0): unos grados de más se recortan.
            servos = [min(180.0, max(0.0, s)) for s in servos]
            if postura_segura(servos):
                return servos
    return [float(s) for s in config.HOME]


def tramo_seguro(a, b, paso_max=5.0):
    """Si al ir en línea recta (en servos, como lo hace el Arduino) de a a b nada toca la superficie."""
    n = max(1, int(np.ceil(max(abs(x - y) for x, y in zip(a, b)) / paso_max)))
    return all(postura_segura([x + (y - x) * k / n for x, y in zip(a, b)]) for k in range(1, n + 1))


def ruta_segura(desde, hasta, paso=10.0):
    """Puntos intermedios (servos) para ir de 'desde' a 'hasta' sin tocar la pared, la mesa ni la base.

    El Arduino mueve cada servo a la misma velocidad, no en línea recta, así que se manda la ruta en
    pasos de como mucho 'paso' grados, todos comprobados. Si el camino directo no es seguro, primero
    se separa el brazo de la superficie (hombro perpendicular a ella, o el brazo entero estirado hacia
    afuera) y luego se va al destino. Inalcanzable si ni así: mejor no moverlo a ciegas.
    """
    desde, hasta = [float(v) for v in desde[:4]], [float(v) for v in hasta[:4]]

    def partir(a, b):
        n = max(1, int(np.ceil(max(abs(x - y) for x, y in zip(a, b)) / paso)))
        return [[x + (y - x) * k / n for x, y in zip(a, b)] for k in range(1, n + 1)]

    # Escalas, de menos a más rodeo: hombro perpendicular a la superficie y, si hace falta, el brazo
    # entero estirado hacia afuera (lo más lejos posible de la pared, la mesa y la base).
    separado = list(desde)
    separado[1] = MAPEO["hombro"][0] + MAPEO["hombro"][1] * 90.0
    base = (desde[0] - MAPEO["base"][0]) / MAPEO["base"][1]
    estirado = [min(180.0, max(0.0, s)) for s in
                servos_desde_geo({"base": base, "hombro": 90.0, "codo": 0.0, "muneca": 0.0})]
    for escalas in ([], [separado], [estirado], [separado, estirado]):
        puntos = [desde] + escalas + [hasta]
        if all(0.0 <= v <= 180.0 for p in escalas for v in p) and all(
                tramo_seguro(a, b) for a, b in zip(puntos, puntos[1:])):
            return [q for a, b in zip(puntos, puntos[1:]) for q in partir(a, b)]
    raise Inalcanzable("no encuentro un camino seguro hasta esa postura: aleja el brazo de la pared (servo o ir) y repite")



def rangos_seguimiento():
    """Rango de servo permitido a cada articulación mientras el robot te sigue: [lo, hi] por articulación.

    ARTICULAR_LIMITES, dentro de los servos (0..180), y para base y hombro además como mucho
    ARTICULAR_LIMITE_HOMBRO grados desde el estado inicial (flexión/extensión y abducción).
    """
    inicial = postura_inicial()
    rangos = []
    for i, nombre in enumerate(ARTICULACIONES_SERVO):
        lo, hi = config.ARTICULAR_LIMITES[nombre]
        lo, hi = max(0.0, float(lo)), min(180.0, float(hi))
        if nombre in ("base", "hombro") and config.ARTICULAR_LIMITE_HOMBRO is not None:
            margen = abs(MAPEO[nombre][1]) * float(config.ARTICULAR_LIMITE_HOMBRO)    # grados -> servo
            lo, hi = max(lo, inicial[i] - margen), min(hi, inicial[i] + margen)
        rangos.append((lo, hi))
    return rangos


def limitar_seguimiento(servos, actual=None, rangos=None):
    """Servos recortados a rangos_seguimiento() antes de enviarlos al robot.

    actual: si el robot ya está fuera de un rango (lo movió la consola), puede volver hacia él pero
    nunca alejarse más; así el recorte no provoca un salto.
    """
    rangos = rangos_seguimiento() if rangos is None else rangos
    salida = []
    for i, (v, (lo, hi)) in enumerate(zip(servos[:4], rangos)):
        if actual is not None:
            lo, hi = min(lo, actual[i]), max(hi, actual[i])
        salida.append(min(hi, max(lo, float(v))))
    return salida + [float(v) for v in servos[4:]]
