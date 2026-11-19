"""Correspondencia por articulaciones y límite de 90° del hombro desde el estado inicial (robot al costado).

Al costado, la base gira sobre el eje que sale de la caja (tu eje izquierda-derecha): hace la flexión
y la extensión de tu hombro. El hombro del robot separa el brazo de la pared: la abducción.
"""

from math import cos, radians, sin

import numpy as np
import pytest

from brazo_robot import config
from brazo_robot.cinematica import (MAPEO, limitar_seguimiento, postura_inicial, postura_segura,
                                    rangos_seguimiento, servos_desde_geo)
from brazo_robot.seguimiento.control import ControlSeguimiento
from brazo_robot.seguimiento.gemelo import SeguidorGemelo, a_ejes_robot
from brazo_robot.seguimiento.mapeo import MODO_ARTICULAR, Mano, angulo_entre
from brazo_robot.vista3d import puntos_robot


@pytest.fixture(autouse=True)
def costado(monkeypatch):
    monkeypatch.setattr(config, "MONTAJE", "costado")
    monkeypatch.setattr(config, "ARTICULAR_BASE", 90.0)
    monkeypatch.setattr(config, "ARTICULAR_SIGNO_PROFUNDIDAD", 1.0)
    monkeypatch.setattr(config, "Z_RECOGIDA", 10.0)
    monkeypatch.setattr(config, "ARTICULAR_LIMITE_ABAJO", None)
    monkeypatch.setattr(config, "GEMELO_CORRESPONDENCIA", "articulaciones")
    monkeypatch.setattr(config, "ARTICULAR_LIMITE_HOMBRO", 90.0)


def unitario(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def brazo_humano(flexion=0.0, abduccion=0.0):
    """Dirección de tu brazo (afuera, adelante, arriba) desde colgando: primero la flexión (hacia
    adelante, negativa = extensión hacia atrás) y luego la abducción (separarlo del cuerpo)."""
    f, a = radians(flexion), radians(abduccion)
    plano = np.array((0.0, sin(f), -cos(f)))             # colgando, girado hacia adelante
    return np.array((sin(a), 0.0, 0.0)) + cos(a) * plano


def lectura(brazo, antebrazo, mano=None):
    return Mano(MODO_ARTICULAR, 0.5, 0.5, (0.0, 0.0, 0.0), largo=0.55, angulos=(None, 0.0, 0.0, None),
                brazo3d=(tuple(brazo), tuple(antebrazo), None if mano is None else tuple(mano)))


def recto(d, mano=True):
    d = unitario(d)
    return lectura(0.3 * d, 0.25 * d, 0.08 * d if mano else None)


def estable(m, servos=None, segundos=10.0):
    servos = postura_inicial() if servos is None else servos
    s = SeguidorGemelo([m], 0.0, servos)
    t, actual = 0.0, list(servos)
    for _ in range(int(segundos / 0.05)):
        t += 0.05
        actual, _ = s.objetivo(m, t, actual)
    return actual, s


def geo(servos):
    return {k: (v - MAPEO[k][0]) / MAPEO[k][1] for k, v in zip(("base", "hombro", "codo", "muneca"), servos)}


# ---------------------------------------------------------------- hombro: flexión, extensión, abducción
def test_brazo_colgando_es_el_estado_inicial():
    servos, s = estable(recto(brazo_humano()))
    assert servos == pytest.approx(postura_inicial(), abs=0.5)
    assert s.info["topes"] == []


@pytest.mark.parametrize("flexion", [30.0, 60.0, 90.0])
def test_flexion_y_extension_las_hace_la_base(flexion):
    adelante, _ = estable(recto(brazo_humano(flexion=flexion)))
    atras, _ = estable(recto(brazo_humano(flexion=-flexion)))
    assert adelante[0] - 90.0 == pytest.approx(flexion, abs=0.5)
    assert atras[0] - 90.0 == pytest.approx(-flexion, abs=0.5)
    for servos in (adelante, atras):                       # sin abducción: el hombro no se separa
        assert geo(servos)["hombro"] == pytest.approx(0.0, abs=0.5)
        assert geo(servos)["codo"] == pytest.approx(0.0, abs=0.5)


@pytest.mark.parametrize("abduccion", [30.0, 60.0, 89.0])
def test_abduccion_la_hace_el_hombro(abduccion):
    servos, _ = estable(recto(brazo_humano(abduccion=abduccion)))
    assert geo(servos)["hombro"] == pytest.approx(abduccion, abs=0.5)
    assert servos[0] == pytest.approx(90.0, abs=0.5)       # la base no se mueve


@pytest.mark.parametrize("flexion, abduccion", [(150.0, 0.0), (-130.0, 0.0), (0.0, 150.0), (120.0, 40.0)])
def test_mas_alla_de_90_grados_se_queda_en_el_limite(flexion, abduccion):
    servos, s = estable(recto(brazo_humano(flexion=flexion, abduccion=abduccion)))
    g, inicial = geo(servos), geo(postura_inicial())
    assert abs(g["base"] - inicial["base"]) <= 90.0 + 1e-6
    assert -1e-6 <= g["hombro"] - inicial["hombro"] <= 90.0 + 1e-6
    assert all(0.0 <= v <= 180.0 for v in servos)
    assert s.info["topes"]                                  # y se avisa que está en el tope


def test_el_brazo_del_robot_apunta_como_tu_brazo_dentro_de_los_90_grados():
    comprobadas = 0
    for flexion in range(-80, 81, 20):
        for abduccion in range(0, 71, 10):
            d = brazo_humano(flexion=flexion, abduccion=abduccion)
            servos, s = estable(recto(d))
            if s.info["brazo_vertical"]:
                continue
            comprobadas += 1
            _, hombro, codo, _, _ = (np.array(p) for p in puntos_robot(servos + [0]))
            esperado = unitario(a_ejes_robot(tuple(d)))
            assert unitario(codo - hombro) @ esperado > 0.9995, (flexion, abduccion)
    assert comprobadas > 60


# ---------------------------------------------------------------- codo y muñeca
def test_el_codo_copia_tu_flexion():
    brazo = brazo_humano(abduccion=60.0)
    # antebrazo doblado 50° dentro del plano del brazo y el eje 'afuera' (el plano del robot), hacia arriba
    eje = unitario(np.cross(brazo, (1.0, 0.0, 0.0)))
    f = radians(50.0)
    antebrazo = brazo * cos(f) + np.cross(eje, brazo) * sin(f)
    servos, _ = estable(lectura(0.3 * brazo, 0.25 * unitario(antebrazo), 0.08 * unitario(antebrazo)))
    assert geo(servos)["codo"] == pytest.approx(-50.0, abs=0.5)
    assert geo(servos)["hombro"] == pytest.approx(60.0, abs=0.5)   # el hombro sigue copiando tu brazo


@pytest.mark.parametrize("doblez", [-35.0, 25.0])
def test_la_muneca_copia_tu_mano_respecto_al_antebrazo(doblez):
    brazo = unitario(brazo_humano(abduccion=60.0))
    eje = unitario(np.cross(brazo, (1.0, 0.0, 0.0)))
    d = radians(doblez)
    mano = brazo * cos(d) - np.cross(eje, brazo) * sin(d)
    servos, _ = estable(lectura(0.3 * brazo, 0.25 * brazo, 0.08 * mano))
    _, _, _, muneca, garra = (np.array(p) for p in puntos_robot(servos + [0]))
    _, hombro, codo, _, _ = (np.array(p) for p in puntos_robot(servos + [0]))
    assert angulo_entre(garra - muneca, codo - hombro) == pytest.approx(abs(doblez), abs=0.5)
    assert abs(geo(servos)["muneca"]) == pytest.approx(abs(doblez), abs=0.5)


def test_con_el_brazo_colgando_y_el_codo_doblado_el_hombro_se_separa_de_la_pared():
    # Colgando, el codo del robot sólo puede doblarse hacia la pared: el hombro se separa lo justo.
    brazo = unitario(brazo_humano())
    antebrazo = unitario(brazo_humano(flexion=90.0))
    servos, s = estable(lectura(0.3 * brazo, 0.25 * antebrazo))
    assert postura_segura(servos)
    assert s.info["hombro_ajustado"]
    assert geo(servos)["codo"] == pytest.approx(-90.0, abs=0.5)    # la flexión se conserva
    assert 0.0 < geo(servos)["hombro"] <= 90.0


# ---------------------------------------------------------------- límites antes de enviar
def test_rangos_dentro_de_los_servos_y_de_los_90_grados(monkeypatch):
    rangos = rangos_seguimiento()
    inicial = postura_inicial()
    assert rangos[0] == pytest.approx((0.0, 180.0))        # base 90 ± 90
    assert rangos[1] == pytest.approx((0.0, 90.0))         # hombro 0..90 (la pared no deja bajar de 0)
    for (lo, hi), v in zip(rangos, inicial):
        assert 0.0 <= lo <= v <= hi <= 180.0
    monkeypatch.setattr(config, "ARTICULAR_BASE", 120.0)   # base calibrada en otro servo
    assert rangos_seguimiento()[0] == pytest.approx((30.0, 180.0))
    monkeypatch.setattr(config, "ARTICULAR_LIMITES", dict(config.ARTICULAR_LIMITES, hombro=(0.0, 60.0)))
    assert rangos_seguimiento()[1] == pytest.approx((0.0, 60.0))


def test_limitar_nunca_aleja_mas_y_no_salta():
    rangos = [(0.0, 180.0), (0.0, 90.0), (0.0, 180.0), (0.0, 180.0)]
    assert limitar_seguimiento([90, 130, 20, 90, 50], rangos=rangos) == [90, 90, 20, 90, 50]
    # el robot ya estaba en 120 (lo movió la consola): puede bajar, pero no subir más
    assert limitar_seguimiento([90, 130, 20, 90], actual=[90, 120, 20, 90], rangos=rangos)[1] == 120
    assert limitar_seguimiento([90, 110, 20, 90], actual=[90, 120, 20, 90], rangos=rangos)[1] == 110


class BrazoFalso:
    def __init__(self, servos):
        self.ultimo = list(servos) + [60.0]
        self.velocidad = 2.0
        self.continuos = []

    def ajustar_velocidad(self, v, guardar=True):
        self.velocidad = v

    def mover_continuo(self, valores):
        self.continuos.append(list(valores))
        self.ultimo = list(valores)
        return True


@pytest.mark.parametrize("flexion, abduccion", [(170.0, 0.0), (0.0, 170.0), (-170.0, 30.0)])
def test_el_control_nunca_envia_mas_alla_del_limite(flexion, abduccion):
    m = recto(brazo_humano(flexion=flexion, abduccion=abduccion))
    control = ControlSeguimiento(BrazoFalso(postura_inicial()), -90.0)
    control.activar(m, 0.0)
    for i in range(1, 400):
        control.actualizar(m, i * config.SEGUIR_PERIODO)
    assert control.brazo.continuos
    rangos = rangos_seguimiento()
    for orden in control.brazo.continuos:
        assert all(lo - 1e-6 <= v <= hi + 1e-6 for v, (lo, hi) in zip(orden[:4], rangos))
        assert postura_segura(orden[:4])


def test_el_control_sigue_coordinando_las_articulaciones():
    # brazo hacia adelante y afuera, codo y muñeca doblados: se mueven base, hombro, codo y muñeca
    brazo = unitario(brazo_humano(flexion=40.0, abduccion=50.0))
    eje = unitario(np.cross(brazo, (1.0, 0.0, 0.0)))
    antebrazo = brazo * cos(radians(40)) + np.cross(eje, brazo) * sin(radians(40))
    mano = antebrazo * cos(radians(20)) + np.cross(eje, antebrazo) * sin(radians(20))
    m = lectura(0.3 * brazo, 0.25 * antebrazo, 0.08 * mano)
    inicial = postura_inicial()
    control = ControlSeguimiento(BrazoFalso(inicial), -90.0)
    control.activar(m, 0.0)
    for i in range(1, 600):
        control.actualizar(m, i * config.SEGUIR_PERIODO)
    final = control.brazo.ultimo[:4]
    assert all(abs(a - b) > 10.0 for a, b in zip(final, inicial)), final
    assert final == pytest.approx(servos_desde_geo({"base": 40.0, "hombro": 50.0, "codo": -40.0, "muneca": -20.0}),
                                  abs=1.5)


# ---------------------------------------------------------------- regresión: la base no cambia de lado
@pytest.mark.parametrize("extremo, tope", [(150.0, 180.0), (120.0, 180.0), (-150.0, 0.0), (-110.0, 0.0)])
def test_mas_alla_del_limite_la_base_se_queda_del_mismo_lado_y_vuelve(extremo, tope):
    # Antes, el ángulo de la base se desenrollaba dos veces: 150° hacia adelante acababa 30° hacia atrás
    # y, al volver a colgar, la base se quedaba en el tope (sesión real con dos cámaras).
    inicial = postura_inicial()
    s = SeguidorGemelo([recto(brazo_humano())], 0.0, inicial)
    t, actual, bases = 0.0, list(inicial), []
    for flexion in (0.0, extremo, 0.0):
        for _ in range(100):
            t += 0.05
            actual, _ = s.objetivo(recto(brazo_humano(flexion=flexion)), t, actual)
        bases.append(actual[0])
    assert bases == pytest.approx([90.0, tope, 90.0], abs=0.5)


def test_barrido_continuo_por_todos_los_extremos_vuelve_al_estado_inicial():
    inicial = postura_inicial()
    s = SeguidorGemelo([recto(brazo_humano())], 0.0, inicial)
    camino = ([(f, 0.0) for f in np.linspace(0, 170, 60)] + [(f, 0.0) for f in np.linspace(170, -170, 120)]
              + [(f, 0.0) for f in np.linspace(-170, 0, 60)] + [(0.0, a) for a in np.linspace(0, 170, 60)]
              + [(0.0, a) for a in np.linspace(170, 0, 60)] + [(0.0, 0.0)] * 100)
    t, actual = 0.0, list(inicial)
    for flexion, abduccion in camino:
        t += 0.05
        actual, _ = s.objetivo(recto(brazo_humano(flexion=flexion, abduccion=abduccion)), t, actual)
        assert 0.0 <= actual[0] <= 180.0 and 0.0 <= actual[1] <= 90.0
    assert actual == pytest.approx(inicial, abs=0.5)
