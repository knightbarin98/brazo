from math import log

import pytest

from brazo_robot.seguimiento.mapeo import (MODO_BRAZO, FiltroUnEuro, MapeoRelativo, SeguidorGarra, a_robot,
                                           apertura_de_ratio, cabeceo_mano, limitar_paso, medir_brazo, medir_mano,
                                           proyectar_alcanzable, ratio_apertura)

LIMITES = ((120.0, 260.0), (-150.0, 150.0), (0.0, 60.0))
TRANSPARENTE = ((1000.0, 0.0),) * 3      # filtro casi transparente: aquí se prueba el mapeo


def mapeo_listo(pose=(200.0, 0.0, 30.0), mm=(350.0, 600.0, 250.0)):
    m = MapeoRelativo(LIMITES, TRANSPARENTE)
    m.enganchar((0.0, 0.0, 0.0), pose, mm, 0.0)
    return m


def test_enganchar_no_produce_salto():
    assert mapeo_listo().objetivo((0.0, 0.0, 0.0), 0.1) == pytest.approx((200.0, 0.0, 30.0))


@pytest.mark.parametrize("eje", [0, 1, 2])
def test_cada_eje_responde_a_su_componente(eje):
    v = [0.0, 0.0, 0.0]
    v[eje] = 0.05
    base = (200.0, 0.0, 30.0)
    objetivo = mapeo_listo().objetivo(tuple(v), 0.5)
    assert objetivo[eje] > base[eje] + 1
    for i in range(3):
        if i != eje:
            assert objetivo[i] == pytest.approx(base[i], abs=1e-6)


def test_los_limites_nunca_se_superan():
    assert mapeo_listo().objetivo((9.0, 9.0, -9.0), 0.5) == pytest.approx((260.0, 150.0, 0.0))
    assert mapeo_listo().objetivo((-9.0, -9.0, 9.0), 0.5) == pytest.approx((120.0, -150.0, 60.0))


def test_cambiar_factor_no_salta_y_escala_lo_que_sigue():
    m = mapeo_listo()
    pose = m.objetivo((0.0, 0.05, 0.0), 0.5)
    m.cambiar_factor(2.0, pose)
    assert m.objetivo((0.0, 0.05, 0.0), 0.6) == pytest.approx(pose, abs=0.05)
    assert m.objetivo((0.0, 0.06, 0.0), 0.7)[1] == pytest.approx(pose[1] + 2.0 * 600.0 * 0.01, abs=0.1)


def test_limitar_paso():
    assert limitar_paso((0, 0, 0), (3, 4, 0), 10) == (3, 4, 0)
    assert limitar_paso((0, 0, 0), (30, 40, 0), 10) == pytest.approx((6, 8, 0))


def test_proyectar_devuelve_el_objetivo_si_llega():
    assert proyectar_alcanzable((0, 0, 0), (5, 0, 0), lambda p: True) == (5, 0, 0)


def test_proyectar_se_queda_en_el_borde():
    p = proyectar_alcanzable((0.0, 0.0, 0.0), (20.0, 0.0, 0.0), lambda q: q[0] <= 10.0)
    assert p[0] == pytest.approx(10.0, abs=0.1) and p[0] <= 10.0


def test_proyectar_sin_centro_alcanzable_es_none():
    assert proyectar_alcanzable((0, 0, 0), (9, 0, 0), lambda q: False) is None


def test_proyectar_desliza_por_el_borde():
    # Círculo de radio 10: con objetivos fuera y a distintos ángulos, el punto recorre el borde.
    dentro = lambda q: q[0] ** 2 + q[1] ** 2 <= 100.0   # noqa: E731
    a = proyectar_alcanzable((0.0, 0.0, 0.0), (20.0, 0.0, 0.0), dentro)
    b = proyectar_alcanzable((0.0, 0.0, 0.0), (14.0, 14.0, 0.0), dentro)
    assert a[1] == pytest.approx(0.0)
    assert b[0] == pytest.approx(b[1]) and 6.5 < b[0] <= 7.1


def test_filtro_converge():
    f = FiltroUnEuro(1.0, 0.0)
    f(0.0, 0.0)
    primero = f(1.0, 0.033)
    assert 0.0 < primero < 0.5
    for i in range(2, 200):
        ultimo = f(1.0, i * 0.033)
    assert ultimo == pytest.approx(1.0, abs=0.01)


def test_filtro_ignora_tiempo_repetido():
    f = FiltroUnEuro(1.0, 0.0)
    f(0.5, 1.0)
    assert f(9.0, 1.0) == 0.5


# ---------------------------------------------------------------- medidas de la mano
def mano_3d(pulgar_a_dedos):
    """21 puntos 3D: palma de 0.1 m y el pulgar a la distancia pedida de las otras puntas."""
    mundo = [(0.0, 0.0, 0.0)] * 21
    mundo[9] = (0.0, -0.1, 0.0)
    for i in (8, 12, 16, 20):
        mundo[i] = (0.0, -0.15, 0.0)
    mundo[4] = (pulgar_a_dedos, -0.15, 0.0)
    return mundo


def test_ratio_apertura_en_unidades_de_palma():
    assert ratio_apertura(mano_3d(0.05)) == pytest.approx(0.5)


def test_ratio_palma_degenerada():
    assert ratio_apertura([(0.0, 0.0, 0.0)] * 21) is None


def test_apertura_de_ratio_calibrada_y_limitada():
    assert apertura_de_ratio(0.6, 1.0, 0.2) == pytest.approx(0.5)
    assert apertura_de_ratio(5.0, 1.0, 0.2) == 1.0
    assert apertura_de_ratio(0.0, 1.0, 0.2) == 0.0
    assert apertura_de_ratio(None, 1.0, 0.2) is None
    assert apertura_de_ratio(0.5, 0.2, 0.2) is None


def test_a_robot_sentidos_sobre_imagen_en_espejo():
    adelante, izquierda, arriba = a_robot((-0.1, -0.2, -0.3))  # a la izquierda de la pantalla, arriba, hacia la cámara
    assert adelante > 0 and izquierda > 0 and arriba > 0


def test_cabeceo_mano():
    assert cabeceo_mano(mano_3d(0.0)) == pytest.approx(90.0)   # muñeca -> nudillo: y negativa = arriba


def test_medir_mano():
    puntos = [(0.5, 0.5)] * 21
    puntos[0] = (0.5, 0.7)
    m = medir_mano(puntos, 4 / 3, mano_3d(0.05), abierta=1.0, cerrada=0.0)
    assert m.vector[0] == pytest.approx(log(0.2))
    assert m.vector[1] == pytest.approx(-m.x) and m.vector[2] == pytest.approx(-m.y)
    assert m.apertura == pytest.approx(0.5)


def test_medir_brazo_vector_y_largo():
    m = medir_brazo((0.0, 0.0, 0.0), (0.3, 0.0, 0.0), (0.3, 0.25, 0.0), (0.4, 0.6))
    assert m.modo == MODO_BRAZO
    assert m.vector == pytest.approx((0.3, 0.25, 0.0))
    assert m.largo == pytest.approx(0.55)
    assert (m.x, m.y) == (0.4, 0.6)


# ---------------------------------------------------------------- la garra sigue a la mano
def seguidor():
    return SeguidorGarra(15.0, (1000.0, 0.0))


def test_garra_sin_apertura_no_se_mueve():
    assert seguidor().actualizar(None, 0.0, 50.0, 100.0, 0.0) is None


def test_garra_cerca_del_angulo_actual_sigue_de_inmediato():
    s = seguidor()
    assert s.actualizar(0.55, 0.0, 50.0, 100.0, 0.0) == pytest.approx(55.0)
    assert s.sigue


def test_mano_abierta_no_suelta_la_pieza_hasta_alcanzar_la_garra():
    s = seguidor()
    assert s.actualizar(1.0, 0.0, 0.0, 100.0, 0.0) is None    # garra cerrada, mano abierta: quieta
    assert s.actualizar(0.8, 0.1, 0.0, 100.0, 0.0) is None
    assert not s.sigue
    assert s.actualizar(0.1, 0.2, 0.0, 100.0, 0.0) is not None
    assert s.sigue


def test_movimiento_rapido_que_cruza_el_angulo_tambien_engancha():
    s = seguidor()
    assert s.actualizar(0.9, 0.0, 50.0, 100.0, 0.0) is None
    assert s.actualizar(0.1, 0.1, 50.0, 100.0, 0.0) is not None   # pasó de 90 a 10 sin parar cerca de 50


def test_soltar_deja_de_seguir():
    s = seguidor()
    s.actualizar(0.5, 0.0, 50.0, 100.0, 0.0)
    s.soltar()
    assert not s.sigue
    assert s.actualizar(1.0, 0.1, 0.0, 100.0, 0.0) is None


def test_garra_con_angulos_invertidos():
    assert seguidor().actualizar(1.0, 0.0, 5.0, 0.0, 100.0) == pytest.approx(0.0)   # abierta=0°, cerrada=100°


# ---------------------------------------------------------------- modo articular
from brazo_robot.seguimiento.mapeo import angulos_articulares, envolver  # noqa: E402

HOMBRO = (0.0, 0.0, 0.0)
# Puntos en (adelante, izquierda, arriba). Brazo derecho en la imagen en espejo: sale hacia Y- (afuera=-1).


@pytest.mark.parametrize("codo, muneca, mano, esperado", [
    ((0, -0.3, 0), (0, -0.55, 0), (0, -0.1, 0), (0, 0, 0)),         # extendido hacia afuera
    ((0, 0, 0.3), (0, 0, 0.55), (0, 0, 0.1), (90, 0, 90)),          # hacia arriba
    ((0, 0, -0.3), (0, 0, -0.55), (0, 0, -0.1), (-90, 0, -90)),     # colgando
    ((0, -0.3, 0), (0, -0.3, 0.25), (0, 0, 0.1), (0, 90, 90)),      # afuera y antebrazo arriba
    ((0, -0.3, 0), (0, -0.3, -0.25), (0, 0, -0.1), (0, -90, -90)),  # afuera y antebrazo abajo
    ((0, 0.3, 0), (0, 0.55, 0), None, (180, 0, None)),              # cruzado hacia el otro lado
])
def test_angulos_en_el_plano_frontal(codo, muneca, mano, esperado):
    base, *resto = angulos_articulares(HOMBRO, codo, muneca, mano)
    assert base is None                                             # la base no sigue a la persona
    for valor, objetivo in zip(resto, esperado):
        if objetivo is None:
            assert valor is None
        else:
            assert envolver(valor - objetivo) == pytest.approx(0, abs=0.5)


def test_la_profundidad_no_cuenta():
    plano = angulos_articulares(HOMBRO, (0, -0.3, 0), (0, -0.3, 0.25))
    con_profundidad = angulos_articulares(HOMBRO, (0.2, -0.3, 0), (-0.1, -0.3, 0.25))
    assert con_profundidad == pytest.approx(plano)


def test_brazo_izquierdo_sale_hacia_el_otro_lado():
    assert angulos_articulares(HOMBRO, (0, 0.3, 0.3), (0, 0.6, 0.3), afuera=+1)[1] == pytest.approx(45)


def test_envolver():
    assert envolver(190) == pytest.approx(-170)
    assert envolver(-180) == pytest.approx(180)
    assert envolver(45) == pytest.approx(45)
