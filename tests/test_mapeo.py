from math import log

import pytest

from brazo_robot.seguimiento.mapeo import FiltroUnEuro, Mano, MapeoRelativo, Pinza, limitar_paso, medir_mano

LIMITES = ((120.0, 260.0), (-150.0, 150.0), (0.0, 60.0))


def mano(x=0.5, y=0.5, escala=0.2, pinza=1.0):
    return Mano(x, y, escala, pinza, [])


def mapeo_listo(pose=(200.0, 0.0, 30.0)):
    # Filtro casi transparente: aquí se prueba el mapeo, no el suavizado.
    m = MapeoRelativo((350.0, 600.0, 250.0), LIMITES, (1000.0, 0.0), (1000.0, 0.0))
    m.enganchar(mano(), pose, 0.0)
    return m


def test_enganchar_no_produce_salto():
    assert mapeo_listo().objetivo(mano(), 0.1) == pytest.approx((200.0, 0.0, 30.0))


@pytest.mark.parametrize("cambio, eje, signo", [
    (dict(x=0.45), 1, +1),       # mano a la izquierda de la pantalla -> Y+
    (dict(x=0.55), 1, -1),
    (dict(y=0.45), 2, +1),       # mano arriba -> subir
    (dict(y=0.55), 2, -1),
    (dict(escala=0.22), 0, +1),  # más cerca de la cámara -> adelante
    (dict(escala=0.18), 0, -1),
])
def test_cada_eje_responde_en_su_sentido(cambio, eje, signo):
    base = (200.0, 0.0, 30.0)
    objetivo = mapeo_listo().objetivo(mano(**cambio), 0.5)
    assert (objetivo[eje] - base[eje]) * signo > 1.0
    otros = [i for i in range(3) if i != eje]
    assert [objetivo[i] for i in otros] == pytest.approx([base[i] for i in otros], abs=1e-6)


def test_escala_es_logaritmica():
    objetivo = mapeo_listo().objetivo(mano(escala=0.2 * 1.1), 0.5)
    assert objetivo[0] == pytest.approx(200.0 + 350.0 * log(1.1), abs=0.05)


def test_los_limites_nunca_se_superan():
    assert mapeo_listo().objetivo(mano(x=0.0, y=1.0, escala=2.0), 0.5) == pytest.approx((260.0, 150.0, 0.0))
    assert mapeo_listo().objetivo(mano(x=1.0, y=0.0, escala=0.01), 0.5) == pytest.approx((120.0, -150.0, 60.0))


def test_limitar_paso():
    assert limitar_paso((0, 0, 0), (3, 4, 0), 10) == (3, 4, 0)
    assert limitar_paso((0, 0, 0), (30, 40, 0), 10) == pytest.approx((6, 8, 0))


def test_filtro_converge_y_reduce_ruido():
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


def test_pinza_requiere_tiempo_sostenido():
    p = Pinza(0.3, 0.5, 0.2)
    assert not p.actualizar(0.1, 0.0)
    assert not p.actualizar(0.1, 0.1)
    assert p.actualizar(0.1, 0.25)


def test_pinza_cuenta_una_vez_y_hay_que_abrir():
    p = Pinza(0.3, 0.5, 0.2)
    p.actualizar(0.1, 0.0)
    assert p.actualizar(0.1, 0.3)
    assert not p.actualizar(0.1, 0.6)
    assert not p.actualizar(0.4, 0.7)     # zona de histéresis: todavía no se rearma
    assert not p.actualizar(0.1, 0.8)
    assert not p.actualizar(0.1, 1.2)
    p.actualizar(0.8, 1.3)                # abierta del todo
    p.actualizar(0.1, 1.4)
    assert p.actualizar(0.1, 1.7)


def test_pinza_parpadeo_no_cuenta():
    p = Pinza(0.3, 0.5, 0.2)
    p.actualizar(0.1, 0.0)
    p.actualizar(0.8, 0.1)
    assert not p.actualizar(0.1, 0.25)


def test_pinza_ya_cerrada_al_enganchar_no_cuenta():
    p = Pinza(0.3, 0.5, 0.2)
    p.reiniciar(0.1)
    p.actualizar(0.1, 0.0)
    assert not p.actualizar(0.1, 0.5)


def test_medir_mano_centro_escala_y_pinza():
    puntos = [(0.5, 0.5)] * 21
    puntos[0] = (0.5, 0.7)    # muñeca
    puntos[9] = (0.5, 0.5)    # base del dedo medio: palma de 0.2
    puntos[4] = (0.5, 0.4)    # pulgar
    puntos[8] = (0.5, 0.45)   # índice: a 0.05
    m = medir_mano(puntos, 4 / 3)
    assert m.escala == pytest.approx(0.2)
    assert m.pinza == pytest.approx(0.25)
    assert m.x == pytest.approx(0.5)
    assert m.y == pytest.approx(0.54)
