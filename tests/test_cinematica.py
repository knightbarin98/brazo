import pytest

from brazo_robot import config
from brazo_robot.ajustes import AJUSTES, aplicar_altura
from brazo_robot.cinematica import Inalcanzable, ik, radio_max_teorico, z_transito


def test_angulos_conocidos():
    # (200, 0, 30) con la garra vertical: base al centro, resto calculado a mano.
    base, hombro, codo, muneca = ik(200, 0, 30)
    assert base == pytest.approx(90.0)
    assert (hombro, codo, muneca) == pytest.approx((64.8, 96.5, 31.7), abs=0.1)


def test_servo_fuera_de_rango_se_rechaza():
    with pytest.raises(Inalcanzable, match="servo muneca fuera de rango"):
        ik(280, 100, -1, -98.0)


def test_punto_lejano_se_rechaza():
    with pytest.raises(Inalcanzable, match="fuera de alcance"):
        ik(900, 0, 30)


def test_cambio_de_longitud_se_ve_en_todos_los_modulos(monkeypatch):
    # L3 y Z_RECOGIDA cambian en ejecución: ningún módulo debe quedarse con una copia vieja.
    antes = ik(200, 0, 30)
    monkeypatch.setattr(config, "L3", 100.0)
    assert ik(200, 0, 30) != antes


def test_aplicar_altura_actualiza_config(monkeypatch):
    monkeypatch.setitem(AJUSTES, "z_recogida", config.Z_RECOGIDA)
    monkeypatch.setattr(config, "Z_RECOGIDA", config.Z_RECOGIDA)
    monkeypatch.setattr(config, "BAJADA_EXTRA_AGARRE", config.BAJADA_EXTRA_AGARRE)
    radio = radio_max_teorico()
    aplicar_altura(5.0)
    assert config.Z_RECOGIDA == 5.0
    assert radio_max_teorico() != radio


def test_z_transito_baja_la_altura_si_no_alcanza():
    # Lejos, la muñeca no llega a Z_SEGURO con la garra inclinada: la aproximación debe ser menor.
    assert z_transito(250, 100, 0.0, 60.0, -95.0) < 60.0
