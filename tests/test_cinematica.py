import pytest

from brazo_robot import config
from brazo_robot.ajustes import AJUSTES, aplicar_altura
from brazo_robot.cinematica import Inalcanzable, ik, postura_desde_servos, radio_max_teorico, z_transito


def test_angulos_conocidos(monkeypatch):
    # (200, 0, 30) con el efector vertical y los 90 mm del electroimán: base al centro, resto calculado a mano.
    monkeypatch.setattr(config, "L3", 90.0)
    base, hombro, codo, muneca = ik(200, 0, 30)
    assert base == pytest.approx(90.0)
    assert (hombro, codo, muneca) == pytest.approx((64.8, 96.5, 31.7), abs=0.1)


def test_servo_fuera_de_rango_se_rechaza(monkeypatch):
    monkeypatch.setattr(config, "L3", 90.0)                 # el caso de P3 que se vio con el electroimán
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


@pytest.mark.parametrize("x, y, z, inclinacion", [(200, 0, 30, -90.0), (150, 100, 9, -90.0), (230, -60, 20, -100.0)])
def test_postura_desde_servos_invierte_ik(x, y, z, inclinacion):
    from math import atan2, cos, degrees, hypot, radians, sin
    base, puntos = postura_desde_servos(ik(x, y, z, inclinacion))
    assert base == pytest.approx(degrees(atan2(y, x)))
    assert puntos[0] == pytest.approx((0.0, 45.0))
    r, zg = puntos[-1]
    assert (r, zg) == pytest.approx((hypot(x, y), z), abs=1e-6)
    muneca = puntos[2]
    assert muneca == pytest.approx((r - config.L3 * cos(radians(inclinacion)), z - config.L3 * sin(radians(inclinacion))), abs=1e-6)


def test_la_garra_mide_12_cm_por_defecto():
    assert config.LONGITUD_GARRA == 120.0


def test_la_longitud_del_iman_guardada_ya_no_se_usa(tmp_path, monkeypatch):
    import json
    from brazo_robot import ajustes
    archivo = tmp_path / "ajustes.json"
    archivo.write_text(json.dumps({"longitud_iman": 90, "z_recogida": 9.0, "velocidad": 2.0}))
    monkeypatch.setattr(ajustes, "AJUSTES_ARCHIVO", str(archivo))
    monkeypatch.setattr(config, "L3", config.L3)
    monkeypatch.setattr(config, "Z_RECOGIDA", config.Z_RECOGIDA)
    monkeypatch.setattr(config, "BAJADA_EXTRA_AGARRE", config.BAJADA_EXTRA_AGARRE)
    for k in ("longitud_iman", "longitud_garra", "z_recogida", "velocidad"):
        monkeypatch.setitem(ajustes.AJUSTES, k, ajustes.AJUSTES.get(k))
    ajustes.AJUSTES.pop("longitud_garra")
    ajustes.cargar_ajustes()
    assert config.L3 == 120.0 and "longitud_iman" not in ajustes.AJUSTES
    ajustes.AJUSTES["longitud_garra"] = 118.0
    ajustes.cargar_ajustes()
    assert config.L3 == 118.0
