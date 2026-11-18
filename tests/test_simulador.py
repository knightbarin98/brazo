"""El simulador: separado del control físico y con el mismo control que el servidor."""

import subprocess
import sys

import pytest

from brazo_robot import config
from brazo_robot.seguimiento.control import SIGUIENDO, ControlSeguimiento
from brazo_robot.seguimiento.mapeo import MODO_ARTICULAR, Mano
from brazo_robot.simulador import BrazoVirtual, _tabla


def test_el_simulador_no_carga_el_modulo_serial():
    codigo = "import sys, brazo_robot.simulador; print('serial' in sys.modules)"
    salida = subprocess.run([sys.executable, "-c", codigo], capture_output=True, text=True, check=True)
    assert salida.stdout.strip() == "False"


def test_robot_virtual_recorta_como_el_real():
    b = BrazoVirtual()
    assert b.mover_continuo([200, -5, 90, 90, 250]) is True
    assert b.ultimo == [180.0, 0.0, 90.0, 90.0, config.GARRA_MAX_GRADOS]


def test_el_control_mueve_el_robot_virtual(monkeypatch):
    monkeypatch.setattr(config, "Z_RECOGIDA", 0.0)
    monkeypatch.setattr(config, "MONTAJE", "de_pie")          # de pie, brazo arriba = hombro a 90
    control = ControlSeguimiento(BrazoVirtual(), -90.0)
    m = Mano(MODO_ARTICULAR, 0.5, 0.5, (0.0, 0.0, 0.0), largo=0.55, angulos=(None, 0.0, 0.0, None),
             brazo3d=((0.0, 0.0, 0.3), (0.0, 0.0, 0.25), None))      # brazo hacia arriba
    control.activar(m, 0.0)
    for i in range(1, 200):
        control.actualizar(m, i * config.SEGUIR_PERIODO)
    assert control.estado == SIGUIENDO
    assert control.brazo.ultimo[1] == pytest.approx(90, abs=1.5)
    assert _tabla(control, m, 1280).shape == (118, 1280, 3)
