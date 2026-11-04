import pytest

from brazo_robot import config
from brazo_robot.seguimiento.control import PAUSADO, SIGUIENDO, ControlSeguimiento
from brazo_robot.seguimiento.mapeo import Mano


class BrazoFalso:
    def __init__(self):
        self.ultimo = [90.0, 120.0, 97.0, 97.0, 0]
        self.velocidad = 2.0
        self.continuos = []
        self.bloqueantes = []

    def ajustar_velocidad(self, v, guardar=True):
        self.velocidad = v

    def ir(self, x, y, z, iman, inclinacion=-90.0):
        self.bloqueantes.append((x, y, z, iman))

    def mover_continuo(self, valores):
        self.continuos.append(list(valores))
        self.ultimo = list(valores)


def mano(x=0.5, y=0.5, escala=0.2, pinza=1.0):
    return Mano(x, y, escala, pinza, [])


@pytest.fixture
def control(monkeypatch):
    monkeypatch.setattr(config, "Z_RECOGIDA", 0.0)
    return ControlSeguimiento(BrazoFalso(), -90.0)


def enganchado(control, t=0.0):
    control.activar(None, t)         # primera vez: va al punto de partida
    control.activar(mano(), t)       # segunda vez: empieza a seguir
    assert control.estado == SIGUIENDO
    return control


def test_primer_activar_solo_va_al_inicio_y_no_sigue(control):
    control.activar(mano(), 0.0)
    assert control.estado == PAUSADO
    assert len(control.brazo.bloqueantes) == 1
    assert control.brazo.continuos == []


def test_no_engancha_sin_mano(control):
    control.activar(None, 0.0)
    control.activar(None, 0.1)
    assert control.estado == PAUSADO


def test_pausado_nunca_mueve(control):
    control.activar(None, 0.0)
    for i in range(20):
        control.actualizar(mano(x=0.1 * (i % 5)), i * 0.1)
    assert control.brazo.continuos == []


def test_siguiendo_mueve_hacia_la_mano(control):
    enganchado(control)
    y0 = control.pose[1]
    x = 0.5
    for i in range(1, 40):
        x -= 0.005
        control.actualizar(mano(x=x), i * 0.05)
    assert control.brazo.continuos
    assert control.pose[1] > y0 + 10


def test_mano_quieta_no_envia_ordenes(control):
    enganchado(control)
    for i in range(1, 40):
        control.actualizar(mano(), i * 0.05)
    assert control.brazo.continuos == []


def test_mano_perdida_pausa_y_detiene(control):
    enganchado(control)
    control.actualizar(mano(), 0.1)
    control.actualizar(None, 0.2)
    assert control.estado == SIGUIENDO      # un parpadeo no detiene
    control.actualizar(None, 0.8)
    assert control.estado == PAUSADO
    enviadas = len(control.brazo.continuos)
    control.actualizar(mano(x=0.1), 0.7)
    assert len(control.brazo.continuos) == enviadas


def test_salto_de_la_mano_pausa_sin_mover(control):
    enganchado(control)
    control.actualizar(mano(x=0.9), 0.1)
    assert control.estado == PAUSADO
    assert control.brazo.continuos == []


def test_velocidad_limitada_tras_atasco(control):
    enganchado(control)
    x0 = control.pose
    control.actualizar(mano(x=0.3), 5.0)    # 5 s sin actualizar y objetivo lejano
    avance = sum((a - b) ** 2 for a, b in zip(control.pose, x0)) ** 0.5
    assert avance <= config.SEGUIR_VELOCIDAD_MM_S * 4 * config.SEGUIR_PERIODO + 1e-6


def test_z_nunca_baja_de_la_recogida(control):
    enganchado(control)
    y = 0.5
    for i in range(1, 400):
        y = min(0.95, y + 0.01)
        control.actualizar(mano(y=y), i * 0.05)
    assert control.pose[2] >= config.Z_RECOGIDA


def test_pinza_alterna_iman_y_perder_mano_no_lo_apaga(control):
    enganchado(control)
    control.actualizar(mano(pinza=0.1), 0.1)
    control.actualizar(mano(pinza=0.1), 0.35)
    assert control.iman_encendido
    control.actualizar(None, 1.0)
    assert control.estado == PAUSADO and control.iman_encendido


def test_reanudar_no_produce_salto(control):
    enganchado(control)
    control.pausar("prueba")
    pose = control.pose
    control.activar(mano(x=0.2, y=0.8), 2.0)
    assert control.estado == SIGUIENDO
    for i in range(1, 10):
        control.actualizar(mano(x=0.2, y=0.8), 2.0 + i * 0.05)
    assert control.pose == pytest.approx(pose)


def test_objetivo_inalcanzable_mantiene_la_ultima_pose(control, monkeypatch):
    enganchado(control)
    from brazo_robot.seguimiento import control as modulo
    from brazo_robot.cinematica import Inalcanzable

    def rechaza(*a):
        raise Inalcanzable("servo fuera de rango")
    monkeypatch.setattr(modulo, "ik", rechaza)
    pose = control.pose
    control.actualizar(mano(x=0.4), 0.1)
    assert control.pose == pose and control.brazo.continuos == []
    assert "Límite" in control.aviso


def test_activar_y_pausar_no_alternan(control):
    enganchado(control)
    control.activar(mano(), 1.0)            # repetido por el retardo de red: sigue siguiendo
    assert control.estado == SIGUIENDO
    control.pausar("a")
    control.pausar("b")
    assert control.estado == PAUSADO


def test_home_solo_en_pausa_y_con_iman_apagado(control):
    llamadas = []
    control.brazo.home = lambda: llamadas.append(1)
    enganchado(control)
    control.ir_a_home()
    assert not llamadas and "Pausa" in control.aviso
    control.pausar("x")
    control.alternar_iman()
    control.ir_a_home()
    assert not llamadas and "imán" in control.aviso
    control.alternar_iman()
    control.ir_a_home()
    assert llamadas and control.pose is None
