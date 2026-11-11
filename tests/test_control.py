import pytest

from brazo_robot import config
from brazo_robot.cinematica import Inalcanzable, ik
from brazo_robot.seguimiento import control as modulo
from brazo_robot.seguimiento.control import ENGANCHANDO, PAUSADO, SIGUIENDO, ControlSeguimiento
from brazo_robot.seguimiento.mapeo import MODO_BRAZO, MODO_MANO, Mano


class BrazoFalso:
    def __init__(self):
        self.ultimo = [90.0, 120.0, 97.0, 97.0, 100.0]
        self.velocidad = 2.0
        self.continuos = []
        self.bloqueantes = []
        self.homes = []
        self.confirmar = True

    def ajustar_velocidad(self, v, guardar=True):
        self.velocidad = v

    def ir(self, x, y, z, garra, inclinacion=-90.0):
        self.bloqueantes.append((x, y, z, garra))

    def home(self, garra=None):
        self.homes.append(garra)

    def mover_continuo(self, valores):
        if not self.confirmar:
            return False
        self.continuos.append(list(valores))
        self.ultimo = list(valores)
        return True


def mano(x=0.5, y=0.5, adelante=0.0, apertura=None, cabeceo=None, modo=MODO_MANO, largo=None):
    return Mano(modo, x, y, (adelante, -x, -y), apertura, cabeceo, largo)


@pytest.fixture
def control(monkeypatch):
    monkeypatch.setattr(config, "Z_RECOGIDA", 0.0)
    return ControlSeguimiento(BrazoFalso(), -90.0)


def enganchado(control, t=0.0, lectura=None):
    lectura = lectura or mano()
    control.activar(None, t)                        # primera vez: va al punto de partida
    control.activar(lectura, t)                     # segunda: toma la referencia
    assert control.estado == ENGANCHANDO
    control.actualizar(lectura, t + config.SEGUIR_TIEMPO_REFERENCIA)
    assert control.estado == SIGUIENDO
    return t + config.SEGUIR_TIEMPO_REFERENCIA


def test_primer_activar_solo_va_al_inicio_y_no_sigue(control):
    control.activar(mano(), 0.0)
    assert control.estado == PAUSADO
    assert len(control.brazo.bloqueantes) == 1
    assert control.brazo.continuos == []


def test_ir_al_inicio_no_cambia_la_garra(control):
    control.brazo.ultimo[4] = 33.0
    control.activar(None, 0.0)
    assert control.brazo.bloqueantes[0][3] == 33.0


def test_no_engancha_sin_lectura(control):
    control.activar(None, 0.0)
    control.activar(None, 0.1)
    assert control.estado == PAUSADO


def test_enganchando_no_mueve_el_brazo(control):
    control.activar(None, 0.0)
    control.activar(mano(), 0.0)
    for i in range(1, 8):
        control.actualizar(mano(x=0.5 - i * 0.01), i * 0.05)
    assert control.brazo.continuos == []


def test_referencia_con_mediana_ignora_una_lectura_con_ruido(control):
    control.activar(None, 0.0)
    control.activar(mano(), 0.0)
    for i in range(1, 11):
        control.actualizar(mano(x=0.5 + (0.2 if i == 4 else 0.0)), i * 0.05)
    assert control.estado == SIGUIENDO
    assert control.mapeo.ref[1] == pytest.approx(-0.5)


def test_perder_datos_al_enganchar_pausa(control):
    control.activar(None, 0.0)
    control.activar(mano(), 0.0)
    control.actualizar(None, 1.0)
    assert control.estado == PAUSADO


def test_modo_brazo_escala_con_el_largo_de_la_persona(control):
    t = enganchado(control, lectura=mano(modo=MODO_BRAZO, largo=0.5))
    assert control.mapeo.mm[0] == pytest.approx((config.L1 + config.L2) / 0.5)
    assert t


def test_pausado_nunca_mueve(control):
    control.activar(None, 0.0)
    for i in range(20):
        control.actualizar(mano(x=0.1 * (i % 5)), i * 0.1)
    assert control.brazo.continuos == []


def test_siguiendo_mueve_hacia_la_mano(control):
    t = enganchado(control)
    y0 = control.pose[1]
    x = 0.5
    for i in range(1, 40):
        x -= 0.005
        control.actualizar(mano(x=x), t + i * 0.05)
    assert control.brazo.continuos
    assert control.pose[1] > y0 + 10


def test_mano_quieta_no_envia_ordenes(control):
    t = enganchado(control)
    for i in range(1, 40):
        control.actualizar(mano(), t + i * 0.05)
    assert control.brazo.continuos == []


def test_perder_datos_pausa_y_detiene(control):
    t = enganchado(control)
    control.actualizar(mano(), t + 0.1)
    control.actualizar(None, t + 0.2)
    assert control.estado == SIGUIENDO      # un parpadeo no detiene
    control.actualizar(None, t + 0.8)
    assert control.estado == PAUSADO
    enviadas = len(control.brazo.continuos)
    control.actualizar(mano(x=0.1), t + 0.9)
    assert len(control.brazo.continuos) == enviadas


def test_al_volver_la_lectura_no_reanuda_solo(control):
    t = enganchado(control)
    control.actualizar(None, t + 1.0)
    for i in range(10):
        control.actualizar(mano(x=0.4), t + 1.1 + i * 0.05)
    assert control.estado == PAUSADO and control.brazo.continuos == []


def test_salto_pausa_sin_mover(control):
    t = enganchado(control)
    control.actualizar(mano(x=0.9), t + 0.1)
    assert control.estado == PAUSADO
    assert control.brazo.continuos == []


def test_cambio_de_modo_pausa(control):
    t = enganchado(control)
    control.actualizar(mano(modo=MODO_BRAZO, largo=0.5), t + 0.1)
    assert control.estado == PAUSADO


def test_velocidad_limitada_tras_atasco(control):
    t = enganchado(control)
    x0 = control.pose
    control.actualizar(mano(x=0.3), t + 5.0)
    avance = sum((a - b) ** 2 for a, b in zip(control.pose, x0)) ** 0.5
    assert avance <= config.SEGUIR_VELOCIDAD_MM_S * 4 * config.SEGUIR_PERIODO + 1e-6


def test_z_nunca_baja_de_la_recogida(control):
    t = enganchado(control)
    y = 0.5
    for i in range(1, 400):
        y = min(0.95, y + 0.01)
        control.actualizar(mano(y=y), t + i * 0.05)
    assert control.pose[2] >= config.Z_RECOGIDA


def test_desliza_por_el_borde_en_vez_de_trabarse(control):
    # Y grande y Z alta: fuera de alcance. El brazo debe llegar al borde y seguir moviéndose en él.
    t = enganchado(control)
    x, trayecto = 0.5, []
    for i in range(1, 200):
        x = max(0.1, x - 0.01)
        control.actualizar(mano(x=x, y=0.5 - min(i, 40) * 0.005), t + i * 0.05)
        trayecto.append(control.pose)
    assert control.estado == SIGUIENDO
    ik(*control.pose, control.inclinacion)   # la pose final sí es alcanzable
    assert len({round(p[1]) for p in trayecto}) > 20


def test_garra_sigue_la_apertura(control):
    t = enganchado(control, lectura=mano(apertura=1.0))
    control.actualizar(mano(apertura=1.0), t + 0.05)      # la mano coincide con la garra abierta: engancha
    for i in range(2, 40):
        control.actualizar(mano(apertura=0.5), t + i * 0.05)
    assert control.garra == pytest.approx(50.0, abs=3)
    assert control.seguidor_garra.sigue


def test_mano_abierta_al_activar_no_suelta_la_pieza(control):
    control.brazo.ultimo[4] = 0.0                  # sostiene una pieza
    t = enganchado(control, lectura=mano(apertura=1.0))
    for i in range(1, 30):
        control.actualizar(mano(apertura=1.0, x=0.5 - i * 0.003), t + i * 0.05)
    assert control.brazo.continuos                 # el brazo sí se mueve
    assert all(v[4] == 0.0 for v in control.brazo.continuos)
    assert "alcance" in control.aviso or "apertura" in control.aviso


def test_perder_la_mano_no_abre_la_garra(control):
    control.brazo.ultimo[4] = 0.0
    t = enganchado(control, lectura=mano(apertura=0.0))
    control.actualizar(None, t + 1.0)
    assert control.estado == PAUSADO and control.garra == 0.0


def test_garra_manual_alterna_y_deja_de_seguir(control):
    control.alternar_garra()
    assert control.garra == 0.0
    control.alternar_garra()
    assert control.garra == 100.0
    assert not control.seguidor_garra.sigue


def test_inclinacion_sigue_a_la_mano_con_tope(control):
    t = enganchado(control, lectura=mano(cabeceo=0.0))
    for i in range(1, 80):
        control.actualizar(mano(cabeceo=60.0), t + i * 0.05)
    assert control.inclinacion == pytest.approx(-90.0 + config.SEGUIR_INCLINACION_LIMITE, abs=1.0)


def test_inclinacion_parte_de_la_calibrada(monkeypatch):
    monkeypatch.setattr(config, "Z_RECOGIDA", 0.0)
    c = ControlSeguimiento(BrazoFalso(), -98.0)
    enganchado(c, lectura=mano(cabeceo=10.0))
    assert c.inclinacion == -98.0


def test_sin_cabeceo_mantiene_la_inclinacion(control):
    t = enganchado(control)
    for i in range(1, 20):
        control.actualizar(mano(x=0.5 - i * 0.003), t + i * 0.05)
    assert control.inclinacion == -90.0


def test_factor_con_limites_y_sin_salto(control):
    t = enganchado(control)
    pose = control.pose
    for _ in range(30):
        control.ajustar_factor(+1)
    assert control.factor == config.SEGUIR_FACTOR_RANGO[1]
    for _ in range(40):
        control.ajustar_factor(-1)
    assert control.factor == config.SEGUIR_FACTOR_RANGO[0]
    control.actualizar(mano(), t + 0.1)
    assert control.pose == pytest.approx(pose)


def test_activar_y_pausar_no_alternan(control):
    t = enganchado(control)
    control.activar(mano(), t + 0.1)        # repetido por el retardo de red: sigue siguiendo
    assert control.estado == SIGUIENDO
    control.pausar("a")
    control.pausar("b")
    assert control.estado == PAUSADO


def test_reanudar_no_produce_salto(control):
    t = enganchado(control)
    control.pausar("prueba")
    pose = control.pose
    control.activar(mano(x=0.2, y=0.8), t + 1.0)
    for i in range(1, 30):
        control.actualizar(mano(x=0.2, y=0.8), t + 1.0 + i * 0.05)
    assert control.estado == SIGUIENDO
    assert control.pose == pytest.approx(pose)


def test_home_solo_en_pausa_y_conserva_la_garra(control):
    enganchado(control)
    control.ir_a_home()
    assert not control.brazo.homes and "Pausa" in control.aviso
    control.pausar("x")
    control.brazo.ultimo[4] = 12.0
    control.ir_a_home()
    assert control.brazo.homes == [12.0] and control.pose is None


def test_objetivo_inalcanzable_mantiene_la_ultima_pose(control, monkeypatch):
    t = enganchado(control)

    def rechaza(*a):
        raise Inalcanzable("servo fuera de rango")
    monkeypatch.setattr(modulo, "ik", rechaza)
    pose = control.pose
    control.actualizar(mano(x=0.4), t + 0.1)
    assert control.pose == pose and control.brazo.continuos == []
    assert "Límite" in control.aviso


def test_orden_sin_confirmar_no_avanza_la_pose(control):
    t = enganchado(control)
    control.brazo.confirmar = False
    pose = control.pose
    control.actualizar(mano(x=0.45), t + 0.1)
    assert control.pose == pose


# ---------------------------------------------------------------- modo articular
from brazo_robot.cinematica import postura_segura  # noqa: E402
from brazo_robot.seguimiento.control import _desenrollar  # noqa: E402
from brazo_robot.seguimiento.mapeo import MODO_ARTICULAR  # noqa: E402


def articular(angulos=(0.0, 45.0, -60.0, 0.0), apertura=None, x=0.5):
    return Mano(MODO_ARTICULAR, x, 0.5, (0.0, 0.0, 0.0), apertura, None, 0.55, angulos=tuple(angulos))


def seguir_hasta(control, lectura, t, segundos):
    pasos = int(segundos / config.SEGUIR_PERIODO)
    for i in range(1, pasos + 1):
        control.actualizar(lectura, t + i * config.SEGUIR_PERIODO)
    return t + pasos * config.SEGUIR_PERIODO


def test_articular_copia_los_angulos_en_cada_motor(control):
    lectura = articular((20.0, 45.0, -60.0, -40.0))     # mano 40° bajo la horizontal
    t = enganchado(control, lectura=lectura)
    seguir_hasta(control, lectura, t, 8.0)
    base, hombro, codo, muneca = control.brazo.ultimo[:4]
    assert base == pytest.approx(90 + 20, abs=1.5)      # MAPEO base (90, +1)
    assert hombro == pytest.approx(45, abs=1.5)         # MAPEO hombro (0, +1)
    assert codo == pytest.approx(60, abs=1.5)           # MAPEO codo (0, -1)
    # antebrazo del robot a 45 - 60 = -15°: la muñeca dobla -25° para que la garra quede a -40°
    assert muneca == pytest.approx(90 - 25, abs=1.5)    # MAPEO muñeca (90, +1)


def test_articular_la_garra_apunta_como_la_mano_aunque_el_hombro_tope(control, monkeypatch):
    lectura = articular((0.0, -90.0, -90.0, 0.0))       # codo pegado al cuerpo, antebrazo y mano al frente
    t = enganchado(control, lectura=lectura)
    monkeypatch.setattr(config, "Z_RECOGIDA", -300.0)   # aquí sólo se prueba la muñeca, no la mesa
    seguir_hasta(control, lectura, t, 10.0)
    base, hombro, codo, muneca = control.brazo.ultimo[:4]
    assert hombro == pytest.approx(0, abs=1.5)          # tope: el robot no baja el hombro de la horizontal
    assert codo == pytest.approx(90, abs=1.5)
    assert muneca == pytest.approx(90 + 90, abs=1.5)    # antebrazo a -90°: la muñeca sube 90 para apuntar al frente


def test_articular_se_acerca_despacio_al_activar(control):
    inicio = list(control.brazo.ultimo[:4])
    lectura = articular((60.0, 45.0, -60.0, 0.0))
    t = enganchado(control, lectura=lectura)
    control.actualizar(lectura, t + config.SEGUIR_PERIODO)
    paso = max(abs(a - b) for a, b in zip(control.brazo.ultimo[:4], inicio))
    assert 0 < paso <= config.ARTICULAR_VELOCIDAD_ACERCAMIENTO * config.SEGUIR_PERIODO + 0.01


def test_articular_nunca_baja_a_la_mesa(control):
    lectura = articular((0.0, 0.0, -150.0, -60.0))      # brazo al frente y antebrazo hacia el suelo
    t = enganchado(control, lectura=lectura)
    seguir_hasta(control, lectura, t, 8.0)
    assert all(postura_segura(c[:4]) for c in control.brazo.continuos)
    assert "mesa" in control.aviso


def test_articular_respeta_los_topes_de_los_servos(control):
    lectura = articular((0.0, -60.0, 40.0, 0.0))        # brazo por debajo de la horizontal: el robot no llega
    t = enganchado(control, lectura=lectura)
    seguir_hasta(control, lectura, t, 8.0)
    assert all(0 <= v <= 180 for c in control.brazo.continuos for v in c[:4])


def test_articular_sin_salto_al_cruzar_180():
    assert _desenrollar(-179.0, 178.0) == pytest.approx(181.0)
    assert _desenrollar(10.0, -350.0) == pytest.approx(-350.0)


def test_articular_sin_mano_la_muneca_no_se_mueve(control):
    t = enganchado(control, lectura=articular((0.0, 45.0, -60.0, None)))
    muneca = control.brazo.ultimo[3]
    seguir_hasta(control, articular((10.0, 50.0, -60.0, None)), t, 3.0)
    assert control.brazo.ultimo[3] == muneca


def test_articular_sin_ganancia(control):
    enganchado(control, lectura=articular())
    control.ajustar_factor(+1)
    assert control.factor == 1.0 and "ganancia" in control.aviso
