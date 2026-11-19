"""Gemelo del brazo con el robot DE PIE: medición 3D respecto al torso y correspondencia de articulaciones.

El montaje al costado tiene sus propias pruebas en test_costado.py.
"""

from math import atan2, cos, degrees, radians, sin

import numpy as np
import pytest

from brazo_robot import config
from brazo_robot.cinematica import postura_segura
from brazo_robot.seguimiento import control as modulo_control
from brazo_robot.seguimiento.control import SIGUIENDO, ControlSeguimiento, SeguidorArticular
from brazo_robot.seguimiento.gemelo import SeguidorGemelo, a_ejes_robot, apertura_hombro_codo
from brazo_robot.seguimiento.mapeo import (MODO_ARTICULAR, REFERENCIA_T, Mano, angulo_entre, aplicar_calibracion, calibracion_gemelo,
                                           en_torso, marco_torso, medidas_brazo3d)
from brazo_robot.vista3d import Camara3D, a_mundo, parametros_vista, puntos_robot

HOME = [90.0, 120.0, 97.0, 97.0]


@pytest.fixture(autouse=True)
def de_pie(monkeypatch):
    """Estas pruebas validan la correspondencia 'muneca' (hombro -> muñeca) sin el límite de 90° del
    hombro; la de articulaciones y el límite están en test_articulaciones.py."""
    monkeypatch.setattr(config, "MONTAJE", "de_pie")
    monkeypatch.setattr(config, "GEMELO_CORRESPONDENCIA", "muneca")
    monkeypatch.setattr(config, "ARTICULAR_LIMITE_HOMBRO", None)


def lectura(brazo, antebrazo, mano=None, x=0.5, y=0.5):
    return Mano(MODO_ARTICULAR, x, y, (0.0, 0.0, 0.0), largo=0.55, angulos=(None, 0.0, 0.0, None),
                brazo3d=(tuple(brazo), tuple(antebrazo), None if mano is None else tuple(mano)))


def estable(m, servos=HOME, segundos=10.0):
    """Servos a los que converge el seguidor con esa lectura fija (los filtros ya asentados)."""
    s = SeguidorGemelo([m], 0.0, servos)
    t, actual = 0.0, list(servos)
    for _ in range(int(segundos / 0.05)):
        t += 0.05
        actual, _ = s.objetivo(m, t, actual)
    return actual, s


def direccion(azimut, elevacion, largo=1.0):
    """Vector (afuera, adelante, arriba) con ese azimut (0 = afuera, 90 = hacia la cámara) y elevación."""
    a, e = radians(azimut), radians(elevacion)
    return (largo * cos(e) * cos(a), largo * cos(e) * sin(a), largo * sin(e))


# ---------------------------------------------------------------- posturas de referencia
def test_brazo_en_t_extendido_hacia_afuera():
    servos, s = estable(lectura((0.3, 0, 0), (0.25, 0, 0), (0.08, 0, 0)))
    # espejo: tu 'afuera' es la X negativa del robot -> hombro en 180 (horizontal hacia afuera de la caja)
    assert servos == pytest.approx([90, 180, 0, 90], abs=0.5)
    assert s.info["topes"] == []


def test_brazo_arriba():
    servos, _ = estable(lectura((0, 0, 0.3), (0, 0, 0.25), (0, 0, 0.08)))
    assert servos == pytest.approx([90, 90, 0, 90], abs=0.5)


def test_brazo_hacia_la_camara_gira_la_base_90():
    servos, _ = estable(lectura((0, 0.3, 0), (0, 0.25, 0), (0, 0.08, 0)))
    # la profundidad la hace la base: el brazo del robot queda horizontal, girado 90° fuera de la caja
    assert servos[0] == pytest.approx(180, abs=0.5)
    assert servos[1] == pytest.approx(0, abs=0.5)


@pytest.mark.parametrize("adelante", [0.1, 0.2, 0.3])
def test_profundidad_simetrica_y_con_su_signo(adelante, monkeypatch):
    hacia_camara, _ = estable(lectura((0.3, adelante, 0), (0.25, adelante, 0)))
    hacia_atras, _ = estable(lectura((0.3, -adelante, 0), (0.25, -adelante, 0)))
    assert hacia_camara[0] - 90 == pytest.approx(-(hacia_atras[0] - 90), abs=0.5)
    assert abs(hacia_camara[0] - 90) > 5
    monkeypatch.setattr(config, "ARTICULAR_SIGNO_PROFUNDIDAD", -1)
    invertido, _ = estable(lectura((0.3, adelante, 0), (0.25, adelante, 0)))
    assert invertido[0] == pytest.approx(hacia_atras[0], abs=0.5)


def test_el_codo_copia_la_flexion_en_3d():
    for flexion in (0, 30, 60, 90, 120):
        brazo = direccion(20, 30, 0.3)
        # el antebrazo gira 'flexion' grados respecto al brazo, fuera de cualquier plano vertical
        eje = np.cross(brazo, (0.3, -0.5, 0.8))
        eje /= np.linalg.norm(eje)
        u = np.array(brazo) / np.linalg.norm(brazo)
        antebrazo = 0.25 * (u * cos(radians(flexion)) + np.cross(eje, u) * sin(radians(flexion)))
        servos, s = estable(lectura(brazo, antebrazo))
        assert s.info["flexion"] == pytest.approx(flexion, abs=0.5)
        if not s.info["topes"]:
            assert servos[2] == pytest.approx(flexion, abs=0.5)   # MAPEO codo (0, -1): servo = flexión


# ---------------------------------------------------------------- correspondencia con la cinemática real
POSTURAS = [(az, el, fl) for az in (-60, -30, 0, 30, 60, 90) for el in (-20, 10, 40, 70) for fl in (0, 40, 80)]


def brazo_con_codo(az, el, fl, lado):
    """Brazo y antebrazo con flexión 'fl' cuya suma apunta en (az, el); el codo a un lado u otro de la línea."""
    d = np.array(direccion(az, el))
    lateral = np.cross(d, (0, 0, 1))
    lateral /= np.linalg.norm(lateral)
    arriba = lado * np.cross(lateral, d)
    return (0.3 * (d * cos(radians(fl / 2)) + arriba * sin(radians(fl / 2))),
            0.3 * (d * cos(radians(fl / 2)) - arriba * sin(radians(fl / 2))))


def test_correspondencia_con_la_cinematica_real():
    """Sin topes: la muñeca del robot apunta como tu brazo y su codo se dobla como el tuyo."""
    comprobadas = 0
    for az, el, fl in POSTURAS:
        for lado in (1, -1):
            u, f = brazo_con_codo(az, el, fl, lado)
            servos, s = estable(lectura(u, f))
            if s.info["topes"] or s.info["brazo_vertical"]:   # límite físico o base retenida a propósito
                continue
            comprobadas += 1
            _, hombro, codo, muneca, _ = (np.array(p) for p in puntos_robot(servos + [0]))
            robot = muneca - hombro
            esperado = np.array(a_ejes_robot(tuple(u + f)))
            assert robot @ esperado / np.linalg.norm(robot) / np.linalg.norm(esperado) > 0.9995, (az, el, fl, lado)
            assert angulo_entre(codo - hombro, muneca - codo) == pytest.approx(fl, abs=0.5)
    # De 144 posturas llegan 52 sin topes: el resto es tope del hombro o de la base (físico) o base
    # retenida con el brazo casi vertical (decisión del algoritmo). Ver docs/gemelo.md.
    assert comprobadas >= 50


def test_brazo_hacia_la_camara_y_abajo_elige_el_giro_alcanzable():
    """Antes, un empate de redondeo elegía el giro opuesto y el hombro quedaba en su tope."""
    servos, s = estable(lectura(*brazo_con_codo(90, -20, 80, 1)))
    assert s.info["topes"] == []
    assert servos[0] == pytest.approx(180, abs=0.5)     # base girada hacia la cámara
    assert 0 < servos[1] < 40                           # hombro apenas sobre la horizontal


def test_ninguna_postura_sale_de_los_servos():
    for az, el, fl in POSTURAS + [(150, -60, 130), (-120, 85, 160), (180, -89, 0)]:
        d = np.array(direccion(az, el))
        servos, _ = estable(lectura(0.3 * d, 0.25 * d), segundos=3.0)
        assert all(0 <= v <= 180 for v in servos), (az, el, fl)


def test_apertura_hombro_codo():
    assert apertura_hombro_codo(0) == pytest.approx(0, abs=0.1)        # recto: brazo sobre la línea
    assert 0 < apertura_hombro_codo(60) < apertura_hombro_codo(120)


# ---------------------------------------------------------------- límites físicos sin saltos
def test_cruzar_el_cuerpo_deja_la_base_en_su_tope_sin_saltar():
    m0 = lectura(*(0.3 * np.array(direccion(60, 0)), 0.25 * np.array(direccion(60, 0))))
    s = SeguidorGemelo([m0], 0.0, HOME)
    actual, t, bases = list(HOME), 0.0, []
    for az in range(60, 176, 2):                   # de afuera-adelante a cruzado delante del cuerpo
        d = np.array(direccion(az, 0))
        for _ in range(10):
            t += 0.05
            actual, _ = s.objetivo(lectura(0.3 * d, 0.25 * d), t, actual)
        bases.append(actual[0])
    assert max(abs(a - b) for a, b in zip(bases, bases[1:])) < 15     # nunca salta al otro tope
    assert bases[-1] in (pytest.approx(0, abs=1), pytest.approx(180, abs=1))
    assert "base" in s.info["topes"]


def test_brazo_vertical_no_mueve_la_base():
    servos, s = estable(lectura(*(0.3 * np.array(direccion(45, 0)), 0.25 * np.array(direccion(45, 0)))))
    base = servos[0]
    t = 10.0
    for _ in range(100):
        t += 0.05
        servos, _ = s.objetivo(lectura((0.01, 0.0, 0.3), (0.0, 0.01, 0.25)), t, servos)
    assert servos[0] == pytest.approx(base, abs=1.0)


def test_brazo_colgando_topa_el_hombro():
    servos, s = estable(lectura((0.05, 0, -0.3), (0.05, 0, -0.25)))
    assert "hombro" in s.info["topes"]
    assert servos[1] in (pytest.approx(0, abs=0.5), pytest.approx(180, abs=0.5))


# ---------------------------------------------------------------- muñeca y mano
def test_la_garra_apunta_como_la_mano():
    servos, _ = estable(lectura((0.3, 0, 0), (0.25, 0, 0), (0.06, 0, 0.06)))   # mano levantada 45°
    _, _, _, muneca, garra = (np.array(p) for p in puntos_robot(servos + [0]))
    v = garra - muneca
    # en el plano de la caja, 'afuera y 45° arriba' es la X negativa del robot y Z positiva: 135°
    assert degrees(atan2(v[2], v[0])) == pytest.approx(135, abs=0.5)


def test_sin_mano_la_muneca_no_se_mueve():
    servos, _ = estable(lectura((0.3, 0, 0), (0.25, 0, 0), None), servos=[90, 150, 20, 33])
    assert servos[3] == pytest.approx(33)


# ---------------------------------------------------------------- medición respecto al torso
PERSONA = {"hombro": (-0.2, -0.5, 0.0), "otro": (0.2, -0.5, 0.0), "caderas": (0.0, 0.0, 0.0)}


def girar_y(p, grados):
    """La persona entera girada sobre la vertical (eje y de MediaPipe)."""
    a = radians(grados)
    return (p[0] * cos(a) + p[2] * sin(a), p[1], -p[0] * sin(a) + p[2] * cos(a))


def test_marco_del_torso():
    afuera, adelante, arriba = marco_torso(PERSONA["hombro"], PERSONA["otro"], PERSONA["caderas"])
    assert afuera == pytest.approx((-1, 0, 0))
    assert arriba == pytest.approx((0, -1, 0))     # y de MediaPipe crece hacia abajo
    assert adelante == pytest.approx((0, 0, -1))   # hacia la cámara
    m = np.array([afuera, adelante, arriba])
    assert m @ m.T == pytest.approx(np.eye(3), abs=1e-9)


@pytest.mark.parametrize("giro", [-40, -15, 20, 45])
def test_girar_el_cuerpo_no_cambia_la_lectura(giro):
    codo = (-0.45, -0.5, -0.1)                     # brazo hacia afuera y un poco hacia la cámara
    base = en_torso(np.subtract(codo, PERSONA["hombro"]), marco_torso(PERSONA["hombro"], PERSONA["otro"], PERSONA["caderas"]))
    p = {k: girar_y(v, giro) for k, v in PERSONA.items()}
    girado = en_torso(np.subtract(girar_y(codo, giro), p["hombro"]), marco_torso(p["hombro"], p["otro"], p["caderas"]))
    assert girado == pytest.approx(base, abs=1e-9)


def test_sin_caderas_usa_la_vertical_de_la_camara():
    _, _, arriba = marco_torso(PERSONA["hombro"], PERSONA["otro"], None)
    assert arriba == pytest.approx((0, -1, 0))


def test_medidas_del_brazo():
    m = medidas_brazo3d((0.3, 0.0, 0.0), (0.0, 0.25, 0.0))
    assert m["flexion"] == pytest.approx(90)
    assert m["azimut"] == pytest.approx(degrees(atan2(0.25, 0.3)))
    assert m["elevacion"] == pytest.approx(0)
    assert 0 < m["hacia_camara"] < 1


# ---------------------------------------------------------------- calibración (brazo colgando)
def _brazo_visto(d, flexion):
    """Brazo y antebrazo como los vería la cámara: dirección d con el codo 'recto' doblado 'flexion'."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    eje = np.cross(d, (0.0, 1.0, 0.3))
    eje /= np.linalg.norm(eje)
    antebrazo = 0.25 * (d * cos(radians(flexion)) + np.cross(eje, d) * sin(radians(flexion)))
    return [(tuple(0.3 * d), tuple(antebrazo), tuple(0.08 * d))] * 5


def test_calibracion_colgando_corrige_direccion_y_flexion():
    # La cámara ve el brazo relajado un poco hacia la cámara y hacia afuera, y el codo "recto" a 15°.
    muestras = _brazo_visto((0.1, 0.15, -1.0), 15)
    cal = calibracion_gemelo(muestras)
    assert cal["flexion"] == pytest.approx(15, abs=0.1)
    assert cal["referencia"] == [0.0, 0.0, -1.0]
    u, f, h = aplicar_calibracion(*muestras[0], cal)
    assert angulo_entre(np.add(u, f), (0, 0, -1)) < 0.5     # vuelve a colgar a plomo: el estado inicial
    assert angulo_entre(u, f) == pytest.approx(0, abs=0.1)
    assert np.linalg.norm(f) == pytest.approx(0.25)


def test_calibracion_vieja_en_t_sigue_valiendo():
    d = np.array(direccion(10, 5))
    muestras = _brazo_visto(d, 15)
    cal = calibracion_gemelo(muestras, referencia=REFERENCIA_T)
    del cal["referencia"]                                    # guardada antes del reposo: sin la clave
    u, f, _ = aplicar_calibracion(*muestras[0], cal)
    assert angulo_entre(np.add(u, f), (1, 0, 0)) < 0.5


def test_calibracion_reescala_la_flexion_hasta_180():
    cal = {"direccion": [1.0, 0.0, 0.0], "flexion": 20.0, "mano": 0.0}
    u = (0.3, 0.0, 0.0)
    for medida, esperada in ((20, 0), (100, 90), (180, 180), (10, 0)):
        f = (0.25 * cos(radians(medida)), 0.0, 0.25 * sin(radians(medida)))
        assert angulo_entre(*aplicar_calibracion(u, f, None, cal)[:2]) == pytest.approx(esperada, abs=0.1)


def test_sin_calibracion_no_cambia_nada():
    datos = ((0.3, 0.1, 0.0), (0.2, 0.0, 0.1), None)
    assert aplicar_calibracion(*datos, None) == datos


def test_calibrar_sin_lecturas_falla():
    with pytest.raises(ValueError):
        calibracion_gemelo([])


# ---------------------------------------------------------------- integrado en el control
class BrazoFalso:
    def __init__(self):
        self.ultimo = HOME + [100.0]
        self.velocidad = 2.0
        self.continuos = []

    def ajustar_velocidad(self, v, guardar=True):
        self.velocidad = v

    def mover_continuo(self, valores):
        self.continuos.append(list(valores))
        self.ultimo = list(valores)
        return True


@pytest.fixture
def control(monkeypatch):
    monkeypatch.setattr(config, "Z_RECOGIDA", 0.0)
    return ControlSeguimiento(BrazoFalso(), -90.0)


def seguir(control, m, segundos, t=0.0):
    for i in range(1, int(segundos / config.SEGUIR_PERIODO) + 1):
        control.actualizar(m, t + i * config.SEGUIR_PERIODO)
    return t + segundos


def test_el_control_usa_el_gemelo_y_llega_a_la_postura(control):
    m = lectura((0.3, 0.15, 0.1), (0.2, 0.2, 0.15), (0.05, 0.05, 0.0))
    control.activar(m, 0.0)
    t = seguir(control, m, 12.0)
    assert control.estado == SIGUIENDO and isinstance(control.articular, SeguidorGemelo)
    esperado, _ = estable(m)
    assert control.brazo.ultimo[:4] == pytest.approx(esperado, abs=1.5)
    assert all(postura_segura(c[:4]) for c in control.brazo.continuos)
    assert t


def test_sin_profundidad_vuelve_al_modo_plano(control, monkeypatch):
    monkeypatch.setattr(modulo_control, "ARTICULAR_PROFUNDIDAD", False)
    m = lectura((0.3, 0.15, 0.1), (0.2, 0.2, 0.15))
    control.activar(m, 0.0)
    seguir(control, m, 1.0)
    assert isinstance(control.articular, SeguidorArticular)


def test_se_acerca_despacio_y_nunca_baja_a_la_mesa(control):
    m = lectura((0.05, 0.25, -0.25), (0.0, 0.2, -0.2))     # brazo bajado hacia la cámara: la mesa lo impide
    inicio = list(control.brazo.ultimo[:4])
    control.activar(m, 0.0)
    seguir(control, m, config.SEGUIR_TIEMPO_REFERENCIA + config.SEGUIR_PERIODO)
    assert max(abs(a - b) for a, b in zip(control.brazo.ultimo[:4], inicio)) <= \
        config.ARTICULAR_VELOCIDAD_ACERCAMIENTO * config.SEGUIR_PERIODO * 2 + 0.01
    seguir(control, m, 10.0, 1.0)
    assert all(postura_segura(c[:4]) for c in control.brazo.continuos)


def test_perder_la_lectura_detiene_el_gemelo(control):
    m = lectura((0.3, 0, 0), (0.25, 0, 0))
    control.activar(m, 0.0)
    t = seguir(control, m, 2.0)
    control.actualizar(None, t + config.ARTICULAR_TIMEOUT + 0.1)
    assert control.estado == "PAUSADO"


# ---------------------------------------------------------------- vista
@pytest.mark.parametrize("espejo", [True, False])
@pytest.mark.parametrize("signo", [1, -1])
def test_la_vista_muestra_al_robot_como_tu_brazo_en_la_webcam(espejo, signo, monkeypatch):
    monkeypatch.setattr(config, "ARTICULAR_ESPEJO", espejo)
    monkeypatch.setattr(config, "ARTICULAR_SIGNO_PROFUNDIDAD", signo)
    camara = Camara3D(azimut=0.0, elevacion=0.0)
    vista = parametros_vista()
    afuera = a_ejes_robot((1.0, 0.0, 0.0))        # tu 'afuera' en ejes del robot (base en 90 = geo 0)
    hacia_camara = a_ejes_robot((0.0, 1.0, 0.0))
    assert camara.proyectar(a_mundo(afuera, vista))[0] == pytest.approx(1.0)        # a la derecha, como en la webcam
    assert camara.proyectar(a_mundo(hacia_camara, vista))[2] == pytest.approx(1.0)  # hacia quien mira
