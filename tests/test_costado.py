"""Robot atornillado al costado de la caja: su Z sale de la caja y la base gira sobre ese eje, como un hombro."""

from math import cos, radians, sin

import numpy as np
import pytest

from brazo_robot import config
from brazo_robot.cinematica import Inalcanzable, piso_seguro, postura_inicial, postura_segura, ruta_segura, servos_desde_geo, tramo_seguro
from brazo_robot.seguimiento.gemelo import SeguidorGemelo, a_ejes_robot, de_ejes_robot
from brazo_robot.seguimiento.mapeo import MODO_ARTICULAR, Mano, angulo_entre
from brazo_robot.vista3d import Camara3D, a_mundo, dibujar_robot, parametros_vista, puntos_robot

HOME = [90.0, 120.0, 97.0, 97.0]


@pytest.fixture(autouse=True)
def costado(monkeypatch):
    monkeypatch.setattr(config, "MONTAJE", "costado")
    monkeypatch.setattr(config, "ARTICULAR_BASE", 90.0)
    monkeypatch.setattr(config, "ARTICULAR_SIGNO_PROFUNDIDAD", 1.0)
    monkeypatch.setattr(config, "Z_RECOGIDA", 10.0)
    monkeypatch.setattr(config, "ARTICULAR_LIMITE_ABAJO", None)


def lectura(brazo, antebrazo, mano=None):
    return Mano(MODO_ARTICULAR, 0.5, 0.5, (0.0, 0.0, 0.0), largo=0.55, angulos=(None, 0.0, 0.0, None),
                brazo3d=(tuple(brazo), tuple(antebrazo), None if mano is None else tuple(mano)))


def estable(m, servos=HOME, segundos=10.0):
    s = SeguidorGemelo([m], 0.0, servos)
    t, actual = 0.0, list(servos)
    for _ in range(int(segundos / 0.05)):
        t += 0.05
        actual, _ = s.objetivo(m, t, actual)
    return actual, s


def direccion(afuera, adelante, arriba):
    v = np.array((afuera, adelante, arriba), float)
    return v / np.linalg.norm(v)


def recto(d):
    return 0.3 * np.asarray(d), 0.25 * np.asarray(d)


# ---------------------------------------------------------------- ejes
def test_ejes_del_robot_al_costado():
    assert a_ejes_robot((1, 0, 0)) == (0, 0, 1)            # tu 'afuera' sale de la caja: la Z del robot
    assert a_ejes_robot((0, 0, -1)) == (1, 0, 0)           # tu 'abajo' es su X (base en ARTICULAR_BASE)
    assert a_ejes_robot((0, 1, 0)) == (0, 1, 0)            # 'hacia la cámara', su Y
    for v in ((1, 2, 3), (-0.5, 0.2, 0.9)):
        assert de_ejes_robot(a_ejes_robot(v)) == pytest.approx(v)


# ---------------------------------------------------------------- posturas de referencia
def test_brazo_colgando():
    servos, s = estable(lectura(*recto(direccion(0, 0, -1))))
    # pegado a la pared hacia abajo: base en su referencia, hombro en 0 (a lo largo de su X)
    assert servos[:3] == pytest.approx([90, 0, 0], abs=0.5)


def test_brazo_en_t_sale_recto_de_la_caja():
    servos, s = estable(lectura(*recto(direccion(1, 0, 0))))
    assert servos[1] == pytest.approx(90, abs=0.5)          # hombro a 90: perpendicular a la pared
    assert servos[2] == pytest.approx(0, abs=0.5)
    assert s.info["topes"] == []


def test_brazo_arriba():
    servos, _ = estable(lectura(*recto(direccion(0, 0, 1))))
    assert servos[1] == pytest.approx(180, abs=0.5)         # a lo largo de su X negativa: hacia arriba
    assert servos[0] == pytest.approx(90, abs=0.5)


def test_brazo_hacia_la_camara_lo_hace_la_base():
    servos, _ = estable(lectura(*recto(direccion(0, 1, 0))))
    assert servos[0] in (pytest.approx(0, abs=0.5), pytest.approx(180, abs=0.5))
    assert servos[1] in (pytest.approx(0, abs=0.5), pytest.approx(180, abs=0.5))


@pytest.mark.parametrize("adelante", [0.3, 0.6])
def test_profundidad_simetrica_y_con_su_signo(adelante, monkeypatch):
    abajo_adelante, _ = estable(lectura(*recto(direccion(0.5, adelante, -1))))
    abajo_atras, _ = estable(lectura(*recto(direccion(0.5, -adelante, -1))))
    assert abajo_adelante[0] - 90 == pytest.approx(-(abajo_atras[0] - 90), abs=0.5)
    assert abs(abajo_adelante[0] - 90) > 10
    monkeypatch.setattr(config, "ARTICULAR_SIGNO_PROFUNDIDAD", -1.0)
    invertido, _ = estable(lectura(*recto(direccion(0.5, adelante, -1))))
    assert invertido[0] == pytest.approx(abajo_atras[0], abs=0.5)


# ---------------------------------------------------------------- correspondencia y alcance
def brazo_con_codo(d, flexion, lado):
    d = np.asarray(d, float)
    eje = np.cross(d, (0.3, 0.5, 0.8))
    eje /= np.linalg.norm(eje)
    arriba = lado * np.cross(eje, d)
    m = radians(flexion / 2)
    return 0.3 * (d * cos(m) + arriba * sin(m)), 0.3 * (d * cos(m) - arriba * sin(m))


def direcciones_hemisferio():
    """Direcciones de tu brazo hacia afuera del cuerpo (la mitad que el robot al costado puede cubrir)."""
    for az in range(-150, 181, 30):          # alrededor del eje 'afuera', desde abajo hacia adelante
        for ap in (20, 45, 70, 89):           # ángulo desde el eje 'afuera'
            a, p = radians(az), radians(ap)
            yield direccion(cos(p), sin(p) * sin(a), -sin(p) * cos(a))


def test_correspondencia_y_alcance_al_costado():
    comprobadas = total = 0
    for d in direcciones_hemisferio():
        for flexion, lado in ((0, 1), (40, 1), (40, -1), (80, 1), (80, -1)):
            total += 1
            u, f = brazo_con_codo(d, flexion, lado)
            servos, s = estable(lectura(u, f))
            assert all(0 <= v <= 180 for v in servos)
            if s.info["topes"] or s.info["brazo_vertical"]:
                continue
            comprobadas += 1
            _, hombro, codo, muneca, _ = (np.array(p) for p in puntos_robot(servos + [0]))
            robot = muneca - hombro
            esperado = np.array(a_ejes_robot(tuple(u + f)))
            assert robot @ esperado / np.linalg.norm(robot) / np.linalg.norm(esperado) > 0.9995
            assert angulo_entre(codo - hombro, muneca - codo) == pytest.approx(flexion, abs=0.5)
    # Al costado el robot cubre casi todo el hemisferio de afuera; sin topes ni base retenida, la mayoría.
    assert comprobadas / total > 0.5, (comprobadas, total)


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


def test_cruzar_el_brazo_por_delante_del_cuerpo_nunca_mete_el_robot_en_la_caja():
    from brazo_robot.seguimiento.control import ControlSeguimiento
    m = lectura(*brazo_con_codo(direccion(-0.6, 0.8, 0), 60, 1))
    _, s = estable(m)
    assert s.info["topes"]                                  # el objetivo cae fuera del alcance
    control = ControlSeguimiento(BrazoFalso(), -90.0)
    control.activar(m, 0.0)
    for i in range(1, 300):
        control.actualizar(m, i * config.SEGUIR_PERIODO)
    assert control.brazo.continuos and all(postura_segura(c[:4]) for c in control.brazo.continuos)


# ---------------------------------------------------------------- seguridad
def test_el_piso_al_costado_es_la_pared():
    config.Z_RECOGIDA = -51.0                               # calibración vieja, con el robot de pie
    assert piso_seguro() == 0.0
    config.MONTAJE = "de_pie"
    assert piso_seguro() == -51.0


def test_ninguna_articulacion_entra_en_la_pared():
    dentro = servos_desde_geo({"base": 0.0, "hombro": 10.0, "codo": -150.0, "muneca": 0.0})   # antebrazo hacia la caja
    fuera = servos_desde_geo({"base": 0.0, "hombro": 90.0, "codo": 0.0, "muneca": 0.0})
    assert not postura_segura(dentro)
    assert postura_segura(fuera)


def test_limite_hacia_abajo_hasta_la_mesa():
    colgando = servos_desde_geo({"base": 0.0, "hombro": 10.0, "codo": 0.0, "muneca": 0.0})
    assert postura_segura(colgando)
    config.ARTICULAR_LIMITE_ABAJO = 200.0                   # la mesa a 20 cm por debajo del eje
    assert not postura_segura(colgando)
    hacia_adelante = servos_desde_geo({"base": 90.0, "hombro": 10.0, "codo": 0.0, "muneca": 0.0})
    assert postura_segura(hacia_adelante)                   # paralelo a la mesa: no baja


# ---------------------------------------------------------------- vista
def test_la_vista_al_costado_coincide_con_tu_brazo():
    camara = Camara3D(azimut=0.0, elevacion=0.0)
    vista = parametros_vista()
    afuera = camara.proyectar(a_mundo(a_ejes_robot((1.0, 0.0, 0.0)), vista))
    arriba = camara.proyectar(a_mundo(a_ejes_robot((0.0, 0.0, 1.0)), vista))
    assert afuera[0] == pytest.approx(1.0) and arriba[1] == pytest.approx(1.0)
    img = dibujar_robot(puntos_robot(HOME + [50.0]), 50.0, 640, 480, vista)
    assert img.shape == (480, 640, 3)


# ---------------------------------------------------------------- estado inicial
def test_estado_inicial_es_el_brazo_colgando_con_la_garra_hacia_abajo():
    servos = postura_inicial()
    _, hombro, codo, muneca, garra = (np.array(p) for p in puntos_robot(servos + [0]))
    abajo = np.array(a_ejes_robot((0.0, 0.0, -1.0)))       # tu 'abajo', en ejes del robot
    for a, b in ((hombro, codo), (codo, muneca), (muneca, garra)):
        d = (b - a) / np.linalg.norm(b - a)
        assert d @ abajo > 0.999                            # brazo, antebrazo y garra a plomo
    assert postura_segura(servos)


def test_estado_inicial_sigue_a_la_calibracion_del_montaje(monkeypatch):
    monkeypatch.setattr(config, "ARTICULAR_BASE", 100.0)
    assert postura_inicial()[0] == pytest.approx(100.0)
    monkeypatch.setattr(config, "MONTAJE", "de_pie")
    assert postura_inicial() == pytest.approx(config.HOME)


def test_estado_inicial_se_separa_de_la_pared_si_colgando_tocaria():
    config.Z_RECOGIDA = config.H + 10.0                     # el piso queda por encima del brazo colgando
    servos = postura_inicial()
    assert postura_segura(servos) and servos[1] > 0


def test_ruta_segura_desde_una_postura_pegada_a_la_pared():
    junto = servos_desde_geo({"base": 0.0, "hombro": 70.0, "codo": -110.0, "muneca": -90.0})
    assert postura_segura(junto)
    assert not tramo_seguro(junto, postura_inicial())       # en línea recta el codo rozaría la pared
    ruta = ruta_segura(junto, postura_inicial())
    assert ruta[-1] == pytest.approx(postura_inicial())
    previo = junto
    for p in ruta:
        assert max(abs(a - b) for a, b in zip(p, previo)) <= 10.0 + 1e-9
        assert tramo_seguro(previo, p)
        previo = p


def test_ruta_segura_rechaza_un_destino_que_toca():
    dentro = servos_desde_geo({"base": 0.0, "hombro": 10.0, "codo": -150.0, "muneca": 0.0})
    with pytest.raises(Inalcanzable):
        ruta_segura(postura_inicial(), dentro)


def test_home_del_robot_simulado_va_al_estado_inicial_por_la_ruta_segura():
    from brazo_robot.simulador import BrazoVirtual
    robot = BrazoVirtual()
    assert robot.ultimo[:4] == pytest.approx(postura_inicial())
    robot.mover(servos_desde_geo({"base": 0.0, "hombro": 70.0, "codo": -110.0, "muneca": -90.0}) + [40.0])
    robot.home(40.0)
    assert robot.ultimo == pytest.approx(postura_inicial() + [40.0])
