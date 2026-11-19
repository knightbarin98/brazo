"""El detector con la segunda cámara (vistas.py), sin MediaPipe: cámaras sintéticas de test_vistas."""

import numpy as np
import pytest

from brazo_robot.seguimiento.mano import DetectorBrazo
from brazo_robot.seguimiento.mapeo import MODO_ARTICULAR, Mano, angulo_entre, calibracion_gemelo, muestra_calibracion
from brazo_robot.seguimiento.vistas import LARGOS_DEFECTO, METODO_VISTAS, Orientaciones

from test_vistas import FRENTE, LADO, LATERAL, cuerpo, direccion, ver


class CamaraFalsa:
    viva = True


class LateralFalsa:
    def __init__(self):
        self.error, self.camara, self.lectura = None, CamaraFalsa(), None

    def ultimo(self, t, max_edad=0.2):
        if self.lectura is None or abs(t - self.lectura[0]) > max_edad:
            return None
        return self.lectura


def detector(lateral=None, cal=None):
    d = DetectorBrazo.__new__(DetectorBrazo)               # sin cargar MediaPipe
    d.cal = {"abierta": 1.6, "cerrada": 1.0, **({"gemelo": cal} if cal else {})}
    d.lateral, d.estado_lateral, d.fusionado = lateral, "", False
    d.lado, d.orientaciones, d._fuente_mano = LADO, Orientaciones(), ""
    d.indices, d.hombro_otro = (11, 13, 15), 12
    return d


def lectura():
    return Mano(MODO_ARTICULAR, 0.5, 0.5, (0.0, 0.0, 0.0))


def seguir(d, u, f, h, segundos=2.0, lateral=None, rng=None, edad=0.0):
    """Unos segundos en la postura, como en vivo; devuelve la última lectura."""
    rng = rng or np.random.default_rng(0)
    p, t = cuerpo(u, f, h), 0.0
    for _ in range(max(1, int(segundos * 30))):
        t += 1 / 30
        if lateral is not None:
            v = ver(p, LATERAL, rng, 0.05)
            lateral.lectura = (t - edad, v.mundo, v.imagen, v.tam)
        m = lectura()
        d._medir_3d(m, None, None, ver(p, FRENTE, rng, 0.05), t)
    return m


def test_con_la_lateral_combina_las_dos():
    lateral = LateralFalsa()
    d = detector(lateral)
    u = direccion(0, 85)                                    # brazo al costado, casi horizontal
    m = seguir(d, u, u, u, lateral=lateral)
    assert d.fusionado and "combinando" in d.estado_lateral
    assert m.medidas["lateral"] and m.medidas["metodo"] == METODO_VISTAS
    assert angulo_entre(m.brazo3d[0], u) < 8.0
    assert m.medidas["abduccion_hombro"] == pytest.approx(85, abs=8)
    assert "abduccion_hombro_frente" in m.medidas            # para comparar en el registro


def test_lateral_vieja_o_sin_persona_sigue_con_la_de_frente():
    lateral = LateralFalsa()
    d = detector(lateral)
    u = direccion(0, 60)
    m = seguir(d, u, u, u, lateral=lateral, edad=1.0)       # lecturas de hace 1 s
    assert not d.fusionado and "no te ve" in d.estado_lateral
    assert angulo_entre(m.brazo3d[0], u) < 12.0


def test_calibrar_colgando_mide_los_largos_y_el_codo():
    d = detector()
    u = direccion()
    rng = np.random.default_rng(1)
    muestras = [muestra_calibracion(seguir(d, u, u, u, segundos=0.0, rng=rng).medidas) for _ in range(20)]
    cal = calibracion_gemelo(muestras)
    assert cal["metodo"] == METODO_VISTAS
    for nombre in ("brazo", "antebrazo"):
        assert cal["largos"][nombre] == pytest.approx(LARGOS_DEFECTO[nombre], rel=0.08)
    assert cal["flexion"] < 8.0                             # con el codo recto, casi recto
    assert cal["direccion"] == cal["referencia"]            # sin giro


def test_una_calibracion_vieja_del_3d_no_se_aplica():
    vieja = {"direccion": [0.3, 0.3, -0.9], "flexion": 28.0, "mano": 13.5, "referencia": [0.0, 0.0, -1.0]}
    u = direccion(0, 45)
    m_vieja = seguir(detector(cal=vieja), u, u, u)
    m_sin = seguir(detector(), u, u, u)
    assert m_vieja.brazo3d[0] == pytest.approx(m_sin.brazo3d[0])
    assert not m_vieja.medidas["calibrado"]


# ---------------------------------------------------------------- elegir las webcams
from brazo_robot.seguimiento.lateral import AUTO, NO, elegir_webcams, fuente_lateral  # noqa: E402
from brazo_robot.vision.camara import es_camara_real  # noqa: E402


def test_una_camara_virtual_con_imagen_fija_no_cuenta():
    rng = np.random.default_rng(0)
    real = [100 + rng.normal(0, 2, (48, 64)).astype("float32") for _ in range(5)]   # ruido de sensor
    fija = [np.full((48, 64), 100, "float32") for _ in range(5)]                     # OBS sin emitir
    assert es_camara_real(real)
    assert not es_camara_real(fija)
    assert not es_camara_real(real[:1])


def test_fuente_lateral():
    assert fuente_lateral("auto") == AUTO and fuente_lateral("NO") == NO and fuente_lateral(None) == AUTO
    assert fuente_lateral("2") == 2 and fuente_lateral("/dev/video4") == "/dev/video4"


def test_elegir_webcams():
    dos = lambda: [0, 1]                                      # noqa: E731
    una = lambda: [3]                                         # noqa: E731
    frente_es_la_segunda = lambda a, b: (b, a)                # noqa: E731  (te ve de frente la 1)
    assert elegir_webcams(None, AUTO, dos, frente_es_la_segunda) == (1, 0)
    assert elegir_webcams(None, AUTO, una, frente_es_la_segunda) == (3, None)      # sólo hay una
    assert elegir_webcams(None, NO, dos, frente_es_la_segunda) == (0, None)        # apagada
    assert elegir_webcams(1, AUTO, dos, frente_es_la_segunda) == (1, 0)            # la otra, de lado
    assert elegir_webcams(1, 0, dos, frente_es_la_segunda) == (1, 0)               # las dos forzadas
    assert elegir_webcams(None, 1, dos, frente_es_la_segunda) == (0, 1)            # sólo la lateral forzada


# ---------------------------------------------------------------- cámaras cruzadas, en marcha
from types import SimpleNamespace  # noqa: E402

from brazo_robot.remoto.app import _Deteccion  # noqa: E402


def puntos(de_frente):
    """Puntos de imagen de una persona vista de frente (hombros separados) o de perfil."""
    p = {i: (0.5, 0.5, 0.0) for i in range(33)}
    p[0] = (0.5, 0.15, 0.9)
    if de_frente:
        p[11], p[12], p[23], p[24] = (0.4, 0.3, 0.9), (0.6, 0.3, 0.9), (0.45, 0.7, 0.9), (0.55, 0.7, 0.9)
    else:
        p[11], p[12], p[23], p[24] = (0.5, 0.3, 0.9), (0.52, 0.3, 0.9), (0.5, 0.7, 0.9), (0.51, 0.7, 0.9)
    return p


def como_pose(p):
    return [SimpleNamespace(x=x, y=y, visibility=v) for x, y, v in (p[i] for i in range(33))]


@pytest.mark.parametrize("frente_de_frente, segundos, cambia", [(False, 3.0, True), (True, 3.0, False), (False, 1.0, False)])
def test_webcams_cruzadas_se_intercambian(frente_de_frente, segundos, cambia):
    lateral = LateralFalsa()
    d = detector(lateral)
    d.cambiar_camaras, d._t_cruzadas = False, None
    t = 0.0
    while t < segundos:
        t += 1 / 30
        lateral.lectura = (t, {}, puntos(not frente_de_frente), (640, 480))
        d.revisar_camaras(como_pose(puntos(frente_de_frente)), (640, 480), t)
    assert d.cambiar_camaras is cambia


def test_la_deteccion_intercambia_las_camaras():
    frente, lado = object(), object()
    lateral = SimpleNamespace(camara=lado)
    d = detector(lateral)
    d.cambiar_camaras, d._t_cruzadas = True, None
    det = _Deteccion.__new__(_Deteccion)
    det.webcam, det.detector = frente, d
    det._cambiar_camaras()
    assert det.webcam is lado and lateral.camara is frente and not d.cambiar_camaras


# ---------------------------------------------------------------- la mano, con más tolerancia
def test_un_dedo_fuera_no_quita_la_direccion_de_la_mano():
    from brazo_robot.seguimiento.mano import PALMA
    d = detector()
    puntos_mano = [(0.5, 0.5)] * 21
    puntos_mano[8] = (1.02, 0.5)                             # la punta del índice fuera de la imagen
    d._manos = SimpleNamespace(pedir=lambda *a: None, ultimo=lambda t: {
        "puntos": puntos_mano, "ratio": 1.2, "cabeceo": 0.0, "direccion": (1, 0, 0), "direccion_mundo": (1, 0, 0)})
    m = lectura()
    direccion, _ = d._leer_mano(np.zeros((480, 640, 3), np.uint8), m, 640, 480, [(0.4, 0.4)] * 3, 0.0)
    assert direccion == (1, 0, 0) and not d.mano_cortada
    assert m.apertura is not None                            # 20 de 21 dentro: también la apertura
    puntos_mano[PALMA[1]] = (1.02, 0.5)                      # un nudillo fuera: la palma, cortada
    direccion, _ = d._leer_mano(np.zeros((480, 640, 3), np.uint8), lectura(), 640, 480, [(0.4, 0.4)] * 3, 0.0)
    assert direccion is None and d.mano_cortada


# ---------------------------------------------------------------- las posiciones llegan al Arduino
def test_las_posiciones_se_envian_al_arduino():
    import json
    from brazo_robot.remoto.protocolo import mano_a_mensaje, mano_de_mensaje
    from brazo_robot.robot import Brazo
    from brazo_robot.seguimiento.control import ControlSeguimiento
    brazo = Brazo(simular=True)
    enviados = []
    brazo.ser.write = lambda datos: enviados.append(datos.decode().strip())
    d, rng, t = detector(), np.random.default_rng(0), 0.0
    control = ControlSeguimiento(brazo, -90.0)

    def paso(u):
        nonlocal t
        t += 1 / 30
        m = lectura()
        m.largo, m.angulos = 0.55, (None, 0.0, 0.0, None)
        d._medir_3d(m, None, None, ver(cuerpo(u, u, u), FRENTE, rng, 0.03), t)
        # por la red, como entre el operador y el servidor
        control.actualizar(mano_de_mensaje(json.loads(json.dumps(mano_a_mensaje(m, t)))["mano"]), t)
        return m

    control.activar(paso(direccion()), t)
    for abduccion in np.linspace(0, 60, 90):
        paso(direccion(0, abduccion))
    for _ in range(90):
        paso(direccion(0, 60))
    ordenes = [e for e in enviados if e.startswith("S,")]
    assert len(ordenes) > 20
    hombro = float(ordenes[-1].split(",")[2])
    assert hombro == pytest.approx(60, abs=6)                # el hombro del robot fue a 60°


def test_tipo_de_camara():
    from brazo_robot.vision.camara import tipo_de_camara
    rng = np.random.default_rng(1)
    assert tipo_de_camara([100 + rng.normal(0, 2, (48, 64)).astype("float32") for _ in range(5)]) == "real"
    assert tipo_de_camara([np.full((48, 64), 46, "float32")] * 5) == "fija"          # OBS sin emitir
    assert tipo_de_camara([rng.normal(0.5, 0.2, (48, 64)).astype("float32")] * 5) == "negra"   # tapada
    assert tipo_de_camara([]) == "sin imagen"
