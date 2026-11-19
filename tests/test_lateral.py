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
