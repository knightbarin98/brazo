"""Segunda webcam a un costado: giro entre cámaras calculado solo, fusión por punto y descartes."""

from math import cos, radians, sin

import numpy as np
import pytest

from brazo_robot.config import LATERAL_MUESTRAS_MIN
from brazo_robot.seguimiento.lateral import DESCARTES_PARA_REINICIAR, FusionLateral, cambiar_lados
from brazo_robot.seguimiento.mano import DetectorBrazo, Punto

PROFUNDIDAD, IMAGEN = 0.08, 0.01            # m de error típico: la profundidad, mucho peor que la imagen


def giro(eje, grados):
    eje = np.asarray(eje, float) / np.linalg.norm(eje)
    k = np.array(((0, -eje[2], eje[1]), (eje[2], 0, -eje[0]), (-eje[1], eje[0], 0)))
    a = radians(grados)
    return np.eye(3) + sin(a) * k + (1 - cos(a)) * k @ k


# La lateral mira desde tu costado: girada ~85° alrededor de la vertical (y de MediaPipe) e inclinada 5°.
R_LATERAL = giro((0, 1, 0), 85) @ giro((1, 0, 0), 5)


def cuerpo(rng):
    """Persona en ejes de la cámara de frente (y hacia abajo): torso fijo y brazo derecho moviéndose."""
    hombro = np.array((-0.18, -0.45, 0.0))
    d1 = rng.normal(size=3)
    d1 /= np.linalg.norm(d1)
    d2 = rng.normal(size=3)
    d2 /= np.linalg.norm(d2)
    codo = hombro + 0.3 * d1
    muneca = codo + 0.25 * d2
    return {11: hombro, 12: np.array((0.18, -0.45, 0.02)), 13: codo, 14: np.array((0.22, -0.2, 0.0)),
            15: muneca, 16: np.array((0.24, 0.02, 0.0)), 23: np.array((-0.1, 0.0, 0.0)),
            24: np.array((0.1, 0.0, 0.0))}


def ver(puntos, R, rng, vis=0.95):
    """Lo que da Pose en una cámara girada R: los puntos en sus ejes, con mucho error en su profundidad."""
    salida = {}
    for i, p in puntos.items():
        q = R.T @ p
        q = q + rng.normal(0, (IMAGEN, IMAGEN, PROFUNDIDAD))
        salida[i] = (q[0], q[1], q[2], vis)
    return salida


def angulo_de_giro(A, B):
    return np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1)))


def entrenada(rng, n=200, R=R_LATERAL):
    f = FusionLateral()
    for k in range(n):
        p = cuerpo(rng)
        f.actualizar(ver(p, np.eye(3), rng), ver(p, R, rng), 0.05 * k)
    return f


def test_calcula_solo_el_giro_entre_las_camaras():
    f = entrenada(np.random.default_rng(1))
    assert angulo_de_giro(f.R, R_LATERAL) < 3.0
    assert np.linalg.det(f.R) == pytest.approx(1.0)


def test_sin_suficientes_lecturas_no_combina():
    rng = np.random.default_rng(2)
    f = FusionLateral()
    p = cuerpo(rng)
    for k in range(LATERAL_MUESTRAS_MIN - 1):
        f.actualizar(ver(p, np.eye(3), rng), ver(p, R_LATERAL, rng), 0.05 * k)
    assert f.fusionar(ver(p, np.eye(3), rng), ver(p, R_LATERAL, rng)) is None


def test_combinar_reduce_el_error_de_profundidad_del_brazo():
    rng = np.random.default_rng(3)
    f = entrenada(rng)
    error_frente, error_combinado = [], []
    for _ in range(300):
        p = cuerpo(rng)
        frente = ver(p, np.eye(3), rng)
        combinado = f.fusionar(frente, ver(p, R_LATERAL, rng))
        for i in (13, 15):                                  # codo y muñeca
            error_frente.append(np.linalg.norm(np.array(frente[i][:3]) - p[i]))
            error_combinado.append(np.linalg.norm(np.array(combinado[i][:3]) - p[i]))
    # la profundidad de una la corrige la imagen de la otra: el error baja a menos de la mitad
    assert np.mean(error_combinado) < 0.5 * np.mean(error_frente)
    assert np.mean(error_combinado) < 0.035


def test_cada_camara_pesa_en_lo_que_ve_bien():
    rng = np.random.default_rng(4)
    f = entrenada(rng)
    p = cuerpo(rng)
    frente = {i: (q[0], q[1], q[2] + 0.3, 0.95) for i, q in ver(p, np.eye(3), rng).items()}   # profundidad muy mal
    combinado = f.fusionar(frente, ver(p, R_LATERAL, rng))
    # la lateral ve de lado la profundidad de la de frente: el error de 30 cm casi desaparece
    assert abs(combinado[15][2] - p[15][2]) < 0.06


def test_izquierda_y_derecha_cambiadas_no_estropean_el_brazo():
    # Pose, de lado, a veces confunde tu brazo derecho con el izquierdo: esos puntos no se combinan.
    rng = np.random.default_rng(5)
    f = entrenada(rng)
    R = f.R.copy()
    error_frente, error_combinado = [], []
    for k in range(200):
        p = cuerpo(rng)
        frente, lateral = ver(p, np.eye(3), rng), ver(p, R_LATERAL, rng)
        cambiada = dict(lateral)
        for a, b in ((11, 12), (13, 14), (15, 16), (23, 24)):
            cambiada[a], cambiada[b] = lateral[b], lateral[a]
        f.actualizar(frente, cambiada, 100.0 + 0.05 * k)
        combinado = f.fusionar(frente, cambiada)
        for i in (13, 15):                                  # el codo y la muñeca que se siguen
            error_frente.append(np.linalg.norm(np.array(frente[i][:3]) - p[i]))
            error_combinado.append(np.linalg.norm(np.array(combinado[i][:3]) - p[i]))
    assert np.mean(error_combinado) < 0.6 * np.mean(error_frente)   # se reconocen y se usan bien
    assert angulo_de_giro(f.R, R) < 2.0                     # y el giro entre cámaras no se estropea


def test_lados_cambiados_desde_el_principio():
    rng = np.random.default_rng(9)
    f = FusionLateral()
    for k in range(100):
        p = cuerpo(rng)
        f.actualizar(ver(p, np.eye(3), rng), cambiar_lados(ver(p, R_LATERAL, rng)), 0.05 * k)
    assert angulo_de_giro(f.R, R_LATERAL) < 3.0
    p = cuerpo(rng)
    frente = ver(p, np.eye(3), rng)
    combinado = f.fusionar(frente, cambiar_lados(ver(p, R_LATERAL, rng)))
    assert np.linalg.norm(np.array(combinado[15][:3]) - p[15]) < 0.08


def test_una_lectura_de_otra_persona_se_descarta():
    rng = np.random.default_rng(8)
    f = entrenada(rng)
    p, otra = cuerpo(rng), cuerpo(rng)
    otra = {i: q + np.array((0.6, 0.0, 0.0)) for i, q in otra.items()}   # alguien a 60 cm, en la lateral
    assert f.actualizar(ver(p, np.eye(3), rng), ver(otra, R_LATERAL, rng), 100.0) is False


def test_si_la_camara_se_mueve_se_vuelve_a_alinear():
    rng = np.random.default_rng(6)
    f = entrenada(rng)
    nueva = giro((0, 1, 0), 60) @ giro((1, 0, 0), -5)       # alguien giró la cámara lateral
    t = 100.0
    for _ in range(DESCARTES_PARA_REINICIAR + 300):
        p = cuerpo(rng)
        t += 0.05
        f.actualizar(ver(p, np.eye(3), rng), ver(p, nueva, rng), t)
    assert angulo_de_giro(f.R, nueva) < 3.0


class LateralFalsa:
    def __init__(self):
        self.error = None
        self.lectura = None
        self.camara = type("C", (), {"viva": True})()

    def ultimo(self, t, max_edad=0.2):
        if self.lectura is None or abs(t - self.lectura[0]) > max_edad:
            return None
        return self.lectura


def detector_con(lateral):
    d = DetectorBrazo.__new__(DetectorBrazo)               # sin cargar MediaPipe
    d.lateral, d.fusion, d.estado_lateral, d.fusionado = lateral, FusionLateral(), "", False
    return d


def mundo_de(puntos):
    lista = [Punto(0.0, 0.0, 0.0, 0.0)] * 33
    for i, (x, y, z, v) in puntos.items():
        lista[i] = Punto(x, y, z, v)
    return lista


def test_el_detector_combina_con_la_lateral_y_sin_ella_sigue_solo():
    rng = np.random.default_rng(7)
    lateral = LateralFalsa()
    d = detector_con(lateral)
    t = 0.0
    for _ in range(LATERAL_MUESTRAS_MIN + 5):
        p = cuerpo(rng)
        t += 0.05
        lateral.lectura = (t, ver(p, R_LATERAL, rng), [])
        mundo = mundo_de(ver(p, np.eye(3), rng))
        salida = d._con_lateral(mundo, t)
    assert d.fusionado and "combinando" in d.estado_lateral
    assert salida is not mundo and len(salida) == 33
    lateral.lectura = (t - 1.0, lateral.lectura[1], [])     # lectura vieja: sólo la de frente
    assert d._con_lateral(mundo, t) is mundo and not d.fusionado
    assert "no te ve" in d.estado_lateral
    sin = detector_con(None)
    assert sin._con_lateral(mundo, t) is mundo
