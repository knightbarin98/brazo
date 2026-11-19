"""Simulador del gemelo: tu brazo en la webcam y el robot virtual lado a lado, sin hardware.

No abre ningún puerto serial ni conexión de red (no importa pyserial): sirve para comprobar la
correspondencia antes de conectar el robot. Usa el mismo control que el servidor (ControlSeguimiento
en modo articular) con un robot virtual, así que lo que se ve aquí es lo que haría el robot real con
la misma configuración: filtros, topes, velocidad máxima, acercamiento lento y pausa sin lectura.
"""

import csv
import time

import cv2
import numpy as np

from . import config
from .ajustes import AJUSTES, garra_abierta
from .cinematica import ik, postura_inicial, ruta_segura
from .config import ARTICULAR_PROFUNDIDAD, GARRA_MAX_GRADOS, GEMELO_CALIBRACION_SEGUNDOS, WEBCAM_RESOLUCION
from .remoto.app import AMBAR, ROJO, VERDE, _Deteccion, _panel_webcam, _poner_lateral, _sin_acentos, crear_detector_brazo
from .seguimiento.control import ACTIVOS, SIGUIENDO, ControlSeguimiento
from .seguimiento.gemelo import SeguidorGemelo
from .seguimiento.mano import cargar_calibracion_mano, guardar_calibracion_mano
from .seguimiento.mapeo import calibracion_gemelo
from .ui import leer_tecla, texto_con_borde, ventana_cerrada
from .vision.camara import Camara
from .vista3d import NOMBRES, dibujar_robot, parametros_vista, puntos_robot

VENTANA = "Simulador del gemelo (sin robot)"
ALTO_PANEL, ALTO_TABLA = 480, 118


class BrazoVirtual:
    """Robot sin hardware: tiene lo que usa el control del modo articular y nada más.

    Las órdenes se aplican al instante (la velocidad la limita el control, igual que con el robot
    real) y se recortan como en Brazo: servos 0..180 y garra hasta GARRA_MAX_GRADOS.
    """

    def __init__(self):
        self.ultimo = postura_inicial() + [garra_abierta()]
        self.velocidad = float(AJUSTES["velocidad"])
        self.ordenes = 0

    def ajustar_velocidad(self, valor, guardar=False):
        self.velocidad = float(valor)

    def mover_continuo(self, valores):
        a = [max(0.0, min(180.0, float(v))) for v in valores]
        a[4] = min(a[4], GARRA_MAX_GRADOS)
        self.ultimo = a
        self.ordenes += 1
        return True

    def mover(self, valores):
        self.mover_continuo(valores)

    def ir(self, x, y, z, garra, inclinacion=-90.):
        self.mover(ik(x, y, z, inclinacion) + [garra])

    def home(self, garra=None):
        garra = garra_abierta() if garra is None else garra
        for servos in ruta_segura(self.ultimo, postura_inicial()):
            self.mover(servos + [garra])


class _SinRed:
    """El hilo de detección de la app envía cada lectura; aquí no hay a quién."""

    def enviar(self, mensaje):
        return True


def _tabla(control, mano, ancho):
    """Lo que haces tú frente a lo que hace el robot, y qué articulaciones están en su tope."""
    img = np.zeros((ALTO_TABLA, ancho, 3), np.uint8)
    color = VERDE if control.estado == SIGUIENDO else AMBAR
    texto_con_borde(img, f"{control.estado} | {_sin_acentos(control.aviso)[:100]}", (8, 20), 0.5, color)
    m = mano.medidas if mano is not None else {}
    if m.get("flexion") is not None:
        tu = (f"Tu brazo:  azimut {m['azimut']:5.0f}  elevacion {m['elevacion']:5.0f}  codo {m['flexion']:5.0f}"
              f"  hacia la camara {m['hacia_camara'] * 100:3.0f}%" + ("" if m.get("caderas") else "  (sin caderas)"))
    else:
        tu = "Tu brazo: no te veo (deben verse los dos hombros, el codo y la muneca)"
    texto_con_borde(img, tu, (8, 46), 0.48)
    if isinstance(control.articular, SeguidorGemelo) and control.articular.info.get("brazo_vertical"):
        posicion = "en T" if config.MONTAJE == "costado" else "vertical"
        texto_con_borde(img, f"brazo casi {posicion}: la base se queda quieta", (ancho - 360, 46), 0.45, AMBAR)
    servos = control.brazo.ultimo
    topes = control.articular.info.get("topes", []) if isinstance(control.articular, SeguidorGemelo) else []
    x = 8
    texto_con_borde(img, "Robot (servo):", (x, 72), 0.48)
    x += 120
    for nombre, valor in zip(NOMBRES + ("garra",), servos):
        texto_con_borde(img, f"{nombre} {valor:5.1f}", (x, 72), 0.48, ROJO if nombre in topes else (255, 255, 255))
        x += 125
    texto_con_borde(img, "ESPACIO activar/pausar | C calibrar (brazo colgando) | G garra | A/D W/S girar vista | V vista inicial | ESC",
                    (8, 100), 0.42, (190, 190, 190))
    return img


def ejecutar_simulador(fuente_webcam, brazo="derecho", registro=None, fuente_lateral=None):
    if not ARTICULAR_PROFUNDIDAD:
        print("ARTICULAR_PROFUNDIDAD = False: se simula el modo plano (base fija).")
    cal = cargar_calibracion_mano()
    detector = crear_detector_brazo(cal, brazo, True, fuente_lateral)
    try:
        webcam = Camara(fuente_webcam, "webcam", WEBCAM_RESOLUCION, mjpg=True)
    except RuntimeError:
        detector.cerrar()
        raise
    robot = BrazoVirtual()
    control = ControlSeguimiento(robot, AJUSTES["inclinacion"])
    deteccion = _Deteccion(webcam, detector, _SinRed())
    archivo = open(registro, "w", newline="") if registro else None
    escritor = csv.writer(archivo) if archivo else None
    if escritor:
        escritor.writerow(["t", "estado", "azimut", "elevacion", "flexion", "hacia_camara", "caderas",
                           "base", "hombro", "codo", "muneca", "garra", "topes"])
    vista = [30.0, 20.0]
    calibrando, aviso_local, t_aviso = None, "", 0.0
    print("Simulador: ESPACIO activa/pausa | C calibra con el brazo colgando | G garra | A/D W/S giran la vista | ESC sale.")
    try:
        cv2.namedWindow(VENTANA, cv2.WINDOW_NORMAL)
        visto = [False]
        webcam.iniciar_hilo()
        deteccion.start()
        ultimo_n = -1
        while True:
            if deteccion.error is not None:
                raise RuntimeError(f"Falló la detección: {deteccion.error}")
            if not webcam.viva:
                raise RuntimeError("Se perdió la señal de la webcam.")
            t = time.monotonic()
            n, f, mano = deteccion.ultimo_numerado()
            nueva = n != ultimo_n
            ultimo_n = n
            control.actualizar(mano if nueva else None, t)
            if nueva and calibrando is not None and mano is not None and mano.medidas.get("crudo"):
                calibrando["muestras"].append(mano.medidas["crudo"])
            if calibrando is not None and t >= calibrando["fin"]:
                try:
                    cal["gemelo"] = calibracion_gemelo(calibrando["muestras"])
                    guardar_calibracion_mano(cal)
                    aviso_local = f"Calibrado con {len(calibrando['muestras'])} lecturas (codo recto medido: {cal['gemelo']['flexion']:.0f})"
                except ValueError:
                    aviso_local = "No se pudo calibrar: no te vi el brazo."
                calibrando, t_aviso = None, t
                control.ir_a_home()                # el robot vuelve al estado inicial, como tu brazo
            if nueva and escritor and mano is not None and mano.medidas.get("flexion") is not None:
                m = mano.medidas
                topes = control.articular.info.get("topes", []) if isinstance(control.articular, SeguidorGemelo) else []
                escritor.writerow([f"{t:.3f}", control.estado, f"{m['azimut']:.1f}", f"{m['elevacion']:.1f}",
                                   f"{m['flexion']:.1f}", f"{m['hacia_camara']:.3f}", int(m.get("caderas", False)),
                                   *(f"{v:.1f}" for v in robot.ultimo), "|".join(topes)])

            topes = control.articular.info.get("topes", []) if isinstance(control.articular, SeguidorGemelo) else []
            panel_robot = dibujar_robot(puntos_robot(robot.ultimo), robot.ultimo[4], 640, ALTO_PANEL,
                                        parametros_vista(), vista[0], vista[1], topes, "Robot simulado")
            panel_tu = _poner_lateral(_panel_webcam(f, mano), detector)
            if calibrando is not None:
                texto_con_borde(panel_tu, f"Brazo relajado colgando, recto y quieto... {calibrando['fin'] - t:.1f}",
                                (8, 70), 0.55, AMBAR)
            elif aviso_local and t - t_aviso < 4.0:
                texto_con_borde(panel_tu, _sin_acentos(aviso_local)[:70], (8, 70), 0.5, AMBAR)
            arriba = np.hstack([panel_tu, cv2.resize(panel_robot, (panel_robot.shape[1], panel_tu.shape[0]))])
            cv2.imshow(VENTANA, np.vstack([arriba, _tabla(control, mano, arriba.shape[1])]))

            k = leer_tecla(5)
            if k == 27 or ventana_cerrada(VENTANA, visto):
                break
            if k == 32:
                if control.estado in ACTIVOS:
                    control.pausar("En pausa.")
                else:
                    control.activar(mano, t)
            elif k in (ord('c'), ord('C')):
                control.pausar("Calibrando.")
                calibrando = {"fin": t + GEMELO_CALIBRACION_SEGUNDOS, "muestras": []}
            elif k in (ord('g'), ord('G')):
                control.alternar_garra()
            elif k in (ord('a'), ord('A'), ord('d'), ord('D')):
                vista[0] += 10.0 if k in (ord('d'), ord('D')) else -10.0
            elif k in (ord('w'), ord('W'), ord('s'), ord('S')):
                vista[1] = max(-10.0, min(90.0, vista[1] + (10.0 if k in (ord('w'), ord('W')) else -10.0)))
            elif k in (ord('v'), ord('V')):
                vista[:] = [30.0, 20.0]
    finally:
        deteccion.parar()
        webcam.cerrar()
        detector.cerrar()
        if archivo:
            archivo.close()
        cv2.destroyWindow(VENTANA)
        print("Simulador cerrado.")
