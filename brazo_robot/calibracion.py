"""Calibraciones del robot montado, desde la consola: montaje del gemelo y garra.

Como 'calibrar recogida' de v13: ventana con instrucciones, teclas para mover el robot y G o ENTER
para guardar en ajustes.json. No necesitan cámara: se calibra mirando el robot.
"""

import cv2
import numpy as np

from . import config
from .ajustes import configurar_recogida, garra_abierta, garra_cerrada, guardar_montaje
from .cinematica import Inalcanzable, postura_segura, servos_desde_geo
from .config import GARRA_MAX_GRADOS, MAPEO
from .ui import leer_tecla, tecla_flecha, texto_con_borde, ventana_cerrada
from .vista3d import dibujar_robot, parametros_vista, puntos_robot

GIRO_PRUEBA_PROFUNDIDAD = 25.0     # grados que gira la base para preguntar hacia dónde se fue
VELOCIDAD_CALIBRACION = 1.0        # grados/paso de 15 ms: despacio, con el robot cerca de la gente
AMBAR = (0, 200, 255)


def postura_referencia(paso, base_servo):
    """Ángulos geométricos de la postura de cada paso.

    plano/profundidad: al costado, el brazo colgando recto (el estado inicial: hombro en 0, a lo
    largo de la pared hacia abajo); de pie, recto a 30° hacia adelante y arriba.
    ceros: brazo perpendicular a la superficie de montaje y antebrazo a 90° (al costado: el antebrazo
    paralelo a la pared, hacia abajo; de pie: horizontal), con la garra siguiendo al antebrazo. Así
    cada servo queda a mitad de recorrido y su cero se puede corregir hacia los dos lados (con el
    codo recto su servo estaría en 0, en el tope).
    """
    base = (base_servo - MAPEO["base"][0]) / MAPEO["base"][1]
    if paso == "profundidad":
        base += GIRO_PRUEBA_PROFUNDIDAD
    if paso == "ceros":
        return {"base": base, "hombro": 90.0, "codo": -90.0, "muneca": 0.0}
    if config.MONTAJE == "costado":
        return {"base": base, "hombro": 0.0, "codo": 0.0, "muneca": 0.0}
    return {"base": base, "hombro": 30.0, "codo": 0.0, "muneca": 0.0}


def servos_seguros(geo):
    """Servos de esa postura, o ValueError si se sale de los servos o toca la superficie o la base."""
    servos = servos_desde_geo(geo)
    if not all(0.0 <= s <= 180.0 for s in servos):
        raise ValueError("esa postura se sale del recorrido de algún servo")
    if not postura_segura(servos):
        raise ValueError("esa postura tocaría la superficie de montaje o la base")
    return servos


INSTRUCCIONES = {
    "plano": ["PASO 1/3: plano de referencia de la base",
              "Al costado: el brazo debe colgar A PLOMO, como tu brazo relajado, sin ir hacia la",
              "camara ni hacia atras (usa una plomada). De pie: en el plano de la caja.",
              "A/D (o flechas) giran la base | 1/5: paso | ENTER: siguiente | ESC: cancelar"],
    "profundidad": ["PASO 2/3: hacia donde gira la base",
                    "La base giro un poco. Desde donde te ve la webcam:",
                    "el brazo se fue HACIA LA CAMARA (adelante)?",
                    "S: si | N: no, se fue hacia atras | ESC: cancelar"],
    "ceros": ["PASO 3/3: cero de cada articulacion. Compruebalo con una escuadra:",
              "brazo perpendicular a la superficie de montaje, antebrazo a 90 grados del brazo",
              "(al costado: paralelo a la pared, hacia abajo) y la garra siguiendo al antebrazo.",
              "1 hombro, 2 codo, 3 muneca | A/D corrigen | 5: paso 1 o 5 | G: guardar | ESC: cancelar"],
}


def _ventana(win, brazo, paso, base_servo, signo, ceros, elegido, avance, aviso):
    img = dibujar_robot(puntos_robot(brazo.ultimo), brazo.ultimo[4], 640, 400, parametros_vista(),
                        titulo="Calibrar montaje (" + config.MONTAJE + ")")
    panel = np.zeros((560, 640, 3), np.uint8)
    panel[:400] = img
    lineas = INSTRUCCIONES[paso] + [
        f"base {base_servo:.1f} | profundidad {'+1' if signo > 0 else '-1'} | ceros hombro {ceros['hombro']:.1f} "
        f"codo {ceros['codo']:.1f} muneca {ceros['muneca']:.1f} | paso {avance:g}"
        + (f" | corrigiendo: {elegido}" if paso == "ceros" else "")]
    for i, linea in enumerate(lineas):
        texto_con_borde(panel, linea, (8, 420 + 22 * i), 0.45, AMBAR if i == 0 else (255, 255, 255))
    if aviso:
        texto_con_borde(panel, aviso[:95], (8, 548), 0.45, (60, 60, 255))
    cv2.imshow(win, panel)


def calibrar_montaje(brazo):
    """Base de referencia, sentido de la profundidad y ceros de hombro, codo y muñeca. G guarda."""
    mapeo_antes = dict(MAPEO)
    base_servo, signo = float(config.ARTICULAR_BASE), float(config.ARTICULAR_SIGNO_PROFUNDIDAD)
    ceros = {k: float(MAPEO[k][0]) for k in ("hombro", "codo", "muneca")}
    paso, elegido, avance, aviso = "plano", "hombro", 1.0, ""
    velocidad_antes = brazo.velocidad
    win = "Calibrar montaje"
    print(f"Montaje: {config.MONTAJE}. El brazo se moverá despacio; despeja su alcance.")
    guardado = False
    try:
        brazo.ajustar_velocidad(VELOCIDAD_CALIBRACION, guardar=False)

        def ir():
            for k, cero in ceros.items():          # los ceros que se están probando, ya aplicados
                MAPEO[k] = (cero, MAPEO[k][1])
            brazo.mover(servos_seguros(postura_referencia(paso, base_servo)) + [brazo.ultimo[4]])

        ir()
        cv2.namedWindow(win)
        visto = [False]
        while True:
            _ventana(win, brazo, paso, base_servo, signo, ceros, elegido, avance, aviso)
            k = leer_tecla(40)
            if k == 27 or ventana_cerrada(win, visto):
                print("Calibración del montaje cancelada: no se guardó nada.")
                break
            if k == -1:
                continue
            flecha = tecla_flecha(k)
            delta = (-avance if flecha == "izquierda" or k in (ord('a'), ord('A')) else
                     avance if flecha == "derecha" or k in (ord('d'), ord('D')) else 0.0)
            try:
                aviso = ""
                if paso == "plano":
                    if delta:
                        base_servo = min(180.0, max(0.0, base_servo + delta))
                        ir()
                    elif k in (ord('1'), ord('5')):
                        avance = float(chr(k))
                    elif k in (10, 13):
                        paso = "profundidad"
                        ir()
                elif paso == "profundidad":
                    if k in (ord('s'), ord('S'), ord('n'), ord('N')):
                        signo = 1.0 if k in (ord('s'), ord('S')) else -1.0
                        paso = "ceros"
                        ir()
                elif paso == "ceros":
                    if k in (ord('1'), ord('2'), ord('3')):
                        elegido = ("hombro", "codo", "muneca")[k - ord('1')]
                    elif k in (ord('5'),):
                        avance = 5.0 if avance == 1.0 else 1.0
                    elif delta:
                        ceros[elegido] += delta
                        ir()
                    elif k in (ord('g'), ord('G'), 10, 13):
                        todos = dict(ceros, base=MAPEO["base"][0])
                        guardar_montaje(base_servo, signo, todos)
                        guardado = True
                        print(f"Montaje guardado: base {base_servo:.1f}, profundidad {signo:+.0f}, ceros {ceros}.")
                        print("Los puntos enseñados con 'calibrar manual' antes de esto ya no se usan: repítelos.")
                        break
            except ValueError as e:
                aviso = f"Rechazado: {e}"
                if paso == "plano" and delta:
                    base_servo -= delta
                elif paso == "ceros" and delta:
                    ceros[elegido] -= delta
                for k2, cero in ceros.items():
                    MAPEO[k2] = (cero, MAPEO[k2][1])
    finally:
        if not guardado:
            MAPEO.update(mapeo_antes)              # cancelar deja todo como estaba
        try:
            cv2.destroyWindow(win)
        except cv2.error:
            pass
        volver_al_inicio(brazo)                    # despacio todavía
        brazo.ajustar_velocidad(velocidad_antes, guardar=False)
    return guardado


def volver_al_inicio(brazo):
    """Al terminar una calibración, al estado inicial (brazo colgando) sin soltar lo que tenga la garra."""
    try:
        brazo.home(brazo.ultimo[4])
        print("El brazo volvió al estado inicial.")
    except Inalcanzable as e:
        print(f"No volví al estado inicial: {e}")


def calibrar_garra(brazo):
    """Prueba la garra en vivo y guarda los ángulos de abierta (O) y cerrada (C)."""
    angulo, avance = float(brazo.ultimo[4]), 5.0
    win = "Calibrar garra"
    print("Mueve la garra con A/D. O guarda abierta, C guarda cerrada (sin forzar el servo contra la pieza).")
    cv2.namedWindow(win)
    visto = [False]
    try:
        while True:
            panel = np.zeros((230, 640, 3), np.uint8)
            lineas = [f"Garra ahora: {angulo:.0f} grados (0..{GARRA_MAX_GRADOS:.0f})",
                      f"Guardada abierta: {garra_abierta():.0f} | cerrada: {garra_cerrada():.0f}",
                      f"Largo hasta el punto de agarre: {config.L3:.0f} mm (se cambia con 'longitud garra <mm>')",
                      "A/D (o flechas): mover | 1/5: paso | O: guardar abierta | C: guardar cerrada | ESC: salir",
                      f"paso {avance:g}"]
            for i, linea in enumerate(lineas):
                texto_con_borde(panel, linea, (10, 30 + 36 * i), 0.5, AMBAR if i == 0 else (255, 255, 255))
            cv2.imshow(win, panel)
            k = leer_tecla(40)
            if k == 27 or ventana_cerrada(win, visto):
                break
            if k == -1:
                continue
            flecha = tecla_flecha(k)
            if flecha in ("izquierda", "derecha") or k in (ord('a'), ord('A'), ord('d'), ord('D')):
                signo = -1 if flecha == "izquierda" or k in (ord('a'), ord('A')) else 1
                angulo = min(GARRA_MAX_GRADOS, max(0.0, angulo + signo * avance))
                brazo.mover(brazo.ultimo[:4] + [angulo])
            elif k in (ord('1'), ord('5')):
                avance = float(chr(k))
            elif k in (ord('o'), ord('O')):
                configurar_recogida("garra_abierta", angulo)
            elif k in (ord('c'), ord('C')):
                configurar_recogida("garra_cerrada", angulo)
    finally:
        try:
            cv2.destroyWindow(win)
        except cv2.error:
            pass

