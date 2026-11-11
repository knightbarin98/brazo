"""Órdenes de alto nivel: interpretar texto, ajustar altura y recoger fichas."""

import re
import unicodedata

import cv2
import numpy as np

from . import config
from .ajustes import AJUSTES, aplicar_altura, guardar_ajustes, garra_abierta, numero_en_rango
from .cinematica import Inalcanzable, ik
from .config import COLORES_ACTIVOS, FONT, RAICES, Z_CAL_MANUAL, Z_SEGURO
from .ui import leer_tecla, tecla_flecha, ventana_cerrada


def normalizar(t):
    t = unicodedata.normalize("NFD", t.lower())
    return "".join(c for c in t if unicodedata.category(c) != "Mn")


def interpretar(txt):
    colores = [c for c in COLORES_ACTIVOS if any(r in txt for r in RAICES[c])]
    formas = [f for f, raiz in (("cuadrado", "cuadr"), ("triangulo", "triang"), ("circulo", "circ")) if raiz in txt]
    todo = any(p in txt for p in ("todo", "todas", "todos"))
    return colores, formas, todo


def destino_de(txt):
    m = re.search(r"\b(?:region|zona|en|a)\s+([ab])\b", txt)
    return m.group(1).upper() if m else "A"


def mover_vertical(brazo, x, y, z_actual, z_destino, inclinacion=-90.):
    """Comprueba toda la bajada/subida antes de ejecutar un paso manual."""
    z_destino = numero_en_rango(z_destino, -100, Z_SEGURO, "Altura")
    for z in np.linspace(z_actual, z_destino, max(2, int(abs(z_destino-z_actual))+1)):
        ik(x, y, float(z), inclinacion)
    brazo.ir(x, y, z_destino, garra_abierta(), inclinacion)
    return z_destino


def ajustar_altura(brazo, vision, x, y):
    """Enseña Z manteniendo XY fijo. G guarda; ESC cancela los cambios.

    Z es una coordenada del modelo, no una medición de contacto. El programa
    no tiene sensor de mesa. No hace una bajada automática al valor guardado.
    """
    if not np.isfinite([x, y]).all():
        raise ValueError("X e Y deben ser finitos")
    cx, cy = vision.corregir_xy(x, y)
    inclinacion = AJUSTES["inclinacion"]
    # Conserva la aproximación adaptativa del archivo aportado, pero comienza
    # arriba: nunca baja de golpe hasta los -51 mm del ajuste anterior.
    z_inicio = None
    for candidata in np.arange(Z_SEGURO, Z_CAL_MANUAL - 1, -1):
        try:
            ik(cx, cy, float(candidata), inclinacion)
            z_inicio = float(candidata)
            break
        except Inalcanzable:
            continue
    if z_inicio is None:
        raise Inalcanzable("No hay altura de aproximación alcanzable en ese XY")
    win = "V13 Recogida | W/S altura J/L inclinacion G guardar ESC cancelar"
    cv2.namedWindow(win)  # Si no hay interfaz gráfica, no mover el brazo.
    visto = [False]
    velocidad_anterior = brazo.velocidad
    guardado = False
    paso = 1.0
    z = z_inicio
    aviso = "W sube / S baja. G guarda altura y velocidad. ESC cancela."
    print(f"Ajuste XY real ({x},{y}) -> comando ({cx:.1f},{cy:.1f}); inicio Z={z:.1f}")
    print("Z menor = bajar. Observe contacto; no hay sensor de mesa. Al salir queda en su posición.")
    try:
        brazo.ajustar_velocidad(min(velocidad_anterior, 0.5), guardar=False)
        brazo.home()
        brazo.ir(cx, cy, z, garra_abierta(), inclinacion)
        while True:
            f = vision.frame_o_lienzo()
            lineas = [f"XY fijo: {x:.1f}, {y:.1f} | Z actual: {z:.1f} mm",
                      f"Z guardada: {config.Z_RECOGIDA:.1f} | paso {paso:.1f} mm | bajada {brazo.velocidad:.2f}",
                      "W / flecha arriba: SUBIR | S / flecha abajo: BAJAR",
                      "0: paso 0.5 mm | 1: 1 mm | 2: 2 mm | 5: 5 mm",
                      "+/-: velocidad bajada | G guarda | ESC cancela",
                      f"J/L: inclinar -/+ 1 grado | inclinacion actual {inclinacion:.1f}", aviso]
            for i, linea in enumerate(lineas):
                cv2.putText(f, linea, (8, 22 + 25*i), FONT, 0.45, (0,0,0), 3)
                cv2.putText(f, linea, (8, 22 + 25*i), FONT, 0.45, (255,255,255), 1)
            cv2.imshow(win, f)
            k = leer_tecla(40)
            if ventana_cerrada(win, visto) or k == 27:
                break
            if k < 0:
                continue
            subir = tecla_flecha(k) == "arriba"
            bajar = tecla_flecha(k) == "abajo"
            try:
                if subir or bajar:
                    z = mover_vertical(brazo, cx, cy, z, z + (paso if subir else -paso), inclinacion)
                    aviso = f"Posición ordenada: Z={z:.1f}. Revise distancia al objeto."
                elif k in (ord('j'),ord('J'),ord('l'),ord('L')):
                    nueva = numero_en_rango(inclinacion + (-1 if k in (ord('j'),ord('J')) else 1), -120, -60, "Inclinación")
                    # Recalcula hombro/codo/muñeca para conservar la posición de la cara.
                    brazo.ir(cx, cy, z, garra_abierta(), nueva)
                    inclinacion = nueva
                    aviso = f"Inclinación {inclinacion:.1f}: compruebe físicamente la garra."
                elif k in (ord('0'),ord('1'),ord('2'),ord('5')):
                    paso = {ord('0'):0.5,ord('1'):1.,ord('2'):2.,ord('5'):5.}[k]
                elif k in (ord('+'),ord('='),ord('-')):
                    nueva = np.clip(brazo.velocidad + (-0.25 if k == ord('-') else 0.25), 0.5, 4)
                    brazo.ajustar_velocidad(float(nueva), guardar=False)
                elif k in (ord('g'),ord('G')):
                    anterior = dict(AJUSTES)
                    z_anterior = config.Z_RECOGIDA
                    try:
                        aplicar_altura(z)
                        AJUSTES["velocidad_bajada"] = brazo.velocidad
                        AJUSTES["inclinacion"] = inclinacion
                        guardar_ajustes()
                    except OSError:
                        AJUSTES.clear(); AJUSTES.update(anterior)
                        aplicar_altura(z_anterior)
                        raise
                    guardado = True
                    print(f"Guardado: Z={z:.1f} mm; inclinación={inclinacion:.1f}; velocidad bajada={brazo.velocidad:.2f}")
                    break
            except (Inalcanzable, ValueError) as e:
                aviso = str(e)
                print("Movimiento rechazado:", e)
    finally:
        cv2.destroyWindow(win)
        brazo.ajustar_velocidad(velocidad_anterior, guardar=False)
        print(f"Brazo permanece en la última posición ordenada. Z guardada={config.Z_RECOGIDA:.1f} mm.")


def recoger(brazo, vision, txt):
    colores, formas, todo = interpretar(txt)
    if not colores and not formas and not todo:
        print("Indica color o figura, ej: recoge cafe | recoge triangulo en b | recoge azul circulo | recoge todo en a")
        return

    destino = destino_de(txt)
    if destino not in vision.slots:
        print(f"La región {destino} no está calibrada ('calibrar regiones').")
        return

    objs = vision.detectar()
    sel = [o for o in objs
           if (not colores or o["color"] in colores) and (not formas or o["forma"] in formas)]
    print(f"Detectadas {len(objs)} fichas, {len(sel)} cumplen la orden. Destino: región {destino}.")

    for o in sel:
        idx = vision.ocupados[destino]
        if idx >= len(vision.slots[destino]):
            print(f"Región {destino} llena. Vacíala y escribe 'reiniciar'.")
            break
        dx, dy = vision.slots[destino][idx]
        sx_cmd, sy_cmd = vision.corregir_xy(o["x"], o["y"])
        dx_cmd, dy_cmd = vision.corregir_xy(dx, dy)
        print(f"  -> {o['color']} {o['forma']} en ({o['x']:.0f}, {o['y']:.0f}) a casilla {idx + 1} de {destino}")
        if len(vision.registros_brazo) >= 4:
            print(f"     corrección brazo: origen ({sx_cmd:.1f},{sy_cmd:.1f}) | destino ({dx_cmd:.1f},{dy_cmd:.1f})")
        try:
            brazo.recoger_y_colocar(sx_cmd, sy_cmd, dx_cmd, dy_cmd)
            vision.ocupados[destino] += 1
        except Inalcanzable as e:
            print("  ! Omitida:", e)
    print("Listo.")
