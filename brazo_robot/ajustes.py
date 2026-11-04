"""Ajustes persistentes (ajustes.json), ángulos de la garra y colores guardados."""

import json
import os

import numpy as np

from . import config
from .config import (AJUSTES_ARCHIVO, COLORES_ACTIVOS, COLORES_ARCHIVO, GARRA_MAX_GRADOS, PASO_GRADOS, RANGOS,
                     Z_AGARRE, Z_SEGURO)

AJUSTES = {"z_recogida": config.Z_RECOGIDA, "velocidad": PASO_GRADOS}
PARAMETROS_RECOGIDA = {
    # nombre: (valor inicial, mínimo, máximo, explicación)
    "inclinacion": (-90., -120., -60., "grados de la garra; -90 apunta verticalmente abajo"),
    "velocidad_bajada": (0.5, 0.5, 4., "grados/paso durante bajada"),
    "velocidad_subida": (0.5, 0.5, 4., "grados/paso durante levantamiento"),
    "paso_bajada": (2., 0.5, 5., "mm por segmento vertical"),
    "pausa_contacto": (0.8, 0., 10., "segundos quieto abajo antes de cerrar la garra"),
    "tiempo_agarre": (2., 0.2, 15., "segundos con la garra cerrada antes de levantar"),
    "pausa_arriba": (0.5, 0., 10., "segundos estabilizando arriba"),
    "garra_abierta": (100., 0., GARRA_MAX_GRADOS, "grados del servo con la garra abierta"),
    "garra_cerrada": (0., 0., GARRA_MAX_GRADOS, "grados del servo al agarrar; ajústalo para no forzar el servo contra la pieza"),
    "tiempo_soltar": (0.8, 0.2, 10., "segundos con la garra abierta en destino"),
}
AJUSTES.update({k: v[0] for k, v in PARAMETROS_RECOGIDA.items()})


def numero_en_rango(valor, minimo, maximo, nombre):
    v = float(valor)
    if not np.isfinite(v) or not minimo <= v <= maximo:
        raise ValueError(f"{nombre}: usa un número entre {minimo} y {maximo}")
    return v


def aplicar_altura(valor):
    config.Z_RECOGIDA = numero_en_rango(valor, -100, Z_SEGURO, "Altura")
    config.BAJADA_EXTRA_AGARRE = Z_AGARRE - config.Z_RECOGIDA
    AJUSTES["z_recogida"] = config.Z_RECOGIDA


def cargar_ajustes():
    if os.path.exists(AJUSTES_ARCHIVO):
        with open(AJUSTES_ARCHIVO) as f:
            AJUSTES.update(json.load(f))
    config.L3 = numero_en_rango(AJUSTES.get("longitud_iman", 90), 20, 200, "Longitud del actuador")
    # "longitud_iman" es el nombre guardado de L3: se conserva para no perder el valor de ajustes.json.
    aplicar_altura(AJUSTES["z_recogida"])
    AJUSTES["velocidad"] = numero_en_rango(AJUSTES["velocidad"], 0.5, 4, "Velocidad")
    for k, (_, lo, hi, _) in PARAMETROS_RECOGIDA.items():
        AJUSTES[k] = numero_en_rango(AJUSTES[k], lo, hi, k)


def guardar_ajustes():
    with open(AJUSTES_ARCHIVO + ".tmp", "w") as f:
        json.dump(AJUSTES, f)
    os.replace(AJUSTES_ARCHIVO + ".tmp", AJUSTES_ARCHIVO)


def configurar_recogida(nombre=None, valor=None):
    if nombre is None:
        print(f"Altura de recogida guardada: {config.Z_RECOGIDA:.1f} mm")
        for k, (_, lo, hi, descripcion) in PARAMETROS_RECOGIDA.items():
            print(f"  {k} = {AJUSTES[k]} | {descripcion} | rango {lo}..{hi}")
        return
    if nombre not in PARAMETROS_RECOGIDA:
        raise ValueError("Parámetro desconocido. Escribe: ajustes recogida")
    _, lo, hi, _ = PARAMETROS_RECOGIDA[nombre]
    nuevo = numero_en_rango(valor, lo, hi, nombre)
    anterior = AJUSTES[nombre]
    AJUSTES[nombre] = nuevo
    try:
        guardar_ajustes()
    except OSError:
        AJUSTES[nombre] = anterior
        raise
    print(f"Guardado {nombre}={nuevo}; se aplica en la siguiente rutina, sin mover ahora.")


def garra_cerrada():
    return AJUSTES["garra_cerrada"]


def garra_abierta():
    return AJUSTES["garra_abierta"]


def cargar_colores():
    if os.path.exists(COLORES_ARCHIVO):
        with open(COLORES_ARCHIVO) as f:
            d = json.load(f)
        for c, rangos in d.items():
            RANGOS[c] = [(tuple(lo), tuple(hi)) for lo, hi in rangos]
        print("Colores calibrados cargados:", ", ".join(d))


def guardar_colores():
    d = {c: [[list(lo), list(hi)] for lo, hi in RANGOS[c]] for c in COLORES_ACTIVOS}
    old = {}
    if os.path.exists(COLORES_ARCHIVO):
        with open(COLORES_ARCHIVO) as f:
            old = json.load(f)
    old.update(d)
    with open(COLORES_ARCHIVO, "w") as f:
        json.dump(old, f)
