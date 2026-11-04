"""Marcadores magenta/verde del brazo: color, detección y modelo píxel-mm."""

import json
import os
from math import hypot

import cv2
import numpy as np

from ..config import (COLOR_DIBUJO_MARCADOR, FONT, MAGENTA_A_MIN, MAGENTA_B_MAX, MAGENTA_H,
                      MARCADOR_AREA_MIN, MARCADORES_ARCHIVO, NOMBRES_MARCADOR, RADIO_BUSQUEDA_PX,
                      VERDE_HSV)

MODELOS_COLOR_MARCADOR = {}     # nombre -> modelo Lab calibrado


# ---------------------------------------------------------------- colores de marcador
def cargar_marcadores():
    MODELOS_COLOR_MARCADOR.clear()
    if not os.path.exists(MARCADORES_ARCHIVO):
        print("Marcadores: reglas por defecto (usa 'calibrar marcador magenta').")
        return
    try:
        with open(MARCADORES_ARCHIVO) as f:
            d = json.load(f)
        for n, m in d.items():
            if n in NOMBRES_MARCADOR:
                np.linalg.inv(np.asarray(m["ab_cov"], float))   # valida
                MODELOS_COLOR_MARCADOR[n] = m
        print("Marcadores calibrados:", ", ".join(MODELOS_COLOR_MARCADOR) or "ninguno")
    except (OSError, ValueError, KeyError, np.linalg.LinAlgError) as e:
        MODELOS_COLOR_MARCADOR.clear()
        print("marcadores.json inválido; se usan reglas por defecto:", e)


def guardar_marcadores():
    with open(MARCADORES_ARCHIVO + ".tmp", "w") as f:
        json.dump(MODELOS_COLOR_MARCADOR, f, indent=2)
    os.replace(MARCADORES_ARCHIVO + ".tmp", MARCADORES_ARCHIVO)


def preparar_color(frame):
    f = cv2.GaussianBlur(frame, (5, 5), 0)
    return cv2.cvtColor(f, cv2.COLOR_BGR2LAB), cv2.cvtColor(f, cv2.COLOR_BGR2HSV)


def _d2_ab(ab, modelo):
    """Distancia de Mahalanobis² en el plano cromático (a, b). ab: (..., 2)."""
    mu = np.asarray(modelo["ab_media"], np.float32)
    inv = np.linalg.inv(np.asarray(modelo["ab_cov"], np.float64)).astype(np.float32)
    da = ab[..., 0].astype(np.float32) - mu[0]
    db = ab[..., 1].astype(np.float32) - mu[1]
    return inv[0, 0] * da * da + 2 * inv[0, 1] * da * db + inv[1, 1] * db * db


def mascara_con_modelo(lab, modelo):
    L = lab[..., 0]
    m = ((_d2_ab(lab[..., 1:], modelo) <= modelo["umbral_d2"])
         & (L >= modelo["L_min"]) & (L <= modelo["L_max"]))
    return m.astype(np.uint8) * 255


def mascara_marcador(lab, hsv, nombre):
    modelo = MODELOS_COLOR_MARCADOR.get(nombre)
    if modelo is not None:
        return mascara_con_modelo(lab, modelo)
    if nombre == "magenta":
        a, b = lab[..., 1], lab[..., 2]
        h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        m = ((a >= MAGENTA_A_MIN) & (b <= MAGENTA_B_MAX)
             & (h >= MAGENTA_H[0]) & (h <= MAGENTA_H[1]) & (s >= 60) & (v >= 50))
        return m.astype(np.uint8) * 255
    return cv2.inRange(hsv, np.uint8(VERDE_HSV[0]), np.uint8(VERDE_HSV[1]))


def modelo_color_desde_muestras(parches_pos, parches_neg=()):
    """Aprende una elipse en (a,b) desde clics; ajusta el umbral con los negativos.

    Devuelve (modelo, informe). Lanza ValueError si las muestras no sirven.
    """
    pos = np.vstack([np.asarray(p, np.float64).reshape(-1, 3) for p in parches_pos])
    croma = np.hypot(pos[:, 1] - 128, pos[:, 2] - 128)
    pos = pos[croma >= 12]
    if len(pos) < 20:
        raise ValueError("Muestras casi grises: haz clic en el CENTRO del marcador, sin reflejos.")
    ab = pos[:, 1:]
    mu = ab.mean(axis=0)
    cov = np.cov(ab.T) + np.eye(2) * 5.0 ** 2     # regularización: >= 5 unidades de desviación
    modelo = {"ab_media": mu.tolist(), "ab_cov": cov.tolist(), "umbral_d2": 12.0,
              "L_min": float(max(0, np.percentile(pos[:, 0], 2) - 40)),
              "L_max": float(min(255, np.percentile(pos[:, 0], 98) + 40))}
    p95 = float(np.percentile(_d2_ab(ab, modelo), 95))
    bajo = p95 * 1.2                              # para incluir el marcador
    informe = f"marcador: a={mu[0]:.0f} b={mu[1]:.0f}; d² p95={p95:.1f}"
    umbral = max(12.0, bajo)
    if parches_neg:
        neg = np.vstack([np.asarray(p, np.float64).reshape(-1, 3) for p in parches_neg])
        neg = neg[(neg[:, 0] >= modelo["L_min"]) & (neg[:, 0] <= modelo["L_max"])]
        if len(neg):
            cerca = float(np.percentile(_d2_ab(neg[:, 1:], modelo), 2))
            alto = 0.7 * cerca                    # para excluir lo rojizo
            informe += f"; negativo más parecido d²={cerca:.1f} (separación x{cerca / max(p95, 1e-6):.1f})"
            if bajo <= alto:
                umbral = min(umbral, alto)
            else:
                umbral = (bajo + alto) / 2
                informe += "\nAVISO: colores casi iguales con esta luz. Usa un marcador más saturado o mejor luz."
    modelo["umbral_d2"] = float(umbral)
    informe += f"; umbral d²={umbral:.1f}"
    return modelo, informe


def candidatos_mascara(mask, area_max):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        a = cv2.contourArea(c)
        if a < MARCADOR_AREA_MIN or a > area_max:
            continue
        if a / max(cv2.contourArea(cv2.convexHull(c)), 1e-6) < 0.6:
            continue
        M = cv2.moments(c)
        out.append({"px": [M["m10"] / M["m00"], M["m01"] / M["m00"]], "area": a, "contorno": c})
    return mask, out


def elegir_candidato(cands, esperado=None, radio=RADIO_BUSQUEDA_PX):
    """Devuelve (candidato o None, motivo). Nunca adivina entre manchas parecidas."""
    if not cands:
        return None, "sin manchas de ese color"
    if esperado is not None:
        e = np.asarray(esperado, float)
        cerca = sorted((hypot(c["px"][0] - e[0], c["px"][1] - e[1]), i) for i, c in enumerate(cands))
        cerca = [t for t in cerca if t[0] <= radio]
        if not cerca:
            return None, f"{len(cands)} mancha(s), ninguna dentro del círculo"
        return cands[cerca[0][1]], f"la más cercana a la punta ({len(cerca)} en el círculo)"
    if len(cands) == 1:
        return cands[0], "única"
    orden = sorted(cands, key=lambda c: c["area"], reverse=True)
    if orden[0]["area"] >= 2.5 * orden[1]["area"]:
        return orden[0], f"la mayor de {len(cands)}"
    return None, f"ambiguo: {len(cands)} manchas parecidas"


def detectar_marcadores(frame, esperados=None, radios=None):
    """esperados: {color: (x, y) px}. Devuelve (encontrados {color: [x, y]}, detalle)."""
    lab, hsv = preparar_color(frame)
    area_max = frame.shape[0] * frame.shape[1] * 0.05
    encontrados, detalle = {}, {}
    for n in NOMBRES_MARCADOR:
        mask, cands = candidatos_mascara(mascara_marcador(lab, hsv, n), area_max)
        esp = (esperados or {}).get(n)
        radio = (radios or {}).get(n, RADIO_BUSQUEDA_PX)
        c, motivo = elegir_candidato(cands, esp, radio)
        if c is not None:
            encontrados[n] = [float(c["px"][0]), float(c["px"][1])]
        detalle[n] = {"mascara": mask, "candidatos": cands, "elegido": c,
                      "motivo": motivo, "esperado": esp, "radio": radio}
    return encontrados, detalle


def marcadores_frame(frame):
    """Compatibilidad con v12: detección sin posición esperada."""
    return detectar_marcadores(frame)[0]


def dibujar_marcadores(img, detalle, tinte=True):
    for n, d in detalle.items():
        col = COLOR_DIBUJO_MARCADOR[n]
        if tinte:
            capa = np.zeros_like(img)
            capa[d["mascara"] > 0] = col
            cv2.addWeighted(capa, 0.45, img, 1.0, 0, img)
        for c in d["candidatos"]:
            cv2.drawContours(img, [c["contorno"]], -1, (255, 255, 255), 1)
        if d["esperado"] is not None:
            e = tuple(int(round(v)) for v in d["esperado"])
            cv2.circle(img, e, int(d["radio"]), col, 1)
        if d["elegido"] is not None:
            p = tuple(int(round(v)) for v in d["elegido"]["px"])
            cv2.drawContours(img, [d["elegido"]["contorno"]], -1, col, 2)
            cv2.drawMarker(img, p, col, cv2.MARKER_CROSS, 18, 2)
            cv2.putText(img, n, (p[0] + 10, p[1]), FONT, 0.5, (255, 255, 255), 1)


def modelo_marcador(muestras, color):
    pares = [r for r in muestras if color in r.get('marcadores', {})]
    if len(pares) < 4:
        return None  # 3 determinan una afín, 4 permiten comprobarla por exclusión.
    px = np.asarray([r['marcadores'][color] for r in pares], float)
    mm = np.asarray([r['objetivo_mm'] for r in pares], float)
    a = np.column_stack((px, np.ones(len(px))))
    if np.linalg.matrix_rank(a) < 3:
        return None
    for i in range(len(a)):
        sub = np.delete(a, i, axis=0)
        if np.linalg.matrix_rank(sub) < 3:
            return None
        fit = np.linalg.lstsq(sub, np.delete(mm, i, axis=0), rcond=None)[0]
        if np.linalg.norm(a[i] @ fit - mm[i]) > 8:
            return None
    return np.linalg.lstsq(a, mm, rcond=None)[0], cv2.convexHull(px.astype(np.float32))
