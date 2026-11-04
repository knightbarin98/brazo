"""Transformación cámara-mm, clasificación de figuras y casillas de destino."""

from math import pi

import cv2
import numpy as np

from ..config import PIEZA_ALTURA, SLOTS_X, SLOTS_Y, UMBRAL_CUADRADO


def validar_transformacion(H1, H2, h1, h2, poligono):
    """Rechaza polos dentro del área, colapso y distorsión extrema.

    Son controles de plausibilidad; no sustituyen medir puntos físicos.
    """
    if H1 is None:
        raise ValueError("Falta calibración cámara-mm")
    q = np.float64(poligono).reshape(-1, 2)
    matrices = [H1] if H2 is None else [H1, H2]
    if H2 is not None and abs(h2 - h1) < 1e-6:
        raise ValueError("Las alturas de calibración son iguales")
    for Hm in matrices:
        Hm = np.asarray(Hm, float)
        if Hm.shape != (3, 3) or not np.isfinite(Hm).all() or np.linalg.matrix_rank(Hm) < 3:
            raise ValueError("Matriz de calibración singular o inválida")
        den = np.c_[q, np.ones(len(q))] @ Hm[2]
        if den.min() <= 0 <= den.max():
            raise ValueError("La transformación tiene una singularidad dentro del área")
    # Examina la escala en distintas posiciones y en ambos planos utilizados.
    centro = q.mean(axis=0)
    sitios = [centro] + [centro + t * (p - centro) for p in q for t in (0.5, 0.9)]
    signos = []
    for altura in (0.0, PIEZA_ALTURA):
        for p in sitios:
            puntos = np.float64([p, p + [1, 0], p + [0, 1]]).reshape(-1, 1, 2)
            xy = cv2.perspectiveTransform(puntos, np.asarray(H1, float)).reshape(-1, 2)
            if H2 is not None:
                xy2 = cv2.perspectiveTransform(puntos, np.asarray(H2, float)).reshape(-1, 2)
                xy += (xy2 - xy) * ((altura - h1) / (h2 - h1))
            if not np.isfinite(xy).all():
                raise ValueError("Coordenadas no finitas")
            J = np.column_stack((xy[1] - xy[0], xy[2] - xy[0]))
            escalas = np.linalg.svd(J, compute_uv=False)
            if escalas[-1] < 0.05 or escalas[0] > 10 or escalas[0] / escalas[-1] > 12:
                raise ValueError("Escala o deformación extrema; revisar puntos medidos y clics")
            signos.append(np.sign(np.linalg.det(J)))
    if min(signos) != max(signos):
        raise ValueError("La transformación cambia de orientación dentro del área")


def clasificar(hull_mm):
    """Combina vértices y ajuste geométrico; no adivina figuras desconocidas."""
    hull_mm = cv2.convexHull(np.float32(hull_mm).reshape(-1, 1, 2))
    area = cv2.contourArea(hull_mm)
    _, r = cv2.minEnclosingCircle(hull_mm)
    r_circ = area / (pi * r * r) if r > 0 else 0
    (_, _), (w, h), _ = cv2.minAreaRect(hull_mm)
    r_rect = area / (w * h) if w * h > 0 else 0
    perimetro = cv2.arcLength(hull_mm, True)
    if area <= 0 or perimetro <= 0:
        return "desconocida", r_circ, r_rect
    vertices = [len(cv2.approxPolyDP(hull_mm, e * perimetro, True))
                for e in (0.015, 0.025, 0.035)]
    area_tri, _ = cv2.minEnclosingTriangle(hull_mm)
    ajuste_tri = area / area_tri if area_tri > 0 else 0
    # Las puntas redondeadas pueden generar más de tres vértices.
    if (vertices.count(3) >= 2 or ajuste_tri >= 0.86) and 0.38 <= r_rect <= 0.68:
        forma = "triangulo"
    elif r_circ > 0.86 and 4 not in vertices:
        forma = "circulo"
    elif 4 in vertices and r_rect > UMBRAL_CUADRADO and min(w, h) / max(w, h) >= 0.70:
        forma = "cuadrado"
    else:
        forma = "desconocida"
    return forma, r_circ, r_rect


def rango_desde_muestras(pix):
    """Percentiles robustos y matiz circular (OpenCV: 0..179).

    Ignora píxeles casi blancos/negros cuyo matiz no es fiable. Muestrear
    zonas claras y oscuras INTERIORES; no bordes, reflejos ni fondo.
    """
    pix = np.asarray(pix).reshape(-1, 3)
    validos = pix[(pix[:, 1] >= 25) & (pix[:, 2] >= 15)]
    if len(validos) < max(8, len(pix) // 4):
        raise ValueError("Muestras sin color suficiente: evita reflejos, sombras negras y fondo.")
    Hh, S, V = validos.astype(float).T
    ang = Hh * (2 * pi / 180)
    centro = (np.arctan2(np.sin(ang).mean(), np.cos(ang).mean()) * 180 / (2 * pi)) % 180
    delta = (Hh - centro + 90) % 180 - 90
    h_lo, h_hi = np.percentile(delta, [3, 97]) + centro + np.array([-5, 5])
    if h_hi - h_lo > 40:
        raise ValueError("Muestras mezcladas: haz los clics dentro de fichas del mismo color.")
    s_lo = max(25, int(np.percentile(S, 3) - 35))
    v_lo = max(15, int(np.percentile(V, 3) - 35))
    v_hi = min(255, int(np.percentile(V, 97) + 40))
    inicio, fin = int(np.floor(h_lo)), int(np.ceil(h_hi))
    tonos = sorted({h % 180 for h in range(inicio, fin + 1)})
    tramos = []
    a = b = tonos[0]
    for h in tonos[1:]:
        if h != b + 1:
            tramos.append((a, b))
            a = h
        b = h
    tramos.append((a, b))
    return [((a, s_lo, v_lo), (b, 255, v_hi)) for a, b in tramos]


def slots_en_cuadrilatero(q, nx=SLOTS_X, ny=SLOTS_Y, margen=0.2):
    q = np.array(q, float)
    us = np.linspace(margen, 1 - margen, nx)
    vs = np.linspace(margen, 1 - margen, ny)
    out = []
    for v in vs:
        for u in us:
            p = (1 - u) * (1 - v) * q[0] + u * (1 - v) * q[1] + u * v * q[2] + (1 - u) * v * q[3]
            out.append((float(p[0]), float(p[1])))
    return out
