"""Visión de la mesa: plano cámara-mm, regiones, colores y detección de fichas."""

import json
import os
from math import cos, hypot, radians, sin

import cv2
import numpy as np

from .. import config
from ..ajustes import guardar_colores
from ..cinematica import Inalcanzable, ik, radio_max_teorico
from ..config import (AREA_MAX_MM2, AREA_MIN_MM2, CALIB_ARCHIVO, COLOR_BGR, COLORES_ACTIVOS,
                      COLORES_REGION, FONT, NOMBRES_REGION, PIEZA_ALTURA, PUNTOS_MM, RANGOS,
                      Z_SEGURO, Z_SOLTAR)
from ..ui import pedir_clics
from .geometria import clasificar, rango_desde_muestras, slots_en_cuadrilatero, validar_transformacion


class Vision:
    def __init__(self, camara):
        self.camara = camara          # la cámara con la que el robot ve la mesa
        self.error_calibracion = None
        self.tam = camara.tam
        self.H1 = self.H2 = None      # píxeles -> mm a la altura h1 / h2
        self.h1 = self.h2 = 0.0
        self.Hinv0 = None             # mm -> píxeles del plano de la mesa (solo para dibujar)
        self.regiones = {}
        self.slots = {}
        self.ocupados = {"A": 0, "B": 0}
        # Registros de enseñanza manual del brazo. Cada entrada conserva el
        # punto real de la mesa, el (x,y,z) que hubo que ordenar para alinear
        # físicamente el electroimán y los ángulos finales enviados al Arduino.
        self.registros_brazo = []
        self.cargar()

    # ---------- persistencia ----------
    def cargar(self):
        print("Archivo de calibración:", CALIB_ARCHIVO)
        if not os.path.exists(CALIB_ARCHIVO):
            print("Sin calibración. Recomendado: 'calibrar plano', 'calibrar manual', 'calibrar regiones', 'calibrar colores'.")
            return
        with open(CALIB_ARCHIVO) as f:
            d = json.load(f)
        if d.get("H1"):
            self.H1 = np.array(d["H1"], float)
            self.h1 = float(d.get("h1", 0.0))
        if d.get("H2"):
            self.H2 = np.array(d["H2"], float)
            self.h2 = float(d.get("h2", 0.0))
        self.regiones = {k: [tuple(p) for p in v] for k, v in d.get("regiones", {}).items()}
        self.registros_brazo = d.get("registros_brazo", [])
        self._actualizar()
        print("Calibración cargada:",
              ("INVALIDA: " + self.error_calibracion) if self.error_calibracion else "geometría plausible; comprobar medidas reales",
              "| regiones:", ", ".join(self.regiones) or "ninguna")
        print("Método:", "manual (un plano)" if self.H2 is None else "automático (dos alturas)")
        if self.registros_brazo:
            print(f"Registros manuales del electroimán: {len(self.registros_brazo)}")

    def guardar(self):
        d = {"H1": self.H1.tolist() if self.H1 is not None else None, "h1": self.h1,
             "H2": self.H2.tolist() if self.H2 is not None else None, "h2": self.h2,
             "regiones": {k: [list(p) for p in v] for k, v in self.regiones.items()},
             "registros_brazo": self.registros_brazo}
        with open(CALIB_ARCHIVO, "w") as f:
            json.dump(d, f)

    # ---------- conversiones ----------
    def px_a_mm(self, pts_px, h=0.0):
        """Píxeles -> mm para puntos a la altura h sobre la mesa (corrige el paralaje).
        Cada píxel es un rayo; su cruce con el plano z=h es lineal en h, así que con dos
        alturas calibradas se obtiene cualquier otra."""
        a = np.float32(pts_px).reshape(-1, 1, 2)
        X1 = cv2.perspectiveTransform(a, self.H1).reshape(-1, 2)
        if self.H2 is not None and self.h2 != self.h1:
            X2 = cv2.perspectiveTransform(a, self.H2).reshape(-1, 2)
            X1 = X1 + (X2 - X1) * ((h - self.h1) / (self.h2 - self.h1))
        return [tuple(p) for p in X1]

    def a_px(self, pts_mm):
        a = np.float32(pts_mm).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(a, self.Hinv0).reshape(-1, 2)

    def _actualizar(self):
        self.slots = {}
        self.Hinv0 = None
        self.error_calibracion = None
        if self.H1 is None:
            return
        try:
            zona = self.regiones.get("fichas")
            if zona is not None:
                validar_transformacion(self.H1, self.H2, self.h1, self.h2, zona)
        except ValueError as e:
            self.error_calibracion = str(e)
            print("CALIBRACION INVALIDA:", e, "| Repite 'calibrar plano' con puntos medidos.")
            return
        if self.H2 is None:
            # Un solo plano: la inversa exacta. La rejilla de toda la imagen podía
            # incluir puntos tras el horizonte y deformar el dibujo de referencias.
            self.Hinv0 = np.linalg.inv(np.asarray(self.H1, float))
        else:
            w, h = self.tam
            gx, gy = np.meshgrid(np.linspace(0.1 * w, 0.9 * w, 9), np.linspace(0.1 * h, 0.9 * h, 7))
            px = np.float32(np.c_[gx.ravel(), gy.ravel()])
            mm = np.float32(self.px_a_mm(px, 0.0))
            self.Hinv0, _ = cv2.findHomography(mm, px, 0)
        for n in ("A", "B"):
            if n in self.regiones:
                self.slots[n] = slots_en_cuadrilatero(self.px_a_mm(self.regiones[n], 0.0))

    # ---------- cámara ----------
    def frame(self):
        return self.camara.leer_fresco()

    def frame_vivo(self):
        """Un solo fotograma, para ventanas en vivo (frame() descarta 5 y es lento)."""
        return self.camara.leer()

    def _hsv(self, f):
        return cv2.cvtColor(cv2.GaussianBlur(f, (5, 5), 0), cv2.COLOR_BGR2HSV)

    def _mascara(self, hsv, color):
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in RANGOS[color]:
            m |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        return m

    # ---------- calibraciones ----------
    def calibrar_plano(self):
        """Calibra cámara -> milímetros con 4 marcas físicas sobre la mesa.

        Este paso NO usa el marcador verde ni mueve el brazo. Es la referencia
        geométrica del plano sobre la que después se enseña la posición real
        de la cara del electroimán.
        """
        print("Mide las marcas desde el eje de la base: x adelante, y izquierda; unidades mm.")
        print("PUNTOS REALES, EN ESTE ORDEN:", PUNTOS_MM)
        print("Haz clic exactamente en las 4 marcas físicas de la mesa.")
        pts = pedir_clics(self.frame(), [f"Punto {i + 1}: {p} mm (sobre la mesa)" for i, p in enumerate(PUNTOS_MM)])
        if pts is None:
            print("Cancelado.")
            return
        src = np.float32(pts).reshape(-1, 1, 2)
        dst = np.float32(PUNTOS_MM).reshape(-1, 1, 2)
        if (not cv2.isContourConvex(src) or not cv2.isContourConvex(dst)
                or cv2.contourArea(src) < 100 or cv2.contourArea(dst) < 100):
            raise ValueError("Puntos cruzados, repetidos o casi alineados. No se guardó.")
        nueva = cv2.getPerspectiveTransform(src.reshape(4, 2), dst.reshape(4, 2))
        validar_transformacion(nueva, None, 0, 0, pts)

        self.H1 = nueva
        self.h1, self.H2, self.h2 = 0.0, None, 0.0
        self._actualizar()
        if self.error_calibracion:
            raise ValueError("Plano rechazado: " + self.error_calibracion)
        self.guardar()
        print("Plano MANUAL guardado con los 4 puntos medidos.")
        print("Ahora ejecuta: calibrar manual")

    def calibrar_regiones(self):
        textos, colores = [], []
        for n in ("fichas", "A", "B"):
            for i in range(4):
                textos.append(f"{NOMBRES_REGION[n]}: esquina {i + 1}/4 (en orden alrededor)")
                colores.append(COLORES_REGION[n])
        pts = pedir_clics(self.frame(), textos, colores)
        if pts is None:
            print("Cancelado.")
            return
        for j, n in enumerate(("fichas", "A", "B")):
            self.regiones[n] = pts[j * 4:(j + 1) * 4]
        self._actualizar()
        self.guardar()
        print("Regiones guardadas.")
        self.revisar_alcance()

    def calibrar_colores(self, color=None):
        N = 4
        colores = [color] if color is not None else COLORES_ACTIVOS
        if any(c not in COLORES_ACTIVOS for c in colores):
            raise ValueError("Color no activo")
        print("Haz 4 clics en zonas interiores claras y oscuras; evita bordes y reflejos.")
        f = self.frame()
        hsv = self._hsv(f)
        textos, cols = [], []
        for c in colores:
            for i in range(N):
                textos.append(f"INTERIOR {c.upper()}: varia zona clara/oscura ({i + 1}/{N})")
                cols.append(COLOR_BGR.get(c, (0, 255, 0)))
        pts = pedir_clics(f, textos, cols)
        if pts is None:
            print("Cancelado.")
            return
        nuevos = {}
        for j, c in enumerate(colores):
            pix = []
            for (x, y) in pts[j * N:(j + 1) * N]:
                x, y = int(x), int(y)
                pix.append(hsv[max(0, y - 3):y + 4, max(0, x - 3):x + 4].reshape(-1, 3))
            nuevos[c] = rango_desde_muestras(np.vstack(pix))
            print(f"  {c}: {nuevos[c]}")
        RANGOS.update(nuevos)
        guardar_colores()
        print("Colores guardados. Usa 'mascaras' para revisar y 'ver' para detectar.")

    def revisar_alcance(self):
        if self.H1 is None or not self.regiones:
            print("Faltan calibraciones (usa 'calibrar plano' o 'calibrar punta', y luego 'calibrar regiones').")
            return
        rmax = radio_max_teorico()
        print(f"Radio máximo teórico del brazo: {rmax:.0f} mm")
        todos = []
        for n in ("fichas", "A", "B"):
            if n not in self.regiones:
                continue
            pts = self.px_a_mm(self.regiones[n], 0.0)
            radios = [hypot(x, y) for x, y in pts]
            todos += radios
            malas = 0
            # En FICHAS hay que bajar hasta la altura de recogida; en A/B solo hasta soltar.
            z_bajo = config.Z_RECOGIDA if n == "fichas" else Z_SOLTAR
            for (x, y) in pts:
                try:
                    ik(x, y, Z_SEGURO)
                    ik(x, y, z_bajo)
                except Inalcanzable:
                    malas += 1
            estado = "OK" if malas == 0 else f"{malas} esquina(s) FUERA de alcance"
            print(f"  {NOMBRES_REGION[n]:9s}: radio {min(radios):.0f} a {max(radios):.0f} mm -> {estado}")
        if todos:
            print(f"Radio máximo usado por las 12 esquinas: {max(todos):.0f} mm (margen {rmax - max(todos):.0f} mm)")

    # ---------- dibujo ----------
    def _overlay(self, img):
        for n, pts in self.regiones.items():
            col = COLORES_REGION[n]
            cv2.polylines(img, [np.int32(pts)], True, col, 2)
            cv2.putText(img, NOMBRES_REGION[n], tuple(int(v) for v in pts[0]), FONT, 0.5, col, 2)
        if self.Hinv0 is not None:
            r = radio_max_teorico()
            arco = [(r * cos(radians(a)), r * sin(radians(a))) for a in range(-90, 91, 5)]
            cv2.polylines(img, [np.int32(self.a_px(arco))], False, (200, 50, 160), 2)
            base = self.a_px([(0, 0)])[0]
            cv2.circle(img, (int(base[0]), int(base[1])), 6, (255, 255, 255), 2)

    def vista(self):
        f = self.frame()
        self._overlay(f)
        return f

    def _mostrar(self, img, nombre="vista.jpg"):
        cv2.imwrite(nombre, img)
        try:
            cv2.imshow(nombre, img)
            cv2.waitKey(1)
        except cv2.error:
            pass

    def mascaras(self):
        """Guarda mascaras.jpg: lo que 've' cada color en TODA la imagen (para afinar)."""
        hsv = self._hsv(self.frame())
        tiras = []
        for c in COLORES_ACTIVOS:
            t = cv2.cvtColor(self._mascara(hsv, c), cv2.COLOR_GRAY2BGR)
            t = cv2.resize(t, None, fx=0.5, fy=0.5)
            cv2.putText(t, c, (8, 24), FONT, 0.7, (0, 255, 0), 2)
            tiras.append(t)
        self._mostrar(np.hstack(tiras), "mascaras.jpg")

    # ---------- detección de fichas ----------
    def diagnostico(self):
        """Inspecciona candidatos sin mover el brazo ni relajar sus filtros.

        Compara geometría en píxeles y mm para revelar una transformación
        deformada. Guarda el fotograma original y todos los motivos de rechazo.
        """
        f = self.frame()
        cv2.imwrite("diagnostico_original.jpg", f)
        hsv = self._hsv(f)
        zona = np.zeros(f.shape[:2], np.uint8)
        hay_region = "fichas" in self.regiones
        if hay_region:
            cv2.fillPoly(zona, [np.int32(self.regiones["fichas"])], 255)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        registros = []
        paneles = []
        for color in COLORES_ACTIVOS:
            completa = self._mascara(hsv, color)
            panel = cv2.cvtColor(completa, cv2.COLOR_GRAY2BGR)
            cv2.putText(panel, color, (10, 25), FONT, 0.7, (0, 255, 0), 2)
            paneles.append(panel)
            print(f"{color}: {cv2.countNonZero(completa)} pixeles de color en imagen; "
                  f"{cv2.countNonZero(cv2.bitwise_and(completa, zona))} dentro de FICHAS")
            # Primero informa de objetos excluidos o recortados por la región.
            contornos, _ = cv2.findContours(completa, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contornos:
                if cv2.contourArea(c) < 40:
                    continue
                interior = np.zeros_like(zona)
                cv2.drawContours(interior, [c], -1, 255, -1)
                fraccion = cv2.countNonZero(cv2.bitwise_and(interior, zona)) / max(1, cv2.countNonZero(interior))
                if fraccion < 0.98:
                    x, y, _, _ = cv2.boundingRect(c)
                    motivo = "FUERA DE FICHAS" if fraccion == 0 else "RECORTADA POR FICHAS"
                    print(f"  {color} cerca de pixel ({x},{y}): {motivo}")
                    registros.append({"color": color, "pixel": [x, y], "motivos": [motivo]})
                    cv2.drawContours(f, [c], -1, (0, 0, 255), 1)
            m = cv2.bitwise_and(completa, zona)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
            m = cv2.bitwise_and(m, zona)
            contornos, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contornos:
                area_px = cv2.contourArea(c)
                if len(c) < 3 or area_px < 10:
                    continue
                hull = cv2.convexHull(c)
                solidez = area_px / max(cv2.contourArea(hull), 1e-6)
                forma_px, _, _ = clasificar(hull)
                r = {"id": len(registros) + 1, "color": color,
                     "area_px": area_px, "forma_px": forma_px,
                     "solidez": solidez, "motivos": []}
                if self.H1 is None:
                    r["motivos"].append("SIN CALIBRACION MM")
                else:
                    mm = np.float32(self.px_a_mm(hull.reshape(-1, 2), PIEZA_ALTURA)).reshape(-1, 1, 2)
                    if not np.isfinite(mm).all():
                        r["motivos"].append("TRANSFORMACION INVALIDA")
                    else:
                        area = cv2.contourArea(mm)
                        forma, rc, rr = clasificar(mm)
                        r.update(area_mm2=area, forma_mm=forma, rc=rc, rr=rr)
                        if not AREA_MIN_MM2 <= area <= AREA_MAX_MM2:
                            r["motivos"].append("AREA FUERA DE RANGO")
                        if forma == "desconocida":
                            r["motivos"].append("FORMA NO RECONOCIDA")
                        momentos = cv2.moments(mm)
                        if abs(momentos["m00"]) > 1e-6:
                            x, y = momentos["m10"] / momentos["m00"], momentos["m01"] / momentos["m00"]
                            r.update(x=x, y=y)
                            try:
                                ik(x, y, Z_SEGURO)
                                ik(x, y, config.Z_RECOGIDA)
                                r["alcance"] = "OK"
                            except Inalcanzable as e:
                                r["alcance"] = str(e)
                        else:
                            r["motivos"].append("AREA DEGENERADA")
                if solidez < 0.82:
                    r["motivos"].append("CONTORNO FRAGMENTADO O PIEZAS UNIDAS")
                registros.append(r)
                estado = "; ".join(r["motivos"]) or "ACEPTADA POR VISION"
                print(f"  #{r['id']} {color}: {estado} | forma px={forma_px}, "
                      f"forma mm={r.get('forma_mm', '?')}, area={r.get('area_mm2', 0):.1f} mm2, "
                      f"solidez={solidez:.2f}")
                if "x" in r:
                    print(f"    x={r['x']:.1f}, y={r['y']:.1f} mm | alcance: {r['alcance']}")
                x, y, _, _ = cv2.boundingRect(c)
                col = (0, 0, 255) if r["motivos"] else (0, 180, 0)
                cv2.drawContours(f, [c], -1, col, 2)
                cv2.putText(f, f"#{r['id']} {color}", (x, max(15, y - 5)), FONT, 0.5, col, 2)
        if not hay_region:
            print("Falta definir FICHAS: calibrar regiones")
        if not registros:
            print("No hay contornos de al menos 10 px2 dentro de FICHAS.")
        self._overlay(f)
        self._mostrar(f, "diagnostico.jpg")
        cv2.imwrite("diagnostico_mascaras.jpg", np.hstack(paneles))
        with open("diagnostico.json", "w") as archivo:
            json.dump({"version": "13", "rangos": RANGOS,
                       "archivo_calibracion": CALIB_ARCHIVO,
                       "H1": self.H1.tolist() if self.H1 is not None else None,
                       "H2": self.H2.tolist() if self.H2 is not None else None,
                       "h1": self.h1, "h2": self.h2,
                       "regiones": self.regiones, "candidatos": registros}, archivo, indent=2)
        print("Guardados diagnostico.jpg, diagnostico_original.jpg, diagnostico_mascaras.jpg y diagnostico.json")

    def detectar(self, mostrar=True):
        if getattr(self, "error_calibracion", None):
            raise RuntimeError("Calibración inválida: " + self.error_calibracion + ". Repite calibrar plano.")
        if self.H1 is None or "fichas" not in self.regiones:
            raise RuntimeError("Falta calibrar: usa 'calibrar plano' o 'calibrar punta', y luego 'calibrar regiones'.")
        f = self.frame()
        hsv = self._hsv(f)
        zona = np.zeros(f.shape[:2], np.uint8)
        cv2.fillPoly(zona, [np.int32(self.regiones["fichas"])], 255)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        objs = []

        for color in COLORES_ACTIVOS:
            m = cv2.bitwise_and(self._mascara(hsv, color), zona)
            # Limpieza suave: los kernels grandes borraban las puntas.
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
            m = cv2.bitwise_and(m, zona)
            cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if len(c) < 3 or cv2.contourArea(c) <= 0:
                    continue
                hull = cv2.convexHull(c)
                hull_mm = np.float32(self.px_a_mm(hull.reshape(-1, 2), PIEZA_ALTURA)).reshape(-1, 1, 2)
                area = cv2.contourArea(hull_mm)
                if area < AREA_MIN_MM2 or area > AREA_MAX_MM2:
                    continue
                forma, rc, rr = clasificar(hull_mm)
                # Evita convertir fragmentos de fondo en objetivos del brazo.
                solidez = cv2.contourArea(c) / max(cv2.contourArea(hull), 1e-6)
                if forma == "desconocida" or solidez < 0.82:
                    continue
                M = cv2.moments(hull_mm)
                cx, cy = M["m10"] / M["m00"], M["m01"] / M["m00"]
                objs.append({"color": color, "forma": forma, "x": float(cx), "y": float(cy),
                             "area": area, "rc": rc, "rr": rr})
                Mp = cv2.moments(hull)
                px_c = (int(Mp["m10"] / Mp["m00"]), int(Mp["m01"] / Mp["m00"]))
                cv2.drawContours(f, [hull], -1, COLOR_BGR[color], 2)
                cv2.putText(f, f"{color} {forma}", (px_c[0] - 25, px_c[1] - 12), FONT, 0.5, (0, 0, 0), 2)

        self._overlay(f)
        if mostrar:
            self._mostrar(f)
        return objs
