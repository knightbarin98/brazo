"""Enseñanza empírica por postura con marcadores verde/magenta.

NO proyecta el marcador sobre la mesa con una homografía: aprende, a la
postura de recogida, qué píxel del marcador corresponde a qué punto real.
"""

import json
import os
import time
from math import hypot

import cv2
import numpy as np

from .. import config
from ..ajustes import AJUSTES, aplicar_altura, guardar_ajustes, garra_abierta, numero_en_rango
from ..cinematica import Inalcanzable, firma_geometria, firma_postura, ik, z_transito
from ..config import (DIST_OFFSET_APRENDIDO, ENSEÑANZA_ARCHIVO, FONT, MARGEN_APROX, NOMBRES_MARCADOR,
                      COLOR_DIBUJO_MARCADOR, PUNTOS_MM, RADIO_BUSQUEDA_APRENDIDO_PX, RADIO_BUSQUEDA_PX,
                      RADIO_CORRECCION, RADIO_DESVANECER, TOL_INCL_MUESTRA, TOL_Z_GUARDAR, TOL_Z_MUESTRA,
                      MARCADORES_ARCHIVO, Z_SEGURO)
from ..ui import leer_tecla, tecla_flecha, texto_con_borde, ventana_cerrada
from .base import Vision
from .marcadores import (MODELOS_COLOR_MARCADOR, cargar_marcadores, detectar_marcadores, dibujar_marcadores,
                         guardar_marcadores, mascara_con_modelo, mascara_marcador, modelo_color_desde_muestras,
                         modelo_marcador, preparar_color)


class VisionEnsenanza(Vision):
    def __init__(self, camara):
        super().__init__(camara)
        self.ensenanza = []
        if os.path.exists(ENSEÑANZA_ARCHIVO):
            with open(ENSEÑANZA_ARCHIVO) as f:
                self.ensenanza = json.load(f).get('muestras', [])
        cargar_marcadores()
        print('V13: calibrar plano -> calibrar recogida x y -> calibrar marcador magenta -> calibrar manual/piezas.')
        self.resumen_muestras()

    def detectar(self, mostrar=True):
        if self.H2 is not None:
            raise ValueError('La geometría cambió: ejecuta calibrar plano antes de detectar o recoger.')
        return super().detectar(mostrar=mostrar)

    # ---------- muestras ----------
    def motivo_incompatible(self, r):
        if self.H1 is None:
            return "sin calibrar plano"
        f = r.get("firma") or {}
        if f.get("geometria") != firma_geometria():
            return "geometría distinta (L1/L2/L3/H/MAPEO)"
        plano = r.get("plano")
        if (plano is None or self.H2 is not None
                or not np.allclose(np.asarray(plano, float), self.H1, rtol=1e-6, atol=1e-9)):
            return "hechas con otro 'calibrar plano'"
        if abs(float(f.get("z", 1e9)) - config.Z_RECOGIDA) > TOL_Z_MUESTRA:
            return f"Z de recogida distinta (>{TOL_Z_MUESTRA:.0f} mm)"
        if abs(float(f.get("inclinacion", 1e9)) - AJUSTES["inclinacion"]) > TOL_INCL_MUESTRA:
            return f"inclinación distinta (>{TOL_INCL_MUESTRA:.0f}°)"
        return None

    def muestras_activas(self):
        return [r for r in self.ensenanza if self.motivo_incompatible(r) is None]

    def resumen_muestras(self):
        motivos = {}
        for r in self.ensenanza:
            m = self.motivo_incompatible(r) or "ACTIVAS"
            motivos[m] = motivos.get(m, 0) + 1
        if not motivos:
            print("Sin muestras de enseñanza.")
        for m, n in sorted(motivos.items(), key=lambda t: t[0] != "ACTIVAS"):
            print(f"  Muestras {m}: {n}" if m == "ACTIVAS" else f"  Ignoradas ({m}): {n}")

    def guardar_ensenanza(self, nuevas):
        with open(ENSEÑANZA_ARCHIVO + '.tmp', 'w') as f:
            json.dump({'version': 13, 'muestras': nuevas}, f, indent=2)
        os.replace(ENSEÑANZA_ARCHIVO + '.tmp', ENSEÑANZA_ARCHIVO)
        self.ensenanza = nuevas

    def corregir_xy(self, x, y):
        """Interpola la corrección (comando - objetivo) por distancia inversa.

        Completa hasta RADIO_CORRECCION del punto enseñado más cercano, y se
        desvanece hasta cero en RADIO_DESVANECER mm más: no se extrapola lejos
        de lo enseñado (por ejemplo hacia A/B si no se enseñaron puntos ahí).
        """
        x, y = float(x), float(y)
        muestras = self.muestras_activas() if self.H1 is not None else []
        if not muestras:
            return x, y
        pesos, deltas, dmin = [], [], float("inf")
        for r in muestras:
            ox, oy = map(float, r['objetivo_mm'][:2])
            cx, cy = map(float, r['comando_mm'][:2])
            d = hypot(x - ox, y - oy)
            if d < 1e-6:
                return cx, cy
            dmin = min(dmin, d)
            pesos.append(1.0 / (d * d))
            deltas.append((cx - ox, cy - oy))
        suma = sum(pesos)
        dx = sum(w * d[0] for w, d in zip(pesos, deltas)) / suma
        dy = sum(w * d[1] for w, d in zip(pesos, deltas)) / suma
        # Distancia al ÁREA enseñada (0 dentro del polígono de puntos), no al punto más cercano.
        fuera = dmin
        if len(muestras) >= 3:
            casco = cv2.convexHull(np.float32([r['objetivo_mm'][:2] for r in muestras]))
            if cv2.contourArea(casco) > 1:
                fuera = max(0.0, -cv2.pointPolygonTest(casco, (x, y), True))
        factor = float(np.clip(1 - (fuera - RADIO_CORRECCION) / RADIO_DESVANECER, 0, 1))
        return x + factor * dx, y + factor * dy

    def offset_marcador(self, color, cerca_de):
        """(píxel del marcador - píxel de la punta real), ponderado por cercanía a cerca_de.

        El marcador va por encima de la cara, así que su desfase en la imagen
        cambia con la posición (paralaje): el aprendido en P1/P2 no vale tal
        cual en P3/P4. Devuelve (offset, distancia mm a la muestra más cercana).
        """
        if self.Hinv0 is None:
            return None, None
        muestras = [r for r in self.muestras_activas() if color in r.get('marcadores', {})]
        if not muestras:
            return None, None
        offs = np.array([np.subtract(r['marcadores'][color], self.a_px([r['objetivo_mm'][:2]])[0])
                         for r in muestras])
        dist = np.array([hypot(r['objetivo_mm'][0] - cerca_de[0], r['objetivo_mm'][1] - cerca_de[1])
                         for r in muestras])
        pesos = 1.0 / np.maximum(dist, 1.0) ** 2
        return (pesos[:, None] * offs).sum(axis=0) / pesos.sum(), float(dist.min())

    def esperados_para(self, pose):
        if pose is None or self.Hinv0 is None:
            return {}, {}
        cx, cy = self.corregir_xy(pose[0], pose[1])
        real = (2 * pose[0] - cx, 2 * pose[1] - cy)       # inversa aproximada de la corrección
        punta = self.a_px([real])[0]
        esperados, radios = {}, {}
        for c in NOMBRES_MARCADOR:
            off, dmin = self.offset_marcador(c, real)
            esperados[c] = punta if off is None else punta + off
            # Círculo estrecho sólo cerca de un punto ya enseñado; lejos, búsqueda amplia.
            cerca = off is not None and dmin <= DIST_OFFSET_APRENDIDO
            radios[c] = RADIO_BUSQUEDA_APRENDIDO_PX if cerca else RADIO_BUSQUEDA_PX
        return esperados, radios

    def estimar_punta(self, detecciones, z, modelos=None):
        if abs(z - config.Z_RECOGIDA) > TOL_Z_GUARDAR:
            return None
        if modelos is None:
            modelos = {c: modelo_marcador(self.muestras_activas(), c) for c in NOMBRES_MARCADOR}
        for color in NOMBRES_MARCADOR:      # magenta en la punta tiene prioridad
            modelo = modelos.get(color)
            if modelo is None or color not in detecciones:
                continue
            px = detecciones[color]
            matriz, hull = modelo
            if cv2.pointPolygonTest(hull, (float(px[0]), float(px[1])), False) < 0:
                continue
            return color, np.asarray([*px, 1.]) @ matriz
        return None

    # ---------- marcadores: diagnóstico y calibración ----------
    def ver_marcadores(self):
        activas = self.muestras_activas() if self.H1 is not None else []
        for c in NOMBRES_MARCADOR:
            n = sum(c in r.get('marcadores', {}) for r in activas)
            ok = modelo_marcador(activas, c) is not None
            print(f"{c}: {'calibrado' if c in MODELOS_COLOR_MARCADOR else 'regla por defecto'}; "
                  f"{n} referencias; modelo {'válido' if ok else 'pendiente (4 puntos distribuidos, error <= 8 mm)'}")
        win = "Marcadores en vivo | ESC salir | G guarda marcadores.jpg"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        visto = [False]
        try:
            while True:
                f = self.frame_vivo()
                _, detalle = detectar_marcadores(f)
                dibujar_marcadores(f, detalle)
                for i, (n, d) in enumerate(detalle.items()):
                    texto_con_borde(f, f"{n}: {d['motivo']}", (8, 22 + 22 * i), 0.5, COLOR_DIBUJO_MARCADOR[n])
                texto_con_borde(f, "Blanco = manchas candidatas. Si algo rojizo se colorea: calibrar marcador magenta",
                                (8, f.shape[0] - 10), 0.42)
                cv2.imshow(win, f)
                k = leer_tecla(30)
                if k == 27 or ventana_cerrada(win, visto):
                    break
                if k in (ord('g'), ord('G')):
                    cv2.imwrite("marcadores.jpg", f)
                    print("Guardado marcadores.jpg")
        finally:
            cv2.destroyWindow(win)

    def calibrar_marcador(self, nombre):
        if nombre not in NOMBRES_MARCADOR:
            raise ValueError("Usa: calibrar marcador magenta | calibrar marcador verde")
        print(f"\n=== CALIBRAR MARCADOR {nombre.upper()} ===")
        print("Clic IZQUIERDO: 3 o más puntos DENTRO del marcador (centro y orillas, sin brillo).")
        print("Clic DERECHO: cosas que NO son el marcador y se confunden (fichas café/rojizas, piel, mesa).")
        print("ENTER calcula y muestra la máscara | G guarda | BACKSPACE deshace | X borra calibración | ESC sale")
        win = f"Calibrar marcador {nombre}"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        clics, hechos = [], []          # hechos: (signo, x, y, parche Lab)
        visto = [False]

        def clic(ev, x, y, flags, param):
            if ev == cv2.EVENT_LBUTTONDOWN:
                clics.append(("+", x, y))
            elif ev == cv2.EVENT_RBUTTONDOWN:
                clics.append(("-", x, y))

        cv2.setMouseCallback(win, clic)
        modelo, informe = None, ""
        aviso = "Haz clics: izquierdo = marcador, derecho = lo que NO es."
        try:
            while True:
                f = self.frame_vivo()
                lab, hsv = preparar_color(f)
                while clics:
                    s, x, y = clics.pop(0)
                    hechos.append((s, x, y, lab[max(0, y - 3):y + 4, max(0, x - 3):x + 4].reshape(-1, 3).copy()))
                    modelo = None
                    aviso = f"{sum(h[0] == '+' for h in hechos)} positivos, {sum(h[0] == '-' for h in hechos)} negativos. ENTER calcula."
                vista = f.copy()
                if modelo is not None:
                    mascara = mascara_con_modelo(lab, modelo)
                    etiqueta = "MASCARA NUEVA (G guarda)"
                else:
                    mascara = mascara_marcador(lab, hsv, nombre)
                    etiqueta = "mascara ACTUAL (" + ("calibrada" if nombre in MODELOS_COLOR_MARCADOR else "regla por defecto") + ")"
                capa = np.zeros_like(vista)
                capa[mascara > 0] = COLOR_DIBUJO_MARCADOR[nombre]
                cv2.addWeighted(capa, 0.5, vista, 1.0, 0, vista)
                for s, x, y, _ in hechos:
                    if s == "+":
                        cv2.circle(vista, (x, y), 6, (0, 255, 0), 2)
                    else:
                        cv2.drawMarker(vista, (x, y), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
                texto_con_borde(vista, etiqueta, (8, 22), 0.55)
                texto_con_borde(vista, "IZQ marcador | DER no-marcador | ENTER calcular | G guardar | X borrar | ESC", (8, 46))
                for i, linea in enumerate((informe.splitlines() or [""])[:2] + [aviso]):
                    texto_con_borde(vista, linea[:110], (8, vista.shape[0] - 52 + 20 * i), 0.42, (0, 230, 255))
                cv2.imshow(win, vista)
                k = leer_tecla(30)
                if k == 27 or ventana_cerrada(win, visto):
                    print("Salida sin guardar cambios de color." if modelo is not None else "Salida.")
                    break
                if k in (8, 127) and hechos:
                    hechos.pop()
                    modelo = None
                    aviso = "Último clic deshecho."
                elif k in (10, 13):
                    pos = [h[3] for h in hechos if h[0] == "+"]
                    neg = [h[3] for h in hechos if h[0] == "-"]
                    if len(pos) < 3:
                        aviso = "Faltan clics: mínimo 3 dentro del marcador."
                        continue
                    try:
                        modelo, informe = modelo_color_desde_muestras(pos, neg)
                        print(informe)
                        aviso = "Revisa: solo el marcador debe quedar coloreado. G guarda."
                    except ValueError as e:
                        modelo, aviso = None, str(e)
                        print(aviso)
                elif k in (ord('g'), ord('G')):
                    if modelo is None:
                        aviso = "Primero ENTER para calcular."
                        continue
                    anterior = MODELOS_COLOR_MARCADOR.get(nombre)
                    MODELOS_COLOR_MARCADOR[nombre] = modelo
                    try:
                        guardar_marcadores()
                    except OSError:
                        if anterior is None:
                            MODELOS_COLOR_MARCADOR.pop(nombre, None)
                        else:
                            MODELOS_COLOR_MARCADOR[nombre] = anterior
                        raise
                    print(f"Color del marcador {nombre} guardado en {MARCADORES_ARCHIVO}.")
                    break
                elif k in (ord('x'), ord('X')):
                    if MODELOS_COLOR_MARCADOR.pop(nombre, None) is not None:
                        guardar_marcadores()
                        print(f"Calibración de {nombre} borrada: vuelve la regla por defecto.")
                    modelo, hechos[:] = None, []
                    aviso = "Regla por defecto activa."
        finally:
            cv2.destroyWindow(win)

    # ---------- compatibilidad de órdenes antiguas ----------
    def calibrar_manual(self, brazo):
        self.ensenar(brazo, piezas=False)

    def calibrar_punta(self, brazo):
        print('V13: usa calibrar plano y calibrar manual/piezas. No mezcles marcadores en una homografía.')

    def comprobar(self, brazo, x, y, z=None):
        print('Para verificar: calibrar piezas (la ventana muestra la punta estimada) o probar x y.')

    def mostrar_registros_brazo(self):
        self.resumen_muestras()
        for i, r in enumerate(self.muestras_activas(), 1):
            o, c = r['objetivo_mm'], r['comando_mm']
            print(f"  {i}: objetivo=({o[0]:.1f},{o[1]:.1f}) -> comando=({c[0]:.1f},{c[1]:.1f},{c[2]:.1f}) "
                  f"delta=({c[0]-o[0]:+.1f},{c[1]-o[1]:+.1f}) | marcadores: {list(r.get('marcadores', {}))}")

    def probar_correccion_manual(self, brazo, x, y, z=None):
        z = config.Z_RECOGIDA if z is None else z
        cx, cy = self.corregir_xy(x, y)
        print(f"Objetivo ({x:.1f},{y:.1f}) -> comando ({cx:.1f},{cy:.1f},{z:.1f})")
        plan = brazo.plan_vertical(cx, cy, Z_SEGURO, z)
        brazo.mover(plan[0] + [garra_abierta()])
        brazo.ejecutar_vertical(plan, garra_abierta(), AJUSTES['velocidad_bajada'])
        brazo.ajustar_velocidad(AJUSTES['velocidad'], guardar=False)
        print('En posición; garra abierta. No se regresa automáticamente (usa home).')

    # ---------- enseñanza ----------
    def ensenar(self, brazo, piezas=False):
        if self.H1 is None or self.H2 is not None or self.error_calibracion:
            raise ValueError('Primero calibrar plano con medidas reales; no usar el mapa automático antiguo.')
        objetivos = (self.detectar(mostrar=False) if piezas else
                     [dict(x=float(x), y=float(y), color='marca', forma=str(i + 1)) for i, (x, y) in enumerate(PUNTOS_MM)])
        if not objetivos:
            print('No se detectaron piezas. No se mueve el brazo.')
            return
        incl = AJUSTES['inclinacion']
        print('Piezas/referencias congeladas: no las muevas durante la sesión.')
        print(f'N/APROXIMAR deja la cara {MARGEN_APROX:.0f} mm sobre Z objetivo={config.Z_RECOGIDA:.1f}. '
              f'Z baja a la altura objetivo. U/J ajustan altura.')
        print(f'ENTER guarda XY y Z actuales; si Z difiere más de {TOL_Z_GUARDAR:.0f} mm, '
              'esa Z sobrescribe la Z de recogida. K omite, BACKSPACE punto anterior, ESC termina.')
        print('Garra abierta. Los marcadores son una ayuda; confirma mirando la garra de lado.')
        cv2.destroyAllWindows()
        win = 'V13 Calibracion'
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        visto = [False]
        estado = {"pose": None}
        indice, paso, posicionado = 0, 1., -1
        aviso = 'Haz clic en APROXIMAR o pulsa N para comenzar.'
        pendientes, botones = [], []

        def clic(evento, x, y, flags, param):
            if evento == cv2.EVENT_LBUTTONDOWN and not pendientes:
                for x1, y1, x2, y2, _, tecla in botones:
                    if x1 <= x < x2 and y1 <= y < y2:
                        pendientes.append(tecla)
                        break

        cv2.setMouseCallback(win, clic)
        anterior = brazo.velocidad
        vel_lenta = 0.5
        vel_transito = float(np.clip(AJUSTES['velocidad'], 0.5, 2.0))
        modelos = {c: modelo_marcador(self.muestras_activas(), c) for c in NOMBRES_MARCADOR}

        def validar_linea(a, b):
            n = max(2, int(np.ceil(np.linalg.norm(np.subtract(b, a)))) + 1)
            for p in np.linspace(a, b, n):
                ik(*map(float, p), incl)

        def mover_linea(desde, hasta, paso_mm=2., vel=vel_lenta):
            validar_linea(desde, hasta)
            if abs(brazo.velocidad - vel) > 1e-6:
                brazo.ajustar_velocidad(vel, guardar=False)
            n = max(1, int(np.ceil(np.linalg.norm(np.subtract(hasta, desde)) / paso_mm)))
            for p in np.linspace(desde, hasta, n + 1)[1:]:
                brazo.mover(ik(*map(float, p), incl) + [garra_abierta()])
                estado["pose"] = [float(v) for v in p]

        def aproximar(o):
            cx, cy = self.corregir_xy(o['x'], o['y'])
            z_obj = float(config.Z_RECOGIDA)
            tope_dest = z_transito(cx, cy, z_obj)
            llegada = [cx, cy, min(z_obj + MARGEN_APROX, tope_dest)]
            pose = estado["pose"]
            if pose is None:            # desde HOME: movimiento articular arriba del destino
                brazo.ajustar_velocidad(vel_transito, guardar=False)
                brazo.ir(cx, cy, tope_dest, garra_abierta(), incl)
                estado["pose"] = [cx, cy, tope_dest]
                mover_linea(estado["pose"], llegada)
                return
            tope_orig = z_transito(pose[0], pose[1], pose[2])
            z_t = min(tope_orig, tope_dest)
            while z_t >= llegada[2]:
                tramos = [(pose, [pose[0], pose[1], z_t]),
                          ([pose[0], pose[1], z_t], [cx, cy, z_t]),
                          ([cx, cy, z_t], llegada)]
                try:
                    for a, b in tramos:     # se valida TODA la ruta antes de mover
                        validar_linea(a, b)
                    break
                except Inalcanzable:
                    z_t -= 2.0
            else:
                raise Inalcanzable("no hay altura de tránsito común entre los dos puntos")
            if z_t < Z_SEGURO - 0.5:
                print(f"  Tránsito a Z={z_t:.0f} mm (Z_SEGURO={Z_SEGURO} no alcanzable en algún tramo)")
            mover_linea(pose, [pose[0], pose[1], z_t], 2., vel_lenta)
            mover_linea(estado["pose"], [cx, cy, z_t], 5., vel_transito)
            mover_linea(estado["pose"], llegada, 2., vel_lenta)

        controles = [('APROXIMAR', ord('n')), ('X+', ord('w')), ('X-', ord('s')),
                     ('Y+', ord('a')), ('Y-', ord('d')), ('SUBIR', ord('u')),
                     ('BAJAR', ord('j')), ('IR A Z', ord('z')),
                     ('GUARDAR', 13), ('OMITIR', ord('k')), ('ANTERIOR', 8),
                     ('PASO 0.5', ord('0')), ('PASO 1', ord('1')), ('PASO 2', ord('2')),
                     ('PASO 5', ord('5')), ('SALIR', 27)]
        try:
            while indice < len(objetivos):
                o = objetivos[indice]
                pose = estado["pose"]
                f = self.frame_vivo()
                esperados, radios = self.esperados_para(pose)
                detecciones, detalle = detectar_marcadores(f, esperados, radios)
                dibujar_marcadores(f, detalle, tinte=False)
                for i, obj in enumerate(objetivos):
                    p = tuple(np.int32(np.round(self.a_px([(obj['x'], obj['y'])])[0])))
                    cv2.drawMarker(f, p, (0, 0, 255) if i == indice else (255, 255, 255), cv2.MARKER_CROSS, 16, 2)
                    cv2.putText(f, str(i + 1), p, FONT, .6, (0, 0, 0), 2)
                if posicionado == indice and pose is not None:
                    dz = pose[2] - config.Z_RECOGIDA
                    estado_txt = f'XYZ ({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f})  Z-obj={dz:+.1f} mm'
                else:
                    estado_txt = 'pulsa APROXIMAR'
                est = self.estimar_punta(detecciones, pose[2], modelos) if pose is not None else None
                lineas = [f'P{indice + 1}/{len(objetivos)} {o["color"]} {o["forma"]} | {estado_txt}',
                          f'Z objetivo {config.Z_RECOGIDA:.1f} | inclinacion {incl:.1f} | paso {paso} mm',
                          *(f'{c}: {detalle[c]["motivo"]}' for c in NOMBRES_MARCADOR),
                          f'Punta estimada ({est[0]}): {np.round(est[1], 1)}' if est else
                          'Sin estimacion de punta: confirma mirando de lado.']
                for i, t in enumerate(lineas):
                    texto_con_borde(f, t, (8, 22 + 22 * i))
                ancho, h = max(640, f.shape[1]), f.shape[0]
                por_fila = 8
                panel = np.zeros((h + 2 * 36 + 28, ancho, 3), np.uint8)
                panel[:h, :f.shape[1]] = f
                botones.clear()
                for i, (etiqueta, tecla) in enumerate(controles):
                    col, fila = i % por_fila, i // por_fila
                    x1, y1 = col * (ancho // por_fila), h + fila * 36
                    x2, y2 = x1 + ancho // por_fila - 3, y1 + 32
                    botones.append((x1, y1, x2, y2, etiqueta, tecla))
                    cv2.rectangle(panel, (x1, y1), (x2, y2), (65, 65, 65), -1)
                    cv2.putText(panel, etiqueta, (x1 + 3, y1 + 21), FONT, .40, (255, 255, 255), 1)
                cv2.putText(panel, aviso[:100], (8, h + 2 * 36 + 18), FONT, .43, (0, 230, 255), 1)
                cv2.imshow(win, panel)
                k = leer_tecla(30)
                if pendientes:
                    k = pendientes.pop(0)
                if 65 <= k <= 90:
                    k += 32                     # mayúsculas -> minúsculas
                if ventana_cerrada(win, visto) or k == 27:
                    break
                if k == -1:
                    continue
                try:
                    if k in (ord('0'), ord('1'), ord('2'), ord('5')):
                        paso = .5 if k == ord('0') else float(chr(k))
                    elif k == ord('n'):
                        aviso = 'Moviendo...'
                        aproximar(o)
                        posicionado = indice
                        aviso = 'Listo: ajusta XY y la altura con U/J (Z = objetivo actual); ENTER guarda esa XYZ.'
                    elif k == ord('k'):
                        indice += 1
                        aviso = 'Punto omitido; APROXIMAR para el siguiente.'
                    elif k in (8, 127):
                        if indice > 0:
                            indice -= 1
                            aviso = f'De vuelta al punto {indice + 1}: APROXIMAR para repetirlo.'
                    elif k in (10, 13):
                        if pose is None or posicionado != indice:
                            aviso = 'Primero APROXIMAR este punto.'
                        else:
                            if abs(pose[2] - config.Z_RECOGIDA) > TOL_Z_GUARDAR:
                                # La Z enseñada manda: se adopta como nueva Z de recogida.
                                z_previa, activas_previas = config.Z_RECOGIDA, len(self.muestras_activas())
                                aplicar_altura(pose[2])
                                try:
                                    guardar_ajustes()
                                except OSError:
                                    aplicar_altura(z_previa)
                                    raise
                                print(f'Z de recogida sobrescrita: {z_previa:.1f} -> {config.Z_RECOGIDA:.1f} mm (ajustes.json)')
                                perdidas = activas_previas - len(self.muestras_activas())
                                if perdidas:
                                    print(f'  {perdidas} muestra(s) hechas a otra Z quedan ignoradas; repítelas a esta Z.')
                            r = {'objetivo_mm': [float(o['x']), float(o['y'])], 'comando_mm': list(pose),
                                 'firma': firma_postura(), 'plano': self.H1.tolist(), 'marcadores': detecciones,
                                 'color': o['color'], 'forma': o['forma'], 'timestamp': time.time()}
                            nuevas = [v for v in self.ensenanza
                                      if not (self.motivo_incompatible(v) is None and
                                              hypot(v['objetivo_mm'][0] - o['x'], v['objetivo_mm'][1] - o['y']) < 2)]
                            self.guardar_ensenanza(nuevas + [r])
                            modelos = {c: modelo_marcador(self.muestras_activas(), c) for c in NOMBRES_MARCADOR}
                            d = (pose[0] - o['x'], pose[1] - o['y'])
                            print(f'P{indice + 1} guardado: delta=({d[0]:+.1f},{d[1]:+.1f}) mm; '
                                  f'marcadores vistos: {list(detecciones) or "ninguno"}')
                            if not detecciones:
                                print('  (sin marcador: la corrección XY sí sirve; la estimación visual de punta no)')
                            indice += 1
                            aviso = 'Guardado. APROXIMAR para el siguiente punto.'
                    elif pose is not None and posicionado == indice:
                        nueva = list(pose)
                        direccion = tecla_flecha(k)
                        if direccion == 'arriba':
                            nueva[0] += paso
                        elif direccion == 'abajo':
                            nueva[0] -= paso
                        elif direccion == 'izquierda':
                            nueva[1] += paso
                        elif direccion == 'derecha':
                            nueva[1] -= paso
                        elif k == ord('u'):
                            nueva[2] += paso
                        elif k == ord('j'):
                            nueva[2] -= paso
                        elif k == ord('z'):
                            nueva[2] = float(config.Z_RECOGIDA)
                        else:
                            continue
                        numero_en_rango(nueva[2], -100, Z_SEGURO, 'Altura')
                        mover_linea(pose, nueva, 1., vel_lenta)
                        aviso = 'Movimiento realizado. ENTER guarda la alineación.'
                    else:
                        aviso = 'Haz clic en APROXIMAR o pulsa N antes de mover.'
                except (Inalcanzable, ValueError) as e:
                    aviso = 'Rechazado: ' + str(e)
                    print(aviso)
        finally:
            cv2.destroyWindow(win)
            brazo.ajustar_velocidad(anterior, guardar=False)
            print('Sesión finalizada. Se conservan los puntos confirmados; brazo quieto, garra abierta.')
            self.resumen_muestras()
