#!/usr/bin/env python3
"""
Brazo robótico - Programa de la Raspberry Pi (v13)

Cambios v13:
  - Marcador MAGENTA detectado en espacio Lab (magenta: a alto y b<=~130;
    rojo/café/piel/madera tienen b>=150). Ya no se confunde con rojizos.
  - 'calibrar marcador magenta|verde': clic izq = marcador, clic der = cosas
    que NO son el marcador (fichas rojizas, etc.). Ajusta el umbral para
    excluirlas y guarda en marcadores.json.
  - El marcador se busca cerca de donde debe estar la punta (círculo en pantalla)
    y se ignoran manchas lejanas; ya no falla si hay dos manchas parecidas.
  - 'marcadores' abre una vista en vivo con máscara y candidato elegido.
  - Enseñanza (calibrar manual/piezas): altura de tránsito adaptativa (P3/P4 ya
    no se rechazan), APROXIMAR baja cerca de la Z objetivo, tecla Z/botón IR A Z,
    ENTER guarda la Z enseñada (si difiere >3 mm sobrescribe la Z de recogida).
  - Las muestras ya no se pierden por cambios mínimos de Z/inclinación
    (tolerancia) y se informa cuántas se ignoran y por qué.
  - Corrección XY interpolada entre puntos enseñados (no solo a <20 mm).
  - Teclado robusto con NumLock activo (bits de modificador) y cierre de ventana.
  - Errores de OpenCV ya no cierran el programa.

Cambios anteriores:
  - V10: inclinación J/L en calibración de recogida, compensada por cinemática.
  - V10: descenso/ascenso segmentados, velocidades por fase y pausas configurables.
  - V10: probar agarre x y ejecuta una recogida sin trasladar al destino.
  - V9: ajustar altura x y: cámara en vivo, W/S para subir/bajar y G para guardar.
  - V9: velocidad 0.5..4 grados/paso, persistente en ajustes.json.
  - Actuador = electroimán ON/OFF (el 5º valor que se envía al Arduino es 1 o 0).
  - Marcador verde a 90 mm de la punta: la calibración se hace a DOS alturas para
    corregir el paralaje (el marcador está más cerca de la cámara que las fichas).
  - Clasificación de figuras por proporciones (más robusta para triángulos),
    con filtro de tamaño en mm (no depende de la distancia de la cámara).
  - 'calibrar colores': aprende el color de tus fichas con unos clics.
  - Calibración manual enseñada: cámara en vivo + flechas para alinear la cara
    del electroimán sobre 4 puntos reales y guardar correcciones mecánicas XY.

Regiones (4 esquinas cada una, clics sobre la imagen aérea): FICHAS, A y B.
Coordenadas (mm): origen en el eje de la base, x = adelante, y = izquierda, z = altura.

Instalar:  sudo apt install python3-opencv python3-numpy python3-serial
Ejecutar:  python3 brazo_pi_v13.py
"""

import json
import os
import re
import time
import traceback
import unicodedata
from math import acos, atan2, cos, degrees, hypot, pi, radians, sin, sqrt

import cv2
import numpy as np
import serial

# ======================================================================
#  CONFIGURACIÓN  (lo marcado con  # CALIBRAR  hay que ajustarlo)
# ======================================================================
PUERTO = "/dev/ttyACM0"
BAUD = 115200
CAMARA = 0

# Geometría del brazo (mm)
L1, L2 = 170, 150
L3 = 90.0        # eje de muñeca -> cara de contacto; ajuste: longitud iman 90
H = 45           # altura del eje del hombro

PASO_GRADOS = 2.0            # grados por paso de 15 ms (se envía al Arduino)

# servo = cero + signo * angulo_geometrico
MAPEO = {                    # CALIBRAR  (cero, signo)
    "base":   (90, +1),
    "hombro": (0,  +1),
    "codo":   (0,  -1),
    "muneca": (90, +1),
}

HOME = [90, 120, 97, 97]                # base, hombro, codo, muñeca (debe quedar FUERA de la región de fichas)
ARDUINO_HOME = [90, 90, 97, 97, 0]       # posición inicial del firmware (5º valor = imán)

# --- Marcador verde y electroimán ---
DIST_MARCADOR = 90           # mm del marcador verde a la cara del electroimán (a lo largo del actuador)
PIEZA_ALTURA = 8             # CALIBRAR: altura de la cara superior de las fichas sobre la mesa (mm)

# Alturas de la punta (cara del electroimán) sobre la mesa (mm)
Z_AGARRE = PIEZA_ALTURA + 1  # altura teórica original de agarre
BAJADA_EXTRA_AGARRE = 60.0     # mm extra que debe bajar el electroimán al recoger
Z_RECOGIDA = Z_AGARRE - BAJADA_EXTRA_AGARRE
Z_SEGURO = 60
Z_SOLTAR = 25

# --- Calibración con el marcador (dos alturas de la punta) ---
Z_CAL_BAJO = 10
Z_CAL_ALTO = 90
PUNTOS_CAL_PUNTA = [(160, -60), (160, 60), (240, -60), (240, 60), (200, 0), (270, 0)]   # CALIBRAR (mm, alcanzables)
MARCADOR_HSV = ((40, 100, 80), (85, 255, 255))   # marcador VERDE   # CALIBRAR
MARCADOR_AREA_MIN = 40

# --- Calibración manual del plano y enseñanza del electroimán ---
# Estos 4 puntos deben estar marcados físicamente SOBRE LA MESA y medidos desde
# el eje de la base. Primero se usan en "calibrar plano"; después, en
# "calibrar manual", se alinea sobre ellos la CARA DEL ELECTROIMÁN.
# P3/P4 a x=250: con inclinación cercana a -98° la muñeca pasa de 0° en x=280.
PUNTOS_MM = [(150, -100), (150, 100), (250, 100), (250, -100)]   # CALIBRAR
Z_CAL_MANUAL = Z_AGARRE       # altura de la cara del electroimán durante la enseñanza
PASO_CAL_INICIAL = 5.0        # mm por pulsación; 1/2/5 cambian el paso durante la calibración

SLOTS_X, SLOTS_Y = 3, 3      # casillas dentro de cada región de acomodo

# Una ruta fija evita cargar un JSON diferente según desde dónde se ejecuta.
DIRECTORIO = os.path.dirname(os.path.abspath(__file__))
CALIB_ARCHIVO = os.path.join(DIRECTORIO, "calibracion.json")
AJUSTES_ARCHIVO = os.path.join(DIRECTORIO, "ajustes.json")
COLORES_ARCHIVO = os.path.join(DIRECTORIO, "colores.json")

# --- Colores de fichas ---
# Cambia aquí los colores en uso. Opciones ya definidas: cafe, azul, amarillo, rojo, violeta
COLORES_ACTIVOS = ["cafe", "azul", "amarillo"]
RAICES = {                   # palabras que reconoce la orden
    "cafe": ("caf", "marr"),
    "azul": ("azul", "celest"),
    "amarillo": ("amarill",),
    "rojo": ("roj",),
    "violeta": ("viol", "morad"),
}
# Rangos HSV (OpenCV: H 0-180). Son un punto de partida: usa 'calibrar colores'.   # CALIBRAR
RANGOS = {
    "cafe":     [((0, 45, 20), (21, 255, 225)),
                 ((22, 60, 20), (25, 255, 160))],  # café rojizo, claro y oscuro
    "azul":     [((90, 50, 90), (125, 255, 255))],     # azul claro
    "amarillo": [((22, 100, 165), (25, 255, 255)),
                 ((26, 100, 120), (35, 255, 255))],  # separa ocre oscuro/café
    "rojo":     [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "violeta":  [((125, 60, 40), (160, 255, 255))],
}
COLOR_BGR = {"cafe": (30, 70, 120), "azul": (230, 180, 60), "amarillo": (0, 220, 255),
             "rojo": (0, 0, 255), "violeta": (200, 0, 160)}

# --- Figuras: se asume triángulo y cuadrado inscritos en un círculo de 40 mm ---
AREA_MIN_MM2, AREA_MAX_MM2 = 250, 1800   # círculo ≈ 1257, cuadrado ≈ 800, triángulo ≈ 518
UMBRAL_CIRCULO = 0.90        # área / área del círculo envolvente (círculo ≈ 0.95-1.0, cuadrado 0.64, triángulo 0.41)
UMBRAL_CUADRADO = 0.80       # área / área del rectángulo mínimo (cuadrado ≈ 0.95, triángulo 0.50)

FONT = cv2.FONT_HERSHEY_SIMPLEX
COLORES_REGION = {"fichas": (0, 200, 0), "A": (255, 120, 0), "B": (255, 0, 160)}
NOMBRES_REGION = {"fichas": "FICHAS", "A": "Region A", "B": "Region B"}


# ======================================================================
#  AJUSTES (electroimán invertido) Y COLORES GUARDADOS
# ======================================================================
AJUSTES = {"invertir_iman": False, "z_recogida": Z_RECOGIDA, "velocidad": PASO_GRADOS}
PARAMETROS_RECOGIDA = {
    # nombre: (valor inicial, mínimo, máximo, explicación)
    "inclinacion": (-90., -120., -60., "grados del electroimán; -90 apunta verticalmente abajo"),
    "velocidad_bajada": (0.5, 0.5, 4., "grados/paso durante bajada"),
    "velocidad_subida": (0.5, 0.5, 4., "grados/paso durante levantamiento"),
    "paso_bajada": (2., 0.5, 5., "mm por segmento vertical"),
    "pausa_contacto": (0.8, 0., 10., "segundos quieto abajo antes de activar imán"),
    "tiempo_agarre": (2., 0.2, 15., "segundos con imán encendido antes de levantar"),
    "pausa_arriba": (0.5, 0., 10., "segundos estabilizando arriba"),
    "tiempo_soltar": (0.8, 0.2, 10., "segundos con imán apagado en destino"),
}
AJUSTES.update({k: v[0] for k, v in PARAMETROS_RECOGIDA.items()})


def numero_en_rango(valor, minimo, maximo, nombre):
    v = float(valor)
    if not np.isfinite(v) or not minimo <= v <= maximo:
        raise ValueError(f"{nombre}: usa un número entre {minimo} y {maximo}")
    return v


def aplicar_altura(valor):
    global Z_RECOGIDA, BAJADA_EXTRA_AGARRE
    Z_RECOGIDA = numero_en_rango(valor, -100, Z_SEGURO, "Altura")
    BAJADA_EXTRA_AGARRE = Z_AGARRE - Z_RECOGIDA
    AJUSTES["z_recogida"] = Z_RECOGIDA


def cargar_ajustes():
    global L3
    if os.path.exists(AJUSTES_ARCHIVO):
        with open(AJUSTES_ARCHIVO) as f:
            AJUSTES.update(json.load(f))
    L3 = numero_en_rango(AJUSTES.get("longitud_iman", 90), 20, 200, "Longitud del actuador")
    # Conserva los ajustes antiguos (incluido invertir_iman).
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
        print(f"Altura de recogida guardada: {Z_RECOGIDA:.1f} mm")
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


def iman_on():
    return 0 if AJUSTES["invertir_iman"] else 1


def iman_off():
    return 1 if AJUSTES["invertir_iman"] else 0


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


# ======================================================================
#  CINEMÁTICA INVERSA
# ======================================================================
class Inalcanzable(Exception):
    pass


def ik(x, y, z, inclinacion=-90.):
    """IK de la cara del imán, compensando el desplazamiento de L3 al inclinar.

    -90° reproduce la geometría anterior; el ángulo no es el valor del servo.
    """
    theta = radians(numero_en_rango(inclinacion, -120, -60, "Inclinación"))
    r = hypot(x, y) - L3 * cos(theta)
    zw = z - H - L3 * sin(theta)
    if r <= 0:
        raise Inalcanzable("Inclinación incompatible con ese radio")
    d = hypot(r, zw)
    if d > L1 + L2 - 1 or d < abs(L1 - L2) + 1:
        raise Inalcanzable(f"({x:.0f}, {y:.0f}, {z:.0f}) fuera de alcance")

    base = degrees(atan2(y, x))
    a = atan2(zw, r)
    b = acos(max(-1, min(1, (L1**2 + d**2 - L2**2) / (2 * L1 * d))))
    sh = a + b
    el = acos(max(-1, min(1, (L1**2 + L2**2 - d**2) / (2 * L1 * L2))))
    l2 = sh - (pi - el)
    wr = theta - l2

    geo = {"base": base, "hombro": degrees(sh), "codo": degrees(l2 - sh), "muneca": degrees(wr)}
    servo = []
    for k in ("base", "hombro", "codo", "muneca"):
        cero, signo = MAPEO[k]
        v = cero + signo * geo[k]
        if v < 0 or v > 180:
            raise Inalcanzable(f"({x:.0f}, {y:.0f}, {z:.0f}): servo {k} fuera de rango ({v:.0f}°)")
        servo.append(v)
    return servo


def radio_max_teorico():
    zw = Z_RECOGIDA + L3 - H
    return sqrt((L1 + L2 - 1) ** 2 - zw**2)


# ======================================================================
#  BRAZO (serial con el Arduino)
# ======================================================================
class Brazo:
    def __init__(self):
        self.ser = serial.Serial(PUERTO, BAUD, timeout=2)
        time.sleep(2.5)                      # el Arduino se reinicia al abrir el puerto
        self.ser.reset_input_buffer()
        self.ultimo = list(ARDUINO_HOME)
        self.velocidad = float(AJUSTES["velocidad"])
        self._cmd(f"V,{self.velocidad:.2f}")
        self.home()

    def _cmd(self, txt):
        self.ser.write((txt + "\n").encode())
        r = self.ser.readline().decode(errors="ignore").strip()
        if r != "OK":
            raise RuntimeError("Arduino no confirmó la orden: " + repr(r) + ". Revisa conexión antes de continuar.")

    def ajustar_velocidad(self, valor, guardar=True):
        v = numero_en_rango(valor, 0.5, 4, "Velocidad")
        self._cmd(f"V,{v:.2f}")
        self.velocidad = v
        if guardar:
            AJUSTES["velocidad"] = v
            guardar_ajustes()
        print(f"Velocidad: {v:.2f} grados/paso de 15 ms (mayor = más rápido)")

    def mover(self, valores):
        """valores = [base, hombro, codo, muñeca, imán(0/1)]"""
        a = [max(0.0, min(180.0, float(v))) for v in valores]
        self._cmd("S," + ",".join(f"{v:.1f}" for v in a))
        delta = max(abs(p - q) for p, q in zip(a[:4], self.ultimo[:4]))
        time.sleep(delta / self.velocidad * 0.015 + 0.15)
        self.ultimo = a

    def home(self):
        self.mover(HOME + [iman_off()])

    def ir(self, x, y, z, iman, inclinacion=-90.):
        self.mover(ik(x, y, z, inclinacion) + [iman])

    def verificar(self, x, y, z_bajo):
        ik(x, y, Z_SEGURO)
        ik(x, y, z_bajo)

    def plan_vertical(self, x, y, z1, z2):
        n = max(1, int(np.ceil(abs(z2-z1) / AJUSTES["paso_bajada"])))
        return [ik(x, y, float(z), AJUSTES["inclinacion"])
                for z in np.linspace(z1, z2, n+1)]

    def ejecutar_vertical(self, plan, iman, velocidad):
        self.ajustar_velocidad(velocidad, guardar=False)
        for angulos in plan[1:]:
            self.mover(angulos + [iman])

    def recoger_en_origen(self, x, y):
        """Prueba sin traslado: acaba arriba con imán encendido."""
        plan = self.plan_vertical(x, y, Z_SEGURO, Z_RECOGIDA)
        self.mover(plan[0] + [iman_off()])
        print("Bajada por segmentos, manteniendo la inclinación configurada.")
        self.ejecutar_vertical(plan, iman_off(), AJUSTES["velocidad_bajada"])
        print(f"Contacto: pausa {AJUSTES['pausa_contacto']:.1f} s")
        time.sleep(AJUSTES["pausa_contacto"])
        self.mover(plan[-1] + [iman_on()])
        print(f"Imán activado: esperando {AJUSTES['tiempo_agarre']:.1f} s antes de subir")
        time.sleep(AJUSTES["tiempo_agarre"])
        self.ejecutar_vertical(list(reversed(plan)), iman_on(), AJUSTES["velocidad_subida"])
        time.sleep(AJUSTES["pausa_arriba"])

    def probar_agarre(self, x, y):
        anterior = self.velocidad
        try:
            self.ajustar_velocidad(AJUSTES["velocidad_bajada"], guardar=False)
            self.recoger_en_origen(x, y)
            print("Prueba terminada: imán encendido arriba. 'iman off' libera la pieza.")
        finally:
            self.ajustar_velocidad(anterior, guardar=False)

    def recoger_y_colocar(self, x, y, dx, dy):
        # Calcula la totalidad de ambas verticales antes de empezar a recoger.
        self.plan_vertical(x, y, Z_SEGURO, Z_RECOGIDA)
        destino = self.plan_vertical(dx, dy, Z_SEGURO, Z_SOLTAR)
        anterior = self.velocidad
        try:
            self.recoger_en_origen(x, y)
            self.ajustar_velocidad(anterior, guardar=False)
            self.mover(destino[0] + [iman_on()])
            self.ejecutar_vertical(destino, iman_on(), AJUSTES["velocidad_bajada"])
            self.mover(destino[-1] + [iman_off()])
            time.sleep(AJUSTES["tiempo_soltar"])
            self.ejecutar_vertical(list(reversed(destino)), iman_off(), AJUSTES["velocidad_subida"])
            self.ajustar_velocidad(anterior, guardar=False)
            self.home()
        finally:
            self.ajustar_velocidad(anterior, guardar=False)


# ======================================================================
#  UTILIDADES DE VISIÓN
# ======================================================================
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


# Códigos de flechas: GTK/Linux, Qt, Windows.
FLECHAS = {
    "arriba": (65362, 0x1000013, 2490368),
    "abajo": (65364, 0x1000015, 2621440),
    "izquierda": (65361, 0x1000012, 2424832),
    "derecha": (65363, 0x1000014, 2555904),
}
_ESPECIALES = {0xFF0D: 13, 0xFF8D: 13, 0x1000004: 13, 0x1000005: 13,
               0xFF1B: 27, 0x1000000: 27, 0xFF08: 8, 0x1000003: 8, 0xFFFF: 127}


def leer_tecla(ms=30):
    """waitKeyEx normalizado.

    Con NumLock/CapsLock activos GTK devuelve el código con bits extra
    (p. ej. 0x10006E en vez de 'n'), y entonces NINGUNA tecla respondía.
    Devuelve: -1, un código ASCII, o el código GTK de la flecha (65361..65364).
    """
    k = cv2.waitKeyEx(ms)
    if k == -1:
        return -1
    for nombre, codigos in FLECHAS.items():
        if (k in codigos or (k & 0xFFFF) == (codigos[0] & 0xFFFF)
                or (k & 0x0FFFFFFF) == codigos[1]):
            return codigos[0]
    if k in _ESPECIALES:
        return _ESPECIALES[k]
    if (k & 0xFFFF) in _ESPECIALES:
        return _ESPECIALES[k & 0xFFFF]
    return k & 0xFF


def ventana_cerrada(win, visto):
    """True si el usuario cerró la ventana con la X.

    Algunos backends devuelven WND_PROP_VISIBLE=0 aunque la ventana esté
    abierta; por eso solo se confía en 0 si antes se vio visible (visto=[bool]).
    """
    try:
        if cv2.getWindowProperty(win, cv2.WND_PROP_AUTOSIZE) < 0:
            return True
        v = cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE)
    except cv2.error:
        return True
    if v >= 1:
        visto[0] = True
        return False
    return visto[0]


def pedir_clics(f, textos, colores=None):
    """Muestra la imagen y recoge un clic por texto. ESC cancela, Backspace deshace."""
    pts = []
    win = "Calibracion (clic = punto | Backspace = deshacer | ESC = cancelar)"

    def click(ev, x, y, flags, param):
        if ev == cv2.EVENT_LBUTTONDOWN and len(pts) < len(textos):
            pts.append((x, y))

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, click)
    visto = [False]
    while len(pts) < len(textos):
        img = f.copy()
        for i, p in enumerate(pts):
            col = colores[i] if colores else (0, 255, 0)
            cv2.circle(img, p, 5, col, -1)
            cv2.putText(img, str(i + 1), (p[0] + 8, p[1] - 4), FONT, 0.6, col, 2)
        t = textos[len(pts)]
        cv2.putText(img, t, (10, 25), FONT, 0.6, (0, 0, 0), 3)
        cv2.putText(img, t, (10, 25), FONT, 0.6, (255, 255, 255), 1)
        cv2.imshow(win, img)
        k = leer_tecla(30)
        if k == 27 or ventana_cerrada(win, visto):
            cv2.destroyWindow(win)
            return None
        if k in (8, 127) and pts:
            pts.pop()
    cv2.destroyWindow(win)
    return pts


# ======================================================================
#  VISIÓN
# ======================================================================
class Vision:
    def __init__(self):
        self.cap = cv2.VideoCapture(CAMARA)
        if not self.cap.isOpened():
            raise RuntimeError("No se pudo abrir la cámara")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # menos retraso en vivo (si el driver lo admite)
        self.error_calibracion = None
        self.tam = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640,
                    int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480)
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
        for _ in range(5):
            self.cap.read()
        ok, f = self.cap.read()
        if not ok:
            raise RuntimeError("No se pudo leer la cámara")
        return f

    def frame_vivo(self):
        """Un solo fotograma, para ventanas en vivo (frame() descarta 5 y es lento)."""
        ok, f = self.cap.read()
        if not ok:
            raise RuntimeError("No se pudo leer la cámara")
        return f

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

    @staticmethod
    def _tecla_flecha(k):
        """Normaliza flechas de OpenCV en Linux/Raspberry Pi y Windows.

        También admite WASD como respaldo, porque algunos backends de OpenCV
        no entregan los códigos extendidos de las flechas de forma uniforme.
        """
        if k in FLECHAS["arriba"] or k in (ord('w'), ord('W')):
            return "arriba"
        if k in FLECHAS["abajo"] or k in (ord('s'), ord('S')):
            return "abajo"
        if k in FLECHAS["izquierda"] or k in (ord('a'), ord('A')):
            return "izquierda"
        if k in FLECHAS["derecha"] or k in (ord('d'), ord('D')):
            return "derecha"
        return None

    def _dibujar_calibracion_manual(self, img, indice, cmd_x, cmd_y, paso, confirmados):
        """Superpone referencias del plano e instrucciones sobre la cámara en vivo."""
        if self.Hinv0 is None:
            return img
        puntos_px = self.a_px(PUNTOS_MM)
        for i, p in enumerate(puntos_px):
            px, py = int(round(p[0])), int(round(p[1]))
            if i < confirmados:
                col, radio = (0, 220, 0), 7
            elif i == indice:
                col, radio = (0, 0, 255), 10
            else:
                col, radio = (200, 200, 200), 6
            cv2.circle(img, (px, py), radio, col, 2)
            cv2.drawMarker(img, (px, py), col, cv2.MARKER_CROSS, 18, 2)
            cv2.putText(img, f"P{i+1}", (px + 10, py - 8), FONT, 0.55, col, 2)

        tx, ty = PUNTOS_MM[indice]
        lineas = [
            f"CALIBRACION ELECTROIMAN  Punto {indice+1}/{len(PUNTOS_MM)}",
            f"Objetivo real: X={tx:.1f}  Y={ty:.1f} mm",
            f"Orden actual: X={cmd_x:.1f}  Y={cmd_y:.1f}  Z={Z_CAL_MANUAL:.1f} mm",
            f"Paso: {paso:.1f} mm | Flechas/WASD: mover | 1/2/5: paso",
            "ENTER: confirmar | BACKSPACE: punto anterior | R: reiniciar punto | ESC: cancelar",
            "ALINEA EL CENTRO DE LA CARA DEL ELECTROIMAN; el verde es solo referencia.",
        ]
        y = 24
        for texto in lineas:
            cv2.putText(img, texto, (10, y), FONT, 0.50, (0, 0, 0), 3)
            cv2.putText(img, texto, (10, y), FONT, 0.50, (255, 255, 255), 1)
            y += 22
        return img

    def calibrar_manual(self, brazo):
        """Enseña al robot la corrección XY real de la cara del electroimán.

        Requiere una calibración de plano de un solo nivel ('calibrar plano').
        No detecta ni usa el marcador verde para calcular posiciones.
        """
        if self.H1 is None or self.Hinv0 is None:
            print("Falta la calibración del plano. Ejecuta primero: calibrar plano")
            return
        if self.H2 is not None:
            print("La calibración activa es automática (dos alturas).")
            print("Para una enseñanza totalmente independiente del marcador verde, ejecuta primero: calibrar plano")
            return

        print("\n=== CALIBRACION MANUAL DEL ELECTROIMAN ===")
        print("Retira todas las fichas y obstáculos de la mesa.")
        print("Se usarán estas referencias:", PUNTOS_MM)
        print(f"Altura de enseñanza: Z={Z_CAL_MANUAL:.1f} mm.")
        print("Las flechas mueven coordenadas cartesianas, NO servos individuales:")
        print("  ARRIBA = X+ | ABAJO = X- | IZQUIERDA = Y+ | DERECHA = Y-")
        print("WASD funciona como alternativa a las flechas.")
        print("El marcador verde NO interviene en el cálculo; sólo queda visible como referencia.")

        win = "Calibracion manual - electroiman"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        paso = float(PASO_CAL_INICIAL)
        registros = []
        indice = 0
        cmd_x, cmd_y = map(float, PUNTOS_MM[0])
        off = iman_off()

        def ir_seguro(x, y):
            """Aproxima al punto usando la mayor altura alcanzable.

            Algunos puntos lejanos (por ejemplo P3/P4) pueden ser alcanzables
            a Z_CAL_MANUAL pero no a Z_SEGURO=60 porque la muñeca requeriría
            un ángulo menor que 0°. En vez de rechazar el punto, se busca una
            altura de aproximación menor, sin forzar ni recortar ángulos.
            """
            z_aprox = float(Z_SEGURO)

            while z_aprox > Z_CAL_MANUAL:
                try:
                    ik(x, y, z_aprox)
                    break
                except Inalcanzable:
                    z_aprox -= 1.0

            # El punto de enseñanza final sí debe ser físicamente alcanzable.
            ik(x, y, Z_CAL_MANUAL)

            if z_aprox < Z_SEGURO:
                print(
                    f"  Aproximación adaptada para ({x:.0f}, {y:.0f}): "
                    f"Z={z_aprox:.0f} mm en lugar de {Z_SEGURO:.0f} mm"
                )

            brazo.ir(x, y, z_aprox, off)
            brazo.ir(x, y, Z_CAL_MANUAL, off)

        try:
            ir_seguro(cmd_x, cmd_y)
            while indice < len(PUNTOS_MM):
                ok, f = self.cap.read()
                if not ok:
                    raise RuntimeError("No se pudo leer la cámara durante la calibración manual")
                self._dibujar_calibracion_manual(f, indice, cmd_x, cmd_y, paso, len(registros))
                cv2.imshow(win, f)
                k = leer_tecla(30)
                if k == -1:
                    continue
                if k == 27:  # ESC
                    print("Calibración cancelada. No se reemplazaron los registros anteriores.")
                    return

                flecha = self._tecla_flecha(k)
                if flecha is not None:
                    nx, ny = cmd_x, cmd_y
                    if flecha == "arriba":
                        nx += paso
                    elif flecha == "abajo":
                        nx -= paso
                    elif flecha == "izquierda":
                        ny += paso
                    elif flecha == "derecha":
                        ny -= paso
                    try:
                        brazo.ir(nx, ny, Z_CAL_MANUAL, off)
                        cmd_x, cmd_y = nx, ny
                    except Inalcanzable as e:
                        print("Movimiento rechazado:", e)
                    continue

                if k in (ord('1'), ord('2'), ord('5')):
                    paso = float(chr(k))
                    print(f"Paso de calibración: {paso:.0f} mm")
                    continue

                if k in (ord('r'), ord('R')):
                    cmd_x, cmd_y = map(float, PUNTOS_MM[indice])
                    ir_seguro(cmd_x, cmd_y)
                    print(f"Punto {indice+1} reiniciado a su coordenada teórica.")
                    continue

                if k in (8, 127):  # Backspace
                    if indice == 0:
                        print("Ya estás en el primer punto.")
                        continue
                    anterior = registros.pop() if registros else None
                    indice -= 1
                    if anterior is not None:
                        cmd_x, cmd_y = map(float, anterior["comando_mm"][:2])
                    else:
                        cmd_x, cmd_y = map(float, PUNTOS_MM[indice])
                    # Para editar el punto anterior se parte de la posición que ya se había confirmado.
                    ir_seguro(cmd_x, cmd_y)
                    print(f"Regresaste al punto {indice+1}.")
                    continue

                if k in (10, 13):  # Enter
                    tx, ty = map(float, PUNTOS_MM[indice])
                    registro = {
                        "punto": indice + 1,
                        "objetivo_mm": [tx, ty],
                        "comando_mm": [float(cmd_x), float(cmd_y), float(Z_CAL_MANUAL)],
                        "delta_mm": [float(cmd_x - tx), float(cmd_y - ty)],
                        "servos": [float(v) for v in brazo.ultimo[:4]],
                        "timestamp": time.time(),
                    }
                    registros.append(registro)
                    print(f"P{indice+1} confirmado | objetivo=({tx:.1f},{ty:.1f}) "
                          f"comando=({cmd_x:.1f},{cmd_y:.1f}) "
                          f"delta=({cmd_x-tx:+.1f},{cmd_y-ty:+.1f}) mm")
                    indice += 1
                    if indice >= len(PUNTOS_MM):
                        break
                    cmd_x, cmd_y = map(float, PUNTOS_MM[indice])
                    ir_seguro(cmd_x, cmd_y)

            self.registros_brazo = registros
            self.guardar()
            print("\nCalibración manual del electroimán GUARDADA.")
            self.mostrar_registros_brazo()
            print("Estas correcciones se aplicarán a las coordenadas de recogida y depósito.")
        finally:
            try:
                cv2.destroyWindow(win)
            except cv2.error:
                pass
            brazo.home()

    def mostrar_registros_brazo(self):
        if not self.registros_brazo:
            print("No hay registros manuales del electroimán.")
            return
        print("Registros de calibración del electroimán:")
        for r in self.registros_brazo:
            o = r.get("objetivo_mm", [0, 0])
            c = r.get("comando_mm", [0, 0, Z_CAL_MANUAL])
            d = r.get("delta_mm", [c[0] - o[0], c[1] - o[1]])
            print(f"  P{r.get('punto','?')}: objetivo=({o[0]:.1f},{o[1]:.1f}) "
                  f"-> comando=({c[0]:.1f},{c[1]:.1f},{c[2]:.1f}) "
                  f"delta=({d[0]:+.1f},{d[1]:+.1f}) mm | servos={r.get('servos', [])}")

    def corregir_xy(self, x, y):
        """Interpola la corrección manual de XY mediante distancia inversa.

        Los 4 registros dicen qué coordenada hubo que ORDENAR para que la cara
        del electroimán quedara realmente sobre cada punto objetivo. Para un
        punto intermedio se interpola el desplazamiento (comando - objetivo).
        """
        if len(self.registros_brazo) < 4:
            return float(x), float(y)
        x, y = float(x), float(y)
        pesos = []
        deltas = []
        for r in self.registros_brazo:
            tx, ty = map(float, r["objetivo_mm"][:2])
            cx, cy = map(float, r["comando_mm"][:2])
            d = hypot(x - tx, y - ty)
            if d < 1e-6:
                return cx, cy
            pesos.append(1.0 / (d * d))
            deltas.append((cx - tx, cy - ty))
        suma = sum(pesos)
        dx = sum(w * d[0] for w, d in zip(pesos, deltas)) / suma
        dy = sum(w * d[1] for w, d in zip(pesos, deltas)) / suma
        return x + dx, y + dy

    def probar_correccion_manual(self, brazo, x, y, z=Z_CAL_MANUAL):
        """Mueve el electroimán a un punto real usando la corrección aprendida."""
        if len(self.registros_brazo) < 4:
            raise RuntimeError("Primero completa 'calibrar manual' con los 4 puntos.")
        cx, cy = self.corregir_xy(x, y)
        off = iman_off()
        print(f"Objetivo real ({x:.1f},{y:.1f},{z:.1f}) -> comando corregido ({cx:.1f},{cy:.1f},{z:.1f})")
        brazo.ir(cx, cy, Z_SEGURO, off)
        brazo.ir(cx, cy, z, off)
        print("El brazo queda en el punto para inspección. Usa 'home' cuando termines.")

    def buscar_marcador(self):
        f = self.frame()
        m = cv2.inRange(self._hsv(f), np.array(MARCADOR_HSV[0]), np.array(MARCADOR_HSV[1]))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = [c for c in cnts if cv2.contourArea(c) >= MARCADOR_AREA_MIN]
        if not cnts:
            return None
        M = cv2.moments(max(cnts, key=cv2.contourArea))
        return (M["m10"] / M["m00"], M["m01"] / M["m00"])

    def _pasada(self, brazo, z_punta):
        px, mm = [], []
        for (x, y) in PUNTOS_CAL_PUNTA:
            try:
                brazo.ir(x, y, z_punta, iman_off())
            except Inalcanzable as e:
                print("  omitido:", e)
                continue
            time.sleep(0.5)
            p = self.buscar_marcador()
            if p is None:
                print(f"  no se vio el marcador en ({x}, {y}, z={z_punta})")
                continue
            px.append(p)
            mm.append((x, y))
            print(f"  ({x:4d}, {y:4d}) mm -> pixel ({p[0]:.0f}, {p[1]:.0f})")
        if len(px) < 4:
            return None
        Hm, _ = cv2.findHomography(np.float32(px), np.float32(mm), 0)
        try:
            validar_transformacion(Hm, None, 0, 0, cv2.convexHull(np.float32(px)).reshape(-1, 2))
        except ValueError as e:
            print("Pasada rechazada:", e)
            return None
        proy = cv2.perspectiveTransform(np.float32(px).reshape(-1, 1, 2), Hm).reshape(-1, 2)
        err = np.linalg.norm(proy - np.float32(mm), axis=1)
        print(f"  {len(px)} puntos. Error medio {err.mean():.1f} mm, máximo {err.max():.1f} mm")
        if not np.isfinite(err).all() or err.max() > 8:
            print("Error excesivo: no se acepta esta pasada.")
            return None
        return Hm

    def calibrar_punta(self, brazo):
        h1, h2 = Z_CAL_BAJO + DIST_MARCADOR, Z_CAL_ALTO + DIST_MARCADOR
        print("Retira las fichas. El marcador verde debe verse siempre desde la cámara.")
        print(f"Pasada 1: punta a {Z_CAL_BAJO} mm (marcador a {h1} mm de la mesa)")
        H1 = self._pasada(brazo, Z_CAL_BAJO)
        H2 = None
        if H1 is not None:
            print(f"Pasada 2: punta a {Z_CAL_ALTO} mm (marcador a {h2} mm de la mesa)")
            H2 = self._pasada(brazo, Z_CAL_ALTO)
        brazo.home()
        if H1 is None or H2 is None:
            print("Faltaron puntos (mín. 4 por pasada). Revisa MARCADOR_HSV, la luz, el MAPEO y PUNTOS_CAL_PUNTA.")
            return
        zona = self.regiones.get("fichas")
        if zona is not None:
            validar_transformacion(H1, H2, h1, h2, zona)
        self.H1, self.H2, self.h1, self.h2 = H1, H2, float(h1), float(h2)
        self._actualizar()
        self.guardar()
        print("Calibración guardada (con corrección de paralaje del marcador).")

    def comprobar(self, brazo, x, y, z=Z_SEGURO):
        """Mueve la punta y compara con lo que ve la cámara en el marcador."""
        if self.H1 is None:
            raise RuntimeError("Primero 'calibrar punta'.")
        brazo.ir(x, y, z, iman_off())
        time.sleep(0.6)
        p = self.buscar_marcador()
        if p is None:
            print("No se ve el marcador verde.")
            return
        mx, my = self.px_a_mm([p], z + DIST_MARCADOR)[0]
        print(f"Pedido ({x:.0f}, {y:.0f}) | cámara ve el marcador en ({mx:.1f}, {my:.1f}) | "
              f"diferencia {hypot(mx - x, my - y):.1f} mm")

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
            z_bajo = Z_RECOGIDA if n == "fichas" else Z_SOLTAR
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
                                ik(x, y, Z_RECOGIDA)
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


# ======================================================================
#  ÓRDENES
# ======================================================================
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
    brazo.ir(x, y, z_destino, iman_off(), inclinacion)
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
        brazo.ir(cx, cy, z, iman_off(), inclinacion)
        while True:
            f = vision.frame_vivo()
            lineas = [f"XY fijo: {x:.1f}, {y:.1f} | Z actual: {z:.1f} mm",
                      f"Z guardada: {Z_RECOGIDA:.1f} | paso {paso:.1f} mm | bajada {brazo.velocidad:.2f}",
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
            subir = Vision._tecla_flecha(k) == "arriba"
            bajar = Vision._tecla_flecha(k) == "abajo"
            try:
                if subir or bajar:
                    z = mover_vertical(brazo, cx, cy, z, z + (paso if subir else -paso), inclinacion)
                    aviso = f"Posición ordenada: Z={z:.1f}. Revise distancia al objeto."
                elif k in (ord('j'),ord('J'),ord('l'),ord('L')):
                    nueva = numero_en_rango(inclinacion + (-1 if k in (ord('j'),ord('J')) else 1), -120, -60, "Inclinación")
                    # Recalcula hombro/codo/muñeca para conservar la posición de la cara.
                    brazo.ir(cx, cy, z, iman_off(), nueva)
                    inclinacion = nueva
                    aviso = f"Inclinación {inclinacion:.1f}: compruebe físicamente la cara del imán."
                elif k in (ord('0'),ord('1'),ord('2'),ord('5')):
                    paso = {ord('0'):0.5,ord('1'):1.,ord('2'):2.,ord('5'):5.}[k]
                elif k in (ord('+'),ord('='),ord('-')):
                    nueva = np.clip(brazo.velocidad + (-0.25 if k == ord('-') else 0.25), 0.5, 4)
                    brazo.ajustar_velocidad(float(nueva), guardar=False)
                elif k in (ord('g'),ord('G')):
                    anterior = dict(AJUSTES)
                    z_anterior = Z_RECOGIDA
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
        print(f"Brazo permanece en la última posición ordenada. Z guardada={Z_RECOGIDA:.1f} mm.")


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


AYUDA = """Comandos:
  ajustes recogida            muestra inclinación, velocidades, pasos y pausas
  ajuste nombre valor         guarda un parámetro, ej: ajuste tiempo_agarre 3
  probar agarre x y           baja, activa imán, espera y levanta; queda con imán encendido
  calibrar recogida x y       alias del ajuste visual de altura e inclinación
  ajustar altura x y          cámara en vivo: W subir/S bajar; 0/1/2/5 paso; +/- velocidad; G guardar
  altura                     muestra Z de recogida guardada (menor Z = más abajo)
  velocidad [0.5..4]          consulta o guarda velocidad, mayor = más rápido; 2 era la original
  recoge <color y/o figura> [en a|b]   ej: recoge cafe | recoge triangulo en b | recoge azul circulo | recoge todo en a
  ver                         detecta fichas (vista.jpg) y muestra área y proporciones de cada una
  diagnostico                 muestra candidatos y motivos de rechazo; no mueve el brazo
  mascaras                    guarda mascaras.jpg: lo que ve cada color (para afinar)
  alcance                     revisa si las 12 esquinas están dentro del alcance y dibuja el arco
  calibrar plano              4 clics sobre marcas medidas en la mesa (cámara -> mm)
  calibrar manual             enseña la posición REAL del electroimán con flechas + cámara en vivo
  calibrar punta              alternativa automática con marcador verde (2 alturas)
  calibrar regiones           12 clics: esquinas de FICHAS, A y B
  calibrar colores [color]    aprende todos o solo uno: calibrar colores cafe
  registros                   muestra las 4 posiciones guardadas del electroimán
  probar x y [z]              mueve el electroimán usando la corrección manual (z opcional)
  comprobar x y [z]           prueba antigua con el marcador verde
  reiniciar                   marca A y B como vacías
  home | ir x y z | servo a b c d e   (e = imán 0/1)
  iman on | iman off | iman invertir
  salir"""


# ======================================================================
#  V11-V13: enseñanza empírica por postura con marcadores verde/magenta.
#  NO proyecta el marcador sobre la mesa con una homografía: aprende, a la
#  postura de recogida, qué píxel del marcador corresponde a qué punto real.
# ======================================================================
ENSEÑANZA_ARCHIVO = os.path.join(DIRECTORIO, "ensenanza_v11.json")   # mismo archivo: conserva muestras
MARCADORES_ARCHIVO = os.path.join(DIRECTORIO, "marcadores.json")
NOMBRES_MARCADOR = ("magenta", "verde")       # orden de prioridad para estimar la punta
COLOR_DIBUJO_MARCADOR = {"magenta": (255, 0, 255), "verde": (0, 255, 0)}

# Reglas por defecto (sin calibrar). Lab de OpenCV en 8 bits: a y b llevan +128.
#   magenta / fucsia / rosa mexicano: a >= ~180 y b <= ~128 (tiene componente AZUL)
#   rojo, café, piel, madera, naranja: b >= ~150 (componente AMARILLA) -> quedan fuera
# En HSV el rango 135..179 de v12 incluía los rojos (H 170..179): ese era el fallo.
MAGENTA_A_MIN = 150
MAGENTA_B_MAX = 135
MAGENTA_H = (125, 175)          # respaldo HSV; rojo puro (0 / 179) queda fuera
VERDE_HSV = ((40, 100, 80), (85, 255, 255))

RADIO_BUSQUEDA_PX = 200         # sin referencias guardadas: radio alrededor de la punta
RADIO_BUSQUEDA_APRENDIDO_PX = 120
DIST_OFFSET_APRENDIDO = 50.0    # mm: más lejos de toda muestra se usa RADIO_BUSQUEDA_PX
TOL_Z_MUESTRA = 5.0             # mm: muestras reutilizables si Z_RECOGIDA cambió poco
TOL_INCL_MUESTRA = 2.0          # grados
TOL_Z_GUARDAR = 3.0             # mm: ENTER acepta |Z - Z objetivo| <= esto
MARGEN_APROX = 10.0             # APROXIMAR deja la cara a Z objetivo + este margen
RADIO_CORRECCION = 40.0         # mm: corrección completa hasta esta distancia del punto enseñado
RADIO_DESVANECER = 40.0         # mm extra en los que la corrección baja linealmente a cero

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


def texto_con_borde(img, texto, xy, escala=0.45, col=(255, 255, 255)):
    cv2.putText(img, texto, xy, FONT, escala, (0, 0, 0), 3)
    cv2.putText(img, texto, xy, FONT, escala, col, 1)


# ---------------------------------------------------------------- firmas y modelo marcador->mm
def firma_geometria():
    return [L1, L2, L3, H, *[v for par in MAPEO.values() for v in par]]


def firma_postura():
    return {"geometria": firma_geometria(), "z": float(Z_RECOGIDA),
            "inclinacion": float(AJUSTES["inclinacion"])}


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


def z_transito(x, y, z_bajo, z_alto=None, inclinacion=None):
    """Altura más alta (<= z_alto) desde la que se puede bajar en vertical hasta z_bajo.

    Algunos puntos lejanos son alcanzables abajo pero no a Z_SEGURO (la muñeca
    saldría de 0..180°). v12 rechazaba esos puntos; aquí se baja la altura de
    aproximación sin forzar ni recortar ángulos.
    """
    z_alto = Z_SEGURO if z_alto is None else z_alto
    inclinacion = AJUSTES["inclinacion"] if inclinacion is None else inclinacion
    ik(x, y, z_bajo, inclinacion)                 # el punto bajo DEBE ser alcanzable
    tope = z_bajo
    for z in np.arange(z_bajo + 1, z_alto + 1e-6, 1.0):
        try:
            ik(x, y, float(z), inclinacion)
        except Inalcanzable:
            break
        tope = float(z)
    if z_alto > z_bajo and tope - z_bajo < min(10.0, z_alto - z_bajo):
        raise Inalcanzable(f"({x:.0f}, {y:.0f}): no hay ni 10 mm de altura libre sobre Z={z_bajo:.0f}")
    return min(tope, z_alto)


# El plan vertical de recogida/depósito también usa la altura adaptativa.
def _plan_vertical_adaptativo(self, x, y, z1, z2):
    alto, bajo = max(z1, z2), min(z1, z2)
    tope = z_transito(x, y, bajo, alto)
    if tope < alto - 0.5:
        print(f"  Aproximación adaptada en ({x:.0f}, {y:.0f}): Z={tope:.0f} en lugar de {alto:.0f}")
    z1, z2 = (tope, z2) if z1 >= z2 else (z1, tope)
    n = max(1, int(np.ceil(abs(z2 - z1) / AJUSTES["paso_bajada"])))
    return [ik(x, y, float(z), AJUSTES["inclinacion"]) for z in np.linspace(z1, z2, n + 1)]


Brazo.plan_vertical = _plan_vertical_adaptativo


# ---------------------------------------------------------------- Visión con enseñanza
class VisionV11(Vision):
    def __init__(self):
        super().__init__()
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
        if abs(float(f.get("z", 1e9)) - Z_RECOGIDA) > TOL_Z_MUESTRA:
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
        if abs(z - Z_RECOGIDA) > TOL_Z_GUARDAR:
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
        z = Z_RECOGIDA if z is None else z
        cx, cy = self.corregir_xy(x, y)
        print(f"Objetivo ({x:.1f},{y:.1f}) -> comando ({cx:.1f},{cy:.1f},{z:.1f})")
        plan = brazo.plan_vertical(cx, cy, Z_SEGURO, z)
        brazo.mover(plan[0] + [iman_off()])
        brazo.ejecutar_vertical(plan, iman_off(), AJUSTES['velocidad_bajada'])
        brazo.ajustar_velocidad(AJUSTES['velocidad'], guardar=False)
        print('En posición; imán apagado. No se regresa automáticamente (usa home).')

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
        print(f'N/APROXIMAR deja la cara {MARGEN_APROX:.0f} mm sobre Z objetivo={Z_RECOGIDA:.1f}. '
              f'Z baja a la altura objetivo. U/J ajustan altura.')
        print(f'ENTER guarda XY y Z actuales; si Z difiere más de {TOL_Z_GUARDAR:.0f} mm, '
              'esa Z sobrescribe la Z de recogida. K omite, BACKSPACE punto anterior, ESC termina.')
        print('Imán apagado. Los marcadores son una ayuda; confirma mirando la cara del imán de lado.')
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
                brazo.mover(ik(*map(float, p), incl) + [iman_off()])
                estado["pose"] = [float(v) for v in p]

        def aproximar(o):
            cx, cy = self.corregir_xy(o['x'], o['y'])
            z_obj = float(Z_RECOGIDA)
            tope_dest = z_transito(cx, cy, z_obj)
            llegada = [cx, cy, min(z_obj + MARGEN_APROX, tope_dest)]
            pose = estado["pose"]
            if pose is None:            # desde HOME: movimiento articular arriba del destino
                brazo.ajustar_velocidad(vel_transito, guardar=False)
                brazo.ir(cx, cy, tope_dest, iman_off(), incl)
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
                    dz = pose[2] - Z_RECOGIDA
                    estado_txt = f'XYZ ({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f})  Z-obj={dz:+.1f} mm'
                else:
                    estado_txt = 'pulsa APROXIMAR'
                est = self.estimar_punta(detecciones, pose[2], modelos) if pose is not None else None
                lineas = [f'P{indice + 1}/{len(objetivos)} {o["color"]} {o["forma"]} | {estado_txt}',
                          f'Z objetivo {Z_RECOGIDA:.1f} | inclinacion {incl:.1f} | paso {paso} mm',
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
                            if abs(pose[2] - Z_RECOGIDA) > TOL_Z_GUARDAR:
                                # La Z enseñada manda: se adopta como nueva Z de recogida.
                                z_previa, activas_previas = Z_RECOGIDA, len(self.muestras_activas())
                                aplicar_altura(pose[2])
                                try:
                                    guardar_ajustes()
                                except OSError:
                                    aplicar_altura(z_previa)
                                    raise
                                print(f'Z de recogida sobrescrita: {z_previa:.1f} -> {Z_RECOGIDA:.1f} mm (ajustes.json)')
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
                        direccion = self._tecla_flecha(k)
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
                            nueva[2] = float(Z_RECOGIDA)
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
            print('Sesión finalizada. Se conservan los puntos confirmados; brazo quieto, imán apagado.')
            self.resumen_muestras()


Vision = VisionV11
AYUDA += '''
V13 (marcadores y enseñanza):
  calibrar marcador magenta   clic izq = marcador, clic der = rojizos que NO son; ENTER, G guarda
  calibrar marcador verde     igual para el verde
  marcadores                  vista en vivo: máscara de cada color, candidatos y el elegido
  longitud iman [mm]          eje de muñeca -> cara; inicial 90 mm
  calibrar manual             enseña las 4 marcas de la mesa (APROXIMAR, Z, ajustar, ENTER)
  calibrar piezas             detecta piezas y enseña el centro de cada una
  registros                   muestras activas e ignoradas (con el motivo)
'''


def main():
    global L3
    cargar_ajustes()
    cargar_colores()
    brazo = Brazo()
    vision = Vision()
    print(AYUDA)
    print("Colores en uso:", ", ".join(COLORES_ACTIVOS))
    print(f"Altura de recogida: Z={Z_RECOGIDA:.1f} mm (bajada extra {BAJADA_EXTRA_AGARRE:.0f} mm)")
    if AJUSTES["invertir_iman"]:
        print("(electroimán invertido: encendido = 0)")

    while True:
        try:
            txt = normalizar(input("\n> ").strip())
        except (EOFError, KeyboardInterrupt):
            break
        if not txt:
            continue
        try:
            if txt in ("salir", "exit", "q"):
                break
            elif txt == "ayuda":
                print(AYUDA)
            elif txt == "home":
                brazo.home()
            elif txt.startswith("ajustar altura "):
                valores = txt.split()[2:]
                if len(valores) != 2:
                    raise ValueError("Usa: ajustar altura x y (coordenadas reales en mm)")
                ajustar_altura(brazo, vision, *map(float, valores))
            elif txt == "altura":
                print(f"Z de recogida={Z_RECOGIDA:.1f} mm; bajada extra={BAJADA_EXTRA_AGARRE:.1f} mm")
            elif txt == "velocidad" or txt.startswith("velocidad "):
                partes = txt.split()
                if len(partes) == 1:
                    print(f"Velocidad actual={brazo.velocidad:.2f} grados/paso; rango 0.5..4")
                elif len(partes) == 2:
                    brazo.ajustar_velocidad(float(partes[1]))
                else:
                    raise ValueError("Usa: velocidad 1")
            elif txt == "ajustes recogida":
                configurar_recogida()
            elif txt.startswith("ajuste "):
                partes = txt.split()
                if len(partes) != 3:
                    raise ValueError("Usa: ajuste nombre valor")
                configurar_recogida(partes[1], partes[2])
            elif txt.startswith("calibrar recogida "):
                partes = txt.split()[2:]
                if len(partes) != 2:
                    raise ValueError("Usa: calibrar recogida x y")
                ajustar_altura(brazo, vision, *map(float, partes))
            elif txt.startswith("probar agarre "):
                partes = txt.split()[2:]
                if len(partes) != 2:
                    raise ValueError("Usa: probar agarre x y")
                x, y = map(float, partes)
                if not np.isfinite([x, y]).all():
                    raise ValueError("Coordenadas no finitas")
                brazo.probar_agarre(*vision.corregir_xy(x, y))
            elif txt == "calibrar piezas":
                vision.ensenar(brazo, piezas=True)
            elif txt == "marcadores":
                vision.ver_marcadores()
            elif txt.startswith("longitud iman"):
                partes = txt.split()
                if len(partes) == 2:
                    print(f"Eje de muñeca a cara de contacto: {L3:.1f} mm")
                elif len(partes) == 3:
                    valor = numero_en_rango(partes[2], 20, 200, "Longitud")
                    anterior = AJUSTES.get("longitud_iman", L3)
                    AJUSTES["longitud_iman"] = valor
                    try:
                        guardar_ajustes()
                    except OSError:
                        AJUSTES["longitud_iman"] = anterior
                        raise
                    L3 = valor
                    print("Longitud guardada. Repite enseñanza: las muestras con otra geometría no se usan.")
                else:
                    raise ValueError("Usa: longitud iman 90")
            elif txt.startswith("calibrar marcador"):
                partes = txt.split()
                if len(partes) != 3:
                    raise ValueError("Usa: calibrar marcador magenta | calibrar marcador verde")
                vision.calibrar_marcador(partes[2])
            elif txt.startswith("calibrar"):
                if "punta" in txt:
                    vision.calibrar_punta(brazo)
                elif "region" in txt:
                    vision.calibrar_regiones()
                elif "color" in txt:
                    elegidos = interpretar(txt)[0]
                    vision.calibrar_colores(elegidos[0] if elegidos else None)
                elif "plano" in txt:
                    vision.calibrar_plano()
                elif "manual" in txt:
                    vision.calibrar_manual(brazo)
                else:
                    print("Usa: calibrar plano | manual | punta | regiones | colores")
            elif txt == "registros":
                vision.mostrar_registros_brazo()
            elif txt.startswith("probar "):
                v = list(map(float, txt.split()[1:4]))
                if len(v) not in (2, 3):
                    print("Usa: probar x y [z]")
                else:
                    vision.probar_correccion_manual(brazo, *v)
            elif txt == "diagnostico":
                vision.diagnostico()
            elif txt == "ver":
                for o in vision.detectar():
                    print(f"  {o['color']:9s} {o['forma']:10s} x={o['x']:6.1f} y={o['y']:6.1f} mm | "
                          f"area={o['area']:5.0f} mm2  r_circ={o['rc']:.2f}  r_rect={o['rr']:.2f}")
            elif txt == "mascaras":
                vision.mascaras()
                print("Guardado en mascaras.jpg")
            elif txt == "alcance":
                vision.revisar_alcance()
                vision._mostrar(vision.vista())
            elif txt.startswith("comprobar"):
                v = list(map(float, txt.split()[1:4]))
                vision.comprobar(brazo, *v)
            elif txt == "reiniciar":
                vision.ocupados = {"A": 0, "B": 0}
                print("Regiones A y B marcadas como vacías.")
            elif txt.startswith("ir "):
                x, y, z = map(float, txt.split()[1:4])
                brazo.ir(x, y, z, brazo.ultimo[4])
            elif txt.startswith("servo "):
                v = list(map(float, txt.split()[1:6]))
                if len(v) != 5:
                    print("Son 5 valores (el último es el imán: 0 o 1).")
                else:
                    brazo.mover(v)
            elif txt.startswith("iman"):
                if "invertir" in txt:
                    AJUSTES["invertir_iman"] = not AJUSTES["invertir_iman"]
                    guardar_ajustes()
                    print("Electroimán invertido." if AJUSTES["invertir_iman"] else "Electroimán en sentido normal.")
                    brazo.mover(brazo.ultimo[:4] + [iman_off()])
                else:
                    estado = iman_on() if "on" in txt else iman_off()
                    brazo.mover(brazo.ultimo[:4] + [estado])
            elif txt.startswith(("recoge", "toma", "agarra")):
                recoger(brazo, vision, txt)
            else:
                print("No entendí. Escribe 'ayuda'.")
        except Inalcanzable as e:
            print("Fuera de alcance:", e)
        except (ValueError, RuntimeError, TypeError, OSError, KeyError) as e:
            print("Error:", e)
        except cv2.error as e:
            print("Error de OpenCV (¿sin entorno gráfico o ventana cerrada?):", str(e).strip().splitlines()[-1])
        except Exception:
            # Antes cualquier error inesperado cerraba el programa a mitad de una rutina.
            traceback.print_exc()
            print("Error inesperado; el programa sigue. Revisa la traza de arriba.")

    brazo.home()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
