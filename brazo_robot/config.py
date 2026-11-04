"""Configuración del brazo (lo marcado con  # CALIBRAR  hay que ajustarlo).

Z_RECOGIDA, BAJADA_EXTRA_AGARRE y L3 cambian en ejecución (ajustes.json):
léelos siempre como config.Z_RECOGIDA, nunca con "from config import".
"""

import os

import cv2

PUERTO = "/dev/ttyACM0"
BAUD = 115200

# Índice (0, 1, 2...) o ruta (/dev/v4l/by-id/...). En Linux cada cámara USB suele ocupar dos
# índices: 'python main.py --listar-camaras' muestra cuáles sirven.
CAMARA_ROBOT = 0             # mira la mesa; toda la calibración depende de esta cámara

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

PIEZA_ALTURA = 8             # CALIBRAR: altura de la cara superior de las fichas sobre la mesa (mm)

# Alturas de la punta (cara del electroimán) sobre la mesa (mm)
Z_AGARRE = PIEZA_ALTURA + 1  # altura teórica original de agarre
BAJADA_EXTRA_AGARRE = 60.0     # mm extra que debe bajar el electroimán al recoger
Z_RECOGIDA = Z_AGARRE - BAJADA_EXTRA_AGARRE
Z_SEGURO = 60
Z_SOLTAR = 25

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
DIRECTORIO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
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

# --- Enseñanza por postura con marcadores verde/magenta ---
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

# --- Control remoto: Raspberry (servidor.py) <-> computadora del operador (app_remota.py) ---
REMOTO_ESCUCHAR = "0.0.0.0"          # interfaces donde escucha la Raspberry
REMOTO_PUERTO = 8765
REMOTO_TOKEN_ARCHIVO = os.path.join(DIRECTORIO, "token_remoto.txt")   # se crea en la Raspberry; es secreto
REMOTO_VIDEO_FPS = 15
REMOTO_VIDEO_CALIDAD = 60            # calidad JPEG (0-100): menos = menos ancho de banda
REMOTO_TIMEOUT_LATIDO = 5.0          # s sin ningún mensaje del operador: se da por desconectado
REMOTO_VIDEO_MAX_EDAD = 1.5          # s sin video nuevo: la app pausa el brazo (no se maneja a ciegas)
REMOTO_EVENTO_MAX_EDAD = 1.0         # s: una orden que esperó más que esto se descarta
REMOTO_MANO_MAX_EDAD = 0.5           # s: para activar hace falta haber visto la mano hace menos que esto

# --- Seguimiento de la mano del operador ---
MODELO_MANO = os.path.join(DIRECTORIO, "modelos", "hand_landmarker.task")
MODELO_MANO_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
                   "hand_landmarker/float16/1/hand_landmarker.task")
MODELO_MANO_SHA256 = "fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1"
WEBCAM_RESOLUCION = (640, 480)       # webcam del operador; más píxeles = más lento
SEGUIR_INICIO = (200.0, 0.0)         # XY de partida; la Z es la mayor alcanzable hasta Z_SEGURO
# Límites del punto objetivo (mm). La Z mínima es siempre la Z de recogida calibrada.
SEGUIR_LIMITE_X = (120.0, 260.0)
SEGUIR_LIMITE_Y = (-150.0, 150.0)
# Ganancias: mm de brazo por unidad de movimiento de la mano. Cambia el signo para invertir un eje.
SEGUIR_GANANCIA_X = 350.0            # mm por log(tamaño de la mano): acercarla a la cámara = adelante
SEGUIR_GANANCIA_Y = 600.0            # mm por ancho de imagen: mano a la izquierda de la pantalla = Y+
SEGUIR_GANANCIA_Z = 250.0            # mm por alto de imagen: mano arriba = subir
SEGUIR_FILTRO_XY = (1.0, 5.0)        # filtro One Euro (corte mínimo Hz, beta) para posición en imagen
SEGUIR_FILTRO_ESCALA = (0.5, 2.0)    # el tamaño de la mano tiembla más: se filtra más
SEGUIR_VELOCIDAD_MM_S = 80.0         # velocidad máxima del punto objetivo
SEGUIR_VELOCIDAD_SERVOS = 1.0        # grados/paso de 15 ms durante el seguimiento
SEGUIR_PERIODO = 0.05                # s mínimos entre órdenes al Arduino
SEGUIR_DELTA_MIN = 1.0               # mm: movimientos menores no se envían (evita vibración)
SEGUIR_TIMEOUT_MANO = 0.6            # s sin datos de la mano (o sin red): el brazo se detiene y queda en pausa
SEGUIR_SALTO_MAX = 0.25              # salto de la mano entre detecciones (fracción de imagen): pausa
SEGUIR_PINZA_CERRAR = 0.30           # pulgar-índice / tamaño de palma para contar como pinza
SEGUIR_PINZA_ABRIR = 0.50            # histéresis: hay que abrir hasta aquí para volver a pinzar
SEGUIR_PINZA_TIEMPO = 0.20           # s sostenida para que cuente (evita cambios por parpadeo)
