"""Acceso a una cámara USB, con hilo opcional para tener siempre el fotograma más reciente."""

import threading
import time

import cv2


def fuente_de_texto(texto):
    """'2' -> índice 2; cualquier otra cosa se deja como ruta (/dev/v4l/by-id/...)."""
    return int(texto) if texto.isdigit() else texto


def _probar_camara(indice):
    """Devuelve un fotograma si esa cámara entrega imagen, o None."""
    cap = cv2.VideoCapture(indice)
    try:
        if cap.isOpened():
            for _ in range(5):            # algunas webcams tardan un par de lecturas en arrancar
                ok, f = cap.read()
                if ok:
                    return f
        return None
    finally:
        cap.release()


def listar_camaras(maximo=10):
    print("Cámaras que entregan imagen:")
    encontradas = 0
    for i in range(maximo):
        f = _probar_camara(i)
        if f is not None:
            print(f"  {i}: {f.shape[1]}x{f.shape[0]}")
            encontradas += 1
    if not encontradas:
        print("  ninguna")


NEGRO = 8.0              # brillo medio (0..255) por debajo del cual un cuadro es negro


def _leer_cuadros(indice, n=5, espera=1.5, pausa=0.05):
    """Hasta n cuadros con imagen (en gris) de esa cámara, o None si no abre.

    Muchas webcams dan cuadros negros al abrirse, mientras ajustan la exposición: se esperan hasta
    'espera' segundos a que salga imagen. Si sólo hay negro, devuelve esos cuadros negros.
    """
    cap = cv2.VideoCapture(indice)
    try:
        if not cap.isOpened():
            return None
        con_imagen, negros, t0 = [], [], time.monotonic()
        while time.monotonic() - t0 < espera and len(con_imagen) < n:
            ok, f = cap.read()
            if not ok:
                time.sleep(pausa)
                continue
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY).astype("float32")
            (con_imagen if g.mean() >= NEGRO else negros).append(g)
            time.sleep(pausa)
        return con_imagen if con_imagen else negros
    finally:
        cap.release()


def tipo_de_camara(cuadros, umbral=0.3):
    """'real', 'negra' (tapada, apagada o la de una laptop cerrada), 'fija' (virtual, como OBS sin
    emitir: la misma imagen siempre) o 'sin imagen'."""
    if not cuadros:
        return "sin imagen"
    if max(float(c.mean()) for c in cuadros) < NEGRO:
        return "negra"
    return "real" if es_camara_real(cuadros, umbral) else "fija"


def es_camara_real(cuadros, umbral=0.3):
    """Una cámara real siempre tiene ruido de sensor; una virtual (OBS sin emitir) da la misma imagen."""
    if len(cuadros) < 2:
        return False
    return max(float(abs(c - cuadros[0]).mean()) for c in cuadros[1:]) > umbral


EXPLICACION = {"negra": "imagen negra (¿tapada, apagada o la tapa de la laptop cerrada?): no se usa",
               "fija": "imagen fija (cámara virtual, por ejemplo OBS): no se usa",
               "sin imagen": "abre pero no da imagen (¿la usa otro programa?): no se usa"}


def detectar_webcams(maximo=6, informe=False):
    """Índices de las cámaras reales que entregan imagen. informe: dice qué encontró en cada una."""
    reales, fallos = [], 0
    for i in range(maximo):
        cuadros = _leer_cuadros(i)
        if cuadros is None:
            fallos += 1
            if fallos >= 2:                 # los índices van seguidos: no hay más
                break
            continue
        fallos = 0
        tipo = tipo_de_camara(cuadros)
        if tipo == "real":
            reales.append(i)
        if informe:
            print(f"  cámara {i}: " + ("con imagen" if tipo == "real" else EXPLICACION[tipo]), flush=True)
    return reales


def detectar_webcam(maximo=10):
    """Índice de la primera cámara real; si todas dan imagen fija, la primera que entrega imagen."""
    reales = detectar_webcams(min(maximo, 6))
    if reales:
        return reales[0]
    for i in range(maximo):
        if _probar_camara(i) is not None:
            print(f"Aviso: ninguna webcam da imagen real (¿desconectada, tapada o apagada?). Uso la {i}, "
                  "pero así no te verá: revisa la conexión y vuelve a arrancar.", flush=True)
            return i
    raise RuntimeError("No se detectó ninguna webcam. Revisa la conexión y los permisos de cámara del sistema.")


class Camara:
    def __init__(self, fuente, nombre="robot", resolucion=None, mjpg=False):
        self.fuente, self.nombre = fuente, nombre
        self.cap = cv2.VideoCapture(fuente)
        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir la cámara {nombre} ({fuente!r}). "
                               "Revisa con: python main.py consola --listar-camaras")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # menos retraso en vivo (si el driver lo admite)
        if mjpg:                                   # menos ancho de banda USB (si el driver lo admite)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        if resolucion is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolucion[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolucion[1])
        self.tam = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640,
                    int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480)
        self.viva = True
        self._hilo = None
        self._activo = False
        self._cerrojo = threading.Lock()
        self._n, self._frame = 0, None

    def leer(self):
        ok, f = self.cap.read()
        if not ok:
            raise RuntimeError(f"No se pudo leer la cámara {self.nombre}")
        return f

    def leer_fresco(self):
        for _ in range(5):
            self.cap.read()
        return self.leer()

    # ---------- lectura continua, para dos cámaras a la vez ----------
    def iniciar_hilo(self):
        """Lee sin parar en segundo plano: una cámara no frena a la otra ni se acumula retraso."""
        if self._hilo is not None:
            return
        self._activo, self.viva = True, True
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()

    def _bucle(self):
        fallos = 0
        while self._activo:
            ok, f = self.cap.read()
            if ok:
                fallos = 0
                with self._cerrojo:
                    self._n, self._frame = self._n + 1, f
            else:
                fallos += 1
                if fallos >= 30:
                    self.viva = False
                    return
                time.sleep(0.02)

    def ultimo(self):
        """(número de fotograma, imagen o None). No modifiques la imagen: cópiala antes."""
        with self._cerrojo:
            return self._n, self._frame

    def detener_hilo(self):
        self._activo = False
        if self._hilo is not None:
            self._hilo.join(timeout=1.0)
        self._hilo = None
        with self._cerrojo:
            self._frame = None

    def cerrar(self):
        self.detener_hilo()
        self.cap.release()
