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


def detectar_webcam(maximo=10):
    """Índice de la primera cámara que entrega imagen."""
    for i in range(maximo):
        if _probar_camara(i) is not None:
            return i
    raise RuntimeError("No se detectó ninguna webcam. Revisa la conexión y los permisos de cámara del sistema.")


class Camara:
    def __init__(self, fuente, nombre="robot", resolucion=None):
        self.fuente, self.nombre = fuente, nombre
        self.cap = cv2.VideoCapture(fuente)
        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir la cámara {nombre} ({fuente!r}). "
                               "Revisa con: python main.py --listar-camaras")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # menos retraso en vivo (si el driver lo admite)
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
