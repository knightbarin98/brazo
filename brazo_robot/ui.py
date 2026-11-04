"""Ventanas de OpenCV: teclado, clics y texto sobre la imagen."""

import cv2

from .config import FONT

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


def tecla_flecha(k):
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


def texto_con_borde(img, texto, xy, escala=0.45, col=(255, 255, 255)):
    cv2.putText(img, texto, xy, FONT, escala, (0, 0, 0), 3)
    cv2.putText(img, texto, xy, FONT, escala, col, 1)
