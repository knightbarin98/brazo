"""Dirección del brazo con las líneas que se ven en la imagen (2D), con una o dos cámaras.

Pose da dos cosas por cámara: puntos en la imagen (2D, precisos) y una estimación 3D (la profundidad
la adivina, y con el brazo levantado se equivocaba en decenas de grados). Aquí:

- De la estimación 3D sólo se usa lo que Pose estima bien: cómo está girada la cámara respecto a tu
  torso (hombros, caderas y nariz), y hacia qué lado de la cámara va cada segmento (para una sola).
- De la imagen, la línea de cada segmento (hombro -> codo, codo -> muñeca, muñeca -> nudillos): su
  proyección en el plano de esa cámara, en largos de torso (caderas -> hombros, que se ve en las dos).
- Con dos cámaras, cada segmento sale de las cuatro ecuaciones (dos por imagen) por mínimos
  cuadrados: lo que una ve de canto, la otra lo ve de lado. No hace falta alinear las cámaras entre
  sí: cada una se orienta con tu torso.
- Con una, a lo que se ve en la imagen le falta la profundidad: sale del largo del segmento (medido
  al calibrar con el brazo colgando) y su signo, de la estimación 3D.

Las dos cámaras ven la imagen en espejo, así que las dos llaman 'derecho' al mismo brazo. Vista de
lado, Pose a veces cambia izquierda y derecha: se usa la asignación que mejor encaja con la de frente.
"""

import numpy as np

METODO_VISTAS = "vistas"   # calibraciones hechas con este método (las de antes eran con el 3D de Pose)
NARIZ = 0
LARGOS_DEFECTO = {"brazo": 0.62, "antebrazo": 0.55, "mano": 0.2}   # en largos de torso (caderas -> hombros)
SEGMENTOS = ("brazo", "antebrazo", "mano")
RESIDUO_MAX = 0.12         # largos de torso: si las dos cámaras discrepan más en el brazo, sólo la de frente
LARGO_TOLERANCIA = 0.12    # y si el brazo que sale de las dos se aleja más de un 12 % de su largo (calibrado con C)
PARES_LADOS = ((1, 4), (2, 5), (3, 6), (7, 8), (9, 10), (11, 12), (13, 14), (15, 16), (17, 18), (19, 20),
               (21, 22), (23, 24), (25, 26), (27, 28), (29, 30), (31, 32))
VISIBLE = 0.5
BORDE = 0.015


class Vista:
    """Lo que da una cámara en un cuadro: puntos 3D {i: (x, y, z, vis)} y de imagen {i: (x, y, vis)}
    normalizados (0..1), y el tamaño de la imagen (ancho, alto) para medir ángulos en píxeles."""

    def __init__(self, mundo, imagen, tam, mano_px=None):
        self.mundo, self.imagen, self.tam = mundo, imagen, tam
        self.mano_px = mano_px          # (muñeca, nudillos) en píxeles del modelo de la mano, si lo hay

    def px(self, i):
        x, y, _ = self.imagen[i]
        return np.array((x * self.tam[0], y * self.tam[1]))

    def visible(self, *indices):
        return all(i in self.imagen and i in self.mundo and self.imagen[i][2] >= VISIBLE
                   and BORDE < self.imagen[i][0] < 1 - BORDE and BORDE < self.imagen[i][1] < 1 - BORDE
                   for i in indices)

    def p3(self, i):
        return np.asarray(self.mundo[i][:3], float)

    def cambiada(self):
        """La misma vista con izquierda y derecha intercambiadas."""
        mundo, imagen = dict(self.mundo), dict(self.imagen)
        for a, b in PARES_LADOS:
            if a in self.mundo and b in self.mundo:
                mundo[a], mundo[b] = self.mundo[b], self.mundo[a]
            if a in self.imagen and b in self.imagen:
                imagen[a], imagen[b] = self.imagen[b], self.imagen[a]
        return Vista(mundo, imagen, self.tam, None)


class Lado:
    """Índices de Pose del brazo que se sigue (ya en espejo, como ARTICULACIONES)."""

    def __init__(self, hombro, codo, muneca, hombro_otro, cadera, cadera_otra, mano):
        self.hombro, self.codo, self.muneca, self.hombro_otro = hombro, codo, muneca, hombro_otro
        self.cadera, self.cadera_otra, self.mano = cadera, cadera_otra, tuple(mano)


def _unitario(v):
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError("vector nulo")
    return v / n


def marco(vista, lado, de_frente=False):
    """Ejes del torso (filas: afuera, adelante, arriba) en los ejes de esa cámara, o None.

    'adelante': en la cámara de frente, hacia ella (la miras); en la lateral, hacia la nariz, que de
    perfil se ve en la imagen. (La nariz no sirve de frente: está delante justo en la profundidad,
    lo que Pose peor estima, y el sentido cambiaría de un cuadro a otro.)
    """
    if not vista.visible(lado.hombro, lado.hombro_otro, lado.cadera, lado.cadera_otra) or (
            not de_frente and NARIZ not in vista.mundo):
        return None
    try:
        hombro, otro = vista.p3(lado.hombro), vista.p3(lado.hombro_otro)
        centro = (hombro + otro) / 2
        caderas = (vista.p3(lado.cadera) + vista.p3(lado.cadera_otra)) / 2
        afuera = _unitario(hombro - otro)
        vertical = centro - caderas
        arriba = _unitario(vertical - (vertical @ afuera) * afuera)
        adelante = np.cross(arriba, afuera)
        if (adelante[2] > 0) if de_frente else ((vista.p3(NARIZ) - centro) @ adelante < 0):
            adelante = -adelante
    except ValueError:
        return None
    return np.array((afuera, adelante, arriba))


class Orientaciones:
    """Orientación de cada cámara respecto a tu torso, suavizada en el tiempo.

    Cambia sólo cuando giras el cuerpo, y la de un solo cuadro tiene el ruido de profundidad de Pose
    (10-20° con 8 cm de error, sobre todo la lateral, que ve la línea de los hombros de canto): se
    promedia con vida media VIDA_MEDIA segundos y se vuelve a dejar ortonormal.
    """

    VIDA_MEDIA = 3.0

    def __init__(self):
        self.M, self.t = {}, {}
        self.lados_cambiados = False     # la lateral con izquierda y derecha cambiadas (se recuerda)
        self.signos = {}                 # con una cámara: hacia qué lado de ella va cada segmento

    def suavizar(self, clave, M, t):
        if clave not in self.M or t < self.t[clave]:
            self.M[clave], self.t[clave] = M, t
            return M
        a = 1.0 - 0.5 ** ((t - self.t[clave]) / self.VIDA_MEDIA)
        S = (1.0 - a) * self.M[clave] + a * M
        U, _, Vt = np.linalg.svd(S)
        R = U @ Vt                               # la matriz ortonormal más cercana (misma orientación)
        self.M[clave], self.t[clave] = R, t
        return R


class Camara:
    """Cómo ve una cámara tu torso: proyección de los ejes del torso a su imagen y su profundidad."""

    def __init__(self, vista, lado, M):
        self.M = M                                   # filas: afuera, adelante, arriba (ejes de la cámara)
        self.A = M[:, :2].T                          # 2x3: un vector del torso -> x, y de la cámara
        self.z = M[:, 2]                             # el eje de profundidad de la cámara, en ejes del torso
        torso = ((vista.px(lado.hombro) + vista.px(lado.hombro_otro)) / 2
                 - (vista.px(lado.cadera) + vista.px(lado.cadera_otra)) / 2)
        # píxeles por largo de torso: el torso que se ve, corregido por lo que se inclina hacia la cámara
        self.escala = np.linalg.norm(torso) / max(np.linalg.norm(self.A[:, 2]), 0.3)

    def en_torso(self, linea_px):
        """Línea de la imagen (píxeles) -> largos de torso en el plano de esa imagen."""
        return np.asarray(linea_px, float) / self.escala


def lineas(vista, lado):
    """{segmento: (línea en la imagen en píxeles, segmento 3D estimado en ejes de la cámara)} de lo visible."""
    salida = {}
    if vista.visible(lado.hombro, lado.codo):
        salida["brazo"] = (vista.px(lado.codo) - vista.px(lado.hombro), vista.p3(lado.codo) - vista.p3(lado.hombro))
    if vista.visible(lado.codo, lado.muneca):
        salida["antebrazo"] = (vista.px(lado.muneca) - vista.px(lado.codo), vista.p3(lado.muneca) - vista.p3(lado.codo))
    if vista.mano_px is not None:
        muneca, nudillos = vista.mano_px
        d3 = (np.mean([vista.p3(i) for i in lado.mano], axis=0) - vista.p3(lado.muneca)
              if all(i in vista.mundo for i in lado.mano) else np.zeros(3))
        salida["mano"] = (np.asarray(nudillos) - np.asarray(muneca), d3)
    elif vista.visible(lado.muneca, *lado.mano):
        medio = np.mean([vista.px(i) for i in lado.mano], axis=0)
        salida["mano"] = (medio - vista.px(lado.muneca),
                          np.mean([vista.p3(i) for i in lado.mano], axis=0) - vista.p3(lado.muneca))
    return salida


SIGNO_CLARO = 0.3          # con una cámara, el lado sólo cambia si la profundidad estimada es >30 % del segmento


def resolver(filas, largo, signo_previo=None):
    """Segmento en ejes del torso (largos de torso) desde sus líneas en una o más cámaras, y el residuo.

    filas: [(Camara, línea en píxeles, segmento 3D estimado en ejes de esa cámara)]. Con dos o más,
    mínimos cuadrados con las dos ecuaciones de cada imagen. Con una, lo que se ve más la profundidad que
    falta para 'largo', hacia el lado de la cámara que estima Pose; si Pose no lo tiene claro, el de antes.
    """
    if len(filas) >= 2:
        A = np.vstack([c.A for c, _, _ in filas])
        b = np.concatenate([c.en_torso(v) for c, v, _ in filas])
        u, *_ = np.linalg.lstsq(A, b, rcond=None)
        return u, float(np.sqrt(np.mean((A @ u - b) ** 2)))
    c, v, d3 = filas[0]
    u0 = np.linalg.pinv(c.A) @ c.en_torso(v)
    u0 = u0 - (u0 @ c.z) * c.z                       # sólo lo que la imagen ve
    d3 = np.zeros(3) if d3 is None else np.asarray(d3, float)
    # La profundidad que falta: del largo del segmento y de la estimación de Pose, promediadas. Con el
    # segmento casi en el plano de la imagen, la del largo amplifica el ruido (3 % de largo -> ~14°).
    por_largo = np.sqrt(max(0.0, largo ** 2 - float(u0 @ u0)))
    por_pose = largo * abs(d3[2]) / max(np.linalg.norm(d3), 1e-9)
    falta = (por_largo + por_pose) / 2
    signo = 1.0 if d3[2] >= 0 else -1.0
    if signo_previo is not None and abs(d3[2]) < SIGNO_CLARO * max(np.linalg.norm(d3), 1e-9):
        signo = signo_previo
    return u0 + signo * falta * c.z, signo


def brazo_desde_vistas(frente, lateral, lado, largos=None, orientaciones=None, t=0.0):
    """(brazo, antebrazo, mano|None) en ejes del torso (afuera, adelante, arriba), en largos de torso, e info.

    orientaciones: Orientaciones que se conserva entre cuadros (sin ella, sólo el cuadro actual).
    None si la cámara de frente no ve hombros, caderas, nariz, codo y muñeca.
    """
    largos = dict(LARGOS_DEFECTO, **(largos or {}))
    M_f = marco(frente, lado, de_frente=True)
    if M_f is None:
        return None
    if orientaciones is not None:
        M_f = orientaciones.suavizar("frente", M_f, t)
    cam_f, lin_f = Camara(frente, lado, M_f), lineas(frente, lado)
    if "brazo" not in lin_f or "antebrazo" not in lin_f:
        return None
    info = {"lateral": False, "residuo": None, "lados_cambiados": False}

    mejor = None
    M_lateral = marco(lateral, lado) if lateral is not None else None
    if M_lateral is not None and orientaciones is not None:
        M_lateral = orientaciones.suavizar("lateral", M_lateral, t)
    if M_lateral is not None:
        opciones = {}
        for cambiada, vista in ((False, lateral), (True, lateral.cambiada())):
            # con los lados cambiados, 'afuera' es el contrario y lo demás no cambia
            M_l = np.diag((-1.0, 1.0, 1.0)) @ M_lateral if cambiada else M_lateral
            cam_l, lin_l = Camara(vista, lado, M_l), lineas(vista, lado)
            if "brazo" in lin_l:
                _, residuo = resolver([(cam_f, lin_f["brazo"][0], None), (cam_l, lin_l["brazo"][0], None)],
                                      largos["brazo"])
                opciones[cambiada] = (residuo, cam_l, lin_l, cambiada)
        if opciones:
            # Se conserva la asignación anterior salvo que la otra encaje claramente mejor (con ruido,
            # las dos pueden encajar parecido en un cuadro y elegir al azar daba direcciones absurdas).
            previa = orientaciones.lados_cambiados if orientaciones is not None else min(opciones, key=lambda k: opciones[k][0])
            elegida = previa if previa in opciones else next(iter(opciones))
            otra = not elegida
            if otra in opciones and opciones[otra][0] < 0.5 * opciones[elegida][0] and opciones[elegida][0] > 0.05:
                elegida = otra
            if orientaciones is not None:
                orientaciones.lados_cambiados = elegida
            mejor = opciones[elegida]
        if mejor is not None:
            info["residuo"] = mejor[0]
            u, _ = resolver([(cam_f, lin_f["brazo"][0], None), (mejor[1], mejor[2]["brazo"][0], None)], largos["brazo"])
            largo = float(np.linalg.norm(u))
            # Cada imagen da dos ecuaciones y sólo comparten la vertical: dos vistas que no encajan pueden
            # dar un residuo pequeño, pero un brazo de un largo imposible.
            if mejor[0] > RESIDUO_MAX or abs(largo - largos["brazo"]) > LARGO_TOLERANCIA * largos["brazo"]:
                mejor = None
    if mejor is not None:
        info.update(lateral=True, lados_cambiados=mejor[3])

    salida = []
    for nombre in SEGMENTOS:
        if nombre not in lin_f:
            salida.append(None)
            continue
        filas = [(cam_f, lin_f[nombre][0], lin_f[nombre][1])]
        if mejor is not None and nombre in mejor[2]:
            filas.append((mejor[1], mejor[2][nombre][0], mejor[2][nombre][1]))
        previo = orientaciones.signos.get(nombre) if orientaciones is not None else None
        u, signo = resolver(filas, largos[nombre], previo)
        if orientaciones is not None and len(filas) == 1:
            orientaciones.signos[nombre] = signo
        salida.append(tuple(float(x) for x in u))
    return tuple(salida), info


def proporciones(frente, lado):
    """Largos de cada segmento visto en la imagen de frente, en largos de torso (para calibrar colgando:
    entonces todo queda en el plano de la imagen y la proporción es el largo real)."""
    M = marco(frente, lado, de_frente=True)
    if M is None:
        return None
    cam = Camara(frente, lado, M)
    # Colgando, el brazo va como el torso (hacia abajo): si la cámara lo ve inclinado, se ve acortado igual.
    inclinacion = max(np.linalg.norm(cam.A[:, 2]), 0.3)
    return {nombre: float(np.linalg.norm(cam.en_torso(linea)) / inclinacion)
            for nombre, (linea, _) in lineas(frente, lado).items()}
