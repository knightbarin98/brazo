"""Decide qué hace el brazo según las lecturas de la persona. Sin cámara ni ventanas."""

from math import cos, radians, sin
from statistics import median

from .. import config
from ..ajustes import AJUSTES, garra_abierta, garra_cerrada
from ..cinematica import (ARTICULACIONES_SERVO, Inalcanzable, ik, postura_desde_servos, postura_segura,
                          servos_desde_geo, z_transito)
from ..config import (ARTICULAR_ACERCADO, ARTICULAR_BASE, ARTICULAR_DELTA_MIN, ARTICULAR_DESFASE,
                      ARTICULAR_ESPEJO, ARTICULAR_FILTRO,
                      ARTICULAR_LIMITES, ARTICULAR_SALTO_MAX, ARTICULAR_TIMEOUT, ARTICULAR_VELOCIDAD, ARTICULAR_VELOCIDAD_ACERCAMIENTO, L1, L2, MAPEO,
                      SEGUIR_DELTA_MIN, SEGUIR_FACTOR_PASO, SEGUIR_FACTOR_RANGO, SEGUIR_FILTRO_ESCALA,
                      SEGUIR_FILTRO_GARRA, SEGUIR_FILTRO_INCLINACION, SEGUIR_FILTRO_XY, SEGUIR_GANANCIA_X,
                      SEGUIR_GANANCIA_Y, SEGUIR_GANANCIA_Z, SEGUIR_GARRA_DELTA_MIN, SEGUIR_GARRA_TOLERANCIA,
                      SEGUIR_INCLINACION_DELTA_MIN, SEGUIR_INCLINACION_LIMITE, SEGUIR_INCLINACION_VEL,
                      SEGUIR_INICIO, SEGUIR_LIMITE_X, SEGUIR_LIMITE_Y, SEGUIR_PERIODO, SEGUIR_SALTO_MAX,
                      SEGUIR_TIEMPO_REFERENCIA, SEGUIR_TIMEOUT_MANO, SEGUIR_VELOCIDAD_MM_S,
                      SEGUIR_VELOCIDAD_SERVOS, Z_SEGURO)
from .mapeo import MODO_ARTICULAR, MODO_BRAZO, FiltroUnEuro, envolver, MapeoRelativo, SeguidorGarra, limitar_paso, proyectar_alcanzable

PAUSADO, ENGANCHANDO, SIGUIENDO = "PAUSADO", "ENGANCHANDO", "SIGUIENDO"
ACTIVOS = (ENGANCHANDO, SIGUIENDO)
TEXTO_SIGUIENDO = "Siguiendo. Abre y cierra la mano para mover la garra."
TEXTO_ARTICULAR = "Siguiendo tu brazo. Abre y cierra la mano para mover la garra."


def _geo_de_servos(servos):
    return [(s - MAPEO[k][0]) / MAPEO[k][1] for k, s in zip(ARTICULACIONES_SERVO, servos)]


def _rango_geo(nombre):
    """Límites de ARTICULAR_LIMITES (servo) pasados a ángulo geométrico."""
    cero, signo = MAPEO[nombre]
    a, b = ((v - cero) / signo for v in ARTICULAR_LIMITES[nombre])
    return min(a, b), max(a, b)


def _desenrollar(angulo, referencia):
    """El equivalente de 'angulo' (±360·k) más cercano a 'referencia': sin saltos al cruzar ±180°."""
    return angulo + 360.0 * round((referencia - angulo) / 360.0)


def _pose_de_servos(servos):
    """Punto de agarre (x, y, z) en mm e inclinación de la garra, a partir de los servos."""
    base, puntos = postura_desde_servos(servos)
    r, z = puntos[-1]
    geo = _geo_de_servos(servos)
    return (r * cos(radians(base)), r * sin(radians(base)), z), geo[1] + geo[2] + geo[3]


class SeguidorArticular:
    """Ángulos de tu brazo en el plano frontal -> servos del robot, que se mueve en el plano de la caja.

    La base queda fija en ARTICULAR_BASE. Con ARTICULAR_ESPEJO el robot es tu brazo visto en un
    espejo (su frente mira hacia tu cuerpo): hombro 180 - el tuyo, codo y mano con el signo invertido.
    Cada ángulo se sigue sin envolver: al pasar por una zona que el robot no alcanza (p. ej. el brazo
    colgando) el motor se queda en el tope por donde salió, sin saltar al otro. La muñeca se calcula
    al final, con el hombro y el codo ya recortados: así la garra apunta como tu mano.
    """

    def __init__(self, muestras, t):
        valores = [[a[k] for a in muestras if a[k] is not None] for k in range(4)]
        self.crudo = [median(v) if v else None for v in valores]
        self.filtros = [FiltroUnEuro(*ARTICULAR_FILTRO) for _ in range(4)]
        self.rangos = [_rango_geo(k) for k in ARTICULACIONES_SERVO]
        self.base = (ARTICULAR_BASE - MAPEO["base"][0]) / MAPEO["base"][1]
        self.acercando = True
        for k, v in enumerate(self.crudo):
            if v is not None:
                self.filtros[k](v, t)

    def _filtrar(self, k, angulo, t):
        if self.crudo[k] is not None:
            angulo = _desenrollar(angulo, self.crudo[k])
        self.crudo[k] = angulo
        return self.filtros[k](angulo, t)

    def _recortar(self, k, valor):
        lo, hi = self.rangos[k]
        return min(hi, max(lo, valor))

    def objetivo(self, angulos, t, servos_actuales):
        """Servos pedidos (lo que no se ve queda donde está) y si alguna articulación quedó en su tope."""
        geo = _geo_de_servos(servos_actuales)
        pedido = list(geo)
        pedido[0] = self.base
        if angulos[1] is not None:
            hombro = self._filtrar(1, angulos[1] + ARTICULAR_DESFASE["hombro"], t)
            pedido[1] = 180.0 - hombro if ARTICULAR_ESPEJO else hombro
        if angulos[2] is not None:
            codo = self._filtrar(2, angulos[2] + ARTICULAR_DESFASE["codo"], t)
            pedido[2] = -codo if ARTICULAR_ESPEJO else codo
        for k in range(3):
            geo[k] = self._recortar(k, pedido[k])
        if angulos[3] is not None:
            mano = self._filtrar(3, angulos[3], t)
            garra = 180.0 - mano if ARTICULAR_ESPEJO else mano
            pedido[3] = envolver(garra - geo[1] - geo[2] + ARTICULAR_DESFASE["muneca"])
            geo[3] = self._recortar(3, pedido[3])
        recortado = any(abs(g - p) > 1e-6 for g, p in zip(geo, pedido))
        return servos_desde_geo(dict(zip(ARTICULACIONES_SERVO, geo))), recortado


class ControlSeguimiento:
    """El brazo sólo se mueve en SIGUIENDO. Cualquier duda (sin datos, salto) lo deja quieto en
    PAUSADO y hace falta volver a activarlo. La garra nunca se abre sola: no suelta la pieza."""

    def __init__(self, brazo, inclinacion):
        self.brazo = brazo
        self.inclinacion_base = self.inclinacion = inclinacion
        self.estado = PAUSADO
        self.pose = self.centro = None
        self.aviso = "ACTIVAR para empezar (el brazo irá al punto de partida)."
        limites = (SEGUIR_LIMITE_X, SEGUIR_LIMITE_Y, (config.Z_RECOGIDA, Z_SEGURO))
        self.mapeo = MapeoRelativo(limites, (SEGUIR_FILTRO_ESCALA, SEGUIR_FILTRO_XY, SEGUIR_FILTRO_XY))
        self.seguidor_garra = SeguidorGarra(SEGUIR_GARRA_TOLERANCIA, SEGUIR_FILTRO_GARRA)
        self.t_mano = self.t_envio = self.t_inicio = 0.0
        self.mano_previa = None
        self._muestras = []
        self.ref_cabeceo = None
        self.filtro_inclinacion = None
        self.modo = None
        self.articular = None             # SeguidorArticular mientras se sigue en modo articular
        self.ordenes = 0                  # órdenes enviadas: para medir órdenes por segundo

    @property
    def factor(self):
        return self.mapeo.factor

    @property
    def garra(self):
        return self.brazo.ultimo[4]

    def pausar(self, motivo):
        self.estado = PAUSADO
        self.mano_previa = None
        self.aviso = motivo + " ACTIVAR para seguir."

    @staticmethod
    def _velocidad_transito():
        return min(max(AJUSTES["velocidad"], 0.5), 2.0)

    def _ir_al_inicio(self):
        """Primer enganche: lleva el brazo a una postura conocida, a la mayor altura alcanzable."""
        x, y = SEGUIR_INICIO
        z = z_transito(x, y, config.Z_RECOGIDA, Z_SEGURO, self.inclinacion_base)
        self.brazo.ajustar_velocidad(self._velocidad_transito(), guardar=False)
        self.brazo.ir(x, y, z, self.garra, self.inclinacion_base)
        self.brazo.ajustar_velocidad(SEGUIR_VELOCIDAD_SERVOS, guardar=False)
        self.pose = self.centro = (x, y, z)
        self.inclinacion = self.inclinacion_base

    def ir_a_home(self):
        """Sólo en pausa. Conserva el ángulo de la garra para no soltar lo que sostenga."""
        if self.estado in ACTIVOS:
            self.aviso = "Pausa el brazo antes de ir a HOME."
            return
        self.brazo.ajustar_velocidad(self._velocidad_transito(), guardar=False)
        self.brazo.home(self.garra)
        self.pose = None
        self.aviso = "En HOME. ACTIVAR lo lleva al punto de partida."

    def activar(self, mano, t):
        """La primera vez sólo lleva el brazo al punto de partida; después toma la referencia."""
        if self.estado in ACTIVOS:
            return
        if mano is not None and self.modo not in (None, mano.modo):
            self.pose = None              # otro modo: se empieza de nuevo desde el punto de partida
            self.modo = mano.modo
        if self.pose is None and mano is not None and mano.modo == MODO_ARTICULAR:
            # El modo articular no usa el punto de partida: se acerca despacio a tu postura desde donde esté.
            self.pose, self.inclinacion = _pose_de_servos(self.brazo.ultimo[:4])
        if self.pose is None:
            try:
                self._ir_al_inicio()
                self.aviso = "En el punto de partida. Ponte frente a la cámara y pulsa ACTIVAR."
            except Inalcanzable as e:
                self.aviso = f"Punto de partida fuera de alcance: {e}"
            return
        if mano is None:
            self.aviso = "No te veo: ponte frente a la cámara y pulsa ACTIVAR."
            return
        self.estado = ENGANCHANDO
        self.modo = mano.modo
        self._muestras = [mano]
        self.t_inicio = self.t_mano = t
        self.mano_previa = mano
        self.aviso = "Mantente quieto un momento..."

    def _enganchar(self, t):
        """Referencia con la mediana: una sola lectura con ruido movería todo lo que sigue."""
        m = self._muestras
        self.seguidor_garra.soltar()
        self.t_envio = t
        self.estado = SIGUIENDO
        if self.modo == MODO_ARTICULAR:
            self.articular = SeguidorArticular([s.angulos for s in m], t)
            self._objetivo, self._recortado, self._garra_objetivo = list(self.brazo.ultimo[:4]), False, self.garra
            self.aviso = "Acercandose despacio a la postura de tu brazo..."
            return
        vector = tuple(median(s.vector[k] for s in m) for k in range(3))
        if m[-1].modo == MODO_BRAZO:
            largo = max(median(s.largo for s in m), 0.1)
            mm_por_unidad = ((L1 + L2) / largo,) * 3
        else:
            mm_por_unidad = (SEGUIR_GANANCIA_X, SEGUIR_GANANCIA_Y, SEGUIR_GANANCIA_Z)
        self.mapeo.enganchar(vector, self.pose, mm_por_unidad, t)
        cabeceos = [s.cabeceo for s in m if s.cabeceo is not None]
        self.ref_cabeceo = median(cabeceos) if cabeceos else None
        self.filtro_inclinacion = FiltroUnEuro(*SEGUIR_FILTRO_INCLINACION)
        self.inclinacion = self.inclinacion_base
        self.aviso = TEXTO_SIGUIENDO

    def alternar_garra(self):
        """Abre o cierra a mano. La garra deja de seguir la mano hasta que esta la alcance otra vez."""
        abierta, cerrada = garra_abierta(), garra_cerrada()
        nuevo = abierta if abs(self.garra - cerrada) < abs(self.garra - abierta) else cerrada
        self.seguidor_garra.soltar()
        self.brazo.mover_continuo(self.brazo.ultimo[:4] + [nuevo])

    def ajustar_factor(self, signo):
        if self.modo == MODO_ARTICULAR:
            self.aviso = "En modo articular no hay ganancia: el brazo copia tus angulos."
            return
        lo, hi = SEGUIR_FACTOR_RANGO
        factor = round(min(hi, max(lo, self.factor + signo * SEGUIR_FACTOR_PASO)), 2)
        self.mapeo.cambiar_factor(factor, self.pose if self.pose is not None else (0.0, 0.0, 0.0))

    def _inclinacion_pedida(self, mano, t, dt):
        if SEGUIR_INCLINACION_LIMITE <= 0 or mano.cabeceo is None:
            return self.inclinacion
        if self.ref_cabeceo is None:
            self.ref_cabeceo = mano.cabeceo   # la mano apareció después de enganchar
        giro = max(-SEGUIR_INCLINACION_LIMITE, min(SEGUIR_INCLINACION_LIMITE, mano.cabeceo - self.ref_cabeceo))
        pedida = self.filtro_inclinacion(self.inclinacion_base + giro, t)
        maximo = SEGUIR_INCLINACION_VEL * dt
        return max(-120.0, min(-60.0, self.inclinacion + max(-maximo, min(maximo, pedida - self.inclinacion))))

    def _alcanzable(self, objetivo, inclinacion):
        """(punto, inclinación): primero la pedida; si ni el centro llega con ella, la calibrada."""
        for inc in (inclinacion, self.inclinacion_base):
            def llega(p, inc=inc):
                try:
                    ik(*p, inc)
                    return True
                except Inalcanzable:
                    return False
            punto = proyectar_alcanzable(self.centro, objetivo, llega)
            if punto is not None:
                return punto, inc
        return None, inclinacion

    def actualizar(self, mano, t):
        """Llamar siempre, aunque no haya lectura nueva (mano=None): así vigila el tiempo."""
        if self.estado not in ACTIVOS:
            return
        if mano is None:
            if t - self.t_mano > (ARTICULAR_TIMEOUT if self.modo == MODO_ARTICULAR else SEGUIR_TIMEOUT_MANO):
                self.pausar("Sin datos de la persona: brazo detenido.")
            elif self.estado == SIGUIENDO and self.modo == MODO_ARTICULAR:
                self._mover_articular(t)          # entre lecturas sigue avanzando hacia la última postura
            return
        previa = self.mano_previa
        if mano.modo != previa.modo:
            self.pausar("Cambió el modo de seguimiento: brazo detenido.")
            return
        salto_max = ARTICULAR_SALTO_MAX if mano.modo == MODO_ARTICULAR else SEGUIR_SALTO_MAX
        if max(abs(mano.x - previa.x), abs(mano.y - previa.y)) > salto_max:
            self.pausar("La lectura saltó (¿otra persona en cuadro?): brazo detenido.")
            return
        self.mano_previa, self.t_mano = mano, t
        if self.estado == ENGANCHANDO:
            self._muestras.append(mano)
            if t - self.t_inicio >= SEGUIR_TIEMPO_REFERENCIA:
                self._enganchar(t)
            return
        if self.modo == MODO_ARTICULAR:
            self._seguir_articular(mano, t)
        else:
            self._seguir(mano, t)

    def _seguir_articular(self, mano, t):
        """Lectura nueva: actualiza la postura objetivo y la garra, y avanza hacia ellas."""
        abierta, cerrada = garra_abierta(), garra_cerrada()
        garra = self.seguidor_garra.actualizar(mano.apertura, t, self.garra, abierta, cerrada)
        self._garra_objetivo = self.garra if garra is None else garra
        self._objetivo, self._recortado = self.articular.objetivo(mano.angulos, t, self.brazo.ultimo[:4])
        self._mover_articular(t)

    def _mover_articular(self, t):
        """Un paso hacia la postura objetivo, a velocidad limitada y sin tocar la mesa ni la base."""
        actual, objetivo, recortado, garra = self.brazo.ultimo[:4], self._objetivo, self._recortado, self._garra_objetivo
        dt = t - self.t_envio
        if dt < SEGUIR_PERIODO:
            return
        dt = min(dt, 4 * SEGUIR_PERIODO)
        velocidad = ARTICULAR_VELOCIDAD_ACERCAMIENTO if self.articular.acercando else ARTICULAR_VELOCIDAD
        paso = limitar_paso(actual, objetivo, velocidad * dt)
        seguro = proyectar_alcanzable(actual, paso, postura_segura)
        if seguro is None:
            self.pausar("La postura actual toca la mesa o la base: brazo detenido.")
            return
        bloqueado = max(abs(a - b) for a, b in zip(seguro, paso)) > 0.5
        if (max(abs(a - b) for a, b in zip(seguro, actual)) < ARTICULAR_DELTA_MIN
                and abs(garra - self.garra) < SEGUIR_GARRA_DELTA_MIN):
            if bloqueado:
                self.aviso = "Alto: con esa postura el brazo tocaria la mesa o la base."
            return
        if not self.brazo.mover_continuo(list(seguro) + [garra]):
            return
        self.t_envio = t
        self.ordenes += 1
        self.pose, self.inclinacion = _pose_de_servos(seguro)
        if self.articular.acercando and max(abs(a - b) for a, b in zip(seguro, objetivo)) < ARTICULAR_ACERCADO:
            self.articular.acercando = False
        if bloqueado:
            self.aviso = "Alto: con esa postura el brazo tocaria la mesa o la base."
        elif self.articular.acercando:
            self.aviso = "Acercandose despacio a la postura de tu brazo..."
        elif not self.seguidor_garra.sigue:
            self.aviso = "Abre o cierra la mano hasta la apertura actual de la garra."
        elif recortado:
            self.aviso = "Alguna articulacion esta en el tope del motor."
        else:
            self.aviso = TEXTO_ARTICULAR

    def _seguir(self, mano, t):
        objetivo = self.mapeo.objetivo(mano.vector, t)
        abierta, cerrada = garra_abierta(), garra_cerrada()
        garra = self.seguidor_garra.actualizar(mano.apertura, t, self.garra, abierta, cerrada)
        dt = t - self.t_envio
        if dt < SEGUIR_PERIODO:
            return
        # Tope de dt: tras un atasco del bucle no se puede recuperar el tiempo de golpe.
        dt = min(dt, 4 * SEGUIR_PERIODO)
        punto, inclinacion = self._alcanzable(objetivo, self._inclinacion_pedida(mano, t, dt))
        if punto is None:
            self.aviso = "Límite del brazo."
            return
        en_borde = max(abs(p - o) for p, o in zip(punto, objetivo)) > 1.0
        paso = limitar_paso(self.pose, punto, SEGUIR_VELOCIDAD_MM_S * dt)
        garra = self.garra if garra is None else garra
        if (max(abs(p - q) for p, q in zip(paso, self.pose)) < SEGUIR_DELTA_MIN
                and abs(inclinacion - self.inclinacion) < SEGUIR_INCLINACION_DELTA_MIN
                and abs(garra - self.garra) < SEGUIR_GARRA_DELTA_MIN):
            return
        try:
            angulos = ik(*paso, inclinacion)
        except Inalcanzable as e:
            self.aviso = f"Límite del brazo: {e}"
            return
        if self.brazo.mover_continuo(angulos + [garra]):
            self.pose, self.inclinacion, self.t_envio = paso, inclinacion, t
            self.ordenes += 1
            if not self.seguidor_garra.sigue:
                self.aviso = "Abre o cierra la mano hasta la apertura actual de la garra."
            elif en_borde:
                self.aviso = "En el límite del brazo: se desliza por el borde."
            else:
                self.aviso = TEXTO_SIGUIENDO
