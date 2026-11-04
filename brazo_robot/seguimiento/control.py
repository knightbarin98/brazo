"""Decide qué hace el brazo según las lecturas de la persona. Sin cámara ni ventanas."""

from statistics import median

from .. import config
from ..ajustes import AJUSTES, garra_abierta, garra_cerrada
from ..cinematica import Inalcanzable, ik, z_transito
from ..config import (L1, L2, SEGUIR_DELTA_MIN, SEGUIR_FACTOR_PASO, SEGUIR_FACTOR_RANGO, SEGUIR_FILTRO_ESCALA,
                      SEGUIR_FILTRO_GARRA, SEGUIR_FILTRO_INCLINACION, SEGUIR_FILTRO_XY, SEGUIR_GANANCIA_X,
                      SEGUIR_GANANCIA_Y, SEGUIR_GANANCIA_Z, SEGUIR_GARRA_DELTA_MIN, SEGUIR_GARRA_TOLERANCIA,
                      SEGUIR_INCLINACION_DELTA_MIN, SEGUIR_INCLINACION_LIMITE, SEGUIR_INCLINACION_VEL,
                      SEGUIR_INICIO, SEGUIR_LIMITE_X, SEGUIR_LIMITE_Y, SEGUIR_PERIODO, SEGUIR_SALTO_MAX,
                      SEGUIR_TIEMPO_REFERENCIA, SEGUIR_TIMEOUT_MANO, SEGUIR_VELOCIDAD_MM_S,
                      SEGUIR_VELOCIDAD_SERVOS, Z_SEGURO)
from .mapeo import MODO_BRAZO, FiltroUnEuro, MapeoRelativo, SeguidorGarra, limitar_paso, proyectar_alcanzable

PAUSADO, ENGANCHANDO, SIGUIENDO = "PAUSADO", "ENGANCHANDO", "SIGUIENDO"
ACTIVOS = (ENGANCHANDO, SIGUIENDO)
TEXTO_SIGUIENDO = "Siguiendo. Abre y cierra la mano para mover la garra."


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
        self._muestras = [mano]
        self.t_inicio = self.t_mano = t
        self.mano_previa = mano
        self.aviso = "Mantente quieto un momento..."

    def _enganchar(self, t):
        """Referencia con la mediana: una sola lectura con ruido movería todo lo que sigue."""
        m = self._muestras
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
        self.seguidor_garra.soltar()
        self.t_envio = t
        self.estado = SIGUIENDO
        self.aviso = TEXTO_SIGUIENDO

    def alternar_garra(self):
        """Abre o cierra a mano. La garra deja de seguir la mano hasta que esta la alcance otra vez."""
        abierta, cerrada = garra_abierta(), garra_cerrada()
        nuevo = abierta if abs(self.garra - cerrada) < abs(self.garra - abierta) else cerrada
        self.seguidor_garra.soltar()
        self.brazo.mover_continuo(self.brazo.ultimo[:4] + [nuevo])

    def ajustar_factor(self, signo):
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
            if t - self.t_mano > SEGUIR_TIMEOUT_MANO:
                self.pausar("Sin datos de la persona: brazo detenido.")
            return
        previa = self.mano_previa
        if mano.modo != previa.modo:
            self.pausar("Cambió el modo de seguimiento: brazo detenido.")
            return
        if max(abs(mano.x - previa.x), abs(mano.y - previa.y)) > SEGUIR_SALTO_MAX:
            self.pausar("La lectura saltó (¿otra persona en cuadro?): brazo detenido.")
            return
        self.mano_previa, self.t_mano = mano, t
        if self.estado == ENGANCHANDO:
            self._muestras.append(mano)
            if t - self.t_inicio >= SEGUIR_TIEMPO_REFERENCIA:
                self._enganchar(t)
            return
        self._seguir(mano, t)

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
