"""Decide qué hace el brazo según la mano detectada. Sin cámara ni ventanas."""

from .. import config
from ..ajustes import AJUSTES, iman_off, iman_on
from ..cinematica import Inalcanzable, ik, z_transito
from ..config import (SEGUIR_DELTA_MIN, SEGUIR_FILTRO_ESCALA, SEGUIR_FILTRO_XY, SEGUIR_GANANCIA_X, SEGUIR_GANANCIA_Y,
                      SEGUIR_GANANCIA_Z, SEGUIR_INICIO, SEGUIR_LIMITE_X, SEGUIR_LIMITE_Y, SEGUIR_PERIODO,
                      SEGUIR_PINZA_ABRIR, SEGUIR_PINZA_CERRAR, SEGUIR_PINZA_TIEMPO, SEGUIR_SALTO_MAX,
                      SEGUIR_TIMEOUT_MANO, SEGUIR_VELOCIDAD_MM_S, SEGUIR_VELOCIDAD_SERVOS, Z_SEGURO)
from .mapeo import MapeoRelativo, Pinza, limitar_paso

PAUSADO, SIGUIENDO = "PAUSADO", "SIGUIENDO"


class ControlSeguimiento:
    """El brazo sólo se mueve en SIGUIENDO. Cualquier duda (sin mano, salto) lo deja quieto en
    PAUSADO y hace falta volver a activarlo. El imán nunca cambia solo: no suelta la pieza."""

    def __init__(self, brazo, inclinacion):
        self.brazo, self.inclinacion = brazo, inclinacion
        self.estado = PAUSADO
        self.pose = None
        self.aviso = "ACTIVAR para empezar (el brazo irá al punto de partida)."
        limites = (SEGUIR_LIMITE_X, SEGUIR_LIMITE_Y, (config.Z_RECOGIDA, Z_SEGURO))
        self.mapeo = MapeoRelativo((SEGUIR_GANANCIA_X, SEGUIR_GANANCIA_Y, SEGUIR_GANANCIA_Z), limites,
                                   SEGUIR_FILTRO_XY, SEGUIR_FILTRO_ESCALA)
        self.pinza = Pinza(SEGUIR_PINZA_CERRAR, SEGUIR_PINZA_ABRIR, SEGUIR_PINZA_TIEMPO)
        self.t_mano = self.t_envio = 0.0
        self.mano_previa = None

    @property
    def iman_encendido(self):
        return self.brazo.ultimo[4] == iman_on()

    def pausar(self, motivo):
        self.estado = PAUSADO
        self.mano_previa = None
        self.aviso = motivo + " ACTIVAR para seguir."

    def _ir_al_inicio(self):
        """Primer enganche: lleva el brazo a una postura conocida, a la mayor altura alcanzable."""
        x, y = SEGUIR_INICIO
        z = z_transito(x, y, config.Z_RECOGIDA, Z_SEGURO, self.inclinacion)
        self.brazo.ajustar_velocidad(self._velocidad_transito(), guardar=False)
        self.brazo.ir(x, y, z, self.brazo.ultimo[4], self.inclinacion)
        self.brazo.ajustar_velocidad(SEGUIR_VELOCIDAD_SERVOS, guardar=False)
        self.pose = (x, y, z)

    @staticmethod
    def _velocidad_transito():
        return min(max(AJUSTES["velocidad"], 0.5), 2.0)

    def ir_a_home(self):
        """Sólo en pausa y con el imán apagado: brazo.home() apaga el imán y soltaría la pieza."""
        if self.estado == SIGUIENDO:
            self.aviso = "Pausa el brazo antes de ir a HOME."
        elif self.iman_encendido:
            self.aviso = "Apaga el imán antes de ir a HOME: soltaría la pieza."
        else:
            self.brazo.ajustar_velocidad(self._velocidad_transito(), guardar=False)
            self.brazo.home()
            self.pose = None
            self.aviso = "En HOME. ACTIVAR lo lleva al punto de partida."

    def activar(self, mano, t):
        """Engancha la mano; la primera vez sólo lleva el brazo al punto de partida."""
        if self.estado == SIGUIENDO:
            return
        if self.pose is None:
            try:
                self._ir_al_inicio()
                self.aviso = "En el punto de partida. Pon la mano frente a la cámara y pulsa ACTIVAR."
            except Inalcanzable as e:
                self.aviso = f"Punto de partida fuera de alcance: {e}"
            return
        if mano is None:
            self.aviso = "No veo la mano: ponla frente a la cámara y pulsa ACTIVAR."
            return
        self.mapeo.enganchar(mano, self.pose, t)
        self.pinza.reiniciar(mano.pinza)
        self.t_mano = self.t_envio = t
        self.mano_previa = mano
        self.estado = SIGUIENDO
        self.aviso = "Siguiendo. Pinza (pulgar-índice) = imán."

    def alternar_iman(self):
        nuevo = iman_off() if self.iman_encendido else iman_on()
        self.brazo.mover_continuo(self.brazo.ultimo[:4] + [nuevo])

    def actualizar(self, mano, t):
        """Llamar siempre, aunque no haya fotograma nuevo (mano=None): así vigila el tiempo."""
        if self.estado != SIGUIENDO:
            return
        if mano is None:
            if t - self.t_mano > SEGUIR_TIMEOUT_MANO:
                self.pausar("Sin datos de la mano: brazo detenido.")
            return
        previa = self.mano_previa
        if max(abs(mano.x - previa.x), abs(mano.y - previa.y)) > SEGUIR_SALTO_MAX:
            self.pausar("La mano saltó (¿otra persona en cuadro?): brazo detenido.")
            return
        self.mano_previa, self.t_mano = mano, t
        if self.pinza.actualizar(mano.pinza, t):
            self.alternar_iman()
        objetivo = self.mapeo.objetivo(mano, t)
        dt = t - self.t_envio
        if dt < SEGUIR_PERIODO:
            return
        # Tope de dt: tras un atasco del bucle no se puede recuperar el tiempo de golpe.
        paso = limitar_paso(self.pose, objetivo, SEGUIR_VELOCIDAD_MM_S * min(dt, 4 * SEGUIR_PERIODO))
        if max(abs(p - q) for p, q in zip(paso, self.pose)) < SEGUIR_DELTA_MIN:
            return
        try:
            angulos = ik(*paso, self.inclinacion)
        except Inalcanzable as e:
            self.aviso = f"Límite del brazo: {e}"
            return
        self.brazo.mover_continuo(angulos + [self.brazo.ultimo[4]])
        self.pose, self.t_envio = paso, t
        self.aviso = "Siguiendo. Pinza (pulgar-índice) = imán."
