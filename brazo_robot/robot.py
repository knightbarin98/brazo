"""Brazo: comunicación serial con el Arduino y rutinas de movimiento."""

import time

import numpy as np
import serial

from . import config
from .ajustes import AJUSTES, guardar_ajustes, garra_abierta, garra_cerrada, numero_en_rango
from .cinematica import ik, z_transito
from .config import (ARDUINO_HOME, BAUD, GARRA_MAX_GRADOS, HOME, PUERTO, SERIAL_FALLOS_MAX, SERIAL_TIMEOUT,
                     SERIAL_TIMEOUT_RAPIDO, Z_SEGURO, Z_SOLTAR)


class SerialSimulado:
    """Sustituye al Arduino (--simular): acepta todas las órdenes sin mover nada."""

    timeout = None

    def write(self, datos):
        pass

    def readline(self):
        return b"OK\n"

    def reset_input_buffer(self):
        pass


class Brazo:
    def __init__(self, puerto=PUERTO, simular=False):
        if simular:
            self.ser = SerialSimulado()
        else:
            self.ser = serial.Serial(puerto, BAUD, timeout=SERIAL_TIMEOUT)
            time.sleep(2.5)                  # el Arduino se reinicia al abrir el puerto
        self.ser.reset_input_buffer()
        self._timeout = SERIAL_TIMEOUT
        self.fallos = 0
        self.ultimo = list(ARDUINO_HOME)
        self.velocidad = float(AJUSTES["velocidad"])
        self._cmd(f"V,{self.velocidad:.2f}")
        self.home()

    def _poner_timeout(self, segundos):
        if segundos != self._timeout:      # cambiarlo reconfigura el puerto: sólo si hace falta
            self.ser.timeout = self._timeout = segundos

    @staticmethod
    def _limitar(valores):
        a = [max(0.0, min(180.0, float(v))) for v in valores]
        a[4] = min(a[4], GARRA_MAX_GRADOS)
        return a

    def _cmd(self, txt):
        self._poner_timeout(SERIAL_TIMEOUT)
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
        """valores = [base, hombro, codo, muñeca, garra(grados)]. Espera a que llegue."""
        a = self._limitar(valores)
        self._cmd("S," + ",".join(f"{v:.1f}" for v in a))
        delta = max(abs(p - q) for p, q in zip(a[:4], self.ultimo[:4]))
        time.sleep(delta / self.velocidad * 0.015 + 0.15)
        self.ultimo = a

    def mover_continuo(self, valores):
        """Para seguir un objetivo que cambia: no espera a que llegue y no frena el control.

        Devuelve False si el Arduino no confirmó a tiempo; con varios fallos seguidos, RuntimeError.
        """
        a = self._limitar(valores)
        self._poner_timeout(SERIAL_TIMEOUT_RAPIDO)
        self.ser.reset_input_buffer()      # un OK tardío no debe contar como el de esta orden
        self.ser.write(("S," + ",".join(f"{v:.1f}" for v in a) + "\n").encode())
        if self.ser.readline().decode(errors="ignore").strip() != "OK":
            self.fallos += 1
            if self.fallos >= SERIAL_FALLOS_MAX:
                self.fallos = 0
                raise RuntimeError(f"El Arduino no confirmó {SERIAL_FALLOS_MAX} órdenes seguidas")
            return False
        self.fallos = 0
        self.ultimo = a
        return True

    def home(self, garra=None):
        """garra=None la abre, como al terminar una rutina; pasa el ángulo actual para no soltar nada."""
        self.mover(HOME + [garra_abierta() if garra is None else garra])

    def ir(self, x, y, z, garra, inclinacion=-90.):
        self.mover(ik(x, y, z, inclinacion) + [garra])

    def verificar(self, x, y, z_bajo):
        ik(x, y, Z_SEGURO)
        ik(x, y, z_bajo)

    # El plan vertical de recogida/depósito usa la altura adaptativa.
    def plan_vertical(self, x, y, z1, z2):
        alto, bajo = max(z1, z2), min(z1, z2)
        tope = z_transito(x, y, bajo, alto)
        if tope < alto - 0.5:
            print(f"  Aproximación adaptada en ({x:.0f}, {y:.0f}): Z={tope:.0f} en lugar de {alto:.0f}")
        z1, z2 = (tope, z2) if z1 >= z2 else (z1, tope)
        n = max(1, int(np.ceil(abs(z2 - z1) / AJUSTES["paso_bajada"])))
        return [ik(x, y, float(z), AJUSTES["inclinacion"]) for z in np.linspace(z1, z2, n + 1)]

    def ejecutar_vertical(self, plan, garra, velocidad):
        self.ajustar_velocidad(velocidad, guardar=False)
        for angulos in plan[1:]:
            self.mover(angulos + [garra])

    def recoger_en_origen(self, x, y):
        """Prueba sin traslado: acaba arriba con la garra cerrada."""
        plan = self.plan_vertical(x, y, Z_SEGURO, config.Z_RECOGIDA)
        self.mover(plan[0] + [garra_abierta()])
        print("Bajada por segmentos, manteniendo la inclinación configurada.")
        self.ejecutar_vertical(plan, garra_abierta(), AJUSTES["velocidad_bajada"])
        print(f"Contacto: pausa {AJUSTES['pausa_contacto']:.1f} s")
        time.sleep(AJUSTES["pausa_contacto"])
        self.mover(plan[-1] + [garra_cerrada()])
        print(f"Garra cerrada: esperando {AJUSTES['tiempo_agarre']:.1f} s antes de subir")
        time.sleep(AJUSTES["tiempo_agarre"])
        self.ejecutar_vertical(list(reversed(plan)), garra_cerrada(), AJUSTES["velocidad_subida"])
        time.sleep(AJUSTES["pausa_arriba"])

    def probar_agarre(self, x, y):
        anterior = self.velocidad
        try:
            self.ajustar_velocidad(AJUSTES["velocidad_bajada"], guardar=False)
            self.recoger_en_origen(x, y)
            print("Prueba terminada: garra cerrada arriba. 'garra abrir' libera la pieza.")
        finally:
            self.ajustar_velocidad(anterior, guardar=False)

    def recoger_y_colocar(self, x, y, dx, dy):
        # Calcula la totalidad de ambas verticales antes de empezar a recoger.
        self.plan_vertical(x, y, Z_SEGURO, config.Z_RECOGIDA)
        destino = self.plan_vertical(dx, dy, Z_SEGURO, Z_SOLTAR)
        anterior = self.velocidad
        try:
            self.recoger_en_origen(x, y)
            self.ajustar_velocidad(anterior, guardar=False)
            self.mover(destino[0] + [garra_cerrada()])
            self.ejecutar_vertical(destino, garra_cerrada(), AJUSTES["velocidad_bajada"])
            self.mover(destino[-1] + [garra_abierta()])
            time.sleep(AJUSTES["tiempo_soltar"])
            self.ejecutar_vertical(list(reversed(destino)), garra_abierta(), AJUSTES["velocidad_subida"])
            self.ajustar_velocidad(anterior, guardar=False)
            self.home()
        finally:
            self.ajustar_velocidad(anterior, guardar=False)
