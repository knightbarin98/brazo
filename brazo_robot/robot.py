"""Brazo: comunicación serial con el Arduino y rutinas de movimiento."""

import time

import numpy as np
import serial

from . import config
from .ajustes import AJUSTES, guardar_ajustes, iman_off, iman_on, numero_en_rango
from .cinematica import ik, z_transito
from .config import ARDUINO_HOME, BAUD, HOME, PUERTO, Z_SEGURO, Z_SOLTAR


class SerialSimulado:
    """Sustituye al Arduino (--simular): acepta todas las órdenes sin mover nada."""

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
            self.ser = serial.Serial(puerto, BAUD, timeout=2)
            time.sleep(2.5)                  # el Arduino se reinicia al abrir el puerto
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
        anterior = self.ultimo
        self.mover_continuo(valores)
        delta = max(abs(p - q) for p, q in zip(self.ultimo[:4], anterior[:4]))
        time.sleep(delta / self.velocidad * 0.015 + 0.15)

    def mover_continuo(self, valores):
        """Como mover(), pero sin esperar a que llegue: para seguir un objetivo que cambia."""
        a = [max(0.0, min(180.0, float(v))) for v in valores]
        self._cmd("S," + ",".join(f"{v:.1f}" for v in a))
        self.ultimo = a

    def home(self):
        self.mover(HOME + [iman_off()])

    def ir(self, x, y, z, iman, inclinacion=-90.):
        self.mover(ik(x, y, z, inclinacion) + [iman])

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

    def ejecutar_vertical(self, plan, iman, velocidad):
        self.ajustar_velocidad(velocidad, guardar=False)
        for angulos in plan[1:]:
            self.mover(angulos + [iman])

    def recoger_en_origen(self, x, y):
        """Prueba sin traslado: acaba arriba con imán encendido."""
        plan = self.plan_vertical(x, y, Z_SEGURO, config.Z_RECOGIDA)
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
        self.plan_vertical(x, y, Z_SEGURO, config.Z_RECOGIDA)
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
