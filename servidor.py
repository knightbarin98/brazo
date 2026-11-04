#!/usr/bin/env python3
"""Se ejecuta en la Raspberry: controla el brazo y espera a la app del operador.

conda activate brazo-pi && python servidor.py
"""

import argparse
import socket

from brazo_robot.ajustes import cargar_ajustes, cargar_colores
from brazo_robot.config import CAMARA_ROBOT, PUERTO, REMOTO_ESCUCHAR, REMOTO_PUERTO
from brazo_robot.remoto.protocolo import token_del_servidor
from brazo_robot.remoto.servidor import ServidorRobot
from brazo_robot.robot import Brazo
from brazo_robot.vision import Camara, Vision, fuente_de_texto


def direccion_local():
    """La IP de esta máquina en la red. No se envía ningún paquete."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
        except OSError:
            return "IP-DE-LA-RASPBERRY"


def main():
    p = argparse.ArgumentParser(description="Servidor del brazo (Raspberry)")
    p.add_argument("--puerto-serial", default=PUERTO, help="puerto serial del Arduino")
    p.add_argument("--camara", type=fuente_de_texto, default=CAMARA_ROBOT, help="índice o ruta de la cámara del robot")
    p.add_argument("--escuchar", default=REMOTO_ESCUCHAR, help="dirección donde escuchar")
    p.add_argument("--puerto", type=int, default=REMOTO_PUERTO, help="puerto de red")
    p.add_argument("--simular", action="store_true", help="sin Arduino: no se mueve nada")
    args = p.parse_args()

    cargar_ajustes()
    cargar_colores()
    camara = Camara(args.camara)          # primero la cámara: si falla, el brazo no llega a moverse
    vision = Vision(camara)
    print("Modo simulado: sin Arduino." if args.simular else "Iniciando el brazo: va a HOME.")
    brazo = Brazo(args.puerto_serial, simular=args.simular)
    token = token_del_servidor()
    servidor = ServidorRobot(brazo, vision, token, args.escuchar, args.puerto)
    print("\nListo. En la computadora del operador ejecuta:")
    print(f"  python app_remota.py --servidor {direccion_local()} --puerto {servidor.puerto}")
    print(f"Token (te lo pedirá la app; es secreto): {token}")
    print("Ctrl+C detiene el servidor; el brazo se queda donde esté.\n")
    try:
        servidor.ejecutar()
    except KeyboardInterrupt:
        pass
    print("Servidor detenido.")


if __name__ == "__main__":
    main()
