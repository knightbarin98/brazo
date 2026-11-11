#!/usr/bin/env python3
"""Se ejecuta en la Raspberry: controla el brazo y espera a la app del operador. No usa cámara.

conda activate brazo-pi && python servidor.py
"""

import argparse
import sys

from brazo_robot.ajustes import cargar_ajustes
from brazo_robot.config import PUERTO, REMOTO_ESCUCHAR, REMOTO_PUERTO
from brazo_robot.remoto.protocolo import direccion_local, token_del_servidor
from brazo_robot.remoto.servidor import ServidorRobot
from brazo_robot.robot import Brazo


def main():
    p = argparse.ArgumentParser(description="Servidor del brazo (Raspberry)")
    p.add_argument("--puerto-serial", default=PUERTO, help="puerto serial del Arduino")
    p.add_argument("--solo-local", action="store_true", help="sólo acepta la app corriendo en esta misma Raspberry")
    p.add_argument("--puerto", type=int, default=REMOTO_PUERTO, help="puerto de red")
    p.add_argument("--simular", action="store_true", help="sin Arduino: no se mueve nada")
    args = p.parse_args()

    cargar_ajustes()
    print("Modo simulado: sin Arduino." if args.simular else "Iniciando el brazo: va a HOME.")
    brazo = Brazo(args.puerto_serial, simular=args.simular)
    token = token_del_servidor()
    escuchar = "127.0.0.1" if args.solo_local else REMOTO_ESCUCHAR
    servidor = ServidorRobot(brazo, token, escuchar, args.puerto)
    extra = "" if servidor.puerto == REMOTO_PUERTO else f" --puerto {servidor.puerto}"
    print("\nListo. Para manejar el brazo (la webcam de esa máquina debe verte a ti):")
    print(f"  En esta misma Raspberry:  python app_operador.py{extra}")
    if not args.solo_local:
        print(f"  Desde otra computadora:   python app_operador.py --servidor {direccion_local() or 'IP-DE-LA-RASPBERRY'}{extra}")
        if sys.stdout.isatty():
            print(f"Token para la otra computadora (es secreto): {token}")
        else:                             # como servicio: no dejarlo en los registros del sistema
            print("Token para la otra computadora: cat token_remoto.txt")
    print("No hay video del robot: quien maneja debe ver el brazo directamente.")
    print("Ctrl+C detiene el servidor; el brazo se queda donde esté.\n")
    try:
        servidor.ejecutar()
    except KeyboardInterrupt:
        pass
    print("Servidor detenido.")


if __name__ == "__main__":
    main()
