#!/usr/bin/env python3
"""App del operador: su webcam guía el brazo. Corre en otra computadora o en la misma Raspberry.

En la Raspberry (con servidor.py ya en marcha):  python app_operador.py
En otra computadora:                             python app_operador.py --servidor IP-DE-LA-RASPBERRY
"""

import argparse
import getpass
import os
import sys

from brazo_robot.config import ARTICULACIONES, REMOTO_PUERTO
from brazo_robot.remoto.app import ejecutar_app
from brazo_robot.remoto.protocolo import es_esta_maquina, leer_token
from brazo_robot.seguimiento.mapeo import MODO_ARTICULAR, MODO_BRAZO, MODO_MANO, MODOS
from brazo_robot.vision import detectar_webcam, fuente_de_texto


def obtener_token(servidor):
    token = os.environ.get("BRAZO_TOKEN")
    if token:
        return token
    if es_esta_maquina(servidor):         # en la misma Raspberry el token ya está aquí
        token = leer_token()
        if token:
            return token
    return getpass.getpass("Token de la Raspberry: ").strip()


def main():
    p = argparse.ArgumentParser(description="App del operador del brazo")
    p.add_argument("--servidor", default="127.0.0.1",
                   help="IP de la Raspberry; por defecto, esta misma máquina")
    p.add_argument("--puerto", type=int, default=REMOTO_PUERTO, help="puerto de red")
    p.add_argument("--webcam", type=fuente_de_texto, default=None, help="índice o ruta de la webcam (por defecto, la primera libre)")
    p.add_argument("--modo", choices=MODOS, default=MODO_ARTICULAR,
                   help=f"{MODO_ARTICULAR}: cada articulación de tu brazo mueve su motor (por defecto) | "
                        f"{MODO_MANO}: ligero, de cerca | {MODO_BRAZO}: la muñeca guía la punta, en metros")
    p.add_argument("--brazo", choices=tuple(ARTICULACIONES), default="derecho",
                   help="brazo que guía en los modos articular y brazo; el otro se ignora")
    args = p.parse_args()

    token = obtener_token(args.servidor)
    try:
        webcam = detectar_webcam() if args.webcam is None else args.webcam
        print(f"Webcam: {webcam}")
        ejecutar_app(args.servidor, args.puerto, token, webcam, args.modo, args.brazo)
    except RuntimeError as e:
        sys.exit(f"Error: {e}")


if __name__ == "__main__":
    main()
