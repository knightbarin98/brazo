#!/usr/bin/env python3
"""Se ejecuta en la computadora del operador: su webcam guía el brazo por la red.

conda activate brazo && python app_remota.py --servidor IP-DE-LA-RASPBERRY
"""

import argparse
import getpass
import os
import sys

from brazo_robot.config import REMOTO_PUERTO
from brazo_robot.remoto.app import ejecutar_app
from brazo_robot.vision import detectar_webcam, fuente_de_texto


def main():
    p = argparse.ArgumentParser(description="App del operador del brazo remoto")
    p.add_argument("--servidor", required=True, help="IP o nombre de la Raspberry")
    p.add_argument("--puerto", type=int, default=REMOTO_PUERTO, help="puerto de red")
    p.add_argument("--webcam", type=fuente_de_texto, default=None, help="índice de la webcam (por defecto, la primera)")
    args = p.parse_args()

    token = os.environ.get("BRAZO_TOKEN") or getpass.getpass("Token de la Raspberry: ").strip()
    try:
        webcam = detectar_webcam() if args.webcam is None else args.webcam
        print(f"Webcam: {webcam}")
        ejecutar_app(args.servidor, args.puerto, token, webcam)
    except RuntimeError as e:
        sys.exit(f"Error: {e}")


if __name__ == "__main__":
    main()
