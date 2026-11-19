#!/usr/bin/env python3
"""Simulador del gemelo: tu brazo derecho en la webcam y el robot virtual al lado. Sin robot.

No abre ningún puerto serial ni conexión: úsalo para comprobar la correspondencia antes de
conectar el hardware. Para el robot real: python main.py servidor (Raspberry) y python main.py operador.

Arráncalo con:  python main.py simulador   (main.py activa solo el entorno)
"""

import argparse
import sys

from brazo_robot.ajustes import cargar_ajustes
from brazo_robot.config import ARTICULACIONES, WEBCAM_LATERAL
from brazo_robot.simulador import ejecutar_simulador
from brazo_robot.seguimiento.lateral import elegir_webcams, fuente_lateral
from brazo_robot.vision import fuente_de_texto


def main(argv=None):
    p = argparse.ArgumentParser(prog="main.py simulador", description="Simulador del gemelo del brazo (sin hardware)")
    p.add_argument("--webcam", type=fuente_de_texto, default=None,
                   help="índice o ruta de la webcam, o un archivo de video (por defecto, la primera webcam)")
    p.add_argument("--webcam-lateral", type=fuente_lateral, default=WEBCAM_LATERAL,
                   help="segunda webcam, a un costado de ti: 'auto' (por defecto: si hay dos, se usan las dos), "
                        "'no' (sólo una) o su índice")
    p.add_argument("--brazo", choices=tuple(ARTICULACIONES), default="derecho", help="brazo que se copia")
    p.add_argument("--registro", metavar="ARCHIVO.csv", help="guarda cada lectura y los servos, para evaluar")
    args = p.parse_args(argv)

    cargar_ajustes()                      # misma geometría y altura de recogida que la consola, si existe ajustes.json
    try:
        webcam, lateral = elegir_webcams(args.webcam, args.webcam_lateral)
        print(f"Webcam: {webcam}" + (f" | lateral: {lateral}" if lateral is not None else " | sin lateral"))
        ejecutar_simulador(webcam, args.brazo, args.registro, lateral)
    except RuntimeError as e:
        sys.exit(f"Error: {e}")


if __name__ == "__main__":
    main()
