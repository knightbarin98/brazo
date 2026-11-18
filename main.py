#!/usr/bin/env python3
"""Punto de entrada único del proyecto. Corre con cualquier Python (el del entorno base de conda sirve):
crea o actualiza el entorno 'brazo-gemelo' si hace falta y se ejecuta dentro de él.

  python main.py                 trabajar con el robot: servidor + consola + operador
  python main.py menu            menú con todas las opciones
  python main.py simulador       gemelo del brazo sin robot
  python main.py operador        manejar el robot con la webcam
  python main.py servidor        servidor del robot (Raspberry)
  python main.py consola         calibrar y probar (Raspberry)
  python main.py pruebas         pruebas automáticas
  python main.py estado          qué está instalado y corriendo
  python main.py instalar        servicio automático en la Raspberry
  python main.py desinstalar     deshacer todo para empezar de cero
"""

import sys

from brazo_robot.arranque import main

if __name__ == "__main__":
    main(sys.argv[1:])
