#!/usr/bin/env python3
"""Consola local en la Raspberry, para calibrar y probar: necesita monitor o VNC.

No la uses junto con servidor.py: los dos abren el Arduino y la cámara.
"""

from brazo_robot.cli import main

if __name__ == "__main__":
    main()
