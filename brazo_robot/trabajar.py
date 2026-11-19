"""'python main.py' sin comando: servidor, consola y operador juntos, para trabajar con el robot.

Sólo un programa puede abrir el puerto del Arduino, así que la consola corre dentro del servidor:
lee las órdenes en su propio hilo y el hilo del servidor las ejecuta (con el seguimiento en pausa).
El operador es otro proceso, con su ventana, conectado a esta misma máquina.
"""

import argparse
import os
import subprocess
import sys
import threading

import cv2
import serial

from .ajustes import cargar_ajustes, cargar_colores
from .arranque import RAIZ
from .calibracion import volver_al_inicio
from .cli import procesar_con_errores
from .config import CAMARA_ROBOT, PUERTO, REMOTO_ESCUCHAR, REMOTO_PUERTO
from .ordenes import normalizar
from .remoto.protocolo import token_del_servidor
from .remoto.servidor import ServidorRobot
from .robot import Brazo
from .vision import Camara, Vision, fuente_de_texto

RUIDO_MEDIAPIPE = ("W0000", "I0000", "INFO:", "WARNING: All log messages", "inference_feedback", "gl_context",
                   "GL version", "landmark_projection", "NORM_RECT",
                   "out device of bound", "camera failed to properly initialize")   # buscando webcams
AYUDA_TRABAJO = """Trabajando con el robot: servidor + consola + operador.
  Escribe órdenes aquí (ayuda: lista completa). Antes de cada una, el seguimiento se pone en pausa.
  calibrar montaje | calibrar garra | calibrar recogida x y | calibrar manual   calibrar el robot
  operador     vuelve a abrir la ventana del operador si la cerraste
  salir        termina todo (el brazo va al estado inicial sin soltar lo que tenga) | Ctrl+C: termina sin moverlo"""


class Operador:
    """La app del operador en su propio proceso, con su salida en esta consola."""

    def __init__(self, puerto, token, extra):
        self.puerto, self.token, self.extra = puerto, token, list(extra)
        self.proceso = None

    def abrir(self):
        if self.proceso is not None and self.proceso.poll() is None:
            print("La ventana del operador ya está abierta.")
            return
        orden = [sys.executable, os.path.join(RAIZ, "main.py"), "operador",
                 "--servidor", "127.0.0.1", "--puerto", str(self.puerto)] + self.extra
        entorno = dict(os.environ, BRAZO_TOKEN=self.token, PYTHONUNBUFFERED="1")   # sus mensajes, al momento
        self.proceso = subprocess.Popen(orden, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, env=entorno, text=True, bufsize=1)
        threading.Thread(target=self._reenviar, args=(self.proceso,), daemon=True).start()

    @staticmethod
    def _reenviar(proceso):
        for linea in proceso.stdout:
            if linea.strip() and not any(r in linea for r in RUIDO_MEDIAPIPE):
                print(f"[operador] {linea}", end="")
        proceso.wait()
        print("[operador] ventana cerrada. Escribe 'operador' para abrirla otra vez.")

    def cerrar(self):
        if self.proceso is not None and self.proceso.poll() is None:
            self.proceso.terminate()
            try:
                self.proceso.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proceso.kill()


def _consola(servidor, brazo, vision, operador):
    """Hilo de la consola: lee una orden, la encarga al hilo del servidor y espera a que termine."""
    while not servidor.detenido:
        try:
            linea = input("\nconsola> ")
        except (EOFError, KeyboardInterrupt):
            servidor.pedir_parada()
            return
        txt = normalizar(linea.strip())
        if not txt:
            continue
        if txt == "operador":
            operador.abrir()
            continue
        resultado, hecho = {}, threading.Event()

        def tarea():
            try:
                resultado["seguir"] = procesar_con_errores(txt, brazo, vision)
            finally:
                hecho.set()

        servidor.en_hilo_principal(tarea)
        hecho.wait()
        if not resultado.get("seguir", True):
            servidor.en_hilo_principal(lambda: volver_al_inicio(brazo))   # como 'salir' de la consola
            servidor.en_hilo_principal(servidor.pedir_parada)
            return


def leer_argumentos(argv):
    p = argparse.ArgumentParser(prog="main.py", description="Trabajar con el robot: servidor, consola y operador juntos")
    p.add_argument("--puerto-serial", default=PUERTO, help="puerto serial del Arduino")
    p.add_argument("--simular", action="store_true", help="sin Arduino: no se mueve nada")
    p.add_argument("--camara-robot", type=fuente_de_texto, default=CAMARA_ROBOT,
                   help="cámara opcional sobre la superficie de trabajo (ver y recoger fichas)")
    p.add_argument("--puerto", type=int, default=REMOTO_PUERTO, help="puerto de red del servidor")
    p.add_argument("--sin-operador", action="store_true", help="no abre la ventana del operador")
    p.add_argument("--webcam", help="webcam del operador (índice o ruta)")
    p.add_argument("--webcam-lateral", help="segunda webcam del operador, a un costado: por defecto 'auto' "
                                            "(si hay dos, se usan las dos), 'no' o su índice")
    p.add_argument("--modo", help="modo del operador (articular, mano o brazo)")
    return p.parse_args(argv)


def ejecutar_trabajo(argv=None):
    args = leer_argumentos(argv)
    cargar_ajustes()
    cargar_colores()
    if not args.simular and os.name != "nt" and not os.path.exists(args.puerto_serial):
        sys.exit(f"No encuentro el Arduino en {args.puerto_serial}.\n"
                 "  ¿Está conectado? En la Raspberry suele ser /dev/ttyACM0 (o --puerto-serial /dev/ttyUSB0).\n"
                 "  Sin Arduino:            python main.py --simular\n"
                 "  Desde otra computadora: python main.py operador --servidor IP-DE-LA-RASPBERRY\n"
                 "  Simulador:              python main.py simulador | Menú: python main.py menu")
    vision = Vision(None if args.camara_robot is None else Camara(args.camara_robot))
    print("Modo simulado: sin Arduino." if args.simular else "Iniciando el brazo: va al estado inicial (colgando).")
    try:
        brazo = Brazo(args.puerto_serial, simular=args.simular)
    except serial.SerialException as e:
        sys.exit(f"No se pudo abrir el Arduino en {args.puerto_serial}: {e}\n"
                 "  ¿Lo usa otro programa? ¿Tu usuario está en el grupo dialout?")
    token = token_del_servidor()
    servidor = ServidorRobot(brazo, token, REMOTO_ESCUCHAR, args.puerto)
    extra = ((["--webcam", args.webcam] if args.webcam else [])
             + (["--webcam-lateral", args.webcam_lateral] if args.webcam_lateral else [])
             + (["--modo", args.modo] if args.modo else []))
    operador = Operador(servidor.puerto, token, extra)
    print(AYUDA_TRABAJO)
    if not args.sin_operador:
        operador.abrir()
    threading.Thread(target=_consola, args=(servidor, brazo, vision, operador), daemon=True).start()
    try:
        servidor.ejecutar()
    except KeyboardInterrupt:
        print("\nTerminando: el brazo se queda donde está.")
    finally:
        operador.cerrar()
        cv2.destroyAllWindows()
