"""Consola de órdenes del brazo."""

import argparse
import traceback

import cv2
import numpy as np

from . import config
from .ajustes import (AJUSTES, cargar_ajustes, cargar_colores, configurar_recogida, garra_abierta, garra_cerrada,
                      guardar_ajustes, numero_en_rango)
from .cinematica import Inalcanzable
from .config import CAMARA_ROBOT, COLORES_ACTIVOS, GARRA_MAX_GRADOS, PUERTO
from .ordenes import ajustar_altura, interpretar, normalizar, recoger
from .robot import Brazo
from .vision import Camara, Vision, fuente_de_texto, listar_camaras

AYUDA = """Comandos:
  ajustes recogida            muestra inclinación, velocidades, pasos y pausas
  ajuste nombre valor         guarda un parámetro, ej: ajuste tiempo_agarre 3
  probar agarre x y           baja, cierra la garra, espera y levanta; queda con la garra cerrada
  calibrar recogida x y       alias del ajuste visual de altura e inclinación
  ajustar altura x y          cámara en vivo: W subir/S bajar; 0/1/2/5 paso; +/- velocidad; G guardar
  altura                     muestra Z de recogida guardada (menor Z = más abajo)
  velocidad [0.5..4]          consulta o guarda velocidad, mayor = más rápido; 2 era la original
  recoge <color y/o figura> [en a|b]   ej: recoge cafe | recoge triangulo en b | recoge azul circulo | recoge todo en a
  ver                         detecta fichas (vista.jpg) y muestra área y proporciones de cada una
  diagnostico                 muestra candidatos y motivos de rechazo; no mueve el brazo
  mascaras                    guarda mascaras.jpg: lo que ve cada color (para afinar)
  alcance                     revisa si las 12 esquinas están dentro del alcance y dibuja el arco
  calibrar plano              4 clics sobre marcas medidas en la mesa (cámara -> mm)
  calibrar manual             enseña la posición REAL de la garra con flechas + cámara en vivo
  calibrar punta              alternativa automática con marcador verde (2 alturas)
  calibrar regiones           12 clics: esquinas de FICHAS, A y B
  calibrar colores [color]    aprende todos o solo uno: calibrar colores cafe
  registros                   muestra las 4 posiciones guardadas de la garra
  probar x y [z]              mueve la garra usando la corrección manual (z opcional)
  comprobar x y [z]           prueba antigua con el marcador verde
  reiniciar                   marca A y B como vacías
  home | ir x y z | servo a b c d e   (e = garra, grados)
  garra abrir | garra cerrar | garra <grados>
  ajuste garra_abierta 100 | ajuste garra_cerrada 20   ángulos al abrir y al agarrar
  salir"""
AYUDA += '''
V13 (marcadores y enseñanza):
  calibrar marcador magenta   clic izq = marcador, clic der = rojizos que NO son; ENTER, G guarda
  calibrar marcador verde     igual para el verde
  marcadores                  vista en vivo: máscara de cada color, candidatos y el elegido
  longitud garra [mm]         eje de muñeca -> punto de agarre; inicial 90 mm
  calibrar manual             enseña las 4 marcas de la mesa (APROXIMAR, Z, ajustar, ENTER)
  calibrar piezas             detecta piezas y enseña el centro de cada una
  registros                   muestras activas e ignoradas (con el motivo)
'''


def leer_argumentos(argv=None):
    p = argparse.ArgumentParser(description="Consola local del brazo: calibración y pruebas en la Raspberry")
    p.add_argument("--puerto", default=PUERTO, help="puerto serial del Arduino")
    p.add_argument("--camara-robot", type=fuente_de_texto, default=CAMARA_ROBOT,
                   help="índice o ruta de la cámara que mira la mesa")
    p.add_argument("--simular", action="store_true", help="trabaja sin Arduino: no se mueve nada")
    p.add_argument("--listar-camaras", action="store_true", help="muestra las cámaras disponibles y termina")
    return p.parse_args(argv)


def procesar_orden(txt, brazo, vision):
    """Ejecuta una orden ya normalizada. Devuelve False si hay que salir."""
    if txt in ("salir", "exit", "q"):
        return False
    elif txt == "ayuda":
        print(AYUDA)
    elif txt == "home":
        brazo.home()
    elif txt.startswith("ajustar altura "):
        valores = txt.split()[2:]
        if len(valores) != 2:
            raise ValueError("Usa: ajustar altura x y (coordenadas reales en mm)")
        ajustar_altura(brazo, vision, *map(float, valores))
    elif txt == "altura":
        print(f"Z de recogida={config.Z_RECOGIDA:.1f} mm; bajada extra={config.BAJADA_EXTRA_AGARRE:.1f} mm")
    elif txt == "velocidad" or txt.startswith("velocidad "):
        partes = txt.split()
        if len(partes) == 1:
            print(f"Velocidad actual={brazo.velocidad:.2f} grados/paso; rango 0.5..4")
        elif len(partes) == 2:
            brazo.ajustar_velocidad(float(partes[1]))
        else:
            raise ValueError("Usa: velocidad 1")
    elif txt == "ajustes recogida":
        configurar_recogida()
    elif txt.startswith("ajuste "):
        partes = txt.split()
        if len(partes) != 3:
            raise ValueError("Usa: ajuste nombre valor")
        configurar_recogida(partes[1], partes[2])
    elif txt.startswith("calibrar recogida "):
        partes = txt.split()[2:]
        if len(partes) != 2:
            raise ValueError("Usa: calibrar recogida x y")
        ajustar_altura(brazo, vision, *map(float, partes))
    elif txt.startswith("probar agarre "):
        partes = txt.split()[2:]
        if len(partes) != 2:
            raise ValueError("Usa: probar agarre x y")
        x, y = map(float, partes)
        if not np.isfinite([x, y]).all():
            raise ValueError("Coordenadas no finitas")
        brazo.probar_agarre(*vision.corregir_xy(x, y))
    elif txt == "calibrar piezas":
        vision.ensenar(brazo, piezas=True)
    elif txt == "marcadores":
        vision.ver_marcadores()
    elif txt.startswith("longitud garra"):
        partes = txt.split()
        if len(partes) == 2:
            print(f"Eje de muñeca a cara de contacto: {config.L3:.1f} mm")
        elif len(partes) == 3:
            valor = numero_en_rango(partes[2], 20, 200, "Longitud")
            anterior = AJUSTES.get("longitud_iman", config.L3)
            AJUSTES["longitud_iman"] = valor
            try:
                guardar_ajustes()
            except OSError:
                AJUSTES["longitud_iman"] = anterior
                raise
            config.L3 = valor
            print("Longitud guardada. Repite enseñanza: las muestras con otra geometría no se usan.")
        else:
            raise ValueError("Usa: longitud garra 90")
    elif txt.startswith("calibrar marcador"):
        partes = txt.split()
        if len(partes) != 3:
            raise ValueError("Usa: calibrar marcador magenta | calibrar marcador verde")
        vision.calibrar_marcador(partes[2])
    elif txt.startswith("calibrar"):
        if "punta" in txt:
            vision.calibrar_punta(brazo)
        elif "region" in txt:
            vision.calibrar_regiones()
        elif "color" in txt:
            elegidos = interpretar(txt)[0]
            vision.calibrar_colores(elegidos[0] if elegidos else None)
        elif "plano" in txt:
            vision.calibrar_plano()
        elif "manual" in txt:
            vision.calibrar_manual(brazo)
        else:
            print("Usa: calibrar plano | manual | punta | regiones | colores")
    elif txt == "registros":
        vision.mostrar_registros_brazo()
    elif txt.startswith("probar "):
        v = list(map(float, txt.split()[1:4]))
        if len(v) not in (2, 3):
            print("Usa: probar x y [z]")
        else:
            vision.probar_correccion_manual(brazo, *v)
    elif txt == "diagnostico":
        vision.diagnostico()
    elif txt == "ver":
        for o in vision.detectar():
            print(f"  {o['color']:9s} {o['forma']:10s} x={o['x']:6.1f} y={o['y']:6.1f} mm | "
                  f"area={o['area']:5.0f} mm2  r_circ={o['rc']:.2f}  r_rect={o['rr']:.2f}")
    elif txt == "mascaras":
        vision.mascaras()
        print("Guardado en mascaras.jpg")
    elif txt == "alcance":
        vision.revisar_alcance()
        vision._mostrar(vision.vista())
    elif txt.startswith("comprobar"):
        v = list(map(float, txt.split()[1:4]))
        vision.comprobar(brazo, *v)
    elif txt == "reiniciar":
        vision.ocupados = {"A": 0, "B": 0}
        print("Regiones A y B marcadas como vacías.")
    elif txt.startswith("ir "):
        x, y, z = map(float, txt.split()[1:4])
        brazo.ir(x, y, z, brazo.ultimo[4])
    elif txt.startswith("servo "):
        v = list(map(float, txt.split()[1:6]))
        if len(v) != 5:
            print("Son 5 valores (el último es la garra, en grados).")
        else:
            brazo.mover(v)
    elif txt.startswith("garra"):
        partes = txt.split()
        if len(partes) != 2:
            raise ValueError("Usa: garra abrir | garra cerrar | garra 50")
        if partes[1] == "abrir":
            angulo = garra_abierta()
        elif partes[1] == "cerrar":
            angulo = garra_cerrada()
        else:
            angulo = numero_en_rango(partes[1], 0, GARRA_MAX_GRADOS, "Garra")
        brazo.mover(brazo.ultimo[:4] + [angulo])
    elif txt.startswith(("recoge", "toma", "agarra")):
        recoger(brazo, vision, txt)
    else:
        print("No entendí. Escribe 'ayuda'.")
    return True


def main(argv=None):
    args = leer_argumentos(argv)
    if args.listar_camaras:
        listar_camaras()
        return
    cargar_ajustes()
    cargar_colores()
    brazo = Brazo(args.puerto, simular=args.simular)
    vision = Vision(Camara(args.camara_robot))
    print(AYUDA)
    print("Colores en uso:", ", ".join(COLORES_ACTIVOS))
    print(f"Altura de recogida: Z={config.Z_RECOGIDA:.1f} mm (bajada extra {config.BAJADA_EXTRA_AGARRE:.0f} mm)")

    while True:
        try:
            txt = normalizar(input("\n> ").strip())
        except (EOFError, KeyboardInterrupt):
            break
        if not txt:
            continue
        try:
            if not procesar_orden(txt, brazo, vision):
                break
        except Inalcanzable as e:
            print("Fuera de alcance:", e)
        except (ValueError, RuntimeError, TypeError, OSError, KeyError) as e:
            print("Error:", e)
        except cv2.error as e:
            print("Error de OpenCV (¿sin entorno gráfico o ventana cerrada?):", str(e).strip().splitlines()[-1])
        except Exception:
            # Antes cualquier error inesperado cerraba el programa a mitad de una rutina.
            traceback.print_exc()
            print("Error inesperado; el programa sigue. Revisa la traza de arriba.")

    brazo.home(brazo.ultimo[4])        # sin soltar lo que tenga agarrado
    cv2.destroyAllWindows()
