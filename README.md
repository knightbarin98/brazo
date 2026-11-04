# Brazo robótico remoto

```
 Junto al robot                                     Computadora del operador
┌───────────────────────────┐        red           ┌──────────────────────────┐
│ Raspberry: servidor.py    │  video de la mesa -> │ app_remota.py            │
│  - cámara del robot       │                      │  - webcam del operador   │
│  - control y protecciones │  <- mano y órdenes   │  - detección de la mano  │
│  - Arduino -> servos/imán │                      │  - video y botones       │
└───────────────────────────┘                      └──────────────────────────┘
```

La mano se detecta en la computadora del operador; el movimiento y las protecciones se
deciden en la Raspberry. Así, si la red o la app fallan, el brazo se detiene solo.

## En la Raspberry

Requiere Raspberry Pi OS de 64 bits y conda (por ejemplo, Miniforge). Copia esta carpeta y ejecuta:

```bash
conda env create -f environment-raspberry.yml
```

```bash
conda activate brazo-pi
```

```bash
python servidor.py
```

Al arrancar, el brazo va a HOME y se muestran la IP y el token que necesita el operador. El token
se crea una vez y se guarda en `token_remoto.txt`; para cambiarlo, borra ese archivo.

Opciones: `--camara 0` (índice o ruta), `--puerto-serial /dev/ttyACM0`, `--puerto 8765` y
`--simular` (sin Arduino: no se mueve nada).

### Calibrar

La calibración sigue siendo local, porque abre ventanas donde se hace clic: se necesita un monitor
o VNC en la Raspberry. Detén antes `servidor.py`, ya que los dos programas usan el Arduino y la cámara.

```bash
python main.py
```

Calibra al menos `calibrar recogida x y`: esa Z es el piso que el seguimiento nunca cruza.

## En la computadora del operador

Copia esta carpeta y ejecuta:

```bash
conda env create -f environment.yml
```

```bash
conda activate brazo
```

```bash
python app_remota.py --servidor IP-DE-LA-RASPBERRY
```

La app pide el token (o lo toma de la variable `BRAZO_TOKEN`) y usa la primera webcam que encuentra.
Si tienes varias, elige con `--webcam 1`. En macOS, permite el acceso a la cámara a tu terminal.

| Acción | Efecto |
|---|---|
| Botón ACTIVAR o `ESPACIO` (primera vez) | El brazo va al punto de partida y espera |
| ACTIVAR o `ESPACIO` (otra vez) | Empieza a seguir tu mano |
| Botón PAUSAR o `ESPACIO` | Se detiene; puedes recolocar la mano sin moverlo |
| Mano a la izquierda / derecha de la pantalla | Y+ / Y- |
| Mano arriba / abajo | Sube / baja |
| Acercar / alejar la mano de la webcam | Adelante / atrás |
| Juntar pulgar e índice (pinza), botón IMAN o `I` | Enciende / apaga el imán |
| Botón HOME o `H` | Va a HOME (sólo en pausa y con el imán apagado) |
| Botón SALIR o `ESC` | Cierra la app; el brazo queda en pausa |

## Protecciones

El brazo se detiene y queda en pausa, y hay que volver a activarlo, si:

- no llegan datos de la mano durante 0.6 s (mano fuera de cuadro o red lenta);
- se cierra la app o se corta la conexión de control o de video;
- deja de llegar video durante 1.5 s: no se maneja a ciegas;
- la mano salta de golpe (por ejemplo, otra persona en cuadro);
- se pierde la cámara del robot o falla el Arduino.

Además, sólo hay un operador a la vez, el brazo no sale de los límites `SEGUIR_LIMITE_*` ni baja de
la Z de recogida, y su velocidad está limitada. El imán nunca cambia solo: no suelta la pieza si
se cae la red. Los parámetros están en `brazo_robot/config.py` (secciones `REMOTO_` y `SEGUIR_`).

**Debe haber alguien junto al robot con acceso para cortar la alimentación.** La pausa es un
programa, no un paro de emergencia físico, y la cámara no muestra todo lo que rodea al brazo.

## Red y seguridad

El token impide que otros manejen el brazo sin conocerlo, pero **el video y las órdenes viajan sin
cifrar**. Úsalo en una red de confianza. Para conectarte desde otra red usa una VPN como Tailscale
o WireGuard, y **nunca abras el puerto 8765 en el router**.

## Estructura

```
servidor.py               programa de la Raspberry
app_remota.py             app del operador
main.py                   consola local de la Raspberry: calibrar y probar
environment.yml           entorno del operador (y de desarrollo)
environment-raspberry.yml entorno de la Raspberry
modelos/                  modelo de manos (sólo operador; se descarga solo si falta)
brazo_robot/
  config.py               geometría, puertos, red y todos los parámetros
  ajustes.py              ajustes.json, electroimán y colores guardados
  cinematica.py           cinemática inversa
  robot.py                clase Brazo: serial con el Arduino y rutinas
  ordenes.py              interpretar órdenes, ajustar altura, recoger fichas
  ui.py                   teclado, clics y texto en ventanas de OpenCV
  cli.py                  consola local
  vision/
    camara.py             una cámara, con lectura continua en segundo plano
    geometria.py          transformación cámara-mm, figuras y casillas
    marcadores.py         marcadores magenta/verde del brazo
    base.py               plano, regiones, colores y detección de fichas
    ensenanza.py          calibrar manual/piezas y corrección XY
  seguimiento/
    mano.py               detección de la mano con MediaPipe (operador)
    mapeo.py              de la mano al punto objetivo: filtro, límites y pinza
    control.py            estados y protecciones (Raspberry)
  remoto/
    protocolo.py          mensajes, autenticación y validación de datos
    servidor.py           lado Raspberry: control, video y red
    cliente.py            lado operador: conexión y reconexión
    app.py                ventana del operador
tests/                    python -m pytest
```

Los archivos de calibración (`calibracion.json`, `ajustes.json`, `colores.json`, `marcadores.json`,
`ensenanza_v11.json`) van en la carpeta de la Raspberry, junto a `servidor.py`: los que ya tienes siguen sirviendo.

`Z_RECOGIDA`, `BAJADA_EXTRA_AGARRE` y `L3` cambian mientras corre el programa: léelos siempre como
`config.Z_RECOGIDA`, nunca con `from .config import Z_RECOGIDA`.
