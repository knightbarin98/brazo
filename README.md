# Brazo robótico con garra, manejado con la webcam

Son dos programas que se hablan por la red. El servidor siempre corre en la Raspberry; la app del
operador puede correr **en otra computadora o en la misma Raspberry**, sin cambiar nada más.

```
 Junto al robot                                     Donde esté el operador
┌───────────────────────────┐   IP local o         ┌──────────────────────────┐
│ Raspberry: servidor.py    │   127.0.0.1          │ app_operador.py          │
│  - cámara del robot       │  video de la mesa -> │  - webcam del operador   │
│  - control y protecciones │                      │  - detección (MediaPipe) │
│  - Arduino -> servos/garra│  <- lecturas/órdenes │  - video y botones       │
└───────────────────────────┘                      └──────────────────────────┘
```

Las protecciones se deciden en la Raspberry: si la red o la app fallan, el brazo se detiene solo.

## 1. Instalar

En la Raspberry (Raspberry Pi OS de 64 bits y conda, por ejemplo Miniforge):

```bash
conda env create -f environment-raspberry.yml
```

```bash
conda activate brazo-pi
```

En otra computadora, si la vas a usar:

```bash
conda env create -f environment.yml
```

```bash
conda activate brazo
```

## 2. Arrancar el servidor (siempre en la Raspberry)

```bash
python servidor.py
```

Al arrancar, el brazo va a HOME con la garra abierta, y se muestran la IP y el token.

Opciones: `--camara 0` (índice o ruta), `--puerto-serial /dev/ttyACM0`, `--puerto 8765`,
`--simular` (sin Arduino: no se mueve nada), `--solo-local` (sólo acepta la app en la misma Raspberry)
y `--mjpg` (menos ancho de banda USB; después revisa que la calibración siga bien).

## 3. Arrancar la app, donde prefieras

**En la misma Raspberry** (necesita monitor o VNC, y una segunda cámara como webcam). No pide
dirección ni token. Arranca primero el servidor para que la app no tome la cámara del robot:

```bash
python app_operador.py
```

**En otra computadora de la misma red.** Pide el token, o lo toma de la variable `BRAZO_TOKEN`:

```bash
python app_operador.py --servidor 192.168.1.50
```

Opciones: `--webcam 1` para elegir la webcam, y `--modo` para elegir el seguimiento:

| Modo | Qué necesita | Ventajas y límites |
|---|---|---|
| `--modo mano` (por defecto) | que se vea la mano, de cerca | Ligero. La respuesta cambia con tu distancia a la cámara |
| `--modo brazo` | que se vean hombro, codo y muñeca | Mide en metros y se adapta al largo de tu brazo. Más pesado; en una Raspberry puede ir lento |

En modo brazo, `--brazo izquierdo` cambia el brazo que guía (por defecto, el derecho).

## 4. Manejar

| Acción | Efecto |
|---|---|
| ACTIVAR o `ESPACIO` (primera vez) | El brazo va al punto de partida y espera |
| ACTIVAR o `ESPACIO` (otra vez) | Quédate quieto medio segundo: toma la referencia y empieza a seguirte |
| PAUSAR o `ESPACIO` | Se detiene; puedes recolocarte sin moverlo |
| Mano a la izquierda / derecha de la pantalla | Y+ / Y- |
| Mano arriba / abajo | Sube / baja |
| Mano hacia / lejos de la webcam | Adelante / atrás |
| Abrir y cerrar la mano | Abre y cierra la garra, en proporción |
| Inclinar la mano arriba / abajo | Inclina la garra, hasta ±20° de la inclinación calibrada |
| GARRA o `G` | Abre o cierra a mano; luego la garra espera a que tu mano la alcance |
| HOME o `H` | Va a HOME sin cambiar la garra (sólo en pausa) |
| `O` / `P` | Guarda tu mano abierta / cerrada del todo (recomendado la primera vez) |
| `+` / `-` | Más o menos movimiento del brazo por el mismo movimiento tuyo |
| SALIR o `ESC` | Cierra la app; el brazo queda en pausa donde está |

Si al activar la garra sostiene una pieza y tu mano está abierta, la garra **no se abre**: no se
mueve hasta que tu mano llegue a su misma apertura. La pantalla lo indica.

## 5. Calibrar

La calibración se hace en la consola local de la Raspberry, con monitor o VNC. Detén antes
`servidor.py`, porque los dos programas usan el Arduino y la cámara:

```bash
python main.py
```

Para la garra, además de lo ya conocido:

- `longitud garra 90`: distancia del eje de la muñeca al **punto de agarre**, en mm. Mídela de nuevo.
- `ajuste garra_abierta 100` y `ajuste garra_cerrada 20`: ángulos del servo. Ajusta el de cerrada para
  sujetar la pieza sin forzar el servo. Si tu garra abre al revés, intercambia los dos valores.
- `garra abrir`, `garra cerrar` y `garra 50`: prueba los ángulos.
- `calibrar recogida x y`: esa Z es el piso que el seguimiento nunca cruza.

Cambiar `longitud garra` invalida los puntos enseñados con `calibrar manual`: hay que repetirlos.

## Protecciones

El brazo se detiene y queda en pausa, y hay que volver a activarlo, si:

- no llegan lecturas durante 0.6 s (persona fuera de cuadro o red lenta);
- se cierra la app o se corta la conexión de control o de video;
- deja de llegar video durante 1.5 s: no se maneja a ciegas;
- la lectura salta de golpe (por ejemplo, otra persona en cuadro);
- se pierde la cámara del robot o el Arduino deja de confirmar 3 órdenes seguidas.

Además, sólo hay un operador a la vez, el brazo no sale de los límites `SEGUIR_LIMITE_*` ni baja de la
Z de recogida, y su velocidad está limitada. Si el objetivo queda fuera de alcance, se desliza por el
borde. Perder la lectura nunca abre la garra. Los parámetros están en `brazo_robot/config.py`
(secciones `REMOTO_` y `SEGUIR_`).

**Debe haber alguien junto al robot con acceso para cortar la alimentación.** La pausa es un
programa, no un paro de emergencia físico, y la cámara no muestra todo lo que rodea al brazo.

## Red y seguridad

El token impide que otros manejen el brazo sin conocerlo, pero **el video y las órdenes viajan sin
cifrar**. Úsalo en una red de confianza. Para conectarte desde otra red usa una VPN como Tailscale
o WireGuard, y **nunca abras el puerto 8765 en el router**.

## Estructura

```
servidor.py               programa de la Raspberry
app_operador.py           app del operador: en otra computadora o en la misma Raspberry
main.py                   consola local de la Raspberry: calibrar y probar
environment.yml           entorno de otra computadora (y de desarrollo)
environment-raspberry.yml entorno de la Raspberry
modelos/                  modelos de MediaPipe (se descargan solos si faltan)
brazo_robot/
  config.py               geometría, puertos, red y todos los parámetros
  ajustes.py              ajustes.json, ángulos de la garra y colores guardados
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
    mano.py               lectura de la persona con MediaPipe (operador)
    mapeo.py              de la lectura al objetivo: filtro, límites, borde y garra
    control.py            estados y protecciones (Raspberry)
  remoto/
    protocolo.py          mensajes, autenticación y validación de datos
    servidor.py           lado Raspberry: control, video y red
    cliente.py            lado operador: conexión y reconexión
    app.py                ventana del operador
tests/                    python -m pytest
```

Los archivos de calibración (`calibracion.json`, `ajustes.json`, `colores.json`, `marcadores.json`,
`ensenanza_v11.json`) van en la carpeta de la Raspberry, junto a `servidor.py`: los que ya tienes siguen
sirviendo. La apertura de la mano de cada operador se guarda en `seguimiento.json`, donde corre la app.

`Z_RECOGIDA`, `BAJADA_EXTRA_AGARRE` y `L3` cambian mientras corre el programa: léelos siempre como
`config.Z_RECOGIDA`, nunca con `from .config import Z_RECOGIDA`.
