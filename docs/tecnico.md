# Detalles técnicos

Para quien programa o mantiene el proyecto. La guía para usarlo está en el [README](../README.md).

Usa **una sola cámara: la webcam que te ve a ti**. Tus movimientos se traducen en movimientos del
brazo. Son dos programas que se hablan por la red: el servidor siempre corre en la Raspberry y la
app del operador puede correr **en otra computadora o en la misma Raspberry**.

```
 Junto al robot                                     Donde esté la webcam
┌───────────────────────────┐   IP local o         ┌──────────────────────────┐
│ Raspberry: servidor.py    │   127.0.0.1          │ app_operador.py          │
│  - control y protecciones │  postura del brazo ->│  - webcam (te ve a ti)   │
│  - Arduino -> servos/garra│                      │  - detección (MediaPipe) │
│  - sin cámara             │  <- lecturas/órdenes │  - dibujo del brazo      │
└───────────────────────────┘                      └──────────────────────────┘
```

No hay video del robot: **quien maneja debe ver el brazo directamente**. La app dibuja la postura
que se le ordena al brazo, calculada con los ángulos de los servos; sirve de guía, no sustituye mirarlo.
Las protecciones se deciden en la Raspberry: si la red o la app fallan, el brazo se detiene solo.

## Empezar: `python main.py`

Todo se arranca desde `main.py`. No hace falta crear ni activar el entorno: con el entorno `base` de
conda activo, como siempre, en la Raspberry con el Arduino conectado:

```bash
python main.py
```

arranca **todo lo necesario para trabajar con el robot**: el servidor, la consola (en esa misma
terminal) y la ventana del operador con la webcam. En la consola puedes calibrar y probar el robot
mientras la app está abierta: antes de cada orden, el seguimiento se pone en pausa. `salir` lo
termina todo; si cierras la ventana del operador, `operador` la vuelve a abrir. Si el servicio
automático ya está corriendo el servidor, sólo abre el operador.

La primera vez crea el entorno **`brazo-gemelo`** (tarda unos minutos); después lo actualiza solo
cuando cambia `environment.yml` y se ejecuta dentro de él. Sólo hace falta tener conda (por ejemplo
Miniforge). Sin Arduino conectado, `python main.py --simular` prueba lo mismo sin mover nada. Cada
parte se puede pedir también por separado:

| Comando | Para qué |
|---|---|
| `python main.py` | Trabajar con el robot: servidor + consola + operador |
| `python main.py menu` | Menú con todas las opciones |
| `python main.py simulador` | Gemelo del brazo con un robot virtual: sin Arduino ni red |
| `python main.py operador` | Manejar el robot con tu webcam (en otra computadora: `--servidor IP`) |
| `python main.py servidor` | Servidor del robot, en la Raspberry |
| `python main.py consola` | Calibrar y probar el robot, en la Raspberry |
| `python main.py pruebas` | Pruebas automáticas |
| `python main.py estado` | Qué está instalado y qué está corriendo |
| `python main.py instalar` | Servicio automático en la Raspberry (auto-deploy) |
| `python main.py desinstalar` | Deshacer todo para empezar de cero |

`python main.py <comando> --help` muestra las opciones de cada uno.

Antes había dos entornos, `brazo` (computadora) y `brazo-pi` (Raspberry). Ahora es uno solo,
`brazo-gemelo`, igual en todas las máquinas. `python main.py estado` avisa si quedan los viejos y
`python main.py desinstalar` los borra.

## Servidor (siempre en la Raspberry)

```bash
python main.py servidor
```

Al arrancar, el brazo va al estado inicial (colgando) con la garra abierta, y se muestran la IP y el token. Si el servicio
automático está instalado, el servidor ya está corriendo: `main.py` lo avisa y no abre otro.

Opciones: `--puerto-serial /dev/ttyACM0`, `--puerto 8765`, `--simular` (sin Arduino: no se mueve
nada) y `--solo-local` (sólo acepta la app en la misma Raspberry).

## App del operador, donde prefieras

**En la misma Raspberry**, con la webcam conectada a ella y un monitor o VNC. No pide dirección ni token:

```bash
python main.py operador
```

**En otra computadora de la misma red.** Pide el token, o lo toma de la variable `BRAZO_TOKEN`:

```bash
python main.py operador --servidor 192.168.1.50
```

Opciones: `--webcam 1` para elegir la webcam, `--webcam-lateral 2` para una segunda a tu costado
(modos articular y brazo) y `--modo` para elegir el seguimiento:

| Modo | Qué necesita | Ventajas y límites |
|---|---|---|
| `--modo articular` (por defecto) | que se vean los dos hombros, el codo, la muñeca y la mano, de frente | El robot es el gemelo de tu brazo, también hacia adelante y atrás. En una Raspberry 4, ~7 lecturas/s |
| `--modo mano` | que se vea la mano, de cerca | Ligero. La respuesta cambia con tu distancia a la cámara |
| `--modo brazo` | que se vean hombro, codo y muñeca | Tu muñeca guía la punta, en metros. Se adapta al largo de tu brazo |

En los modos articular y brazo, `--brazo izquierdo` cambia el brazo que guía (por defecto, el
derecho). Sólo se sigue a una persona y a ese brazo; el otro se ignora.

### Modo articular: el robot es el gemelo de tu brazo derecho

El robot va **atornillado al costado de la caja**, como tu hombro en tu cuerpo: su base gira sobre un
eje horizontal que sale de la caja. Tu brazo se mide **en 3D respecto a tu torso** y se traduce con la
cinemática real del robot:

| Tú | Motor del robot |
|---|---|
| Flexión (brazo hacia adelante) y extensión (hacia atrás) del hombro | Base: gira sobre el eje que sale de la caja, tu eje izquierda-derecha |
| Abducción del hombro (separar el brazo del cuerpo) | Hombro (los dos motores): separa el brazo de la pared igual |
| Flexión del codo, en 3D | Codo: la misma flexión |
| Mano respecto al antebrazo | Muñeca: el mismo ángulo |
| Abrir y cerrar la mano | Garra |

Base y hombro se mueven como mucho **90° desde el estado inicial** (`ARTICULAR_LIMITE_HOMBRO`), dentro
de los topes de los servos; el límite se aplica antes de enviar cada orden. La rotación del brazo
sobre su eje no existe en el robot. `GEMELO_CORRESPONDENCIA = "muneca"` vuelve a la traducción
anterior (base y hombro siguen la dirección hombro -> muñeca).

Así el robot cubre casi todo el espacio hacia afuera de la caja, que es por donde se mueve tu brazo.
Si algún día va de pie sobre la mesa, cambia `MONTAJE = "de_pie"` en `brazo_robot/config.py`.

**Pruébalo primero en el simulador, sin robot:** `python main.py simulador`. Detalles, límites físicos,
evaluación de la profundidad y la segunda cámara: [gemelo.md](gemelo.md).

**Segunda cámara, a tu costado** (opcional): `--webcam-lateral N` en `python main.py`, `operador` o
`simulador`. Tu brazo se mide con las líneas que se ven en la imagen de cada cámara (no con la
profundidad que adivina Pose); cada cámara se orienta sola con tu torso, así que no hay que calibrarla
ni alinearla. Si la lateral no te ve, se sigue con la de frente. Detalles en
[gemelo.md](gemelo.md#cómo-se-mide-tu-brazo-las-líneas-de-la-imagen).

- Con una pulsación de ACTIVAR, el brazo **se acerca despacio** a tu postura desde donde esté y
  después te sigue. No usa el punto de partida ni tiene ganancia (`+`/`-`).
- El panel izquierdo dibuja el robot en 3D **como tu brazo en la webcam**, con la pared de la caja y
  las articulaciones en su tope marcadas en rojo.
- Ningún movimiento lleva la garra, la muñeca o el codo contra la pared de la caja o contra la base:
  el brazo **se detiene en el borde** y la pantalla lo avisa. Si hay una mesa debajo, indica su
  distancia en `ARTICULAR_LIMITE_ABAJO` y tampoco bajará hasta ella.
- Con una sola cámara la profundidad es poco fiable con el brazo apuntando a ella, y la pantalla lo
  avisa. Con la segunda cámara (`--webcam-lateral`) ese aviso desaparece.

**Estado inicial.** El robot empieza y vuelve siempre a la misma postura que tu brazo relajado:
**colgando a plomo junto a la caja, con la garra hacia abajo**. Va ahí al arrancar, con HOME (`H` o
`home`), al terminar cualquier calibración y al salir. La postura sale de `calibrar montaje` (base de
referencia y ceros) y el robot llega a ella por una ruta segura: en pasos de 10° comprobados y, si
el camino directo rozaría la pared, separándose antes de ella. Si no encuentra una ruta segura, no
se mueve y lo avisa.

El montaje se calibra en la consola con `calibrar montaje` (ver la sección Calibrar). De cada
persona, en la app del operador o en el simulador: tecla `C` con el **brazo relajado colgando junto
al cuerpo, recto y quieto** (como el estado inicial). Corrige lo que la cámara ve de más, se guarda
en `seguimiento.json` y al terminar el robot vuelve al estado inicial: al activar, los dos empiezan
en la misma postura y no hay salto.

## Manejar

| Acción | Efecto |
|---|---|
| ACTIVAR o `ESPACIO` (modo articular) | Quédate quieto medio segundo: el brazo se acerca despacio a tu postura y te sigue |
| ACTIVAR o `ESPACIO` (modos mano y brazo) | La primera vez va al punto de partida y espera; la segunda toma la referencia y te sigue |
| PAUSAR o `ESPACIO` | Se detiene; puedes recolocarte sin moverlo |
| Mano a la izquierda / derecha de la pantalla | Y+ / Y- (modos mano y brazo) |
| Mano arriba / abajo | Sube / baja (modos mano y brazo) |
| Mano hacia / lejos de la webcam | Adelante / atrás (modos mano y brazo) |
| Abrir y cerrar la mano | Abre y cierra la garra, en proporción |
| Inclinar la mano arriba / abajo | Inclina la garra (en modos mano y brazo, hasta ±20° de la calibrada) |
| GARRA o `G` | Abre o cierra a mano; luego la garra espera a que tu mano la alcance |
| HOME o `H` | Va al estado inicial (colgando) sin cambiar la garra (sólo en pausa) |
| `C` (modo articular) | Pausa, calibra tu brazo relajado colgando durante 1.5 s y lleva el robot al estado inicial |
| `O` / `P` | Guarda tu mano abierta / cerrada del todo (recomendado la primera vez) |
| `+` / `-` | Más o menos movimiento del brazo por el mismo movimiento tuyo (no en modo articular) |
| SALIR o `ESC` | Cierra la app; el brazo queda en pausa donde está |

Si al activar la garra sostiene una pieza y tu mano está abierta, la garra **no se abre**: no se
mueve hasta que tu mano llegue a su misma apertura. La pantalla lo indica.

## Instalación automática en la Raspberry (auto-deploy)

Con esto, la Raspberry deja el servidor corriendo como servicio, lo arranca al encender y, cada
minuto, baja los commits nuevos de la rama y lo reinicia.

1. Instala Miniforge con tu usuario (sin sudo), si aún no tienes conda:

   ```bash
   curl -L -O https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh
   ```

   ```bash
   bash Miniforge3-Linux-aarch64.sh -b
   ```

2. Clona el repo en `~/brazo` y, desde ahí:

   ```bash
   python main.py instalar
   ```

   Pide la contraseña de sudo. Usa la URL y la rama `main` del repo; para otra:
   `python main.py instalar <URL> <RAMA>`.

Crea el entorno `brazo-gemelo`, da permiso al puerto serial, deja los servicios activos y escribe la
marca **`scripts/.servicio_instalado`**: si existe, el servicio y el auto-deploy están instalados.
`python main.py estado` la lee y comprueba que los servicios corran. La marca no se sube al repo.

Cómo se comporta:

- Cada deploy, cada encendido y la instalación misma **reinician el Arduino: el brazo va al estado inicial (colgando)**.
- Mientras haya un operador conectado, el deploy **espera**: reiniciar movería el brazo y abriría la garra.
- **Si trabajas en la Raspberry**, el deploy no pisa tu trabajo: con cambios sin commit o commits sin
  subir, no actualiza (queda anotado en `~/brazo-deploy.log`) hasta que los subas o los descartes.
- Si cambia `environment.yml`, también actualiza el entorno.
- Las calibraciones (`*.json`) y `token_remoto.txt` no están en el repo y se conservan.
- Para usar la consola, primero `sudo systemctl stop brazo`; `main.py` lo recuerda.

| Para | Ejecuta |
|---|---|
| Ver qué está instalado y corriendo | `python main.py estado` |
| Ver el token de la app | `cat ~/brazo/token_remoto.txt` |
| Ver el servidor en vivo | `journalctl -u brazo -f` |
| Ver el historial de deploys | `tail -f ~/brazo-deploy.log` |
| Detener el brazo | `sudo systemctl stop brazo` |
| Pausar el auto-deploy | `sudo systemctl stop brazo-deploy.timer` |
| Cambiar de rama | `python main.py instalar --reinstalar <URL> <RAMA>` |

### Empezar de cero

```bash
python main.py desinstalar
```

Detiene y borra los servicios, el script de actualización y su configuración, el registro, la marca
y los entornos `brazo-gemelo` y `brazo-pi`. Deshace también lo que dejaba el script original de
auto-deploy (`~/brazo-venv`). Pregunta antes de borrar la carpeta del repo (y entonces guarda un
respaldo `~/brazo-respaldo-FECHA.tar.gz` con calibraciones, token y cambios), la llave SSH y el
entorno `brazo`. Los grupos de usuario sólo se quitan si consta que los agregó el instalador.
`--si` responde que sí a todo. En una Mac o en Windows sólo borra los entornos.

## Calibrar

Se hace en la consola, en la Raspberry con monitor o VNC, mirando el robot. Con `python main.py`
la consola ya está abierta; también sirve `python main.py consola` (con el servicio detenido).
Todo se guarda en `ajustes.json` de la Raspberry, sin editar `config.py`. En este orden:

1. **`calibrar montaje`**, en tres pasos con el robot moviéndose despacio:
   1. El brazo cuelga recto (el estado inicial). Con `A`/`D` giras la base hasta que cuelgue **a
      plomo**, sin ir hacia la cámara ni hacia atrás (una plomada ayuda). `ENTER`.
   2. La base gira un poco: respondes `S` si el brazo se fue hacia la cámara o `N` si fue hacia atrás.
   3. El robot pone el brazo perpendicular a la pared y el antebrazo a 90°. Con una escuadra corriges
      el cero de cada articulación (`1` hombro, `2` codo, `3` muñeca y `A`/`D`). `G` guarda.
   Al guardar o cancelar, el brazo vuelve al estado inicial ya con lo calibrado. `calibrar recogida`
   y `calibrar manual` también terminan ahí.
2. **`calibrar garra`**: mueves la garra con `A`/`D` y guardas su ángulo de abierta (`O`) y de
   cerrada (`C`); el de cerrada debe sujetar sin forzar el servo. El largo se guarda con
   `longitud garra 120` (mide del eje de la muñeca al punto de agarre).
3. **`calibrar recogida x y`**: con `W`/`S` acercas la garra a la superficie donde trabaja (la pared
   de la caja), con `J`/`L` la inclinas, y `G` guarda. Esa distancia es el límite que el robot no cruza.
4. **`calibrar manual`**, las 4 marcas, como en v13. Sin cámara no hace falta `calibrar plano`:
   - Pon 4 marcas en la superficie de trabajo, en las medidas de `PUNTOS_MM` de `config.py`. Se miden
     desde el eje de la base en los ejes del robot: **X** hacia donde apunta el brazo con la base en
     90 e **Y** hacia donde apunta con la base en 180.
   - En cada marca: `N` aproxima, `Z` baja a la altura de recogida, las flechas (o `W`/`A`/`S`/`D`)
     corrigen hasta que la garra quede justo encima, mirándola de lado, y `ENTER` guarda.
   - El robot interpola esas correcciones para todo lo que haga después (`registros` las muestra).

Cambiar el montaje o `longitud garra` invalida los puntos de `calibrar manual`: hay que repetirlos.
Las órdenes de visión de fichas (`ver`, `recoge`, `calibrar plano`, `regiones`, `colores`) sólo
funcionan con una cámara sobre la superficie de trabajo: `python main.py --camara-robot 0`.

Otras órdenes útiles: `garra abrir`, `garra cerrar`, `garra 50`, `ir x y z`, `servo a b c d e`,
`home`, `registros` y `ayuda`.

## Protecciones

El brazo se detiene y queda en pausa, y hay que volver a activarlo, si:

- no llegan lecturas durante 0.6 s (persona fuera de cuadro o red lenta);
- se cierra la app o se corta la conexión;
- la lectura salta de golpe (por ejemplo, otra persona en cuadro);
- el Arduino deja de confirmar 3 órdenes seguidas.

Además, sólo hay un operador a la vez, el brazo no sale de los límites `SEGUIR_LIMITE_*` ni baja de la
Z de recogida, y su velocidad está limitada. Si el objetivo queda fuera de alcance, se desliza por el
borde. Perder la lectura nunca abre la garra. Los parámetros están en `brazo_robot/config.py`
(secciones `REMOTO_` y `SEGUIR_`).

**Debe haber alguien junto al robot con acceso para cortar la alimentación.** La pausa es un
programa, no un paro de emergencia físico, y sin cámara del robot nada detecta obstáculos.

## Red y seguridad

El token impide que otros manejen el brazo sin conocerlo, pero **el video y las órdenes viajan sin
cifrar**. Úsalo en una red de confianza. Para conectarte desde otra red usa una VPN como Tailscale
o WireGuard, y **nunca abras el puerto 8765 en el router**.

## Estructura

```
main.py                   punto de entrada único: menú, entorno automático y cada parte
environment.yml           el entorno brazo-gemelo, igual en todas las máquinas
scripts/
  instalar_autodeploy.sh  instala el servicio y el auto-deploy en la Raspberry (main.py instalar)
  actualizar.sh           lo corre el auto-deploy cada minuto
  desinstalar.sh          deshace todo lo anterior (main.py desinstalar)
  .servicio_instalado     marca: existe si el servicio está instalado (no va al repo)
servidor.py               servidor de la Raspberry (main.py servidor)
simulador.py              gemelo con un robot virtual (main.py simulador)
app_operador.py           app del operador (main.py operador)
docs/gemelo.md            correspondencia, límites, profundidad y segunda cámara
docs/tecnico.md           este documento
modelos/                  modelos de MediaPipe (se descargan solos si faltan)
brazo_robot/
  arranque.py             lo que hace main.py: entorno de conda y comandos (sólo biblioteca estándar)
  config.py               geometría, puertos, red y todos los parámetros
  ajustes.py              ajustes.json, ángulos de la garra y colores guardados
  cinematica.py           cinemática inversa
  robot.py                clase Brazo: serial con el Arduino y rutinas
  ordenes.py              interpretar órdenes, ajustar altura, recoger fichas
  ui.py                   teclado, clics y texto en ventanas de OpenCV
  vista3d.py              dibujo del robot en 3D (simulador y app)
  simulador.py            simulador con robot virtual (no importa pyserial)
  cli.py                  consola local
  trabajar.py             'python main.py': servidor + consola + operador juntos
  calibracion.py          calibrar montaje y calibrar garra (consola)
  vision/                 cámara (la webcam del operador) y visión opcional de la mesa
    camara.py             una cámara, con lectura continua en segundo plano
    geometria.py          transformación cámara-mm, figuras y casillas
    marcadores.py         marcadores magenta/verde del brazo
    base.py               plano, regiones, colores y detección de fichas
    ensenanza.py          calibrar manual/piezas y corrección XY
  seguimiento/
    mano.py               lectura de la persona con MediaPipe (operador)
    mapeo.py              de la lectura al objetivo: filtro, límites, borde, garra, torso y calibración (brazo colgando)
    gemelo.py             tu brazo en 3D -> ángulos de los servos, con la cinemática real
    lateral.py            segunda webcam: lectura y detección en su propio hilo
    vistas.py             tu brazo con las líneas de la imagen de una o dos cámaras
    control.py            estados y protecciones (Raspberry)
  remoto/
    protocolo.py          mensajes, autenticación y validación de datos
    servidor.py           lado Raspberry: control y red
    cliente.py            lado operador: conexión y reconexión
    app.py                ventana del operador
tests/                    python main.py pruebas
```

Los archivos de calibración (`calibracion.json`, `ajustes.json`, `colores.json`, `marcadores.json`,
`ensenanza_v11.json`) van en la carpeta de la Raspberry, junto a `main.py`: los que ya tienes siguen
sirviendo. La apertura de la mano de cada operador se guarda en `seguimiento.json`, donde corre la app.

`Z_RECOGIDA`, `BAJADA_EXTRA_AGARRE` y `L3` cambian mientras corre el programa: léelos siempre como
`config.Z_RECOGIDA`, nunca con `from .config import Z_RECOGIDA`.
