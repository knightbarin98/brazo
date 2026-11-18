# Gemelo del brazo derecho

El robot reproduce la orientación y la flexión de tu brazo derecho, visto con una webcam. Usa la
geometría, los servos, los límites y la comunicación que ya tenía el proyecto; el único cambio
físico es el efector: una garra de 12 cm en lugar del electroimán de 9 cm.

## Cómo probarlo

1. **Simulador, sin robot.** No abre el puerto serial ni la red. Usa el montaje de `config.py`:

   ```bash
   python main.py simulador
   ```

   A la izquierda se ve tu webcam con tu brazo detectado; a la derecha, el robot virtual con el mismo
   control que usa el servidor. `ESPACIO` activa y pausa, `C` calibra en T y `A`/`D` `W`/`S` giran la vista.
2. **Robot real**, cuando el simulador se comporte como esperas: `python main.py servidor` en la
   Raspberry y `python main.py operador` (modo articular, el de por defecto). Empieza despacio y con la mano en el apagado.

## Datos del robot que se usan

Salen de `brazo_robot/config.py` y `ajustes.json`; no se suponen otros.

| Dato | Valor |
|---|---|
| Articulaciones | base (giro vertical), hombro, codo y muñeca (los tres en el plano que elige la base), garra |
| Eslabones | L1 = 170 mm (hombro-codo), L2 = 150 mm (codo-muñeca), L3 = 120 mm (muñeca-punto de agarre) |
| Altura del eje del hombro | H = 45 mm |
| Servos | 0..180° cada uno; `MAPEO` pasa de ángulo geométrico a servo (cero y signo) |
| Garra | quinto valor de `S,base,hombro,codo,muneca,garra`, en grados (0..100), como en `seguimiento_brazo.py`. **Falta confirmarlo con el código del Arduino** |
| Montaje | `MONTAJE = "costado"`: atornillado al costado de la caja, como un hombro. La Z del robot (su antigua vertical) sale de la caja y la base gira sobre ese eje horizontal; la "altura" pasa a ser la distancia a la pared. `"de_pie"` es el montaje original sobre la mesa |

## Correspondencia de articulaciones

Tu brazo se mide en 3D **respecto a tu torso**: los ejes salen de los hombros y las caderas (sin
caderas en cuadro, de la vertical de la cámara). Girar el cuerpo frente a la cámara no cambia la lectura.

| Tú | Robot | Cómo |
|---|---|---|
| Hacia dónde llevas el brazo alrededor del hombro: abajo, adelante, arriba, atrás (la profundidad incluida) | Base | gira el plano del brazo alrededor del eje que sale de la caja hasta el que contiene tu brazo (hombro -> muñeca) |
| Cuánto se separa el brazo del costado | Hombro | deja la muñeca del robot en la dirección de tu muñeca, con su cinemática (L1, L2) |
| Flexión del codo, medida en 3D | Codo | la misma flexión |
| Dirección de la mano | Muñeca | la garra apunta como tu mano, proyectada en el plano del robot |
| Abrir y cerrar la mano | Garra | proporcional, con la protección de "alcanzar" antes de seguir |

La traducción está en `brazo_robot/seguimiento/gemelo.py` y la usan igual el servidor y el simulador.
El montaje sólo cambia cómo se pasan tus ejes a los del robot (`a_ejes_robot`): al costado, tu
"afuera" es su Z, tu "abajo" su X (con la base en `ARTICULAR_BASE`) y tu "hacia la cámara" su Y. Con
`ARTICULAR_PROFUNDIDAD = False` la base se queda en su plano de referencia (sin profundidad).

## Calibración

**Del montaje del robot**, en la consola y mirando el robot (se guarda en `ajustes.json`):
`calibrar montaje` fija la base de referencia (el brazo colgando, sin ir hacia la cámara ni hacia
atrás), el sentido de la profundidad y el cero de hombro, codo y muñeca con una postura de escuadra;
`calibrar garra` los ángulos de abierta y cerrada, y `longitud garra <mm>` su largo. En `config.py`
quedan `MONTAJE`, `ARTICULAR_LIMITE_ABAJO` (distancia a la mesa, si la hay), `ARTICULAR_DESFASE` y
`ARTICULAR_LIMITES` (recorrido permitido de cada servo).

**De cada persona** (tecla `C` en el simulador): extiende el brazo hacia afuera, horizontal, con la
mano recta, y quédate quieto 1.5 s. Se corrigen la dirección que la cámara ve de más, la flexión que
aparenta el codo estando recto y la de la mano. Se guarda en `seguimiento.json`, donde corre la cámara.

## Seguridad y estabilidad

- **Simulación y control físico separados**: el simulador usa un robot virtual y no importa `pyserial`;
  el robot real sólo se mueve con `servidor.py`, y hace falta activarlo.
- **Suavizado**: filtro One Euro por articulación; más fuerte en la base, que depende de la profundidad.
- **Límites**: cada articulación se recorta a su tope sin saltar al otro; ninguna postura lleva la
  garra, la muñeca o el codo por debajo de la altura de recogida ni contra la columna de la base.
- **Velocidad**: máximo por articulación, y un acercamiento lento al activar.
- **Detención segura**: sin lectura durante 1 s, con un salto brusco o un error del Arduino, el robot
  se detiene donde está y hay que volver a activarlo. La garra nunca se abre sola.

## Limitaciones

| Limitación | Origen | Qué pasa |
|---|---|---|
| El robot no entra en la caja (servo del hombro 0..180, pared) | **Física** | Con el brazo cruzado por delante del cuerpo, se queda en el borde |
| El codo se dobla hacia un solo lado | **Física** | Si tu codo queda del otro lado de la línea hombro-muñeca, el robot no puede imitarlo y su hombro topa |
| La base gira 180° en total | **Física** | Al costado alcanza para cubrir todo alrededor del eje que sale de la caja (combinada con el hombro) |
| El robot tiene 4 articulaciones de postura; tu brazo, 7 | **Física** | Tu codo fuera del plano del robot y la rotación del antebrazo no tienen equivalente |
| La pared de la caja, la columna de la base y la mesa (`ARTICULAR_LIMITE_ABAJO`) | **Física** | El robot se detiene en el borde y avisa |
| Con el brazo casi sobre el eje de la base, la base no se mueve | Algoritmo | Al costado, con el brazo casi recto hacia afuera (en T), su plano no está definido y el giro sería ruido (`ARTICULAR_UMBRAL_HORIZONTAL`) |
| El mismo plano se alcanza con dos giros de la base | Algoritmo | Se elige el que cabe en los topes, con histéresis; cambiar de uno a otro es un giro grande y lento |
| La profundidad sale de una sola cámara | Algoritmo y cámara | Ver la sección siguiente |

**Al costado** (`tests/test_costado.py`), de 240 posturas hacia afuera del cuerpo (doce direcciones
alrededor del eje que sale de la caja, cuatro separaciones del costado, flexiones de 0° a 80° y el
codo a cada lado), **150 se reproducen exactamente**: la muñeca del robot apunta como la tuya y su
codo dobla igual. De las demás, 60 tienen el brazo a menos de 20° de la T (base retenida, del
algoritmo; el brazo igual sigue, sólo no gira la base) y 30 topan el hombro por el codo del otro lado
(física). Es bastante más que de pie, donde el hombro no baja de la horizontal.

**De pie** (`tests/test_gemelo.py`), se probaron 144 posturas: azimut de -60° (atrás) a 90°
(hacia la cámara), cuatro elevaciones, tres flexiones y el codo a cada lado de la línea hombro-muñeca.
En las 52 que caben en los topes, la muñeca del robot apunta como la tuya y su codo dobla igual:

| Elevación del brazo | Se reproducen | Las demás |
|---|---|---|
| 40° sobre la horizontal | 36 de 36 | — |
| 10° | 14 de 36 | hombro en su tope (física) |
| 20° por debajo | 2 de 36 | hombro o base en su tope (física) |
| 70° (casi vertical) | 0 de 36 | base retenida a propósito (algoritmo) |

Cuando el hombro llega a su tope, el robot **prioriza la dirección de la muñeca** (donde queda la
garra) y dobla el codo lo que haga falta para conseguirla, en vez de copiar una flexión que ya no
alinearía nada. Ejemplo real: en una foto con los brazos en T, Pose veía el codo doblado 13° a 31°
(un sesgo típico) y el robot habría dejado caer el antebrazo; ahora lo deja recto. La calibración
en T corrige además ese sesgo: con la misma foto calibrada, la lectura queda en azimut 0°,
elevación 0° y codo 0°, y el robot en base 90°, hombro 180° y codo 0°.

## Profundidad con una cámara

**Qué se hace.** Pose estima puntos 3D en metros a partir de una sola imagen. La profundidad no se
mide: el modelo la infiere de cómo suele verse un cuerpo.

**Dónde falla, y por qué importa aquí:**

- **Brazo hacia la cámara o hacia atrás.** El brazo se acorta en la imagen y lo único que cambia es la
  profundidad, que es justo lo que mueve la base. La imagen de un brazo apuntando 40° hacia la cámara
  y la de uno apuntando 40° hacia atrás se parecen mucho; el modelo puede confundirlas.
- **Ruido.** La profundidad tiembla más que la posición en la imagen, sobre todo con el brazo de
  frente a la cámara, y tiende a "volver" a posturas típicas.
- **Oclusiones.** Con la mano delante del codo, Pose tiene que adivinar el codo.
- **Escala.** Las medidas en metros son aproximadas, pero aquí sólo se usan direcciones y ángulos,
  que no dependen de la escala.

**Qué hace el programa para mitigarlo:** filtra más la base, no la mueve con el brazo casi vertical y
avisa en pantalla ("profundidad poco fiable") cuando más del 80 % del brazo apunta hacia la cámara.

**Cómo evaluarlo con tu montaje.** No lo pude medir sin una persona frente a una cámara real:

1. Marca en el suelo, con cinta, direcciones a 0°, 45° y 90° desde tu hombro (afuera, diagonal y
   hacia la cámara) y repítelas hacia atrás (-45°).
2. Graba con `python main.py simulador --registro prueba.csv` mientras apuntas a cada marca, 5 s quieto.
3. Para cada marca compara la columna `azimut` con el ángulo real: el **sesgo** es la diferencia de
   la media y la **estabilidad**, la desviación típica.
4. Repite con el brazo a 30° por encima de la horizontal y con otra ropa y otra luz.

Si el sesgo pasa de unos 15° o la desviación típica de unos 8° hacia 45°-90°, una cámara no basta
para que la base copie la profundidad con fidelidad, y conviene la segunda cámara.

## Propuesta: segunda cámara superior (no implementada)

Una cámara cenital, mirando hacia abajo, ve directamente el movimiento hacia la cámara frontal o
hacia atrás, que es lo que una sola cámara confunde.

1. **Montaje**: cámara fija sobre el operador, a 1.5-2 m, que vea hombros, codo y muñeca desde arriba.
   Las dos cámaras en la misma computadora, para compartir el reloj.
2. **Calibración intrínseca** de cada cámara con un tablero de ajedrez (`cv2.calibrateCamera`), para
   corregir la distorsión.
3. **Calibración extrínseca**: un tablero ChArUco visible a la vez para las dos cámaras;
   `cv2.stereoCalibrate` (o `solvePnP` en cada una hacia el tablero) da la rotación y la traslación
   entre ellas. Hay que comprobar el error de reproyección (menos de 1-2 px).
4. **Sincronización**: emparejar los cuadros por marca de tiempo (como mucho ~15 ms de diferencia) o
   usar cámaras con disparo externo.
5. **Triangulación**: Pose 2D en cada imagen; `cv2.triangulatePoints` con las dos matrices de
   proyección para hombros, codo, muñeca y caderas. Se descartan los puntos con poca visibilidad o
   mucho error de reproyección.
6. **Integración**: un `DetectorEstereo` que entregue el mismo `brazo3d` (ejes del torso) que hoy da
   la cámara única. El protocolo, el servidor, el control y el simulador no cambian. Si una cámara
   pierde a la persona, se vuelve a la estimación monocular.
7. **Costo**: dos inferencias de Pose por cuadro. En una Raspberry 4 sería demasiado lento, así que
   conviene correrlo en la computadora del operador.

## Pruebas

**Automáticas** (`python -m pytest`): correspondencia con la cinemática real (dirección de la muñeca
y flexión del codo), profundidad simétrica y con su signo, topes sin saltos, brazo vertical, elección
del giro alcanzable, muñeca y mano, marco del torso invariante al girar el cuerpo, calibración en T,
integración con el control (acercamiento, mesa, pérdida de lectura), protocolo, vista y separación
del simulador.

**Con hardware**, en este orden:

1. En el simulador, comprueba a simple vista: brazo arriba, en T, hacia la cámara, hacia atrás,
   codo doblado y mano inclinada. Calibra en T.
2. Robot real con `ARTICULAR_VELOCIDAD` bajo (por ejemplo 30) y alguien junto al apagado.
3. En la consola, `calibrar montaje`: base de referencia, sentido de la profundidad y ceros.
4. Lleva el brazo despacio hacia la cámara: la base debe girar hacia ese lado. Si no, repite el paso 2
   de `calibrar montaje`.
5. Mide la garra y guárdala con `longitud garra`; `calibrar garra` para abierta y cerrada.
6. Tapa la cámara mientras se mueve: debe detenerse en menos de 1 s.
