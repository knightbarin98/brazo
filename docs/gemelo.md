# Gemelo del brazo derecho

El robot reproduce el movimiento de tu brazo derecho (hombro, codo y muñeca), visto con una webcam
de frente y, si la hay, una segunda a tu costado. Usa la
geometría, los servos, los límites y la comunicación que ya tenía el proyecto; el único cambio
físico es el efector: una garra de 12 cm en lugar del electroimán de 9 cm.

## Cómo probarlo

1. **Simulador, sin robot.** No abre el puerto serial ni la red. Usa el montaje de `config.py`:

   ```bash
   python main.py simulador
   ```

   A la izquierda se ve tu webcam con tu brazo detectado; a la derecha, el robot virtual con el mismo
   control que usa el servidor. Con la segunda cámara: `python main.py simulador --webcam-lateral 1`. `ESPACIO` activa y pausa, `C` calibra con el brazo relajado colgando (y el robot vuelve al estado inicial) y `A`/`D` `W`/`S` giran la vista.
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

Con `GEMELO_CORRESPONDENCIA = "articulaciones"` (por defecto) **cada motor copia su articulación**,
partiendo del estado inicial (tu brazo y el robot colgando):

| Tú | Robot | Cómo |
|---|---|---|
| **Flexión** (brazo hacia adelante) y **extensión** (hacia atrás) del hombro: giro alrededor de tu eje izquierda-derecha | Base | al costado, la base gira justo sobre ese eje (el que sale de la caja): copia el ángulo de tu brazo (hombro -> codo) hacia adelante o hacia atrás |
| **Abducción** del hombro: separar el brazo del cuerpo | Hombro | separa el brazo de la pared lo mismo que el tuyo se separa del cuerpo |
| Flexión del codo, medida en 3D | Codo | la misma flexión |
| Mano respecto al antebrazo | Muñeca | el mismo ángulo, medido sobre el eje en que giran hombro, codo y muñeca del robot |
| Abrir y cerrar la mano | Garra | proporcional, con la protección de "alcanzar" antes de seguir |

**Grados de libertad reales.** El robot tiene 2 en el hombro (base y hombro), 1 en el codo y 1 en la
muñeca, todos de bisagra. Tu hombro tiene 3: la rotación del brazo sobre su propio eje (rotación
interna/externa) no existe en el robot y se ignora; la base y el hombro cubren las direcciones del
brazo como flexión/extensión seguida de abducción. El codo del robot sólo se dobla hacia un lado (el
de la caja): si el tuyo se dobla hacia el otro, el robot lo pone en espejo con la misma flexión, y la
muñeca también, para que la mano siga doblada igual respecto al antebrazo.

**Límite de 90°.** Base y hombro no se alejan más de `ARTICULAR_LIMITE_HOMBRO = 90` grados del estado
inicial: flexión y extensión hasta la horizontal (base `ARTICULAR_BASE` ± 90) y abducción hasta la
horizontal (hombro 0..90; hacia el cuerpo no hay recorrido, está la pared). Siempre dentro de
`ARTICULAR_LIMITES` y de 0..180 de cada servo. Más allá, la articulación se queda en el límite y se
avisa (en rojo). El límite se aplica dos veces antes de mandar cada orden: al calcular la postura y
justo antes de enviarla (`limitar_seguimiento`); si el robot ya estaba fuera (lo movió la consola),
sólo puede volver hacia dentro, sin saltos.

**Sin chocar.** Colgando, el codo del robot sólo puede doblarse hacia la pared. Si con tu postura el
antebrazo o la garra la tocarían, el hombro se separa lo justo (sin pasar de 90°) y la flexión del
codo se conserva. Si ni así cabe, el robot se detiene en el borde y lo avisa, como siempre.

`GEMELO_CORRESPONDENCIA = "muneca"` es la traducción anterior: base y hombro dejan la muñeca del robot
en la dirección de tu muñeca (hombro -> muñeca) con su cinemática (L1, L2), el codo copia la flexión
y el hombro la compensa; la muñeca apunta la garra como tu mano. El límite de 90° también se aplica.

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

**De cada persona** (tecla `C` en el simulador o en la app del operador): deja el brazo relajado,
colgando junto al cuerpo, recto y con la mano recta, y quédate quieto 1.5 s. Se corrigen la dirección
que la cámara ve de más, la flexión que aparenta el codo estando recto y la de la mano. Se guarda en
`seguimiento.json`, donde corre la cámara.

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
| El codo se dobla hacia un solo lado (hacia la caja) | **Física** | Si el tuyo se dobla hacia el otro, el robot lo pone en espejo (misma flexión). Colgando, para doblarlo el hombro se separa de la pared lo justo |
| Límite de 90° del hombro desde el estado inicial | **Seguridad** (`ARTICULAR_LIMITE_HOMBRO`) | El brazo por encima de la horizontal (adelante, atrás o al costado) se queda en la horizontal |
| La base gira 180° en total | **Física** | Al costado alcanza para cubrir todo alrededor del eje que sale de la caja (combinada con el hombro) |
| El robot tiene 4 articulaciones de postura; tu brazo, 7 | **Física** | Tu codo fuera del plano del robot y la rotación del antebrazo no tienen equivalente |
| La pared de la caja, la columna de la base y la mesa (`ARTICULAR_LIMITE_ABAJO`) | **Física** | El robot se detiene en el borde y avisa |
| Con el brazo casi sobre el eje de la base, la base no se mueve | Algoritmo | Al costado, con el brazo casi recto hacia afuera (en T), su plano no está definido y el giro sería ruido (`ARTICULAR_UMBRAL_HORIZONTAL`) |
| El mismo plano se alcanza con dos giros de la base | Algoritmo | Se elige el que cabe en los topes, con histéresis; cambiar de uno a otro es un giro grande y lento |
| La profundidad sale de una sola cámara | Algoritmo y cámara | Ver la sección siguiente; con la segunda cámara (`--webcam-lateral`) mejora mucho |

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
corrige además ese sesgo (con la misma foto calibrada en T, la lectura quedaba en azimut 0°,
elevación 0° y codo 0°).

**Calibración con el brazo colgando.** Se calibra con el brazo relajado junto al cuerpo, que es
también el estado inicial del robot (colgando a plomo, garra hacia abajo): la dirección medida se
gira hasta 'abajo' y la flexión que se ve con el codo recto pasa a ser 0. Es más cómoda que la T,
se mantiene quieta sin esfuerzo y, al costado, cae en una dirección donde el plano de la base está
bien definido (en T el brazo queda sobre el eje de la base y su giro es ruido). Las calibraciones
guardadas antes, en T, siguen valiendo: sin la clave `referencia` se usa 'afuera'.

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
para que la base copie la profundidad con fidelidad: usa la segunda cámara (abajo).

## Cómo se mide tu brazo: las líneas de la imagen

Está en `brazo_robot/seguimiento/vistas.py`. Pose da, por cámara, puntos en la imagen (precisos) y
una estimación 3D cuya profundidad adivina. Midiendo en vivo con el brazo levantado al costado, ese
3D daba unos 40° cuando la imagen mostraba unos 85°. Por eso:

1. **Orientación de cada cámara respecto a tu torso**, con hombros, caderas y nariz de su estimación
   3D (lo que Pose sí estima bien: la postura global), suavizada unos 3 s. En la de frente, "adelante"
   es hacia la cámara; en la lateral, hacia la nariz, que de perfil se ve en la imagen.
2. **La línea de cada segmento en la imagen** (hombro -> codo, codo -> muñeca, muñeca -> nudillos, con
   el modelo de la mano si la ve entera), en largos de torso (caderas -> hombros).
3. **Con dos cámaras**: cada segmento sale de las cuatro ecuaciones (dos por imagen) por mínimos
   cuadrados; lo que una ve de canto, la otra lo ve de lado. No hay que alinear las cámaras entre sí.
   Si las dos no encajan (residuo grande o un brazo de largo imposible), sólo la de frente. Vista de
   lado, Pose a veces cambia izquierda y derecha: se recuerda la asignación que encaja.
4. **Con una**: a lo que se ve le falta la profundidad; sale del largo del segmento (medido al
   calibrar con `C`) y de la estimación de Pose, promediadas, hacia el lado que estima Pose (con
   histéresis, para que no salte de un cuadro a otro).
5. **Respaldo**: sin caderas en cuadro, la estimación 3D de Pose como antes.

En las pruebas con cámaras sintéticas inclinadas como en tu montaje (`tests/test_vistas.py`) y 5 cm de
error de profundidad: con dos cámaras, 2° de error típico y 6° en el 90 % de los casos; con una, 4° y 10°.

**La calibración** (`C`, brazo colgando) mide el largo de cada segmento y lo que aparentan el codo y
la mano estando rectos. Las calibraciones hechas antes con el 3D de Pose no se aplican: hay que
calibrar de nuevo.

**Límite conocido**: si la cámara lateral ve a otra persona cuya postura casualmente es compatible
con las dos imágenes y con el largo de tu brazo, dos vistas no bastan para notarlo. La lateral debe
verte sólo a ti.

**Colocación de la lateral**: a tu derecha, más o menos a la altura del hombro, a 1.5-2.5 m, que te vea
de cuerpo entero (hombros, brazo, caderas y la cara de perfil). La ventana muestra lo que ve, con tu
brazo dibujado, y su estado: "combinando las 2 camaras", "no te ve", "no ve tu brazo" o "no encaja con
la de frente". Dos inferencias de Pose: en una Raspberry 4 van a la mitad de lecturas por segundo.

## Pruebas

**Automáticas** (`python -m pytest`): correspondencia por articulaciones (flexión, extensión y
abducción del hombro, codo, muñeca, límite de 90° antes de enviar, separarse de la pared), medida con
las líneas de la imagen con una y dos cámaras (lados cambiados, lateral que no encaja, calibración), correspondencia anterior con la
cinemática real (dirección de la muñeca y flexión del codo), profundidad simétrica y con su signo, topes sin saltos, brazo vertical, elección
del giro alcanzable, muñeca y mano, marco del torso invariante al girar el cuerpo, calibración con el brazo colgando (y las viejas en T),
estado inicial colgando y rutas seguras hasta él,
integración con el control (acercamiento, mesa, pérdida de lectura), protocolo, vista y separación
del simulador.

**Con hardware**, en este orden:

1. En el simulador, calibra con `C` (brazo relajado colgando) y comprueba a simple vista: brazo
   colgando (el robot debe quedar igual que en el estado inicial), hacia adelante (base), hacia
   atrás, separado del cuerpo (hombro), más allá de la horizontal (se queda en 90°), codo doblado y
   mano inclinada. Repite con `--webcam-lateral` y compara hacia adelante y hacia atrás.
2. Robot real con `ARTICULAR_VELOCIDAD` bajo (por ejemplo 30) y alguien junto al apagado.
3. En la consola, `calibrar montaje`: base de referencia, sentido de la profundidad y ceros.
4. Lleva el brazo despacio hacia la cámara: la base debe girar hacia ese lado. Si no, repite el paso 2
   de `calibrar montaje`.
5. Mide la garra y guárdala con `longitud garra`; `calibrar garra` para abierta y cerrada.
6. Tapa la cámara mientras se mueve: debe detenerse en menos de 1 s.
