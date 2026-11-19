# Brazo robótico que copia tu brazo

Te paras frente a una webcam, mueves tu brazo derecho y **el robot hace lo mismo**: hombro, codo y
muñeca. Al abrir y cerrar la mano, la garra se abre y se cierra. Si conectas una segunda webcam a
tu costado, el programa la usa sola y es más preciso hacia adelante y hacia atrás.

```
   Tú, frente a la webcam              El robot, atornillado al costado de la caja
   (y, si quieres, otra a tu lado)
        ( )  [cam]                            ┌──────┐
        /|\   ── la computadora ve ──►        │ caja │──┐
         |      tu brazo y se lo              └──────┘  │  ← brazo del robot
        / \     manda al robot                          ▼  (garra)
```

Esta guía es para usarlo **sin saber programar**. Si vas a cambiar el código, los detalles están
en [docs/tecnico.md](docs/tecnico.md).

---

## ⚠️ Antes de empezar: seguridad

- **Siempre debe haber alguien junto al robot, con la mano en el enchufe o el interruptor.** El
  botón PAUSAR es un programa, no un freno físico.
- Nadie debe estar dentro del alcance del brazo mientras se mueve.
- El robot no ve: no sabe si hay algo o alguien en su camino. Quien lo maneja **debe estar mirándolo**.
- Al encenderse, el robot se mueve solo a su posición inicial (brazo colgando). Despeja su alcance.

---

## Palabras que vas a leer

| Palabra | Qué es |
|---|---|
| **Terminal** | Ventana donde se escriben órdenes de texto. En Mac: app *Terminal*. En Windows: *Miniforge Prompt*. En la Raspberry: el icono de pantalla negra. |
| **Comando** | Una línea que escribes en la terminal y confirmas con la tecla `Enter`. |
| **Raspberry** | La computadora pequeña que está junto al robot. |
| **Arduino** | La placa que mueve los motores. Va conectada a la Raspberry por USB. |
| **Estado inicial** | La posición de descanso del robot: brazo colgando recto junto a la caja, con la garra hacia abajo, como tu brazo relajado. |
| **Webcam de frente** | La que te ve de frente. Es la principal: con ella sola ya funciona. |
| **Webcam lateral** | Una segunda, opcional, a tu costado derecho. Ve de lado cuando llevas el brazo hacia adelante o hacia atrás. |

---

## Qué copia el robot de tu brazo

Todo se cuenta desde el **estado inicial**: tu brazo y el del robot colgando.

| Tú haces... | El robot... |
|---|---|
| Llevar el brazo **hacia adelante** o **hacia atrás** | Gira su base: su brazo va hacia adelante o hacia atrás igual que el tuyo. |
| **Separar el brazo del cuerpo**, hacia el costado | Separa su brazo de la caja lo mismo. |
| **Doblar el codo** | Dobla su codo lo mismo. |
| **Doblar la muñeca** | Dobla su muñeca lo mismo. |
| **Abrir y cerrar la mano** | Abre y cierra la garra. |

Lo que el robot **no puede** hacer, y es normal:

- **No sube el brazo más allá de la horizontal** (90° desde que cuelga), ni hacia adelante, ni hacia
  atrás, ni hacia el costado. Es su límite de seguridad: si tú lo subes más, el robot se queda ahí.
- **No gira el brazo sobre sí mismo** (como cuando giras la palma hacia arriba o hacia abajo): no
  tiene ese motor.
- **Su codo sólo se dobla hacia la caja.** Si doblas el tuyo hacia el otro lado, el robot lo dobla
  igual pero en espejo.
- Con el brazo colgando, si doblas el codo, el robot **separa un poco el brazo de la caja** para que
  su antebrazo no choque con ella.

---

## Usar el robot (en la Raspberry)

### 1. Arrancar

1. Conecta el Arduino a la Raspberry por USB y enciende la fuente de los motores.
2. Conecta la webcam a la Raspberry. Si conectas dos, se usan las dos solas (pero en la Raspberry
   va más lento: con dos es mejor manejar desde otra computadora, ver más abajo).
3. Abre una terminal y escribe, uno por uno (`Enter` después de cada uno):

   ```bash
   cd ~/brazo
   ```

   ```bash
   python main.py
   ```

La primera vez tarda varios minutos porque instala lo necesario; las siguientes, segundos.

Se abren dos cosas:

- **Una ventana con tu webcam** y un dibujo del robot. Desde aquí lo manejas. Si hay webcam lateral,
  lo que ve aparece pequeño abajo a la derecha.
- **La consola**, en la misma terminal (donde aparece `>`). Desde aquí se calibra.

> Al arrancar, **ponte frente a la webcam de frente**: si hay dos, así sabe cuál es cuál (y si se
> equivoca, se corrige solo en unos segundos al verte). En la terminal aparece lo que encontró en
> cada cámara (`Buscando webcams...`) y cuáles usa, por ejemplo `Webcam: 0 | lateral: 2`.
> Si dice *"No encuentro el Arduino"*: revisa el cable USB y que la fuente esté encendida.
> Si dice que *el servicio ya está corriendo*: solo se abre la ventana, puedes manejar igual.
> Para calibrar en ese caso, primero escribe `sudo systemctl stop brazo` y luego `python main.py`.

### 2. Calibrar el robot (solo la primera vez, o si lo desatornillan)

Se hace en la **consola**: escribe la orden y pulsa `Enter`. Se abre una ventana con
instrucciones; **las teclas se pulsan con esa ventana seleccionada** (haz clic en ella).

**a) `calibrar montaje`**: le enseña al robot cómo está atornillado. Son tres pasos:

1. El brazo cuelga hacia abajo. Pulsa `A` o `D` hasta que cuelgue **completamente recto hacia
   abajo** (si tienes una plomada o un hilo con peso, compáralo). Pulsa `Enter`.
2. El robot gira un poco. Mirando desde donde está la webcam: si el brazo se acercó a la cámara
   pulsa `S`; si se alejó, pulsa `N`.
3. El robot pone el brazo saliendo recto de la caja y el antebrazo doblado en ángulo recto. Con una
   escuadra revisa cada parte: pulsa `1` (hombro), `2` (codo) o `3` (muñeca) para elegir y
   `A`/`D` para corregir. Pulsa `G` para guardar.

Al terminar, el robot vuelve solo al estado inicial. `ESC` cancela sin guardar nada.

**b) `calibrar garra`**: `A`/`D` mueven la garra. Cuando esté bien abierta pulsa `O`; cuando esté
cerrada sujetando una pieza **sin forzar** el motor, pulsa `C`. `ESC` para salir.

### 3. Calibrar tu brazo (cada persona, la primera vez)

Esta calibración mide **el largo de tu brazo** y cómo se ve recto. Si ya habías calibrado antes de esta
versión, **hazla otra vez**: la anterior ya no se usa.

En la **ventana de la webcam**:

1. Ponte de frente a la cámara, de modo que se vean **tus dos hombros, tu cadera y tu brazo derecho
   completo, con la mano dentro de la imagen**.
2. Deja el brazo derecho **relajado, colgando junto al cuerpo, recto y quieto**.
3. Pulsa `C` y no te muevas durante 2 segundos. Si usas dos webcams, calibra con las dos puestas.

El robot vuelve al estado inicial: ahora tú y el robot están en la misma postura.

Pulsa también `O` con la mano bien abierta y `P` con el puño cerrado, para que la garra
entienda tu mano.

> **Tu mano**: la muñeca del robot la sigue mientras se vea tu palma, aunque algún dedo se salga de
> la imagen. Para abrir y cerrar la garra se necesitan ver casi todos los dedos.

### 4. Manejar

| Tecla o botón | Qué hace |
|---|---|
| `ESPACIO` o **ACTIVAR** | Empieza a copiarte. Quédate quieto medio segundo: el robot se acerca despacio a tu postura y después te sigue. |
| `ESPACIO` o **PAUSAR** | El robot se detiene donde está. Puedes moverte libremente. |
| Abrir / cerrar la mano | Abre / cierra la garra. |
| `G` o **GARRA** | Abre o cierra la garra sin usar la mano. |
| `H` o **HOME** | Lleva el robot al estado inicial (primero pausa). |
| `C` | Vuelve a calibrar tu brazo (ver paso 3). |
| `ESC` o **SALIR** | Cierra la ventana. El robot se queda en pausa donde está. |

**Consejos:**

- Muévete despacio, sobre todo al principio.
- Si sales de la cámara o entra otra persona, el robot **se pausa solo**. Pulsa `ESPACIO` para seguir.
- Si la garra sostiene una pieza, no la suelta aunque abras la mano de golpe: espera a que tu mano
  "la alcance".
- El robot copia tu hombro, tu codo y tu muñeca, y **no sube el brazo más allá de la horizontal**
  (ver "Qué copia el robot de tu brazo").
- Hacia adelante y atrás (acercar el brazo a la cámara) es lo más difícil de ver con una sola
  cámara. Funciona mejor si apuntas un poco hacia un lado, o con una segunda cámara (ver abajo).

### 5. Terminar

En la consola escribe `salir` y pulsa `Enter`. El robot vuelve al estado inicial y todo se cierra.
Después apaga la fuente de los motores.

---

## Probarlo sin robot, en tu computadora (simulador)

Ves un robot dibujado que copia tu brazo. Sirve para practicar sin riesgo.

**Una sola vez**, para preparar tu computadora:

1. Instala **Miniforge** (es gratis): <https://github.com/conda-forge/miniforge#install>.
2. Abre una terminal y descarga el proyecto:

   ```bash
   git clone https://github.com/knightbarin98/brazo.git
   ```

**Cada vez que quieras probar**:

```bash
cd brazo
```

```bash
python main.py simulador
```

Si tu computadora tiene dos webcams (por ejemplo la de la laptop y una USB), usa las dos solas.

Teclas: `ESPACIO` empieza/pausa, `C` calibra tu brazo (colgando, como en el paso 3), `G` garra,
`A`/`D` y `W`/`S` giran el dibujo, `V` lo regresa, `ESC` cierra.

> **En Mac**, la primera vez pide permiso para la cámara. Si no lo pide o no ves la imagen: *Ajustes
> del Sistema → Privacidad y seguridad → Cámara* y activa *Terminal*. Después cierra la Terminal y
> vuelve a abrirla.

---

## Usar una segunda cámara (opcional, más precisa)

Una segunda webcam a tu **costado derecho** ve de lado lo que la de frente ve mal: cuando llevas el
brazo hacia adelante o hacia atrás. No hay que alinearla con la otra: se orienta sola con tu cuerpo.

1. Conecta la segunda webcam a la **misma computadora** que la primera.
2. Colócala a tu derecha, más o menos a la altura de tu hombro, a 1.5-2.5 m, que te vea de cuerpo
   entero (hombros, brazo y cadera).
3. **Arranca como siempre** (`python main.py`, el simulador o el operador) y **ponte frente a la
   cámara de frente**: el programa encuentra las dos webcams solo y usa como "de frente" la que te ve
   de frente. Las cámaras virtuales (por ejemplo la de OBS) se ignoran. En la terminal dice cuáles
   usa, algo como `Webcam: 0 | lateral: 2`.
4. Abajo a la derecha de la ventana verás lo que ve la cámara lateral, con tu brazo dibujado. Debe
   decir **"combinando las 2 camaras"** (en verde). Si dice "no te ve" o "no encaja", acomódala para
   que te vea de cuerpo entero, con la cara de perfil, y sin nadie más en la imagen.
5. Calibra tu brazo con `C` (paso 3) con las dos cámaras puestas.

Si la cámara lateral te pierde, el robot sigue funcionando con la de frente.

Si las eligió al revés (la de frente aparece abajo a la derecha), **se corrige sola**: cuando la
lateral te ve claramente más de frente durante 2 segundos, se intercambian y la terminal dice
*"Las webcams estaban cruzadas"*. También puedes decirle cuál es cuál con los números que mostró,
por ejemplo `python main.py --webcam 2 --webcam-lateral 0`. Para usar sólo una: `--webcam-lateral no`.

> Con dos cámaras la computadora trabaja el doble. En la Raspberry va lento: con dos webcams,
> conéctalas a una computadora normal y maneja desde ahí (ver la sección siguiente).

## Manejarlo desde otra computadora

El robot y su Raspberry se quedan en su lugar; tú usas tu computadora con tu webcam (o tus dos
webcams: se usan solas, igual que en la Raspberry). Las dos computadoras deben estar **en la misma
red WiFi**.

1. En la Raspberry, arranca con `python main.py` (o ya estará corriendo si el servicio está
   instalado). Anota la **IP** que aparece, algo como `192.168.1.50`.
2. En tu computadora (preparada como en el simulador):

   ```bash
   python main.py operador --servidor 192.168.1.50
   ```

   Cambia el número por la IP de tu Raspberry.
3. Te pedirá un **token** (una contraseña). Se ve en la Raspberry con:

   ```bash
   cat ~/brazo/token_remoto.txt
   ```

Los controles son los mismos de la tabla de arriba. **Igual tiene que haber alguien junto al robot.**

---

## Si algo no funciona

| Problema | Qué hacer |
|---|---|
| *"No encuentro el Arduino"* | Revisa el cable USB del Arduino y que la fuente de los motores esté encendida. |
| La ventana dice *"no te veo"* | Aléjate un poco: deben verse tus dos hombros, el codo y la muñeca. Mejor con buena luz y de frente. Con poca luz las cámaras no te ven. |
| No abras otro programa que use la webcam mientras corre | Se la puede quitar ("Se perdió la señal de la webcam"): ciérralo y vuelve a arrancar. |
| No se ve la imagen de la webcam (Mac) | Da permiso de cámara a Terminal (ver el simulador). |
| Usa como "de frente" la cámara equivocada | Ponte frente a la de frente unos segundos: se corrige sola. Si no, usa los números que muestra al arrancar: `--webcam 2 --webcam-lateral 0` (el primero, la de frente). En Mac los números pueden cambiar de un día a otro. |
| Dice *"No te veo en las dos webcams al arrancar"* | No te vio al empezar y eligió por orden. No pasa nada: al verte, si quedaron al revés, se intercambian solas. |
| El robot se queda quieto aunque me muevo | Mira la ventana: si dice "no te veo", la cámara de frente no te ve bien (luz, distancia, o está tapada). Revisa en la terminal qué cámaras encontró al arrancar. |
| Quiero usar sólo una webcam | Agrega `--webcam-lateral no` al comando. |
| Tengo dos webcams y sólo usa una | Al arrancar dice qué encontró en cada cámara ("Buscando webcams..."). *Imagen negra*: está tapada, apagada o es la de la laptop con la tapa cerrada. *Imagen fija*: es una cámara virtual (OBS). Si una no aparece, está desconectada: revisa el cable USB. |
| La cámara lateral dice "no te ve", "no ve tu brazo" o "no encaja" | Acomódala para que te vea de cuerpo entero, con la cara de perfil, y que no haya otra persona en su imagen. |
| Mi mano sale cortada por el borde | Aléjate un poco. Si se sale la palma, la muñeca del robot se queda quieta; si sólo se salen dedos, la muñeca sigue pero la garra puede no responder. |
| La garra no abre ni cierra | Que se vea tu mano entera, con los dedos dentro de la imagen, y calibra `O` (abierta) y `P` (puño). |
| El robot no sube el brazo más allá de la horizontal | Es a propósito: es su límite de seguridad (90° desde que cuelga). |
| Doblo el codo y el robot lo dobla hacia el otro lado | Es normal: su codo sólo se dobla hacia la caja, así que lo hace en espejo. |
| Con el brazo colgando, al doblar el codo el robot separa el brazo de la caja | Es normal: lo separa lo justo para que el antebrazo no choque con la caja. |
| Giro la palma y el robot no hace nada | Es normal: el robot no tiene ese movimiento. |
| El robot no se mueve al pulsar ESPACIO | Mira el mensaje de abajo en la ventana. Quizá no te ve, o está en pausa por una protección: pulsa `ESPACIO` otra vez. |
| El robot no copia bien tu brazo | Calibra tu brazo con `C` (paso 3), viéndote de cuerpo entero (también la cadera). Si el robot está desviado incluso en el estado inicial, repite `calibrar montaje`. |
| *"No encuentro un camino seguro"* | El robot quedó muy pegado a la caja. En la consola usa `servo` o `ir` para alejarlo (pide ayuda a quien programa) o muévelo a mano **con la fuente apagada**. |
| La consola dice que el Arduino está ocupado | El servicio automático ya lo usa. Escribe `sudo systemctl stop brazo` y vuelve a intentar. |
| Quiero ver qué está instalado | `python main.py estado` |
| Quiero ver todas las opciones | `python main.py menu` |

Si nada de esto ayuda, copia **todo** el texto que salió en la terminal y mándaselo a quien programa.

---

## Para quien programa

- Detalles técnicos, instalación automática en la Raspberry, todas las órdenes de la consola,
  protecciones, red y estructura del código: [docs/tecnico.md](docs/tecnico.md).
- Cómo se traduce tu brazo al robot, límites, profundidad y la segunda cámara:
  [docs/gemelo.md](docs/gemelo.md).
- Pruebas automáticas: `python main.py pruebas`.
