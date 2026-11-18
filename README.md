# Brazo robótico que copia tu brazo

Te paras frente a una webcam, mueves tu brazo derecho y **el robot hace lo mismo**. Al abrir y
cerrar la mano, la garra se abre y se cierra.

```
   Tú, frente a la webcam              El robot, atornillado al costado de la caja
        ( )                                   ┌──────┐
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

---

## Usar el robot (en la Raspberry)

### 1. Arrancar

1. Conecta el Arduino a la Raspberry por USB y enciende la fuente de los motores.
2. Conecta la webcam a la Raspberry.
3. Abre una terminal y escribe, uno por uno (`Enter` después de cada uno):

   ```bash
   cd ~/brazo
   ```

   ```bash
   python main.py
   ```

La primera vez tarda varios minutos porque instala lo necesario; las siguientes, segundos.

Se abren dos cosas:

- **Una ventana con tu webcam** y un dibujo del robot. Desde aquí lo manejas.
- **La consola**, en la misma terminal (donde aparece `>`). Desde aquí se calibra.

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

En la **ventana de la webcam**:

1. Ponte de frente a la cámara, de modo que se vean **tus dos hombros y tu brazo derecho completo**.
2. Deja el brazo derecho **relajado, colgando junto al cuerpo, recto y quieto**.
3. Pulsa `C` y no te muevas durante 2 segundos.

El robot vuelve al estado inicial: ahora tú y el robot están en la misma postura.

Pulsa también `O` con la mano bien abierta y `P` con el puño cerrado, para que la garra
entienda tu mano.

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
- Hacia adelante y atrás (acercar el brazo a la cámara) es lo más difícil de ver con una sola
  cámara. Funciona mejor si apuntas un poco hacia un lado.

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

Teclas: `ESPACIO` empieza/pausa, `C` calibra tu brazo (colgando, como en el paso 3), `G` garra,
`A`/`D` y `W`/`S` giran el dibujo, `V` lo regresa, `ESC` cierra.

> **En Mac**, la primera vez pide permiso para la cámara. Si no lo pide o no ves la imagen: *Ajustes
> del Sistema → Privacidad y seguridad → Cámara* y activa *Terminal*. Después cierra la Terminal y
> vuelve a abrirla.

---

## Manejarlo desde otra computadora

El robot y su Raspberry se quedan en su lugar; tú usas tu computadora con tu webcam. Las dos
computadoras deben estar **en la misma red WiFi**.

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
| La ventana dice *"no te veo"* | Aléjate un poco: deben verse tus dos hombros, el codo y la muñeca. Mejor con buena luz y de frente. |
| No se ve la imagen de la webcam (Mac) | Da permiso de cámara a Terminal (ver el simulador). |
| Tengo varias cámaras y abre la equivocada | Agrega `--webcam 1` al final del comando (o `2`, `3`...). |
| El robot no se mueve al pulsar ESPACIO | Mira el mensaje de abajo en la ventana. Quizá no te ve, o está en pausa por una protección: pulsa `ESPACIO` otra vez. |
| El robot no copia bien tu brazo | Calibra tu brazo con `C` (paso 3). Si el robot está desviado incluso en el estado inicial, repite `calibrar montaje`. |
| *"No encuentro un camino seguro"* | El robot quedó muy pegado a la caja. En la consola usa `servo` o `ir` para alejarlo (pide ayuda a quien programa) o muévelo a mano **con la fuente apagada**. |
| La consola dice que el Arduino está ocupado | El servicio automático ya lo usa. Escribe `sudo systemctl stop brazo` y vuelve a intentar. |
| Quiero ver qué está instalado | `python main.py estado` |
| Quiero ver todas las opciones | `python main.py menu` |

Si nada de esto ayuda, copia **todo** el texto que salió en la terminal y mándaselo a quien programa.

---

## Para quien programa

- Detalles técnicos, instalación automática en la Raspberry, todas las órdenes de la consola,
  protecciones, red y estructura del código: [docs/tecnico.md](docs/tecnico.md).
- Cómo se traduce tu brazo al robot, límites y profundidad: [docs/gemelo.md](docs/gemelo.md).
- Pruebas automáticas: `python main.py pruebas`.
