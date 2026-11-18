"""Arranque de 'python main.py': prepara el entorno de conda y lanza cada parte del proyecto.

SÓLO BIBLIOTECA ESTÁNDAR: este módulo corre con cualquier Python, normalmente el del entorno 'base'
que conda deja activo. Si no está ya dentro del entorno del proyecto, lo crea (o lo actualiza si
cambió environment.yml) y se vuelve a ejecutar con su Python. Así nadie tiene que activarlo a mano.
"""

import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENTORNO = "brazo-gemelo"
ARCHIVO_ENTORNO = os.path.join(RAIZ, "environment.yml")
HUELLA = ".brazo-environment.sha256"        # dentro del entorno: el environment.yml con el que se creó
MARCA_SERVICIO = os.path.join(RAIZ, "scripts", ".servicio_instalado")
ENTORNOS_VIEJOS = ("brazo", "brazo-pi")      # nombres de antes de unificar
VARIABLE_ARRANQUE = "BRAZO_ARRANQUE"         # evita un bucle si el Python del entorno no sirve

# Comandos que necesitan el entorno (numpy, OpenCV, MediaPipe...) y su descripción para el menú
EN_ENTORNO = {
    "trabajar": "Trabajar con el robot: servidor + consola + operador (lo que hace 'python main.py')",
    "simulador": "Simulador del gemelo, sin robot (webcam + robot virtual)",
    "operador": "Manejar el robot con la webcam (app del operador)",
    "servidor": "Servidor del robot: controla el Arduino (en la Raspberry)",
    "consola": "Consola local: calibrar y probar el robot (en la Raspberry)",
    "pruebas": "Correr las pruebas automáticas",
}
# Comandos que corren con cualquier Python, sin el entorno
SIN_ENTORNO = {
    "menu": "Este menú",
    "estado": "Ver qué está instalado y qué está corriendo",
    "entorno": "Crear o actualizar el entorno de conda",
    "instalar": "Instalar el servicio automático en la Raspberry (auto-deploy)",
    "desinstalar": "Deshacer todo: servicios, scripts y entornos, para empezar de cero",
}
USA_EL_ARDUINO = ("trabajar", "servidor", "consola")


class ErrorArranque(Exception):
    pass


# ---------------------------------------------------------------- conda y el entorno
def huella_archivo_entorno():
    with open(ARCHIVO_ENTORNO, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def buscar_conda():
    """Ruta del ejecutable de conda, o None."""
    candidatos = [os.environ.get("CONDA_EXE"), shutil.which("conda")]
    casa = os.path.expanduser("~")
    for base in ("miniforge3", "mambaforge", "miniconda3", "anaconda3"):
        candidatos.append(os.path.join(casa, base, "Scripts" if os.name == "nt" else "bin",
                                       "conda.exe" if os.name == "nt" else "conda"))
    candidatos += ["/opt/homebrew/Caskroom/miniconda/base/bin/conda", "/opt/conda/bin/conda"]
    for c in candidatos:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return None


def entornos(conda):
    """{nombre: prefijo} de los entornos de conda (el base incluido como 'base')."""
    salida = subprocess.run([conda, "env", "list", "--json"], stdout=subprocess.PIPE, check=True)
    prefijos = json.loads(salida.stdout.decode())["envs"]
    raiz = os.path.dirname(os.path.dirname(os.path.realpath(conda)))
    return {("base" if os.path.realpath(p) == raiz else os.path.basename(p)): p for p in prefijos}


def python_de(prefijo):
    return os.path.join(prefijo, "python.exe") if os.name == "nt" else os.path.join(prefijo, "bin", "python")


def en_el_entorno():
    return os.path.basename(os.path.normpath(sys.prefix)) == ENTORNO


def al_dia(prefijo):
    try:
        with open(os.path.join(prefijo, HUELLA)) as f:
            return f.read().strip() == huella_archivo_entorno()
    except OSError:
        return False


def preparar_entorno(conda, prefijo=None):
    """Crea el entorno si falta o lo actualiza si cambió environment.yml. Devuelve su prefijo."""
    if prefijo is None:
        prefijo = entornos(conda).get(ENTORNO)
    if prefijo is None:
        print(f"Creando el entorno '{ENTORNO}' (la primera vez tarda varios minutos)...")
        subprocess.run([conda, "env", "create", "-n", ENTORNO, "-f", ARCHIVO_ENTORNO], check=True)
        prefijo = entornos(conda).get(ENTORNO)
        if prefijo is None:
            raise ErrorArranque(f"conda no creó el entorno '{ENTORNO}'.")
    elif not al_dia(prefijo):
        print(f"environment.yml cambió: actualizando el entorno '{ENTORNO}'...")
        subprocess.run([conda, "env", "update", "-n", ENTORNO, "-f", ARCHIVO_ENTORNO, "--prune"], check=True)
    else:
        return prefijo
    with open(os.path.join(prefijo, HUELLA), "w") as f:
        f.write(huella_archivo_entorno() + "\n")
    return prefijo


def exigir_conda():
    conda = buscar_conda()
    if conda is None:
        raise ErrorArranque(
            "No encontré conda. Instala Miniforge (https://github.com/conda-forge/miniforge) y vuelve a "
            "abrir la terminal, o indica la ruta con la variable CONDA_EXE.")
    return conda


def reejecutar_en_entorno(argumentos):
    """Prepara el entorno y vuelve a correr main.py con su Python. No regresa."""
    if os.environ.get(VARIABLE_ARRANQUE):
        raise ErrorArranque(f"El Python del entorno '{ENTORNO}' no arrancó como se esperaba ({sys.executable}). "
                            "Prueba: python main.py desinstalar y después python main.py entorno")
    conda = exigir_conda()
    _reiniciar_con(python_de(preparar_entorno(conda)), argumentos)


def _reiniciar_con(python, argumentos):
    """Vuelve a correr main.py con ese Python y los mismos argumentos. No regresa."""
    entorno = dict(os.environ, **{VARIABLE_ARRANQUE: "1"})
    orden = [python, os.path.join(RAIZ, "main.py")] + list(argumentos)
    if os.name == "nt":                       # en Windows exec no conserva la consola: se espera al hijo
        sys.exit(subprocess.call(orden, env=entorno))
    os.execve(python, orden, entorno)


# ---------------------------------------------------------------- servicio en la Raspberry
def leer_marca():
    """Contenido de scripts/.servicio_instalado como dict, o None si no existe."""
    if not os.path.exists(MARCA_SERVICIO):
        return None
    datos = {}
    with open(MARCA_SERVICIO) as f:
        for linea in f:
            if "=" in linea:
                clave, valor = linea.rstrip("\n").split("=", 1)
                datos[clave] = valor
    return datos


def hay_systemd():
    return platform.system() == "Linux" and shutil.which("systemctl") is not None


def estado_servicio(nombre="brazo"):
    """'active', 'inactive', 'failed'... o None si no hay systemd o la unidad no existe."""
    if not hay_systemd() or not os.path.exists(f"/etc/systemd/system/{nombre}.service"):
        return None
    salida = subprocess.run(["systemctl", "is-active", nombre], stdout=subprocess.PIPE)
    return salida.stdout.decode().strip()


def servicio_ocupa_el_arduino():
    """True si el servicio del brazo está corriendo (y por tanto usa el Arduino), salvo que seamos él."""
    return not os.environ.get("BRAZO_SERVICIO") and estado_servicio() == "active"


# ---------------------------------------------------------------- comandos sin entorno
def comando_estado():
    print(f"Proyecto:  {RAIZ}")
    print(f"Sistema:   {platform.system()} {platform.machine()} | Python {platform.python_version()} ({sys.prefix})")
    conda = buscar_conda()
    print(f"conda:     {conda or 'NO ENCONTRADO'}")
    if conda:
        todos = entornos(conda)
        prefijo = todos.get(ENTORNO)
        if prefijo is None:
            print(f"Entorno:   '{ENTORNO}' no existe (se crea solo al usar main.py)")
        else:
            print(f"Entorno:   '{ENTORNO}' en {prefijo} | {'al día' if al_dia(prefijo) else 'hay que actualizarlo (se hace solo)'}")
        viejos = [n for n in ENTORNOS_VIEJOS if n in todos]
        if viejos:
            print(f"           entornos de antes que ya no se usan: {', '.join(viejos)} ('python main.py desinstalar' los borra)")
    marca = leer_marca()
    if marca:
        print(f"Servicio:  instalado el {marca.get('FECHA', '?')} (rama {marca.get('RAMA', '?')}, usuario {marca.get('USUARIO', '?')})")
    elif os.path.exists("/etc/systemd/system/brazo.service"):
        print("Servicio:  instalado con una versión anterior del instalador (sin marca). "
              "Para empezar de cero: python main.py desinstalar")
    else:
        print("Servicio:  no instalado en esta máquina")
    for unidad in ("brazo", "brazo-deploy.timer"):
        e = estado_servicio(unidad)
        if e is not None:
            print(f"           {unidad}: {e}")
    if servicio_ocupa_el_arduino():
        print("Aviso:     el servicio está usando el Arduino; 'servidor' y 'consola' necesitan 'sudo systemctl stop brazo'.")


def _git(*args):
    salida = subprocess.run(["git", "-C", RAIZ] + list(args), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return salida.stdout.decode().strip() if salida.returncode == 0 else None


def comando_instalar(argumentos):
    if not hay_systemd():
        raise ErrorArranque("El servicio automático es para la Raspberry (Linux con systemd).")
    marca = leer_marca()
    if marca and "--reinstalar" not in argumentos:
        print(f"El servicio ya está instalado ({marca.get('FECHA', '?')}, rama {marca.get('RAMA', '?')}).")
        print("Para instalarlo otra vez: python main.py instalar --reinstalar")
        return
    argumentos = [a for a in argumentos if a != "--reinstalar"]
    repo = argumentos[0] if argumentos else _git("remote", "get-url", "origin")
    rama = argumentos[1] if len(argumentos) > 1 else "main"
    if not repo:
        raise ErrorArranque("No sé la URL del repositorio: python main.py instalar <URL> [RAMA]")
    print(f"Se instalará el servicio con {repo} (rama {rama}). Pedirá tu contraseña de sudo.")
    sys.exit(subprocess.call(["sudo", "bash", os.path.join(RAIZ, "scripts", "instalar_autodeploy.sh"), repo, rama]))


def comando_desinstalar(argumentos):
    if hay_systemd():
        print("Pedirá tu contraseña de sudo.")
        sys.exit(subprocess.call(["sudo", "bash", os.path.join(RAIZ, "scripts", "desinstalar.sh")] + list(argumentos)))
    # Sin systemd (Mac, Windows o Linux sin servicio): sólo hay entornos de conda que borrar.
    conda = exigir_conda()
    todos = entornos(conda)
    for nombre in (ENTORNO,) + ENTORNOS_VIEJOS:
        if nombre not in todos:
            continue
        if nombre != ENTORNO and input(f"¿Borrar también el entorno viejo '{nombre}'? [s/N] ").strip().lower() != "s":
            continue
        print(f"Borrando el entorno '{nombre}'...")
        subprocess.run([conda, "env", "remove", "-y", "-n", nombre], check=True)
    print("Listo. 'python main.py' lo vuelve a crear cuando lo necesites.")


# ---------------------------------------------------------------- comandos con entorno
def ejecutar_en_entorno(comando, argumentos):
    """Ya dentro del entorno: lanza la parte pedida con sus propios argumentos."""
    if comando == "trabajar" and servicio_ocupa_el_arduino():
        print("El servicio automático ya corre el servidor y usa el Arduino: abro sólo el operador.\n"
              "  Para tener también la consola:  sudo systemctl stop brazo  y vuelve a correr  python main.py")
        comando, argumentos = "operador", []
    if comando in USA_EL_ARDUINO and servicio_ocupa_el_arduino():
        raise ErrorArranque("El servicio automático ya está corriendo el servidor y usa el Arduino.\n"
                            "  Detenlo antes:  sudo systemctl stop brazo\n"
                            "  Y al terminar:  sudo systemctl start brazo")
    sys.argv[0] = f"main.py {comando}"
    if comando == "trabajar":
        from brazo_robot.trabajar import ejecutar_trabajo
        ejecutar_trabajo(argumentos)
    elif comando == "simulador":
        import simulador
        simulador.main(argumentos)
    elif comando == "operador":
        import app_operador
        app_operador.main(argumentos)
    elif comando == "servidor":
        import servidor
        servidor.main(argumentos)
    elif comando == "consola":
        from brazo_robot import cli
        cli.main(argumentos)
    elif comando == "pruebas":
        import pytest
        os.chdir(RAIZ)
        sys.exit(pytest.main(["-q"] + list(argumentos)))


# ---------------------------------------------------------------- menú y entrada
def ayuda():
    print("Uso: python main.py [comando] [opciones del comando]\n")
    for nombre, texto in list(EN_ENTORNO.items()) + list(SIN_ENTORNO.items()):
        print(f"  {nombre:12s} {texto}")
    print("\nSin comando hace 'trabajar' (servidor + consola + operador); 'python main.py menu' abre el menú.")
    print("'python main.py <comando> --help' muestra sus opciones.")


def menu():
    opciones = list(EN_ENTORNO) + [c for c in SIN_ENTORNO if c != "menu"]
    print("\nBrazo robótico. ¿Qué quieres hacer?")
    for i, nombre in enumerate(opciones, 1):
        texto = EN_ENTORNO.get(nombre) or SIN_ENTORNO[nombre]
        print(f"  {i}) {texto}")
    print("  0) Salir")
    try:
        eleccion = input("> ").strip()
    except (EOFError, KeyboardInterrupt):
        return None
    if eleccion.isdigit() and 1 <= int(eleccion) <= len(opciones):
        return opciones[int(eleccion) - 1]
    if eleccion in opciones:
        return eleccion
    return None


def main(argumentos):
    try:
        if argumentos and argumentos[0] in ("-h", "--help", "ayuda"):
            ayuda()
            return
        if not argumentos or argumentos[0].startswith("-"):
            # Sin comando (o sólo con opciones): trabajar con el robot. Necesita la terminal para la consola.
            if not sys.stdin.isatty():
                ayuda()
                return
            comando, resto = "trabajar", list(argumentos)
        else:
            comando, resto = argumentos[0], argumentos[1:]
        if comando == "menu":
            if not sys.stdin.isatty():
                ayuda()
                return
            comando, resto = menu(), []
            if comando is None:
                return
        if comando not in EN_ENTORNO and comando not in SIN_ENTORNO:
            print(f"Comando desconocido: {comando}\n")
            ayuda()
            sys.exit(2)
        if comando == "estado":
            comando_estado()
        elif comando == "entorno":
            preparar_entorno(exigir_conda())
            print(f"Entorno '{ENTORNO}' listo.")
        elif comando == "instalar":
            comando_instalar(resto)
        elif comando == "desinstalar":
            comando_desinstalar(resto)
        elif not en_el_entorno():
            reejecutar_en_entorno([comando] + list(resto))
        else:
            conda = buscar_conda()
            if conda and not al_dia(sys.prefix):  # dentro del entorno pero environment.yml cambió
                preparar_entorno(conda, sys.prefix)
                _reiniciar_con(sys.executable, [comando] + list(resto))
            ejecutar_en_entorno(comando, resto)
    except ErrorArranque as e:
        sys.exit(f"Error: {e}")
    except subprocess.CalledProcessError as e:
        sys.exit(f"Error: falló '{' '.join(map(str, e.cmd))}' (código {e.returncode}).")
