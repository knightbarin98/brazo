"""Consola del robot montado: calibraciones nuevas, enseñanza sin cámara y modo de trabajo."""

import json
import threading
import time

import pytest

from brazo_robot import ajustes, arranque, calibracion, config, trabajar
from brazo_robot.cinematica import postura_inicial, postura_segura, ruta_segura


class BrazoFalso:
    def __init__(self):
        self.ultimo = [90.0, 120.0, 97.0, 97.0, 100.0]
        self.velocidad = 2.0
        self.movimientos = []

    def ajustar_velocidad(self, v, guardar=True):
        self.velocidad = v

    def mover(self, valores):
        self.movimientos.append(list(valores))
        self.ultimo = [float(v) for v in valores]

    def mover_continuo(self, valores):
        self.mover(valores)
        return True

    def ir(self, x, y, z, garra, inclinacion=-90.0):
        from brazo_robot.cinematica import ik
        self.mover(ik(x, y, z, inclinacion) + [garra])

    def home(self, garra=None):                     # como Brazo.home: por la ruta segura
        for servos in ruta_segura(self.ultimo, postura_inicial()):
            self.mover(servos + [100.0 if garra is None else garra])


@pytest.fixture
def ajustes_temporales(tmp_path, monkeypatch):
    """ajustes.json en una carpeta temporal y config/MAPEO restaurados al terminar."""
    monkeypatch.setattr(ajustes, "AJUSTES_ARCHIVO", str(tmp_path / "ajustes.json"))
    monkeypatch.setattr(config, "MONTAJE", "costado")
    monkeypatch.setattr(config, "ARTICULAR_BASE", config.ARTICULAR_BASE)
    monkeypatch.setattr(config, "ARTICULAR_SIGNO_PROFUNDIDAD", config.ARTICULAR_SIGNO_PROFUNDIDAD)
    monkeypatch.setattr(config, "Z_RECOGIDA", 10.0)
    mapeo = dict(config.MAPEO)
    guardados = {k: ajustes.AJUSTES.get(k) for k in ("articular_base", "articular_signo_profundidad", "mapeo_cero",
                                                     "garra_abierta", "garra_cerrada")}
    yield tmp_path / "ajustes.json"
    config.MAPEO.update(mapeo)
    for k, v in guardados.items():
        if v is None:
            ajustes.AJUSTES.pop(k, None)
        else:
            ajustes.AJUSTES[k] = v


def teclas(monkeypatch, modulo, secuencia):
    """Ventanas falsas: leer_tecla devuelve la secuencia y luego ESC."""
    cola = [ord(c) if isinstance(c, str) else c for c in secuencia]
    monkeypatch.setattr(modulo, "leer_tecla", lambda ms=30: cola.pop(0) if cola else 27)
    monkeypatch.setattr(modulo, "ventana_cerrada", lambda win, visto: False)
    for nombre in ("namedWindow", "imshow", "destroyWindow"):
        monkeypatch.setattr(modulo.cv2, nombre, lambda *a, **k: None)


# ---------------------------------------------------------------- montaje guardado
def test_guardar_y_cargar_el_montaje(ajustes_temporales):
    ajustes.guardar_montaje(95.0, -1.0, {"base": 90.0, "hombro": 2.0, "codo": -1.5, "muneca": 91.0})
    assert config.ARTICULAR_BASE == 95.0 and config.ARTICULAR_SIGNO_PROFUNDIDAD == -1.0
    assert config.MAPEO["hombro"] == (2.0, 1) and config.MAPEO["codo"] == (-1.5, -1)
    datos = json.loads(ajustes_temporales.read_text())
    assert datos["articular_base"] == 95.0 and datos["mapeo_cero"]["muneca"] == 91.0


def test_un_cero_disparatado_no_se_guarda(ajustes_temporales):
    antes = dict(config.MAPEO)
    with pytest.raises(ValueError):
        ajustes.guardar_montaje(90.0, 1.0, {"base": 90.0, "hombro": 80.0, "codo": 0.0, "muneca": 90.0})
    assert config.MAPEO == antes


# ---------------------------------------------------------------- calibrar montaje
def test_calibrar_montaje_paso_a_paso(ajustes_temporales, monkeypatch):
    # base +3, siguiente, "sí fue hacia la cámara", codo -1, guardar
    teclas(monkeypatch, calibracion, ["d", "d", "d", 13, "s", "2", "a", "g"])
    brazo = BrazoFalso()
    assert calibracion.calibrar_montaje(brazo) is True
    assert config.ARTICULAR_BASE == pytest.approx(93.0)
    assert config.ARTICULAR_SIGNO_PROFUNDIDAD == 1.0
    assert config.MAPEO["codo"][0] == pytest.approx(-1.0)
    assert brazo.velocidad == 2.0                                   # la velocidad vuelve a la de antes
    assert all(postura_segura(m[:4]) for m in brazo.movimientos)    # nunca toca la pared ni la base
    assert brazo.ultimo[:4] == pytest.approx(postura_inicial())     # termina colgando, en el estado inicial
    assert brazo.ultimo[0] == pytest.approx(93.0)                   # ... con la base recién calibrada


def test_calibrar_montaje_cancelado_no_cambia_nada(ajustes_temporales, monkeypatch):
    antes = (config.ARTICULAR_BASE, dict(config.MAPEO))
    teclas(monkeypatch, calibracion, ["d", 13, "n", "1", "d", "d", 27])
    brazo = BrazoFalso()
    assert calibracion.calibrar_montaje(brazo) is False
    assert (config.ARTICULAR_BASE, dict(config.MAPEO)) == antes
    assert brazo.ultimo[:4] == pytest.approx(postura_inicial())
    assert not ajustes_temporales.exists()


def test_la_prueba_de_profundidad_gira_la_base(ajustes_temporales):
    plano = calibracion.postura_referencia("plano", 90.0)
    prueba = calibracion.postura_referencia("profundidad", 90.0)
    assert prueba["base"] - plano["base"] == pytest.approx(calibracion.GIRO_PRUEBA_PROFUNDIDAD)
    ceros = calibracion.postura_referencia("ceros", 90.0)
    assert (ceros["hombro"], ceros["codo"]) == (90.0, -90.0)                 # perpendicular a la pared y codo a 90
    servos = calibracion.servos_seguros(ceros)
    assert all(30 <= s <= 150 for s in servos[1:])                           # todos lejos de sus topes


# ---------------------------------------------------------------- calibrar garra
def test_calibrar_garra(ajustes_temporales, monkeypatch):
    teclas(monkeypatch, calibracion, ["1", "a", "a", "a", "c", "d", "o", 27])
    brazo = BrazoFalso()
    brazo.ultimo[4] = 50.0
    calibracion.calibrar_garra(brazo)
    assert ajustes.garra_cerrada() == 47.0 and ajustes.garra_abierta() == 48.0
    assert brazo.ultimo[4] == 48.0


# ---------------------------------------------------------------- enseñar las 4 marcas sin cámara
def test_calibrar_manual_sin_camara(tmp_path, monkeypatch, ajustes_temporales):
    from brazo_robot.vision import base as vision_base
    from brazo_robot.vision import ensenanza
    monkeypatch.setattr(vision_base, "CALIB_ARCHIVO", str(tmp_path / "calibracion.json"))
    monkeypatch.setattr(ensenanza, "ENSEÑANZA_ARCHIVO", str(tmp_path / "ensenanza.json"))
    monkeypatch.setattr(ensenanza, "PUNTOS_MM", [(150, -100), (150, 100), (250, 100), (250, -100)])
    vision = ensenanza.VisionEnsenanza(None)
    assert vision.corregir_xy(200, 0) == (200, 0)                   # sin muestras, sin corrección
    # en cada marca: aproximar, ir a la Z objetivo, corregir 2 mm en X y guardar
    teclas(monkeypatch, ensenanza, ["n", "z", "1", "w", "w", 13] * 4)
    monkeypatch.setattr(ensenanza.cv2, "setMouseCallback", lambda *a: None)
    monkeypatch.setattr(ensenanza.cv2, "destroyAllWindows", lambda *a: None)
    vision.ensenar(BrazoFalso(), piezas=False)
    activas = vision.muestras_activas()
    assert len(activas) == 4 and all(m["plano"] is None and m["firma"]["montaje"] == "costado" for m in activas)
    x, y = vision.corregir_xy(200, 0)
    assert x == pytest.approx(202.0, abs=0.01) and y == pytest.approx(0.0, abs=0.01)
    config.MONTAJE = "de_pie"                                       # otro montaje: esas marcas ya no valen
    assert vision.muestras_activas() == []


def test_calibrar_piezas_sin_camara_avisa(tmp_path, monkeypatch):
    from brazo_robot.vision import base as vision_base
    from brazo_robot.vision import ensenanza
    monkeypatch.setattr(vision_base, "CALIB_ARCHIVO", str(tmp_path / "calibracion.json"))
    monkeypatch.setattr(ensenanza, "ENSEÑANZA_ARCHIVO", str(tmp_path / "ensenanza.json"))
    with pytest.raises(ValueError, match="cámara"):
        ensenanza.VisionEnsenanza(None).ensenar(BrazoFalso(), piezas=True)


# ---------------------------------------------------------------- trabajar: servidor + consola + operador
@pytest.fixture
def servidor_simulado(monkeypatch):
    from brazo_robot.remoto.servidor import ServidorRobot
    from brazo_robot.robot import Brazo
    monkeypatch.setattr(Brazo, "mover", Brazo.mover_continuo)
    return ServidorRobot(Brazo(simular=True), "token", "127.0.0.1", 0)


def test_la_consola_ordena_dentro_del_servidor_y_salir_termina(servidor_simulado, monkeypatch):
    s = servidor_simulado
    lineas = iter(["garra cerrar", "salir"])
    monkeypatch.setattr("builtins.input", lambda prompt="": next(lineas))
    hilo = threading.Thread(target=trabajar._consola, args=(s, s.brazo, None, None), daemon=True)
    hilo.start()
    hilo_servidor = threading.Thread(target=s.ejecutar, daemon=True)
    hilo_servidor.start()
    hilo_servidor.join(5)
    assert s.detenido and not hilo_servidor.is_alive()
    assert s.brazo.ultimo[4] == pytest.approx(ajustes.garra_cerrada())            # 'garra cerrar'
    assert s.brazo.ultimo[:4] == pytest.approx(postura_inicial())                 # salir: estado inicial sin soltar


def test_fin_de_entrada_detiene_todo(servidor_simulado, monkeypatch):
    def sin_entrada(prompt=""):
        raise EOFError
    monkeypatch.setattr("builtins.input", sin_entrada)
    trabajar._consola(servidor_simulado, servidor_simulado.brazo, None, None)
    assert servidor_simulado.detenido


def test_el_operador_se_abre_en_otro_proceso_contra_esta_maquina(monkeypatch):
    lanzado = {}

    class Proceso:
        stdout = iter(["W0000 ruido\n", "Webcam: 0\n"])

        def poll(self):
            return None

        def wait(self, timeout=None):
            return 0

    def popen(orden, **opciones):
        lanzado.update(orden=orden, **opciones)
        return Proceso()
    monkeypatch.setattr(trabajar.subprocess, "Popen", popen)
    trabajar.Operador(8765, "secreto", ["--webcam", "1"]).abrir()
    assert lanzado["orden"][-7:] == ["operador", "--servidor", "127.0.0.1", "--puerto", "8765", "--webcam", "1"]
    assert lanzado["env"]["BRAZO_TOKEN"] == "secreto"
    assert lanzado["stdin"] == trabajar.subprocess.DEVNULL          # la terminal es de la consola


def test_sin_arduino_explica_que_hacer(monkeypatch, capsys):
    monkeypatch.setattr(trabajar, "cargar_ajustes", lambda: None)
    monkeypatch.setattr(trabajar, "cargar_colores", lambda: None)
    with pytest.raises(SystemExit) as salida:
        trabajar.ejecutar_trabajo(["--puerto-serial", "/dev/no-existe"])
    assert "No encuentro el Arduino" in str(salida.value) and "--simular" in str(salida.value)


# ---------------------------------------------------------------- main.py sin comando
@pytest.fixture
def arranque_en_terminal(monkeypatch):
    llamadas = []
    monkeypatch.setattr(arranque.sys.stdin, "isatty", lambda: True)
    monkeypatch.setattr(arranque, "en_el_entorno", lambda: True)
    monkeypatch.setattr(arranque, "buscar_conda", lambda: None)
    monkeypatch.setattr(arranque, "ejecutar_en_entorno", lambda c, a: llamadas.append((c, list(a))))
    return llamadas


@pytest.mark.parametrize("argumentos, esperado", [([], ("trabajar", [])), (["--simular"], ("trabajar", ["--simular"])),
                                                  (["simulador"], ("simulador", []))])
def test_sin_comando_trabaja_con_el_robot(arranque_en_terminal, argumentos, esperado):
    arranque.main(argumentos)
    assert arranque_en_terminal == [esperado]


def test_con_el_servicio_activo_abre_solo_el_operador(monkeypatch):
    monkeypatch.delenv("BRAZO_SERVICIO", raising=False)
    monkeypatch.setattr(arranque, "estado_servicio", lambda nombre="brazo": "active")
    abierto = []
    import app_operador
    monkeypatch.setattr(app_operador, "main", lambda argv: abierto.append(argv))
    arranque.ejecutar_en_entorno("trabajar", ["--simular"])
    assert abierto == [[]]


def test_el_servidor_atiende_tareas_de_otros_hilos(servidor_simulado):
    hechas = []
    servidor_simulado.en_hilo_principal(lambda: hechas.append(threading.current_thread().name))
    hilo = threading.Thread(target=servidor_simulado.ejecutar, name="principal", daemon=True)
    hilo.start()
    fin = time.monotonic() + 3
    while not hechas and time.monotonic() < fin:
        time.sleep(0.01)
    servidor_simulado.pedir_parada()
    hilo.join(3)
    assert hechas == ["principal"]


def test_una_orden_de_la_consola_pausa_el_seguimiento(servidor_simulado):
    s = servidor_simulado
    s.control.estado = "SIGUIENDO"
    s.control.pose = (200.0, 0.0, 50.0)
    hecho = []
    s.en_hilo_principal(lambda: hecho.append(s.control.estado))
    s._atender_tareas()
    assert hecho == ["PAUSADO"]                                     # ya estaba en pausa al ejecutarse
    assert "consola" in s.control.aviso and s.control.pose is None
