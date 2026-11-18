"""python main.py: entorno automático, marca del servicio y comandos."""

import os
import subprocess
import sys

import pytest

from brazo_robot import arranque

RAIZ = arranque.RAIZ


def test_el_arranque_solo_usa_la_biblioteca_estandar():
    """Corre con el Python del entorno base, que no tiene numpy ni OpenCV."""
    codigo = ("import sys\n"
              "for m in ('numpy', 'cv2', 'mediapipe', 'serial'): sys.modules[m] = None\n"
              "import brazo_robot.arranque\n"
              "print(sorted(m for m in sys.modules if m.split('.')[0] in ('numpy', 'cv2', 'mediapipe', 'serial')"
              " and sys.modules[m] is not None))")
    salida = subprocess.run([sys.executable, "-c", codigo], cwd=RAIZ, capture_output=True, text=True, check=True)
    assert salida.stdout.strip() == "[]"


def test_ayuda_y_comando_desconocido():
    ayuda = subprocess.run([sys.executable, "main.py", "--help"], cwd=RAIZ, capture_output=True, text=True)
    assert ayuda.returncode == 0 and "simulador" in ayuda.stdout and "desinstalar" in ayuda.stdout
    malo = subprocess.run([sys.executable, "main.py", "volar"], cwd=RAIZ, capture_output=True, text=True)
    assert malo.returncode == 2 and "desconocido" in malo.stdout


def test_sin_terminal_muestra_la_ayuda_en_vez_del_menu():
    salida = subprocess.run([sys.executable, "main.py"], cwd=RAIZ, capture_output=True, text=True,
                            stdin=subprocess.DEVNULL)
    assert salida.returncode == 0 and "Uso:" in salida.stdout


def test_el_entorno_tiene_un_nombre_nuevo_y_unico():
    with open(arranque.ARCHIVO_ENTORNO) as f:
        texto = f.read()
    assert "name: brazo-gemelo" in texto
    assert arranque.ENTORNO == "brazo-gemelo" and arranque.ENTORNO not in arranque.ENTORNOS_VIEJOS
    assert not os.path.exists(os.path.join(RAIZ, "environment-raspberry.yml"))


def test_detecta_si_ya_corre_dentro_del_entorno(monkeypatch):
    monkeypatch.setattr(sys, "prefix", "/x/envs/brazo-gemelo")
    assert arranque.en_el_entorno()
    monkeypatch.setattr(sys, "prefix", "/x/envs/brazo")
    assert not arranque.en_el_entorno()


def test_huella_del_entorno(tmp_path):
    assert not arranque.al_dia(str(tmp_path))
    (tmp_path / arranque.HUELLA).write_text(arranque.huella_archivo_entorno() + "\n")
    assert arranque.al_dia(str(tmp_path))
    (tmp_path / arranque.HUELLA).write_text("otra\n")
    assert not arranque.al_dia(str(tmp_path))


def test_buscar_conda_respeta_conda_exe(tmp_path, monkeypatch):
    falso = tmp_path / "conda"
    falso.write_text("#!/bin/sh\n")
    falso.chmod(0o755)
    monkeypatch.setenv("CONDA_EXE", str(falso))
    assert arranque.buscar_conda() == str(falso)


def test_preparar_entorno_crea_actualiza_o_no_toca(tmp_path, monkeypatch):
    llamadas, prefijo = [], tmp_path / "envs" / "brazo-gemelo"
    existentes = {}

    def correr(orden, check=True, **_):
        llamadas.append(orden[1:3])
        if orden[1:3] == ["env", "create"]:
            prefijo.mkdir(parents=True)
            existentes[arranque.ENTORNO] = str(prefijo)
    monkeypatch.setattr(arranque.subprocess, "run", correr)
    monkeypatch.setattr(arranque, "entornos", lambda conda: dict(existentes))
    assert arranque.preparar_entorno("conda") == str(prefijo)            # no existía: se crea
    assert llamadas == [["env", "create"]] and arranque.al_dia(str(prefijo))
    llamadas.clear()
    arranque.preparar_entorno("conda")                                   # al día: nada
    assert llamadas == []
    (prefijo / arranque.HUELLA).write_text("vieja\n")
    arranque.preparar_entorno("conda")                                   # environment.yml cambió: update
    assert llamadas == [["env", "update"]] and arranque.al_dia(str(prefijo))


def test_no_hay_bucle_si_el_python_del_entorno_falla(monkeypatch):
    monkeypatch.setenv(arranque.VARIABLE_ARRANQUE, "1")
    with pytest.raises(arranque.ErrorArranque, match="no arrancó"):
        arranque.reejecutar_en_entorno(["simulador"])


def test_leer_la_marca_del_servicio(tmp_path, monkeypatch):
    marca = tmp_path / ".servicio_instalado"
    monkeypatch.setattr(arranque, "MARCA_SERVICIO", str(marca))
    assert arranque.leer_marca() is None
    marca.write_text("FECHA=2026-11-17 10:00:00\nRAMA=main\nDIR_APP=/home/pi/brazo\n")
    assert arranque.leer_marca() == {"FECHA": "2026-11-17 10:00:00", "RAMA": "main", "DIR_APP": "/home/pi/brazo"}


def test_la_marca_no_se_sube_al_repo():
    salida = subprocess.run(["git", "check-ignore", "scripts/.servicio_instalado"], cwd=RAIZ, capture_output=True, text=True)
    assert salida.stdout.strip() == "scripts/.servicio_instalado"


@pytest.mark.parametrize("comando", ["servidor", "consola"])
def test_no_compite_por_el_arduino_con_el_servicio(comando, monkeypatch):
    monkeypatch.delenv("BRAZO_SERVICIO", raising=False)
    monkeypatch.setattr(arranque, "estado_servicio", lambda nombre="brazo": "active")
    with pytest.raises(arranque.ErrorArranque, match="systemctl stop brazo"):
        arranque.ejecutar_en_entorno(comando, [])


def test_el_propio_servicio_si_puede_arrancar_el_servidor(monkeypatch):
    monkeypatch.setenv("BRAZO_SERVICIO", "1")
    monkeypatch.setattr(arranque, "estado_servicio", lambda nombre="brazo": "active")
    assert not arranque.servicio_ocupa_el_arduino()


def test_el_simulador_no_choca_con_el_servicio(monkeypatch):
    monkeypatch.setattr(arranque, "estado_servicio", lambda nombre="brazo": "active")
    llamado = []
    import simulador
    monkeypatch.setattr(simulador, "main", lambda argv: llamado.append(argv))
    arranque.ejecutar_en_entorno("simulador", ["--webcam", "1"])
    assert llamado == [["--webcam", "1"]]


def test_instalar_fuera_de_la_raspberry_avisa(monkeypatch):
    monkeypatch.setattr(arranque, "hay_systemd", lambda: False)
    with pytest.raises(arranque.ErrorArranque, match="Raspberry"):
        arranque.comando_instalar([])


def test_instalar_dos_veces_avisa_que_ya_esta(monkeypatch, capsys):
    monkeypatch.setattr(arranque, "hay_systemd", lambda: True)
    monkeypatch.setattr(arranque, "leer_marca", lambda: {"FECHA": "hoy", "RAMA": "main"})
    arranque.comando_instalar([])
    assert "ya está instalado" in capsys.readouterr().out


@pytest.mark.parametrize("script", ["instalar_autodeploy.sh", "actualizar.sh", "desinstalar.sh"])
def test_los_scripts_no_tienen_errores_de_sintaxis(script):
    subprocess.run(["bash", "-n", os.path.join(RAIZ, "scripts", script)], check=True)


def test_el_servicio_arranca_por_main_py():
    with open(os.path.join(RAIZ, "scripts", "instalar_autodeploy.sh")) as f:
        texto = f.read()
    assert "main.py servidor" in texto and "BRAZO_SERVICIO=1" in texto and 'ENTORNO="brazo-gemelo"' in texto
    assert ".servicio_instalado" in texto and "environment.yml" in texto


@pytest.mark.parametrize("respuesta, esperado", [("1", "trabajar"), ("2", "simulador"), ("6", "pruebas"),
                                                 ("10", "desinstalar"), ("estado", "estado"), ("0", None),
                                                 ("x", None), ("99", None)])
def test_menu(respuesta, esperado, monkeypatch):
    monkeypatch.setattr("builtins.input", lambda prompt="": respuesta)
    assert arranque.menu() == esperado
