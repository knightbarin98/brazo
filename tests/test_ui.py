"""Detección de la ventana cerrada en los dos backends de OpenCV: GTK (Raspberry) y Cocoa (Mac)."""

import cv2
import pytest

from brazo_robot import ui


def backend(monkeypatch, visible, autosize):
    def propiedad(win, cual):
        valor = visible if cual == cv2.WND_PROP_VISIBLE else autosize
        if valor == "error":
            raise cv2.error("ventana inexistente")
        return valor
    monkeypatch.setattr(ui.cv2, "getWindowProperty", propiedad)


def test_mac_ventana_abierta_no_se_da_por_cerrada(monkeypatch):
    backend(monkeypatch, visible=1.0, autosize=-1.0)             # Cocoa: AUTOSIZE siempre -1
    visto = [False]
    assert not any(ui.ventana_cerrada("v", visto) for _ in range(3))


def test_mac_ventana_cerrada(monkeypatch):
    backend(monkeypatch, visible="error", autosize="error")      # Cocoa: cerrada lanza un error
    assert ui.ventana_cerrada("v", [True])


def test_raspberry_ventana_cerrada(monkeypatch):
    backend(monkeypatch, visible=-1.0, autosize=-1.0)            # GTK: cerrada responde -1
    assert ui.ventana_cerrada("v", [False])


@pytest.mark.parametrize("visto, cerrada", [([False], False), ([True], True)])
def test_visible_cero_solo_cuenta_si_antes_se_vio(monkeypatch, visto, cerrada):
    backend(monkeypatch, visible=0.0, autosize=0.0)              # algunos backends: VISIBLE 0 recién abierta
    assert ui.ventana_cerrada("v", visto) is cerrada
