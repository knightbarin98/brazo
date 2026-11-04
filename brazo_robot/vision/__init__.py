"""Todo lo que usa la cámara que mira la mesa."""

from .camara import Camara, detectar_webcam, fuente_de_texto, listar_camaras
from .ensenanza import VisionEnsenanza as Vision

__all__ = ["Camara", "Vision", "detectar_webcam", "fuente_de_texto", "listar_camaras"]
