"""Selección de render Mesa por proceso y comprobación del contexto OpenGL real."""

from dataclasses import dataclass
import os
from pathlib import Path
import re


GRAPHICS_MODES = ("auto", "intel", "nvidia", "software")


class GraphicsError(RuntimeError):
    """El renderizador solicitado no está disponible o no se activó."""


def find_gpu(vendor: str, devices: Path = Path("/sys/bus/pci/devices")) -> str:
    """Encuentra la dirección PCI de una GPU sin ejecutar utilidades del sistema."""
    identifier = {"intel": "0x8086", "nvidia": "0x10de"}[vendor]
    for device in sorted(devices.glob("*")):
        try:
            if ((device / "class").read_text().strip().startswith("0x03")
                    and (device / "vendor").read_text().strip() == identifier):
                return "pci-" + device.name.replace(":", "_").replace(".", "_")
        except OSError:
            continue
    raise GraphicsError(f"No se detecta GPU {vendor}. Prueba --graphics auto o --graphics software.")


def configure_graphics(mode: str) -> None:
    """Aplicar antes de abrir OpenGL; no modifica controladores ni archivos del sistema.

    Auto respeta las preferencias externas y deja elegir a Mesa/GNOME. Los
    modos explícitos seleccionan una GPU PCI o fuerzan el respaldo software.
    """
    if mode not in GRAPHICS_MODES:
        raise ValueError(f"Modo gráfico desconocido: {mode}.")
    if mode in ("intel", "nvidia"):
        address = find_gpu(mode)
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "0"
        os.environ["DRI_PRIME"] = address
    elif mode == "software":
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
    os.environ.setdefault("LP_NUM_THREADS", "2")
    os.environ["PROYECTOROBOT_GRAPHICS"] = mode


@dataclass(frozen=True)
class GraphicsInfo:
    vendor: str
    renderer: str
    version: str

    @property
    def device(self) -> str:
        renderer = self.renderer.lower()
        combined = (self.vendor + " " + self.renderer).lower()
        if any(marker in renderer for marker in (
            "llvmpipe", "softpipe", "swrast", "software", "swiftshader", "lavapipe", "basic render",
        )):
            return "software"
        if "intel" in combined:
            return "intel"
        if "nvidia" in combined or "nouveau" in combined or re.search(r"\bnv[0-9a-f]{2,}\b", renderer):
            return "nvidia"
        return "unknown"

    @property
    def label(self) -> str:
        return {"software": "CPU software", "intel": "GPU Intel", "nvidia": "GPU NVIDIA"}.get(
            self.device, "OpenGL: " + self.renderer,
        )


def inspect_context(gl, requested: str = "auto") -> GraphicsInfo:
    """Leer la GPU usada, sin confundir proveedor Mesa con renderizador software."""
    def string(parameter: int) -> str:
        value = gl.glGetString(parameter)
        if not value:
            raise GraphicsError("OpenGL no devuelve un contexto válido; prueba --graphics software.")
        return value.decode("utf-8", errors="replace") if isinstance(value, bytes) else str(value)

    info = GraphicsInfo(string(gl.GL_VENDOR), string(gl.GL_RENDERER), string(gl.GL_VERSION))
    if requested != "auto" and info.device != requested:
        raise GraphicsError(
            f"Se solicitó {requested}, pero OpenGL usa {info.renderer}. "
            "Prueba --graphics auto o --graphics software. No se cambiaron controladores."
        )
    return info


def default_draw_target(info: GraphicsInfo) -> str:
    """NV118 evita la copia offscreen que falló durante el redimensionado."""
    direct = info.device == "nvidia" and re.search(r"\bnv118\b", info.renderer.lower())
    return "window" if direct else "offscreen"


def require_stable_context(info: GraphicsInfo, draw_target: str = "offscreen") -> None:
    """Evitar la copia offscreen NV118 que falló en el redimensionado real.

    Un fallo nativo del controlador no se puede capturar con try/except de
    Python. Las herramientas de diagnóstico pueden probarlo de forma aislada;
    el visor normal rechaza esa copia y utiliza dibujo directo a la ventana.
    """
    if draw_target not in ("window", "offscreen"):
        raise ValueError("Destino de dibujo inválido.")
    if default_draw_target(info) == "window" and draw_target == "offscreen":
        raise GraphicsError(
            "La copia offscreen de NVIDIA NV118/MX130 falló al redimensionar el visor. "
            "Este modo no está habilitado para uso normal. Usa dibujo directo, --graphics intel "
            "o --graphics software. No se modificaron controladores."
        )
