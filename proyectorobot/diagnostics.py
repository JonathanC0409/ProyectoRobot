"""Inspección local sin abrir dispositivos, ventanas o conexiones de red."""

from importlib import metadata
import os
from pathlib import Path
import platform
import sys

from proyectorobot.config import HAND_FILENAME, POSE_FILENAME


OPENCV_PACKAGES = (
    "opencv-python", "opencv-contrib-python",
    "opencv-python-headless", "opencv-contrib-python-headless",
)


def check_environment(models_dir: Path) -> int:
    healthy = True
    print(f"Python: {platform.python_version()} | Ejecutable: {sys.executable}")
    print(f"Plataforma: {platform.system()} {platform.machine()} | libc: {platform.libc_ver()}")
    if sys.version_info[:2] != (3, 12):
        print("ERROR: este proyecto se valida con Python 3.12; usa .venv/bin/python.")
        healthy = False
    if sys.prefix == sys.base_prefix:
        print("AVISO: no estás usando un entorno virtual.")
    for name in ("mediapipe", "numpy", *OPENCV_PACKAGES):
        try:
            print(f"{name}: {metadata.version(name)}")
        except metadata.PackageNotFoundError:
            if name in ("mediapipe", "numpy", "opencv-contrib-python"):
                print(f"ERROR: falta {name}. Consulta requirements-phase1.txt.")
                healthy = False
    installed_opencv = []
    for name in OPENCV_PACKAGES:
        try:
            metadata.version(name)
            installed_opencv.append(name)
        except metadata.PackageNotFoundError:
            pass
    if len(installed_opencv) > 1:
        print("ERROR: varias distribuciones comparten cv2: " + ", ".join(installed_opencv))
        print("Se necesita autorización para corregir el venv; pip check no detecta esta colisión.")
        healthy = False
    try:
        import cv2
        import mediapipe as mp
        import numpy as np
        from mediapipe.tasks.python import vision

        print(f"Imports OK: cv2={cv2.__version__}, mediapipe={mp.__version__}, numpy={np.__version__}")
        print(f"Tasks OK: {vision.PoseLandmarker.__name__}, {vision.HandLandmarker.__name__}")
        for line in cv2.getBuildInformation().splitlines():
            if any(key in line for key in ("GUI:", "QT:", "Wayland", "v4l/v4l2")):
                print(line.strip())
    except (ImportError, OSError, AttributeError) as error:
        print(f"ERROR de importación/API: {error}")
        healthy = False
    devices = sorted(Path("/dev").glob("video*"))
    print("Webcams visibles: " + (", ".join(str(path) for path in devices) or "ninguna"))
    if not devices:
        print("AVISO: no se podrá validar captura aquí; revisa desde la sesión local de Fedora.")
    print(f"Sesión: {os.environ.get('XDG_SESSION_TYPE', 'desconocida')}")
    print(f"DISPLAY: {os.environ.get('DISPLAY', 'ausente')}")
    print(f"WAYLAND_DISPLAY: {os.environ.get('WAYLAND_DISPLAY', 'ausente')}")
    for filename in (POSE_FILENAME, HAND_FILENAME):
        path = models_dir / filename
        if path.is_file() and path.stat().st_size > 0:
            print(f"Modelo: {path} ({path.stat().st_size} bytes)")
        else:
            print(f"ERROR: falta modelo {path}; descarga explícita indicada en README.md.")
            healthy = False
    return 0 if healthy else 1
