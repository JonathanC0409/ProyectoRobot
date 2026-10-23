"""Rutas locales estables, independientes del directorio de ejecución."""

from pathlib import Path


DEFAULT_MODELS_DIR = Path(__file__).resolve().parents[1] / "models"
POSE_FILENAME = "pose_landmarker_lite.task"
HAND_FILENAME = "hand_landmarker.task"
