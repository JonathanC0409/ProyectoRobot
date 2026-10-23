"""MediaPipe Tasks en CPU; inferencia síncrona de ambos modelos por frame."""

from contextlib import ExitStack
from pathlib import Path
from types import TracebackType
from typing import Self

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from proyectorobot.config import DEFAULT_MODELS_DIR, HAND_FILENAME, POSE_FILENAME
from proyectorobot.data.types import Landmark, Observation


def model_paths(directory: Path, include_hands: bool = True) -> tuple[Path, Path]:
    paths = (directory / POSE_FILENAME, directory / HAND_FILENAME)
    required = paths if include_hands else paths[:1]
    missing = [str(path) for path in required if not path.is_file() or path.stat().st_size == 0]
    if missing:
        raise FileNotFoundError(
            "Faltan modelos: " + ", ".join(missing)
            + ". Consulta README.md y scripts/download_models.py; no se descargan automáticamente."
        )
    return paths


def convert_landmarks(points: list, require_confidence: bool) -> tuple[Landmark, ...]:
    default = 0.0 if require_confidence else 1.0
    return tuple(
        Landmark(
            x=point.x, y=point.y, z=point.z,
            visibility=point.visibility if point.visibility is not None else default,
            presence=point.presence if point.presence is not None else default,
        )
        for point in points
    )


def convert_world_landmarks(points: list, image_points: list) -> tuple[Landmark, ...]:
    """Conserva scores world explícitos; completa solo ausentes del mismo punto.

    Tasks admite scores opcionales en world. La ausencia no equivale a confianza
    cero ni uno: se usa el score de imagen del mismo landmark si está disponible.
    El control sigue exigiendo imagen visible y geometría world coherente.
    """
    landmarks = []
    for index, point in enumerate(points):
        image = image_points[index] if index < len(image_points) else None
        scores = []
        for field in ("visibility", "presence"):
            score = getattr(point, field, None)
            if score is None:
                score = getattr(image, field, None)
            scores.append(score if score is not None else 0.0)
        landmarks.append(Landmark(point.x, point.y, point.z, *scores))
    return tuple(landmarks)


class Detector:
    """VIDEO permite tracking y resultados alineados sin callbacks concurrentes."""

    def __init__(self, models_dir: Path = DEFAULT_MODELS_DIR, include_hands: bool = True) -> None:
        self.models_dir = models_dir
        self.include_hands = include_hands
        self._stack = ExitStack()
        self._pose = None
        self._hands = None
        self._last_ms = -1

    def __enter__(self) -> Self:
        pose_path, hand_path = model_paths(self.models_dir, self.include_hands)
        try:
            self._pose = self._stack.enter_context(vision.PoseLandmarker.create_from_options(
                vision.PoseLandmarkerOptions(
                    base_options=python.BaseOptions(
                        model_asset_path=str(pose_path), delegate=python.BaseOptions.Delegate.CPU,
                    ),
                    running_mode=vision.RunningMode.VIDEO,
                    num_poses=1, output_segmentation_masks=False,
                )
            ))
            if self.include_hands:
                self._hands = self._stack.enter_context(vision.HandLandmarker.create_from_options(
                    vision.HandLandmarkerOptions(
                        base_options=python.BaseOptions(
                            model_asset_path=str(hand_path), delegate=python.BaseOptions.Delegate.CPU,
                        ),
                        running_mode=vision.RunningMode.VIDEO, num_hands=2,
                    )
                ))
        except BaseException:
            self._stack.close()
            raise
        return self

    def detect(self, frame_bgr: np.ndarray, timestamp_ms: int) -> Observation:
        if self._pose is None or (self.include_hands and self._hands is None):
            raise RuntimeError("Usa Detector dentro de un bloque with.")
        if timestamp_ms < 0 or timestamp_ms <= self._last_ms:
            raise ValueError("MediaPipe requiere timestamps no negativos y crecientes.")
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
        pose_result = self._pose.detect_for_video(image, timestamp_ms)
        hand_result = self._hands.detect_for_video(image, timestamp_ms) if self._hands is not None else None
        self._last_ms = timestamp_ms
        world = getattr(pose_result, "pose_world_landmarks", ())
        return Observation(
            timestamp_ms=timestamp_ms,
            pose=convert_landmarks(pose_result.pose_landmarks[0], True)
            if pose_result.pose_landmarks else (),
            hands=tuple(convert_landmarks(points, False) for points in hand_result.hand_landmarks)
            if hand_result is not None else (),
            pose_world=convert_world_landmarks(
                world[0], pose_result.pose_landmarks[0] if pose_result.pose_landmarks else [],
            ) if world else (),
        )

    def __exit__(
        self, exc_type: type[BaseException] | None,
        exc_value: BaseException | None, traceback: TracebackType | None,
    ) -> None:
        try:
            self._stack.close()
        finally:
            self._pose = None
            self._hands = None
