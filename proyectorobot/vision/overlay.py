"""Dibujo propio usando las conexiones públicas de MediaPipe Tasks."""

import cv2
import numpy as np
from mediapipe.tasks.python import vision

from proyectorobot.control.arms import ArmState, ArmStates
from proyectorobot.data.types import Landmark, Observation


def draw_landmarks(
    frame: np.ndarray, points: tuple[Landmark, ...], connections: list,
    color: tuple[int, int, int], mirror: bool, confidence: float,
) -> None:
    height, width = frame.shape[:2]
    pixels = {
        index: (round((1.0 - point.x if mirror else point.x) * (width - 1)),
                round(point.y * (height - 1)))
        for index, point in enumerate(points) if point.visible(confidence)
    }
    for connection in connections:
        if connection.start in pixels and connection.end in pixels:
            cv2.line(frame, pixels[connection.start], pixels[connection.end], color, 2, cv2.LINE_AA)
    for pixel in pixels.values():
        cv2.circle(frame, pixel, 3, color, -1, cv2.LINE_AA)


def render(
    frame: np.ndarray, observation: Observation, states: ArmStates,
    mirror: bool, fps: float, inference_ms: float, extra_lines: tuple[str, ...] = (),
    compact: bool = False, hands_enabled: bool = True,
) -> np.ndarray:
    output = cv2.flip(frame, 1) if mirror else frame.copy()
    draw_landmarks(output, observation.pose, vision.PoseLandmarksConnections.POSE_LANDMARKS,
                   (70, 220, 70), mirror, 0.6)
    for hand in observation.hands:
        draw_landmarks(output, hand, vision.HandLandmarksConnections.HAND_CONNECTIONS,
                       (0, 200, 255), mirror, 0.0)
    if compact:
        labels = (
            f"Movimiento: {states.movement}",
            f"Izq: {states.left.value} | Der: {states.right.value}",
            f"FPS: {fps:.1f} | Inferencia: {inference_ms:.0f} ms | Dedos: {'SI' if hands_enabled else 'OFF'}",
            *extra_lines,
            "Lados anatomicos | Q / ESC: salir",
        )
        panel = np.full((10 + 20 * len(labels), output.shape[1], 3), 25, dtype=np.uint8)
        for index, label in enumerate(labels):
            text_width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
            scale = min(0.5, 0.5 * max(1, output.shape[1] - 16) / max(1, text_width))
            cv2.putText(panel, label, (8, 18 + index * 20), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, (235, 235, 235), 1, cv2.LINE_AA)
        return np.vstack((output, panel))
    lines = (
        (f"Movimiento: {states.movement}", None),
        (f"Izquierdo: {states.left.value}", states.left),
        (f"Derecho: {states.right.value}", states.right),
        (f"Pose: {'SI' if observation.pose else 'NO'} | Manos: {len(observation.hands)}", None),
        (f"FPS: {fps:.1f} | Inferencia: {inference_ms:.0f} ms | CPU", None),
        ("Lados de la persona | Q / ESC: salir", None),
    ) + tuple((line, None) for line in extra_lines)
    padding = 20
    line_height = 48
    panel_height = max(output.shape[0], 2 * padding + line_height * len(lines))
    panel = np.full((panel_height, 560, 3), 25, dtype=np.uint8)
    for index, (text, state) in enumerate(lines):
        color = (80, 240, 80) if state == ArmState.UP else (235, 235, 235)
        cv2.putText(panel, text, (padding, padding + 24 + index * line_height),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
    output = cv2.copyMakeBorder(
        output, 0, panel_height - output.shape[0], 0, 0,
        cv2.BORDER_CONSTANT, value=(25, 25, 25),
    )
    return np.hstack((output, panel))
