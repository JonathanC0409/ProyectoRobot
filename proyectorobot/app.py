"""Entrada de Fase 1: captura → percepción → estados → pantalla."""

import argparse
import logging
import os
from pathlib import Path
import sys
import time

from proyectorobot.config import DEFAULT_MODELS_DIR
from proyectorobot.control.arms import ArmRecognizer


WINDOW_NAME = "ProyectoRobot - Fase 1"


class FrameClock:
    """Milisegundos monotónicos estrictamente crecientes incluso a FPS altos."""

    def __init__(self) -> None:
        self._origin_ns = time.monotonic_ns()
        self._last_ms = -1

    def next_ms(self) -> int:
        elapsed_ms = (time.monotonic_ns() - self._origin_ns) // 1_000_000
        self._last_ms = max(self._last_ms + 1, elapsed_ms)
        return self._last_ms


def positive_int(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("Se requiere un entero mayor que cero.")
    return number


def camera_index(value: str) -> int:
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("El índice de webcam debe ser >= 0.")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m proyectorobot", description="ProyectoRobot: pose, manos y brazos en CPU.",
    )
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--check", action="store_true", help="Diagnóstico sin webcam, GUI ni red.")
    modes.add_argument("--smoke-test", action="store_true", help="Inferencia sintética sin webcam ni GUI.")
    parser.add_argument("--camera", type=camera_index, default=0, help="Índice V4L2, por defecto 0.")
    parser.add_argument("--width", type=positive_int, default=640)
    parser.add_argument("--height", type=positive_int, default=480)
    parser.add_argument("--fps", type=positive_int, default=20, help="FPS de captura solicitados, no garantizados.")
    parser.add_argument("--window-width", type=positive_int, default=1280,
                        help="Ancho inicial de ventana; no cambia la captura.")
    parser.add_argument("--window-height", type=positive_int, default=720,
                        help="Alto inicial de ventana; no cambia la captura.")
    parser.add_argument("--models-dir", type=Path, default=DEFAULT_MODELS_DIR)
    parser.add_argument("--mirror", action=argparse.BooleanOptionalAction, default=True,
                        help="Espejo de presentación; mantiene lados anatómicos.")
    return parser


def run_camera(args: argparse.Namespace) -> None:
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise RuntimeError("No hay sesión gráfica. Usa --check o ejecuta desde GNOME.")
    if os.environ.get("XDG_SESSION_TYPE") == "wayland" and os.environ.get("DISPLAY"):
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

    import cv2
    from proyectorobot.vision.camera import Camera
    from proyectorobot.vision.detector import Detector
    from proyectorobot.vision.overlay import render

    cv2.setNumThreads(2)
    recognizer = ArmRecognizer()
    clock = FrameClock()
    fps = 0.0
    try:
        with Detector(args.models_dir) as detector:
            with Camera(args.camera, args.width, args.height, args.fps) as camera:
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(WINDOW_NAME, args.window_width, args.window_height)
                logging.info("Captura activa; Q, Escape o cerrar la ventana para salir.")
                previous_end = time.perf_counter()
                while True:
                    frame = camera.read()
                    timestamp_ms = clock.next_ms()
                    inference_start = time.perf_counter()
                    observation = detector.detect(frame, timestamp_ms)
                    inference_ms = (time.perf_counter() - inference_start) * 1000
                    states = recognizer.update(observation)
                    output = render(frame, observation, states, args.mirror, fps, inference_ms)
                    cv2.imshow(WINDOW_NAME, output)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), ord("Q"), 27):
                        break
                    if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                        break
                    frame_end = time.perf_counter()
                    instantaneous_fps = 1.0 / max(frame_end - previous_end, 1e-9)
                    fps = instantaneous_fps if fps == 0 else 0.9 * fps + 0.1 * instantaneous_fps
                    previous_end = frame_end
    finally:
        cv2.destroyAllWindows()


def smoke_test(models_dir: Path) -> None:
    import numpy as np
    from proyectorobot.vision.detector import Detector

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    with Detector(models_dir) as detector:
        for timestamp in (0, 50, 100):
            observation = detector.detect(frame, timestamp)
            logging.info("Inferencia sintética t=%s: pose=%s, manos=%s",
                         timestamp, bool(observation.pose), len(observation.hands))
            if observation.pose or observation.hands:
                raise RuntimeError("Detección inesperada en imagen vacía; revisar los modelos.")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    if sys.version_info[:2] != (3, 12):
        logging.error("Usa Python 3.12: .venv/bin/python -m proyectorobot")
        return 1
    try:
        if args.check:
            from proyectorobot.diagnostics import check_environment

            return check_environment(args.models_dir)
        if args.smoke_test:
            smoke_test(args.models_dir)
        else:
            run_camera(args)
    except KeyboardInterrupt:
        logging.info("Interrumpido por el usuario; recursos liberados.")
        return 130
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        logging.error("%s", error)
        logging.error("Consulta README.md; diagnóstico: .venv/bin/python -m proyectorobot --check")
        return 1
    except Exception as error:
        logging.error("Fallo inesperado (%s): %s", type(error).__name__, error)
        return 1
    return 0
