"""Captura aislada: una inferencia bloqueada no detiene el control del robot."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import multiprocessing
import os
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Thread
import time
from types import TracebackType
from typing import Self

from proyectorobot.data.types import Observation
from proyectorobot.control.motion import ArmTargets

@dataclass(frozen=True)
class VisionConfig:
    camera: int
    width: int
    height: int
    fps: int
    models_dir: Path
    show_camera: bool = False
    window_width: int = 640
    window_height: int = 480
    mirror: bool = True
    track_hands: bool = False
    recovery_ms: int = 80


@dataclass(frozen=True)
class VisionPacket:
    observation: Observation
    captured_at: float
    inference_ms: float
    targets: ArmTargets = ArmTargets()
    status: str = "INICIANDO VISION"
    generation: int = 0
    fps: float = 0.0


class VisionWorkerError(RuntimeError):
    """Error del proceso que posee la webcam y los modelos de visión."""


class VisionStreamClosed(Exception):
    """El usuario cerró normalmente la ventana de cámara."""


def offer_latest(queue, packet: VisionPacket) -> None:
    """Descarta resultados antiguos; nunca espera a que el consumidor esté libre."""
    try:
        queue.put_nowait(packet)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            return
        try:
            queue.put_nowait(packet)
        except Full:
            pass


def limit_frame_size(frame, width: int, height: int):
    """Acota captura real conservando proporciones aunque el driver ignore la resolución."""
    scale = min(1.0, width / frame.shape[1], height / frame.shape[0])
    if scale == 1.0:
        return frame
    import cv2

    return cv2.resize(frame, (max(1, round(frame.shape[1] * scale)), max(1, round(frame.shape[0] * scale))),
                      interpolation=cv2.INTER_AREA)


def capture_worker(config: VisionConfig, queue, stop, errors, generation=None) -> None:
    window_created = False
    try:
        logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
        if os.environ.get("XDG_SESSION_TYPE") == "wayland" and os.environ.get("DISPLAY"):
            os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
        import cv2
        from proyectorobot.control.arms import ArmRecognizer
        from proyectorobot.control.retargeting import PoseRetargeter, RetargetResult
        from proyectorobot.vision.camera import CameraStream
        from proyectorobot.vision.detector import Detector
        from proyectorobot.vision.overlay import render

        cv2.setNumThreads(2)
        last_timestamp_ms = -1
        retargeter = PoseRetargeter(recovery_ms=config.recovery_ms)
        recognizer = ArmRecognizer(allow_upper_body=True, hold_ms=config.recovery_ms)
        current_generation = 0
        report_start = time.monotonic()
        reported_frames = 0
        total_inference_ms = total_read_ms = total_display_ms = 0.0
        fps = 0.0
        previous_capture = None
        camera_window = "ProyectoRobot - Camara e imitacion"
        with Detector(config.models_dir, include_hands=config.track_hands) as detector:
            capture_scale = max(1.0, 640 / config.width)
            capture_width = round(config.width * capture_scale)
            capture_height = round(config.height * capture_scale)
            with CameraStream(config.camera, capture_width, capture_height, config.fps) as camera:
                if config.show_camera:
                    cv2.namedWindow(camera_window, cv2.WINDOW_NORMAL)
                    window_created = True
                    cv2.resizeWindow(camera_window, config.window_width, config.window_height)
                while not stop.is_set():
                    cycle_start = time.monotonic()
                    requested_generation = generation.value if generation is not None else 0
                    if requested_generation != current_generation:
                        current_generation = requested_generation
                        retargeter.recalibrate()
                    captured = camera.read()
                    frame = limit_frame_size(captured.image, config.width, config.height)
                    captured_at = captured.captured_at
                    inference_start = time.monotonic()
                    timestamp_ms = max(last_timestamp_ms + 1, int(captured_at * 1000))
                    observation = detector.detect(frame, timestamp_ms)
                    last_timestamp_ms = timestamp_ms
                    inference_ms = (time.monotonic() - inference_start) * 1000
                    was_calibrated = retargeter.calibrated
                    result = (RetargetResult(ArmTargets(), "REPOSO: C PARA CALIBRAR")
                              if current_generation < 0 else
                              retargeter.update(observation, frame.shape[1], frame.shape[0]))
                    if retargeter.calibrated and not was_calibrated:
                        logging.info("Calibración completada; imitación de brazos habilitada.")
                    if previous_capture is not None:
                        instant = 1 / max(captured_at - previous_capture, 1e-9)
                        fps = instant if fps == 0 else 0.85 * fps + 0.15 * instant
                    previous_capture = captured_at
                    offer_latest(queue, VisionPacket(
                        observation, captured_at, inference_ms, result.targets,
                        result.status, current_generation, fps,
                    ))
                    display_start = time.monotonic()
                    if config.show_camera:
                        states = recognizer.update(observation)
                        output = render(
                            frame, observation, states, config.mirror, fps, inference_ms,
                            extra_lines=(f"Robot: {result.status}", "R: reposo | C: calibrar | Sin grabacion"),
                            compact=True, hands_enabled=config.track_hands,
                        )
                        cv2.imshow(camera_window, output)
                        key = cv2.waitKey(1) & 0xFF
                        if key in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(
                            camera_window, cv2.WND_PROP_VISIBLE,
                        ) < 1:
                            errors.send("__CLOSED__")
                            break
                        if key in (ord("c"), ord("C")) and generation is not None:
                            retargeter.recalibrate()
                            errors.send("__RECALIBRATE__")
                        if key in (ord("r"), ord("R")) and generation is not None:
                            retargeter.recalibrate()
                            errors.send("__RESET__")
                    reported_frames += 1
                    report_time = time.monotonic()
                    total_inference_ms += inference_ms
                    total_read_ms += (inference_start - cycle_start) * 1000
                    total_display_ms += (report_time - display_start) * 1000
                    if report_time - report_start >= 2:
                        logging.info("Visión CPU: %.1f FPS, medias: inferencia %.0f ms, lectura %.0f ms, panel %.0f ms.",
                                     reported_frames / (report_time - report_start),
                                     total_inference_ms / reported_frames,
                                     total_read_ms / reported_frames,
                                     total_display_ms / reported_frames)
                        report_start = report_time
                        reported_frames = 0
                        total_inference_ms = total_read_ms = total_display_ms = 0.0
                    stop.wait(max(0.0, 1 / config.fps - (time.monotonic() - cycle_start)))
    except Exception as error:
        try:
            errors.send(f"{type(error).__name__}: {str(error)[:700]}")
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        if window_created:
            cv2.destroyAllWindows()
        errors.close()
        queue.cancel_join_thread()
        queue.close()


class VisionStream:
    def __init__(self, config: VisionConfig) -> None:
        context = multiprocessing.get_context("spawn")
        self._queue = context.Queue(maxsize=1)
        self._latest = Queue(maxsize=1)
        self._receive_failed = Event()
        self._stop = context.Event()
        self._generation = context.RawValue("i", 0)
        self._errors, self._sender = context.Pipe(duplex=False)
        self._process = context.Process(
            target=capture_worker, args=(config, self._queue, self._stop, self._sender, self._generation),
            name="ProyectoRobotVision", daemon=True,
        )
        self._started = False
        self.received_count = 0
        self._receiver = Thread(target=self._receive, name="ProyectoRobotIPC", daemon=True)

    def _receive(self) -> None:
        while not self._stop.is_set():
            try:
                packet = self._queue.get(timeout=0.1)
            except Empty:
                continue
            except Exception as error:
                if not self._stop.is_set():
                    logging.error("Recepción IPC (%s): %s", type(error).__name__, error)
                    self._receive_failed.set()
                return
            offer_latest(self._latest, packet)
            self.received_count += 1

    def __enter__(self) -> Self:
        try:
            self._process.start()
            self._started = True
            self._sender.close()
            self._receiver.start()
            return self
        except BaseException:
            self.close()
            raise

    def poll(self) -> VisionPacket | None:
        if self._errors.poll():
            try:
                message = self._errors.recv()
            except EOFError:
                message = "El proceso de visión terminó inesperadamente."
            if message == "__CLOSED__":
                raise VisionStreamClosed()
            if message == "__RECALIBRATE__":
                self.request_recalibration()
                return None
            if message == "__RESET__":
                self.request_reset()
                return None
            raise VisionWorkerError(message)
        if not self._process.is_alive():
            raise VisionWorkerError(f"El proceso de visión terminó (código {self._process.exitcode}).")
        if self._receive_failed.is_set():
            raise VisionWorkerError("Se interrumpió la comunicación con visión.")
        try:
            packet = self._latest.get_nowait()
            return packet if packet.generation == self.generation else None
        except Empty:
            return None

    @property
    def generation(self) -> int:
        return self._generation.value

    def request_recalibration(self) -> None:
        """Solo el proceso principal escribe esta generación; visión solo la lee."""
        self._generation.value = abs(self._generation.value) + 1

    def request_reset(self) -> None:
        """Generación negativa pausa el control hasta recalibrar con C."""
        self._generation.value = -abs(self._generation.value) - 1

    def close(self) -> None:
        self._stop.set()
        if self._started:
            self._process.join(timeout=2.0)
            if self._process.is_alive():
                logging.warning("Visión no respondió al cierre; terminando solo su proceso.")
                self._process.terminate()
                self._process.join(timeout=1.0)
            if self._process.is_alive():
                self._process.kill()
                self._process.join(timeout=1.0)
        if self._receiver.ident is not None:
            self._receiver.join(timeout=0.5)
        self._errors.close()
        self._sender.close()
        self._queue.close()
        self._queue.join_thread()

    def __exit__(self, exc_type: type[BaseException] | None,
                 exc_value: BaseException | None, traceback: TracebackType | None) -> None:
        self.close()
