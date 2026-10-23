"""Captura V4L2 con liberación garantizada y fallos explícitos."""

import logging
from dataclasses import dataclass
from queue import Empty, Full, Queue
from threading import Event, Thread
import time
from types import TracebackType
from typing import Self

import cv2
import numpy as np


class CameraError(RuntimeError):
    """La cámara no está disponible o dejó de entregar imágenes."""


class Camera:
    def __init__(self, index: int, width: int, height: int, fps: int) -> None:
        if index < 0 or min(width, height, fps) <= 0:
            raise ValueError("Índice de cámara >= 0 y dimensiones/FPS > 0 requeridos.")
        self.index = index
        self.width = width
        self.height = height
        self.fps = fps
        self._capture = None

    def __enter__(self) -> Self:
        try:
            self._capture = cv2.VideoCapture(self.index, cv2.CAP_V4L2)
            if not self._capture.isOpened():
                raise CameraError(
                    f"No se pudo abrir /dev/video{self.index}. Comprueba el índice, "
                    "permisos, acceso desde el contenedor y otras aplicaciones."
                )
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self._capture.set(cv2.CAP_PROP_FPS, self.fps)
            self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            logging.info(
                "Cámara: %.0fx%.0f, FPS declarados: %.1f (solicitados: %s)",
                self._capture.get(cv2.CAP_PROP_FRAME_WIDTH),
                self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT),
                self._capture.get(cv2.CAP_PROP_FPS), self.fps,
            )
            return self
        except BaseException:
            self.close()
            raise

    def read(self) -> np.ndarray:
        if self._capture is None:
            raise CameraError("La cámara no está abierta.")
        success, frame = self._capture.read()
        if not success or frame is None or frame.size == 0:
            raise CameraError("La webcam dejó de entregar imágenes. Cierra y vuelve a ejecutar.")
        return frame

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def __exit__(
        self, exc_type: type[BaseException] | None,
        exc_value: BaseException | None, traceback: TracebackType | None,
    ) -> None:
        self.close()


@dataclass(frozen=True)
class CapturedFrame:
    image: np.ndarray
    captured_at: float


class CameraStream:
    """Lector de Fase 2 con una sola imagen pendiente y cámara propiedad del hilo.

    Solapa espera de V4L2 con inferencia; descarta imágenes antiguas. La cámara
    se abre, lee y libera siempre en el mismo hilo. Si el driver se bloquea,
    cerrar no espera indefinidamente: el proceso de visión sigue bajo la
    vigilancia y terminación acotada de VisionStream.
    """

    def __init__(self, index: int, width: int, height: int, fps: int) -> None:
        self.arguments = index, width, height, fps
        self._frames: Queue[CapturedFrame] = Queue(maxsize=1)
        self._stop = Event()
        self._error: Exception | None = None
        self._received = False
        self._thread = Thread(target=self._capture, name="ProyectoRobotCamera", daemon=True)

    def _capture(self) -> None:
        try:
            with Camera(*self.arguments) as camera:
                while not self._stop.is_set():
                    frame = CapturedFrame(camera.read(), time.monotonic())
                    if self._stop.is_set():
                        break
                    try:
                        self._frames.put_nowait(frame)
                    except Full:
                        try:
                            self._frames.get_nowait()
                        except Empty:
                            pass
                        self._frames.put_nowait(frame)
        except Exception as error:
            self._error = error

    def __enter__(self) -> Self:
        self._thread.start()
        return self

    def read(self) -> CapturedFrame:
        deadline = time.monotonic() + (0.35 if self._received else 5.0)
        while True:
            if self._error is not None:
                raise CameraError(f"Lectura de webcam: {self._error}") from self._error
            remaining = deadline - time.monotonic()
            if remaining <= 0 or self._stop.is_set():
                raise CameraError("La webcam no entrega imágenes recientes; vuelve a abrir el programa.")
            try:
                frame = self._frames.get(timeout=min(0.05, remaining))
            except Empty:
                continue
            self._received = True
            return frame

    def close(self) -> None:
        self._stop.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=1.0)
            if self._thread.is_alive():
                logging.warning("El driver de cámara no respondió al cierre; el proceso de visión finalizará.")

    def __exit__(self, exc_type: type[BaseException] | None,
                 exc_value: BaseException | None, traceback: TracebackType | None) -> None:
        self.close()
