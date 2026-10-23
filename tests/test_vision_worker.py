"""Cola acotada y errores de un proceso real; nunca se abre una webcam."""

from pathlib import Path
from queue import Empty, Queue
from tempfile import TemporaryDirectory
from threading import Event, Thread
import time
import unittest
from unittest.mock import MagicMock, patch

from proyectorobot.control.motion import ArmAngles, ArmTargets
from proyectorobot.data.types import Landmark, Observation
from proyectorobot.vision.worker import VisionConfig, VisionPacket, VisionStream, VisionWorkerError, offer_latest


def synthetic_capture(config, queue, stop, errors, generation=None):
    try:
        while not stop.is_set():
            packet = VisionPacket(
                Observation(100, (Landmark(0.5, 0.5),) * 33), time.monotonic(), 5.0,
                ArmTargets(ArmAngles(0.5, 0.6), ArmAngles(0.7, 0.8)),
                generation=generation.value if generation is not None else 0,
            )
            offer_latest(queue, packet)
            stop.wait(0.02)
    finally:
        errors.close()
        queue.cancel_join_thread()
        queue.close()


class VisionWorkerTests(unittest.TestCase):
    def test_reset_pauses_generation_and_recalibration_resumes_with_fresh_generation(self):
        config = VisionConfig(0, 480, 360, 15, Path("unused"))
        stream = VisionStream(config)
        try:
            self.assertEqual(stream.generation, 0)
            stream.request_reset()
            self.assertLess(stream.generation, 0)
            paused_generation = stream.generation
            stream.request_reset()
            self.assertLess(stream.generation, paused_generation)
            stream.request_recalibration()
            self.assertGreater(stream.generation, abs(paused_generation))
        finally:
            stream.close()
    def test_actual_camera_frame_is_limited_without_distorting_or_upscaling(self) -> None:
        try:
            import numpy as np
            import cv2
        except (ImportError, OSError):
            self.skipTest("Dependencias de visión no disponibles")
        from proyectorobot.vision.worker import limit_frame_size

        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.assertEqual(limit_frame_size(frame, 480, 360).shape, (270, 480, 3))
        standard = np.zeros((480, 640, 3), dtype=np.uint8)
        self.assertEqual(limit_frame_size(standard, 480, 360).shape, (360, 480, 3))
        small = np.zeros((120, 160, 3), dtype=np.uint8)
        self.assertIs(limit_frame_size(small, 480, 360), small)

    def test_real_process_transfers_pose_and_angles_without_images(self) -> None:
        config = VisionConfig(0, 640, 480, 20, Path("unused"))
        with patch("proyectorobot.vision.worker.capture_worker", synthetic_capture):
            stream = VisionStream(config)
        with stream:
            deadline = time.monotonic() + 5
            packet = None
            while time.monotonic() < deadline and packet is None:
                packet = stream.poll()
                time.sleep(0.01)
            self.assertIsNotNone(packet, "No llegó la observación entre procesos")
            self.assertEqual(packet.observation.timestamp_ms, 100)
            self.assertEqual(len(packet.observation.pose), 33)
            self.assertEqual(packet.targets.left, ArmAngles(0.5, 0.6))
            self.assertFalse(hasattr(packet, "frame"))
        self.assertFalse(stream._process.is_alive())

    def test_queue_drops_old_frame_without_growing_or_blocking(self) -> None:
        queue = Queue(maxsize=1)
        queue.put_nowait("old frame")
        offer_latest(queue, "new frame")
        self.assertEqual(queue.qsize(), 1)
        self.assertEqual(queue.get_nowait(), "new frame")

    def test_child_error_is_reported_and_process_is_reaped(self) -> None:
        with TemporaryDirectory() as directory:
            config = VisionConfig(0, 640, 480, 20, Path(directory))
            stream = VisionStream(config)
            with stream:
                deadline = time.monotonic() + 12
                while time.monotonic() < deadline:
                    try:
                        stream.poll()
                    except VisionWorkerError as error:
                        self.assertIn("Faltan modelos", str(error))
                        break
                    time.sleep(0.02)
                else:
                    self.fail("El error del proceso no llegó a tiempo")
            self.assertFalse(stream._process.is_alive())

    def test_blocked_ipc_does_not_block_the_control_consumer(self) -> None:
        entered = Event()
        release = Event()
        stream = VisionStream(VisionConfig(0, 640, 480, 20, Path("unused")))

        def blocked_receive(*args, **kwargs):
            entered.set()
            release.wait(timeout=2)
            raise Empty

        results = []
        try:
            with patch.object(stream._queue, "get", side_effect=blocked_receive):
                with patch.object(stream._process, "is_alive", return_value=True):
                    stream._receiver.start()
                    self.assertTrue(entered.wait(timeout=1))
                    consumer = Thread(target=lambda: results.append(stream.poll()), daemon=True)
                    consumer.start()
                    consumer.join(timeout=0.2)
                    self.assertFalse(consumer.is_alive(), "El control quedó esperando al IPC")
                    self.assertEqual(results, [None])
        finally:
            release.set()
            stream.close()

    def test_old_generation_is_discarded_after_recalibration(self) -> None:
        stream = VisionStream(VisionConfig(0, 640, 480, 20, Path("unused")))
        try:
            with patch.object(stream._process, "is_alive", return_value=True):
                stream._latest.put_nowait(VisionPacket(Observation(100), 1.0, 5.0))
                stream.request_recalibration()
                self.assertIsNone(stream.poll())
                fresh = VisionPacket(Observation(200), 2.0, 5.0, generation=stream.generation)
                stream._latest.put_nowait(fresh)
                self.assertIs(stream.poll(), fresh)
        finally:
            stream.close()

    def test_camera_window_is_closed_when_capture_fails(self) -> None:
        try:
            import cv2
            from proyectorobot.vision.camera import CameraError
            from proyectorobot.vision.worker import capture_worker
        except (ImportError, OSError):
            self.skipTest("Dependencias de visión no disponibles")
        from contextlib import ExitStack

        config = VisionConfig(0, 480, 360, 20, Path("unused"), show_camera=True)
        camera_manager = MagicMock()
        camera_manager.__enter__.return_value.read.side_effect = CameraError("Fallo de prueba")
        detector_manager = MagicMock()
        errors = MagicMock()
        queue = MagicMock()
        stop = MagicMock()
        stop.is_set.return_value = False
        with ExitStack() as stack:
            camera_constructor = stack.enter_context(patch("proyectorobot.vision.camera.Camera", return_value=camera_manager))
            stack.enter_context(patch("proyectorobot.vision.detector.Detector", return_value=detector_manager))
            for function in ("namedWindow", "resizeWindow", "destroyAllWindows"):
                stack.enter_context(patch.object(cv2, function))
            capture_worker(config, queue, stop, errors)
            cv2.destroyAllWindows.assert_called_once()
            camera_constructor.assert_called_once_with(0, 640, 480, 20)
        self.assertIn("Fallo de prueba", errors.send.call_args.args[0])
        camera_manager.__exit__.assert_called_once()
        detector_manager.__exit__.assert_called_once()
        errors.close.assert_called_once()
