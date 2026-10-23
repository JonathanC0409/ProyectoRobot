"""Captura acotada, errores y propiedad de recursos sin webcam real."""

from contextlib import AbstractContextManager
from threading import Event, get_ident
import time
import unittest
from unittest.mock import patch

import numpy as np

from proyectorobot.vision.camera import CameraError, CameraStream


class CameraStreamTests(unittest.TestCase):
    def test_latest_frame_replaces_old_frames_and_camera_stays_on_its_thread(self):
        completed = Event()
        release = Event()
        events = []

        class Source(AbstractContextManager):
            count = 0

            def __enter__(self):
                events.append(("open", get_ident()))
                return self

            def read(self):
                events.append(("read", get_ident()))
                self.count += 1
                if self.count == 6:
                    completed.set()
                    release.wait(2)
                return np.full((2, 2, 3), self.count, dtype=np.uint8)

            def __exit__(self, *args):
                events.append(("close", get_ident()))

        source = Source()
        with patch("proyectorobot.vision.camera.Camera", return_value=source):
            stream = CameraStream(0, 640, 480, 30)
            stream.__enter__()
            try:
                self.assertTrue(completed.wait(2))
                self.assertEqual(stream._frames.qsize(), 1)
                frame = stream.read()
                self.assertEqual(int(frame.image[0, 0, 0]), 5)
                self.assertLessEqual(frame.captured_at, time.monotonic())
            finally:
                stream._stop.set()
                release.set()
                stream.close()
        self.assertFalse(stream._thread.is_alive())
        self.assertEqual(events[-1][0], "close")
        self.assertEqual(len({identity for _, identity in events}), 1)
        self.assertNotEqual(events[0][1], get_ident())

    def test_camera_failure_is_propagated_and_resources_are_released(self):
        released = Event()

        class Source(AbstractContextManager):
            def __enter__(self):
                return self

            def read(self):
                raise CameraError("fallo sintético")

            def __exit__(self, *args):
                released.set()

        with patch("proyectorobot.vision.camera.Camera", return_value=Source()):
            with CameraStream(0, 640, 480, 30) as stream:
                with self.assertRaisesRegex(CameraError, "fallo sintético"):
                    stream.read()
        self.assertTrue(released.is_set())
        self.assertFalse(stream._thread.is_alive())

    def test_blocked_driver_does_not_make_close_unbounded_or_release_concurrently(self):
        blocked = Event()
        release = Event()
        closed = Event()

        class Source(AbstractContextManager):
            def __enter__(self):
                return self

            def read(self):
                blocked.set()
                release.wait(3)
                return np.zeros((2, 2, 3), dtype=np.uint8)

            def __exit__(self, *args):
                closed.set()

        with patch("proyectorobot.vision.camera.Camera", return_value=Source()):
            stream = CameraStream(0, 640, 480, 30).__enter__()
            try:
                self.assertTrue(blocked.wait(2))
                start = time.monotonic()
                with self.assertLogs(level="WARNING"):
                    stream.close()
                self.assertLess(time.monotonic() - start, 1.5)
                self.assertFalse(closed.is_set())
            finally:
                release.set()
                stream.close()
        self.assertTrue(closed.is_set())

    def test_closed_stream_cannot_return_a_stale_frame(self):
        stream = CameraStream(0, 640, 480, 30)
        stream.close()
        with self.assertRaisesRegex(CameraError, "recientes"):
            stream.read()
