"""CLI y contrato de tiempo sin GUI ni acceso a hardware."""

from contextlib import redirect_stderr
import io
import unittest
from unittest.mock import patch

from proyectorobot.app import FrameClock, build_parser, main


class AppTests(unittest.TestCase):
    def test_defaults_are_cpu_friendly(self) -> None:
        args = build_parser().parse_args([])
        self.assertEqual((args.width, args.height, args.fps), (640, 480, 20))
        self.assertTrue(args.mirror)
        self.assertFalse(build_parser().parse_args(["--no-mirror"]).mirror)

    def test_invalid_camera_and_dimensions_are_rejected(self) -> None:
        for arguments in (
            ["--camera", "-1"], ["--width", "0"], ["--fps", "-10"],
            ["--window-width", "0"], ["--window-height", "-1"],
        ):
            with self.subTest(arguments=arguments), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as caught:
                    build_parser().parse_args(arguments)
                self.assertEqual(caught.exception.code, 2)

    def test_check_and_smoke_are_exclusive(self) -> None:
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            build_parser().parse_args(["--check", "--smoke-test"])

    def test_clock_handles_submillisecond_frames_and_backward_source(self) -> None:
        with patch("proyectorobot.app.time.monotonic_ns", side_effect=[0, 0, 500_000, 0, 20_000_000]):
            clock = FrameClock()
            self.assertEqual([clock.next_ms() for _ in range(4)], [0, 1, 2, 20])

    def test_runtime_error_returns_failure_without_traceback(self) -> None:
        with patch("proyectorobot.app.run_camera", side_effect=RuntimeError("Cámara ausente")):
            with self.assertLogs(level="ERROR") as captured:
                self.assertEqual(main([]), 1)
            self.assertIn("Cámara ausente", " ".join(captured.output))

    def test_ctrl_c_has_distinct_exit_code(self) -> None:
        with patch("proyectorobot.app.run_camera", side_effect=KeyboardInterrupt):
            with self.assertLogs(level="INFO"):
                self.assertEqual(main([]), 130)

    def test_missing_graphical_session_is_actionable(self) -> None:
        with patch.dict("os.environ", {}, clear=True), self.assertLogs(level="ERROR") as captured:
            self.assertEqual(main([]), 1)
        self.assertIn("No hay sesión gráfica", " ".join(captured.output))
