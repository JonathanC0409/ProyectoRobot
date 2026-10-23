"""Modos separados de la Fase 1 y cierre sin ventanas reales."""

from contextlib import redirect_stderr
import io
import unittest
from unittest.mock import MagicMock, patch

from proyectorobot.simulation.app import RenderClock, build_parser, main
from proyectorobot.simulation.viewer import RobotViewer


class Phase2AppTests(unittest.TestCase):
    def test_render_clock_preserves_thirty_fps_with_fifty_hz_control(self):
        clock = RenderClock(30)
        frames = sum(clock.due(index / 50) for index in range(100))
        self.assertGreaterEqual(frames, 59)
        self.assertLessEqual(frames, 60)
        self.assertTrue(clock.due(100))
        self.assertFalse(clock.due(100))
        self.assertLessEqual(clock.next_frame - 100, 1 / 30 + 1e-9)

    def test_fast_profile_keeps_vision_render_and_camera_window_independent(self):
        args = build_parser().parse_args([])
        self.assertEqual((args.width, args.height, args.fps), (480, 360, 30))
        self.assertEqual(args.render_fps, 30)
        self.assertEqual(args.graphics, "auto")
        self.assertEqual((args.window_width, args.window_height), (640, 480))
        args = build_parser().parse_args(["--fps", "20", "--render-fps", "15"])
        self.assertEqual((args.fps, args.render_fps), (20, 15))

    def test_invalid_or_unsafe_modes_are_rejected(self) -> None:
        for arguments in (
            ["--headless"], ["--snapshot", "robot.png"], ["--duration", "nan"],
            ["--tracking-timeout", "0"], ["--demo", "--check"],
        ):
            with self.subTest(arguments=arguments), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    main(arguments)
                self.assertEqual(error.exception.code, 2)

    def test_capture_and_mirror_options_are_independent_of_demo(self) -> None:
        args = build_parser().parse_args(["--no-mirror", "--camera", "2"])
        self.assertFalse(args.mirror)
        self.assertEqual(args.camera, 2)
        self.assertFalse(args.demo)

    def test_viewer_keyboard_requests_do_not_modify_physics_on_gui_thread(self) -> None:
        viewer = RobotViewer(None)
        viewer.on_key(ord("c"))
        self.assertTrue(viewer.recalibrate.is_set())
        viewer.on_key(ord("r"))
        self.assertTrue(viewer.reset.is_set())
        self.assertFalse(viewer.quit.is_set())
        viewer.on_key(256)
        self.assertTrue(viewer.quit.is_set())

    def test_no_display_produces_actionable_error_before_launch(self) -> None:
        with patch.dict("os.environ", {}, clear=True):
            with self.assertRaisesRegex(RuntimeError, "Sin sesión gráfica"):
                with RobotViewer(None):
                    self.fail("No debería abrirse un visor sin sesión gráfica")

    def test_imitation_loop_closes_viewer_and_stream_on_capture_error(self) -> None:
        try:
            from proyectorobot.simulation.engine import RobotSimulation
            from proyectorobot.simulation.app import run_imitation
            from proyectorobot.vision.worker import VisionWorkerError
        except (ImportError, OSError):
            self.skipTest("Dependencias de visión/simulación no disponibles")
        from contextlib import ExitStack

        simulation = RobotSimulation()
        args = build_parser().parse_args([])
        viewer_manager = MagicMock()
        viewer = viewer_manager.__enter__.return_value
        viewer.running.side_effect = [True, False]
        viewer.recalibrate.is_set.return_value = False
        viewer.reset.is_set.return_value = False
        stream_manager = MagicMock()
        stream_manager.__enter__.return_value.generation = 0
        stream_manager.__enter__.return_value.poll.side_effect = VisionWorkerError("Error sintético de cámara")
        with ExitStack() as stack:
            stack.enter_context(patch("proyectorobot.simulation.viewer.RobotViewer", return_value=viewer_manager))
            stack.enter_context(patch("proyectorobot.vision.worker.VisionStream", return_value=stream_manager))
            with self.assertLogs(level="ERROR"), self.assertRaisesRegex(VisionWorkerError, "sintético"):
                run_imitation(args, simulation)
        viewer_manager.__exit__.assert_called_once()
        stream_manager.__exit__.assert_called_once()
        self.assertEqual(simulation.controller.tracking(0), (False, False))

    def test_reset_returns_to_neutral_and_rejects_commands_until_recalibration(self):
        from proyectorobot.control.motion import ArmAngles, ArmTargets
        from proyectorobot.data.types import Observation
        from proyectorobot.simulation.engine import RobotSimulation
        from proyectorobot.simulation.app import run_imitation
        from proyectorobot.vision.worker import VisionPacket
        import time

        simulation = RobotSimulation()
        viewer_manager = MagicMock()
        viewer = viewer_manager.__enter__.return_value
        viewer.running.side_effect = [True, True, False]
        viewer.reset.is_set.side_effect = [True, False]
        viewer.recalibrate.is_set.return_value = False
        stream_manager = MagicMock()
        stream = stream_manager.__enter__.return_value
        stream.generation = 0
        stream.request_reset.side_effect = lambda: setattr(stream, "generation", -1)
        targets = ArmTargets(ArmAngles(1, 1, 1, 1), ArmAngles(1, 1, 1, 1))
        stream.poll.side_effect = [VisionPacket(Observation(index), time.monotonic(), 5,
                                               targets, generation=generation)
                                   for index, generation in ((0, 0), (1, -1))]
        with patch("proyectorobot.simulation.viewer.RobotViewer", return_value=viewer_manager), \
                patch("proyectorobot.vision.worker.VisionStream", return_value=stream_manager), \
                patch.object(simulation.controller, "set_targets") as setter:
            run_imitation(build_parser().parse_args([]), simulation)
        stream.request_reset.assert_called_once()
        setter.assert_not_called()
        self.assertIn("REPOSO", viewer.update.call_args.args[2])
