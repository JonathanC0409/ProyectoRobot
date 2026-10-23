"""Cámara simulada, adaptación de Tasks y dibujo sin abrir ventanas."""

from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

try:
    import cv2
    import numpy as np
    from proyectorobot.vision.camera import Camera, CameraError
    from proyectorobot.vision.detector import Detector, convert_landmarks, model_paths
    from proyectorobot.vision.overlay import draw_landmarks, render
    from proyectorobot.control.arms import ArmState, ArmStates
    from proyectorobot.data.types import Landmark, Observation
    VISION_AVAILABLE = True
except (ImportError, OSError):
    VISION_AVAILABLE = False


@unittest.skipUnless(VISION_AVAILABLE, "Dependencias de Fase 1 no disponibles")
class VisionTests(unittest.TestCase):
    def test_missing_world_scores_use_only_same_landmark_image_scores(self):
        from proyectorobot.vision.detector import convert_world_landmarks
        image = [SimpleNamespace(visibility=0.92, presence=0.85),
                 SimpleNamespace(visibility=0.1, presence=0.2)]
        world = [SimpleNamespace(x=-0.3, y=0.2, z=-0.4, visibility=None, presence=None)] * 2
        converted = convert_world_landmarks(world, image)
        self.assertEqual(converted[0], Landmark(-0.3, 0.2, -0.4, 0.92, 0.85))
        self.assertFalse(converted[1].confident(0.65))
        self.assertFalse(convert_world_landmarks(world, [])[0].confident(0.65))

    def test_explicit_low_world_confidence_is_never_replaced_by_high_image_score(self):
        from proyectorobot.vision.detector import convert_world_landmarks
        image = [SimpleNamespace(visibility=1, presence=1)]
        world = [SimpleNamespace(x=0, y=0, z=0, visibility=0.2, presence=0.0)]
        self.assertEqual(convert_world_landmarks(world, image)[0], Landmark(0, 0, 0, 0.2, 0.0))

    def test_compact_arms_have_distinct_anatomical_colors_without_relabelling(self):
        from test_retargeting import frontal_pose
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        with patch("proyectorobot.vision.overlay.cv2.line", wraps=cv2.line) as lines:
            render(frame, Observation(0, frontal_pose()), ArmStates(ArmState.DOWN, ArmState.DOWN),
                   True, 10, 100, compact=True)
        colors = [call.args[3] for call in lines.call_args_list]
        self.assertEqual(colors.count((70, 220, 70)), 2)
        self.assertEqual(colors.count((0, 145, 255)), 2)

    def test_pose_only_mode_neither_requires_nor_runs_hand_model(self) -> None:
        with TemporaryDirectory() as directory:
            from proyectorobot.config import POSE_FILENAME

            (Path(directory) / POSE_FILENAME).write_bytes(b"test pose asset")
            manager = MagicMock()
            manager.__enter__.return_value.detect_for_video.return_value.pose_landmarks = []
            with patch("proyectorobot.vision.detector.vision.PoseLandmarker.create_from_options", return_value=manager):
                with patch("proyectorobot.vision.detector.vision.HandLandmarker.create_from_options") as hands:
                    with Detector(Path(directory), include_hands=False) as detector:
                        result = detector.detect(np.zeros((32, 32, 3), dtype=np.uint8), 0)
                    hands.assert_not_called()
            self.assertEqual(result.hands, ())
            manager.__exit__.assert_called_once()

    def test_compact_overlay_keeps_video_and_status_without_wide_sidebar(self) -> None:
        frame = np.zeros((360, 480, 3), dtype=np.uint8)
        states = ArmStates(ArmState.UP, ArmState.DOWN)
        with patch("proyectorobot.vision.overlay.cv2.putText", wraps=cv2.putText) as draw:
            output = render(frame, Observation(0), states, False, 15, 60,
                            extra_lines=("Robot: CALIBRANDO: 50%", "C: recalibrar"),
                            compact=True, hands_enabled=False)
        self.assertEqual(output.shape, (490, 480, 3))
        np.testing.assert_array_equal(output[:360], frame)
        labels = [call.args[1] for call in draw.call_args_list]
        self.assertIn("Robot: CALIBRANDO: 50%", labels)
        self.assertTrue(any("Dedos: OFF" in label for label in labels))

    def test_camera_release_when_open_fails(self) -> None:
        with patch("proyectorobot.vision.camera.cv2.VideoCapture") as constructor:
            constructor.return_value.isOpened.return_value = False
            with self.assertRaisesRegex(CameraError, "No se pudo abrir"):
                with Camera(0, 640, 480, 20):
                    self.fail("Una cámara ausente no debe iniciar captura")
            constructor.return_value.release.assert_called_once()

    def test_capture_failure_releases_camera(self) -> None:
        with patch("proyectorobot.vision.camera.cv2.VideoCapture") as constructor:
            capture = constructor.return_value
            capture.get.return_value = 20.0
            capture.read.return_value = (False, None)
            with self.assertRaisesRegex(CameraError, "dejó de entregar"):
                with Camera(0, 640, 480, 20) as camera:
                    camera.read()
            capture.release.assert_called_once()

    def test_successful_capture_is_not_mirrored_before_inference(self) -> None:
        frame = np.zeros((4, 6, 3), dtype=np.uint8)
        with patch("proyectorobot.vision.camera.cv2.VideoCapture") as constructor:
            constructor.return_value.get.return_value = 20.0
            constructor.return_value.read.return_value = (True, frame)
            with Camera(0, 640, 480, 20) as camera:
                self.assertIs(camera.read(), frame)
            constructor.return_value.release.assert_called_once()

    def test_missing_models_have_clear_error(self) -> None:
        with TemporaryDirectory() as directory:
            with self.assertRaisesRegex(FileNotFoundError, "no se descargan automáticamente"):
                model_paths(Path(directory))

    def test_missing_confidence_is_rejected_for_pose_only(self) -> None:
        point = SimpleNamespace(x=0.5, y=0.5, z=0.0, visibility=None, presence=None)
        self.assertFalse(convert_landmarks([point], True)[0].visible(0.6))
        self.assertTrue(convert_landmarks([point], False)[0].visible(0.0))

    def test_partial_detector_initialization_closes_first_model(self) -> None:
        pose = MagicMock()
        with patch("proyectorobot.vision.detector.model_paths", return_value=(Path("pose"), Path("hands"))):
            with patch("proyectorobot.vision.detector.vision.PoseLandmarker.create_from_options", return_value=pose):
                with patch("proyectorobot.vision.detector.vision.HandLandmarker.create_from_options",
                           side_effect=RuntimeError("Modelo inválido")):
                    with self.assertRaisesRegex(RuntimeError, "Modelo inválido"):
                        with Detector():
                            self.fail("Un detector parcial no debe estar disponible")
        pose.__exit__.assert_called_once()

    def test_detector_sends_rgb_and_same_timestamp_to_both_models(self) -> None:
        frame = np.zeros((4, 6, 3), dtype=np.uint8)
        frame[:, :, 0] = 255
        detector = Detector()
        detector._pose = MagicMock()
        detector._hands = MagicMock()
        detector._pose.detect_for_video.return_value.pose_landmarks = []
        detector._hands.detect_for_video.return_value.hand_landmarks = []
        observation = detector.detect(frame, 100)
        self.assertEqual(observation.timestamp_ms, 100)
        pose_image, pose_timestamp = detector._pose.detect_for_video.call_args.args
        hand_image, hand_timestamp = detector._hands.detect_for_video.call_args.args
        self.assertIs(pose_image, hand_image)
        self.assertEqual((pose_timestamp, hand_timestamp), (100, 100))
        np.testing.assert_array_equal(pose_image.numpy_view()[0, 0], [0, 0, 255])
        with self.assertRaises(ValueError):
            detector.detect(frame, 100)

    def test_world_landmarks_keep_metric_coordinates_and_confidence(self) -> None:
        detector = Detector(include_hands=False)
        detector._pose = MagicMock()
        point = SimpleNamespace(x=-0.3, y=1.4, z=-0.6, visibility=0.9, presence=0.8)
        detector._pose.detect_for_video.return_value = SimpleNamespace(
            pose_landmarks=[], pose_world_landmarks=[[point] * 33])
        result = detector.detect(np.zeros((32, 32, 3), dtype=np.uint8), 0)
        self.assertEqual(len(result.pose_world), 33)
        self.assertEqual(result.pose_world[0], Landmark(-0.3, 1.4, -0.6, 0.9, 0.8))
        self.assertTrue(result.pose_world[0].confident(0.65))
        self.assertFalse(result.pose_world[0].visible(0.65))

    def test_mirror_changes_pixels_without_swapping_states_or_mutating_frame(self) -> None:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:, 0] = (255, 0, 0)
        observation = Observation(0)
        states = ArmStates(ArmState.UP, ArmState.DOWN)
        output = render(frame, observation, states, True, 20.0, 15.0)
        self.assertEqual(output.shape, (480, 1200, 3))
        np.testing.assert_array_equal(output[300, 639], [255, 0, 0])
        np.testing.assert_array_equal(output[10, 639], [255, 0, 0])
        np.testing.assert_array_equal(frame[300, 0], [255, 0, 0])
        self.assertEqual(states.left, ArmState.UP)

    def test_movement_text_keeps_anatomical_sides_with_and_without_mirror(self) -> None:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        states = ArmStates(ArmState.UP, ArmState.DOWN)
        for mirror in (False, True):
            with self.subTest(mirror=mirror):
                with patch("proyectorobot.vision.overlay.cv2.putText", wraps=cv2.putText) as draw_text:
                    render(frame, Observation(0), states, mirror, 20.0, 15.0)
                labels = [call.args[1] for call in draw_text.call_args_list]
                self.assertIn("Movimiento: MANO IZQUIERDA ARRIBA", labels)
                self.assertIn("Izquierdo: ARRIBA", labels)
                self.assertIn("Derecho: ABAJO", labels)

    def test_landmark_projection_and_mirror(self) -> None:
        point = Landmark(0.25, 0.5)
        for mirror, expected_horizontal in ((False, 25), (True, 75)):
            with self.subTest(mirror=mirror):
                frame = np.zeros((101, 101, 3), dtype=np.uint8)
                draw_landmarks(frame, (point,), [], (0, 255, 0), mirror, 0.6)
                self.assertEqual(int(frame[50, expected_horizontal, 1]), 255)

    def test_large_text_fits_sidebar_even_with_low_resolution_capture(self) -> None:
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        for state in (ArmState.UP, ArmState.DOWN, ArmState.UNKNOWN):
            with self.subTest(state=state):
                with patch("proyectorobot.vision.overlay.cv2.putText", wraps=cv2.putText) as draw_text:
                    output = render(frame, Observation(0), ArmStates(state, state), False, 20.0, 150.0)
                for call in draw_text.call_args_list:
                    panel, label, origin, font, scale, _, thickness, _ = call.args
                    (text_width, text_height), baseline = cv2.getTextSize(label, font, scale, thickness)
                    self.assertGreaterEqual(origin[0], 0)
                    self.assertGreaterEqual(origin[1] - text_height, 0)
                    self.assertLess(origin[0] + text_width, panel.shape[1])
                    self.assertLess(origin[1] + baseline, output.shape[0])
                np.testing.assert_array_equal(output[:240, :320], frame)


@unittest.skipUnless(VISION_AVAILABLE, "Dependencias de Fase 1 no disponibles")
class ModelIntegrationTests(unittest.TestCase):
    def test_real_models_on_blank_frames(self) -> None:
        from proyectorobot.config import DEFAULT_MODELS_DIR, HAND_FILENAME, POSE_FILENAME

        if not all((DEFAULT_MODELS_DIR / filename).is_file() for filename in (POSE_FILENAME, HAND_FILENAME)):
            self.skipTest("Modelos no descargados; la prueba nunca accede a la red")
        from proyectorobot.app import smoke_test

        smoke_test(DEFAULT_MODELS_DIR)
