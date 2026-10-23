"""Casos sintéticos de elevación, ruido y pérdida de puntos."""

from dataclasses import replace
import unittest

from proyectorobot.control.arms import ArmRecognizer, ArmState, ArmStates, ArmTracker
from proyectorobot.data.types import Landmark, Observation


def make_pose(left_up: bool = False, right_up: bool = False) -> tuple[Landmark, ...]:
    points = [Landmark(0.5, 0.5) for _ in range(33)]
    for shoulder, wrist, hip, horizontal, raised in (
        (11, 15, 23, 0.65, left_up), (12, 16, 24, 0.35, right_up),
    ):
        points[shoulder] = Landmark(horizontal, 0.4)
        points[wrist] = Landmark(horizontal, 0.2 if raised else 0.8)
        points[hip] = Landmark(horizontal, 0.8)
    return tuple(points)


class ArmRecognizerTests(unittest.TestCase):
    def test_upper_body_mode_does_not_claim_tracking_is_lost_for_cropped_hips(self) -> None:
        points = list(make_pose(left_up=True))
        for index in (23, 24):
            points[index] = Landmark(0.5, 1.2, visibility=0.1)
        pose = tuple(points)
        upper = ArmRecognizer(hold_ms=0, allow_upper_body=True).update(Observation(0, pose))
        strict = ArmRecognizer(hold_ms=0).update(Observation(0, pose))
        self.assertEqual((upper.left, upper.right), (ArmState.UP, ArmState.DOWN))
        self.assertEqual(strict.left, ArmState.UNKNOWN)

    def test_combined_movement_handles_all_tracking_combinations(self) -> None:
        expected = {
            (ArmState.DOWN, ArmState.DOWN): "MANOS ABAJO",
            (ArmState.UP, ArmState.DOWN): "MANO IZQUIERDA ARRIBA",
            (ArmState.DOWN, ArmState.UP): "MANO DERECHA ARRIBA",
            (ArmState.UP, ArmState.UP): "AMBAS MANOS ARRIBA",
            (ArmState.UNKNOWN, ArmState.UNKNOWN): "SIN SEGUIMIENTO",
            (ArmState.UNKNOWN, ArmState.DOWN): "SEGUIMIENTO PARCIAL",
            (ArmState.UNKNOWN, ArmState.UP): "SEGUIMIENTO PARCIAL",
            (ArmState.DOWN, ArmState.UNKNOWN): "SEGUIMIENTO PARCIAL",
            (ArmState.UP, ArmState.UNKNOWN): "SEGUIMIENTO PARCIAL",
        }
        for (left, right), movement in expected.items():
            with self.subTest(left=left, right=right):
                self.assertEqual(ArmStates(left, right).movement, movement)

    def test_each_anatomical_side_and_both_arms(self) -> None:
        for left_up, right_up in ((False, False), (True, False), (False, True), (True, True)):
            with self.subTest(left=left_up, right=right_up):
                recognizer = ArmRecognizer()
                pose = make_pose(left_up, right_up)
                initial = recognizer.update(Observation(0, pose))
                self.assertEqual(initial.left, ArmState.UNKNOWN)
                result = recognizer.update(Observation(150, pose))
                self.assertEqual(result.left, ArmState.UP if left_up else ArmState.DOWN)
                self.assertEqual(result.right, ArmState.UP if right_up else ArmState.DOWN)

    def test_spike_does_not_raise_arm(self) -> None:
        recognizer = ArmRecognizer()
        recognizer.update(Observation(0, make_pose()))
        recognizer.update(Observation(150, make_pose()))
        result = recognizer.update(Observation(200, make_pose(True)))
        self.assertEqual(result.left, ArmState.DOWN)
        recognizer.update(Observation(250, make_pose()))
        result = recognizer.update(Observation(400, make_pose()))
        self.assertEqual(result.left, ArmState.DOWN)

    def test_hysteresis_retains_state_in_boundary_band(self) -> None:
        tracker = ArmTracker(hold_ms=0)
        shoulder, hip = Landmark(0.5, 0.4), Landmark(0.5, 0.8)
        self.assertEqual(tracker.update(shoulder, Landmark(0.5, 0.2), hip, 0), ArmState.UP)
        self.assertEqual(tracker.update(shoulder, Landmark(0.5, 0.34), hip, 50), ArmState.UP)
        self.assertEqual(tracker.update(shoulder, Landmark(0.5, 0.39), hip, 100), ArmState.DOWN)
        self.assertEqual(tracker.update(shoulder, Landmark(0.5, 0.34), hip, 150), ArmState.DOWN)

    def test_hysteresis_band_cancels_pending_transition(self) -> None:
        tracker = ArmTracker()
        shoulder, hip = Landmark(0.5, 0.4), Landmark(0.5, 0.8)
        tracker.update(shoulder, Landmark(0.5, 0.2), hip, 0)
        tracker.update(shoulder, Landmark(0.5, 0.34), hip, 100)
        self.assertEqual(tracker.update(shoulder, Landmark(0.5, 0.2), hip, 200), ArmState.UNKNOWN)
        self.assertEqual(tracker.update(shoulder, Landmark(0.5, 0.2), hip, 350), ArmState.UP)

    def test_missing_pose_resets_immediately_and_requires_reacquisition(self) -> None:
        recognizer = ArmRecognizer()
        recognizer.update(Observation(0, make_pose(True, True)))
        recognizer.update(Observation(150, make_pose(True, True)))
        lost = recognizer.update(Observation(200))
        self.assertEqual((lost.left, lost.right), (ArmState.UNKNOWN, ArmState.UNKNOWN))
        reacquired = recognizer.update(Observation(400, make_pose(True, True)))
        self.assertEqual(reacquired.left, ArmState.UNKNOWN)
        self.assertEqual(recognizer.update(Observation(550, make_pose(True, True))).left, ArmState.UP)

    def test_unreliable_left_wrist_does_not_invalidate_right_arm(self) -> None:
        for invalid in (
            Landmark(0.65, 0.2, visibility=0.2),
            Landmark(0.65, 0.2, presence=0.2),
            Landmark(0.65, -0.1),
            Landmark(float("nan"), 0.2),
            Landmark(0.65, 0.2, z=float("inf")),
        ):
            with self.subTest(invalid=invalid):
                recognizer = ArmRecognizer(hold_ms=0)
                points = list(make_pose(True, True))
                points[15] = invalid
                result = recognizer.update(Observation(0, tuple(points)))
                self.assertEqual(result.left, ArmState.UNKNOWN)
                self.assertEqual(result.right, ArmState.UP)

    def test_invalid_or_cropped_torso_returns_unknown(self) -> None:
        for hip in (Landmark(0.65, 0.41), Landmark(0.65, 1.1), Landmark(0.65, 0.2)):
            recognizer = ArmRecognizer(hold_ms=0)
            points = list(make_pose(True))
            points[23] = hip
            self.assertEqual(recognizer.update(Observation(0, tuple(points))).left, ArmState.UNKNOWN)

    def test_vertical_translation_and_scaling_preserve_result(self) -> None:
        original = make_pose(True, False)
        transformed = tuple(replace(point, y=point.y * 0.5 + 0.1) for point in original)
        for pose in (original, transformed):
            states = ArmRecognizer(hold_ms=0).update(Observation(0, pose))
            self.assertEqual((states.left, states.right), (ArmState.UP, ArmState.DOWN))

    def test_wrong_landmark_count_cannot_leave_stale_state(self) -> None:
        recognizer = ArmRecognizer(hold_ms=0)
        recognizer.update(Observation(0, make_pose(True)))
        result = recognizer.update(Observation(1, make_pose(True)[:20]))
        self.assertEqual(result.left, ArmState.UNKNOWN)

    def test_timestamps_must_increase(self) -> None:
        recognizer = ArmRecognizer()
        recognizer.update(Observation(100))
        for timestamp in (-1, 99, 100):
            with self.assertRaises(ValueError):
                recognizer.update(Observation(timestamp))

    def test_invalid_thresholds_are_rejected(self) -> None:
        for options in (
            {"raise_ratio": 0.1, "lower_ratio": 0.2}, {"hold_ms": -1},
            {"visibility": 1.1}, {"raise_ratio": float("nan")},
        ):
            with self.subTest(options=options), self.assertRaises(ValueError):
                ArmTracker(**options)
