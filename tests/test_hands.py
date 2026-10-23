"""Geometría, calibración, lateralidad y seguridad de las manos sin webcam."""

from dataclasses import replace
from math import cos, pi, sin
import unittest

from proyectorobot.control.gestures import (
    HandFeatures, HandGestureRecognizer, associate_hands, measure_hand, normalized_command,
)
from proyectorobot.control.hands import (
    FIST, MAX_CURL, OPEN, PINCH, HandCommand, HandController, HandTargets,
)
from proyectorobot.control.motion import MotionController
from proyectorobot.data.types import Landmark, Observation


def synthetic_hand(closed=False, shift=0, pinch=False):
    """Palma frontal con dedos rectos o flexionados 108° y 54° adicionales."""
    pixels = [(400 + shift, 330, 0)] * 21
    bases = ((350, 320), (365, 285), (390, 280), (412, 282), (435, 292))
    for chain, (base_x, base_y) in zip(((1, 2, 3, 4), (5, 6, 7, 8), (9, 10, 11, 12),
                                       (13, 14, 15, 16), (17, 18, 19, 20)), bases):
        position = (base_x + shift, base_y, 0)
        pixels[chain[0]] = position
        for step, (index, length) in enumerate(zip(chain[1:], (26, 22, 18))):
            angle = (0, pi * 0.6, pi * 0.9)[step] if closed else 0
            position = (position[0], position[1] - length * cos(angle),
                        position[2] + length * sin(angle))
            pixels[index] = position
    if pinch:
        for index, xy in enumerate(((340, 306), (338, 276), (345, 248), (363, 219)), 1):
            pixels[index] = (xy[0] + shift, xy[1], 0)
    return tuple(Landmark(x / 640, y / 480, z / 640) for x, y, z in pixels)


def observation(timestamp, left=None, right=None):
    pose = [Landmark(0, 0, visibility=0)] * 33
    pose[11], pose[12] = Landmark(420 / 640, 230 / 480), Landmark(200 / 640, 230 / 480)
    pose[15], pose[16] = Landmark(400 / 640, 330 / 480), Landmark(220 / 640, 330 / 480)
    return Observation(timestamp, tuple(pose), tuple(hand for hand in (right, left) if hand is not None))


class HandGeometryTests(unittest.TestCase):
    def test_angles_distinguish_extended_flexed_and_pinch(self):
        opened = measure_hand(synthetic_hand(), 640, 480)
        closed = measure_hand(synthetic_hand(True), 640, 480)
        pinch = measure_hand(synthetic_hand(pinch=True), 640, 480)
        self.assertLess(max(opened.curls), 0.01)
        self.assertGreater(min(closed.curls), 0.85)
        self.assertGreater(opened.pinch_gap, 0.5)
        self.assertLess(pinch.pinch_gap, 0.1)

    def test_normalization_invariant_to_scale_translation_and_aspect(self):
        points = synthetic_hand(True)
        expected = measure_hand(points, 640, 480)
        transformed = tuple(Landmark(p.x * 0.5 + 0.1, p.y * 0.5 + 0.1, p.z * 0.5)
                            for p in points)
        actual = measure_hand(transformed, 640, 480)
        self.assertAlmostEqual(actual.pinch_gap, expected.pinch_gap)
        for first, second in zip(actual.curls, expected.curls):
            self.assertAlmostEqual(first, second)
        resized = tuple(replace(p, y=p.y * 0.75) for p in points)
        resized_features = measure_hand(resized, 640, 640)
        self.assertAlmostEqual(resized_features.pinch_gap, expected.pinch_gap)
        for first, second in zip(resized_features.curls, expected.curls):
            self.assertAlmostEqual(first, second)

    def test_invalid_degenerate_tiny_and_offscreen_hands_are_rejected(self):
        valid = synthetic_hand()
        for hand in ((), valid[:20], (Landmark(0.5, 0.5),) * 21,
                     (replace(valid[0], x=float("nan")),) + valid[1:],
                     (replace(valid[0], x=1.2),) + valid[1:],
                     (replace(valid[0], z=10),) + valid[1:]):
            with self.subTest(hand=hand[:1]):
                self.assertIsNone(measure_hand(hand, 640, 480))
        self.assertIsNone(measure_hand(valid, 0, 480))
        tiny = tuple(Landmark(p.x * 0.01, p.y * 0.01, p.z * 0.01) for p in valid)
        self.assertIsNone(measure_hand(tiny, 640, 480))

    def test_classification_keeps_fist_distinct_from_small_pinch_gap(self):
        opened, closed = (0.0,) * 5, (1.0,) * 5
        for features, expected in ((HandFeatures(opened, 1.3), "ABIERTA"),
                                   (HandFeatures(closed, 0.1), "PUNO"),
                                   (HandFeatures((0.1,) * 5, 0.1), "PINZA"),
                                   (HandFeatures((0.5,) * 5, 0.6), "PARCIAL")):
            command, status = normalized_command(features, opened, closed)
            self.assertEqual(status, expected)
            self.assertTrue(all(0 <= value <= 1 for value in command.values))

    def test_anatomical_association_ignores_result_order_and_mirror(self):
        left, right = synthetic_hand(), synthetic_hand(shift=-180)
        sample = observation(0, left, right)
        self.assertEqual(associate_hands(sample, 640, 480), (left, right))
        mirror = lambda hand: tuple(replace(p, x=1 - p.x) for p in hand)
        mirrored = replace(sample, pose=mirror(sample.pose), hands=tuple(map(mirror, sample.hands)))
        self.assertEqual(associate_hands(mirrored, 640, 480), (mirror(left), mirror(right)))

    def test_crossed_nearby_wrists_and_missing_pose_do_not_swap_hands(self):
        sample = observation(0, synthetic_hand())
        self.assertEqual(associate_hands(replace(sample, pose=()), 640, 480), (None, None))
        pose = list(sample.pose)
        pose[16] = pose[15]
        self.assertEqual(associate_hands(replace(sample, pose=tuple(pose)), 640, 480), (None, None))


class GestureCalibrationTests(unittest.TestCase):
    def calibrated(self):
        recognizer = HandGestureRecognizer()
        for timestamp in range(0, 700, 100):
            result = recognizer.update(observation(timestamp, synthetic_hand()), 640, 480)
            self.assertIsNone(result.targets.left)
        for timestamp in range(700, 1400, 100):
            recognizer.update(observation(timestamp, synthetic_hand(True)), 640, 480)
        self.assertIsNotNone(recognizer.states[0].closed)
        return recognizer

    def test_calibration_then_open_fist_pinch_and_independent_missing_hand(self):
        recognizer = self.calibrated()
        timestamp = 1400
        for hand, expected in ((synthetic_hand(), "ABIERTA"), (synthetic_hand(True), "PUNO"),
                               (synthetic_hand(pinch=True), "PINZA")):
            for _ in range(6):
                result = recognizer.update(observation(timestamp, hand), 640, 480)
                timestamp += 100
            self.assertEqual(result.left, expected)
            self.assertEqual(result.right, "SIN MANO")
            self.assertIsNotNone(result.targets.left)
            self.assertIsNone(result.targets.right)

    def test_loss_recovery_uses_fresh_samples_without_erasing_calibration(self):
        recognizer = self.calibrated()
        for timestamp in (1400, 1500, 1600):
            recognizer.update(observation(timestamp, synthetic_hand(True)), 640, 480)
        self.assertIsNone(recognizer.update(observation(1700), 640, 480).targets.left)
        self.assertIsNone(recognizer.update(observation(1800, synthetic_hand()), 640, 480).targets.left)
        recovered = recognizer.update(observation(1900, synthetic_hand()), 640, 480)
        self.assertLess(max(recovered.targets.left.values), 0.01)
        stale = recognizer.update(observation(2400, synthetic_hand(True)), 640, 480)
        self.assertIsNone(stale.targets.left)

    def test_second_hand_calibrates_independently_then_both_follow_different_gestures(self):
        recognizer = self.calibrated()
        for timestamp in range(1400, 2100, 100):
            recognizer.update(observation(timestamp, synthetic_hand(True), synthetic_hand(shift=-180)),
                              640, 480)
        for timestamp in range(2100, 2800, 100):
            recognizer.update(observation(timestamp, synthetic_hand(True), synthetic_hand(True, shift=-180)),
                              640, 480)
        for timestamp in range(2800, 3400, 100):
            result = recognizer.update(observation(timestamp, synthetic_hand(pinch=True),
                                                   synthetic_hand(shift=-180)), 640, 480)
        self.assertEqual((result.left, result.right), ("PINZA", "ABIERTA"))
        self.assertGreater(result.targets.left.index, 0.5)
        self.assertLess(max(result.targets.right.values), 0.01)

    def test_repeated_timestamp_invalidates_command_and_recalibration_clears_references(self):
        recognizer = self.calibrated()
        recognizer.update(observation(1500, synthetic_hand()), 640, 480)
        self.assertIsNone(recognizer.update(observation(1500, synthetic_hand()), 640, 480).targets.left)
        recognizer.recalibrate()
        self.assertIsNone(recognizer.states[0].opened)
        self.assertIsNone(recognizer.states[0].closed)

    def test_fist_cannot_calibrate_as_open(self):
        recognizer = HandGestureRecognizer()
        for timestamp in range(0, 1000, 100):
            recognizer.update(observation(timestamp, synthetic_hand(True)), 640, 480)
        self.assertIsNone(recognizer.states[0].opened)


class HandControlTests(unittest.TestCase):
    def test_limits_speed_and_missing_side(self):
        controller = HandController()
        controller.set_targets(HandTargets(left=HandCommand(10, 10, -1)), 0, 0)
        previous = controller.positions
        for step in range(30):
            actual = controller.step(0.01, step * 0.01)
            self.assertTrue(all(0 <= value <= MAX_CURL for value in actual))
            self.assertTrue(all(abs(new - old) <= 0.02 for new, old in zip(actual, previous)))
            self.assertEqual(actual[2:], (0.0,) * 4)
            previous = actual

    def test_stale_observations_open_and_recovery_is_slew_limited(self):
        controller = HandController()
        for step in range(100):
            now = step * 0.01
            controller.set_targets(HandTargets(left=FIST), now, now)
            controller.step(0.01, now)
        self.assertGreater(controller.positions[0], 0.4)
        for step in range(200):
            controller.step(0.01, 2 + step * 0.01)
        self.assertLess(max(controller.positions), 0.001)
        controller.set_targets(HandTargets(right=PINCH), 4, 4)
        self.assertLessEqual(controller.step(0.01, 4)[3], 0.02)

    def test_invalid_values_and_timestamps_fail_safe(self):
        for value, captured_at, now in ((float("nan"), 0, 0), (1, 2, 0), (1, 0, float("inf"))):
            controller = HandController()
            with self.assertRaises(ValueError):
                controller.set_targets(HandTargets(left=HandCommand(value, 1, 1)), captured_at, now)
            self.assertEqual(controller.tracking(0), (False, False))
        controller = HandController()
        controller.set_targets(HandTargets(left=OPEN), 0, 0)
        with self.assertRaises(ValueError):
            controller.set_targets(HandTargets(left=FIST), 0, 0)

    def test_custom_arm_safe_posture_does_not_change_default_phase(self):
        from proyectorobot.control.motion import NEUTRAL
        from proyectorobot.simulation.hand_model import safe_posture
        self.assertEqual(MotionController().positions, NEUTRAL)
        for side in ("left", "right"):
            neutral = safe_posture(side)
            controller = MotionController(neutral=neutral)
            self.assertEqual(controller.step(0.01, 0), neutral)
        with self.assertRaises(ValueError):
            MotionController(neutral=(0.0,))
