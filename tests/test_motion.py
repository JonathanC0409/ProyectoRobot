"""Suavizado, límites y vigilancia sin MuJoCo ni visión."""

from math import radians
import unittest

from proyectorobot.control.motion import ArmAngles, ArmTargets, JOINTS, NEUTRAL, MotionController


class MotionTests(unittest.TestCase):
    def test_adaptive_response_reduces_lag_without_exceeding_velocity_limits(self):
        fixed = MotionController(smoothing=0.08)
        adaptive = MotionController(smoothing=0.08, adaptive=True)
        target = ArmAngles(NEUTRAL[0], 0, rotation=radians(20))
        for index in range(25):
            now = index * 0.01
            for controller in (fixed, adaptive):
                controller.set_targets(ArmTargets(target), now, now)
                previous = controller.positions
                controller.step(0.01, now)
                self.assertTrue(all(abs(current - old) <= 0.015 + 1e-12
                                    for current, old in zip(controller.positions, previous)))
        self.assertLess(radians(20) - adaptive.positions[2],
                        0.9 * (radians(20) - fixed.positions[2]))
        for index in range(600):
            adaptive.step(0.01, 1 + index * 0.01)
        for actual, neutral in zip(adaptive.positions, NEUTRAL):
            self.assertAlmostEqual(actual, neutral, places=5)

    def test_adaptive_filter_still_suppresses_small_stationary_jitter(self):
        controller = MotionController(smoothing=0.08, adaptive=True)
        for index in range(200):
            now = index * 0.01
            target = ArmAngles(NEUTRAL[0], 0, rotation=0.005 if index % 2 else -0.005)
            controller.set_targets(ArmTargets(target), now, now)
            controller.step(0.01, now)
            self.assertLess(abs(controller.positions[2]), 0.002)

    def test_velocity_and_joint_limits_under_extreme_targets(self) -> None:
        controller = MotionController()
        target = ArmAngles(1000.0, -1000.0, 1000.0, -1000.0)
        for index in range(600):
            now = index * 0.01
            controller.set_targets(ArmTargets(target, target), now, now)
            previous = controller.positions
            positions = controller.step(0.01, now)
            for old, current, spec in zip(previous, positions, JOINTS):
                self.assertLessEqual(abs(current - old), 0.015 + 1e-12)
                self.assertGreaterEqual(current, spec.lower)
                self.assertLessEqual(current, spec.upper)

    def test_stalled_vision_returns_to_neutral_without_new_input(self) -> None:
        controller = MotionController()
        for index in range(100):
            now = index * 0.01
            controller.set_targets(ArmTargets(ArmAngles(radians(100), 1.0), None), now, now)
            controller.step(0.01, now)
        raised = controller.positions[0]
        for index in range(1, 501):
            controller.step(0.01, 0.99 + index * 0.01)
        self.assertGreater(raised, NEUTRAL[0] + 0.5)
        for actual, safe in zip(controller.positions, NEUTRAL):
            self.assertAlmostEqual(actual, safe, places=5)
        self.assertEqual(controller.tracking(6.0), (False, False))

    def test_loss_is_independent_for_each_arm(self) -> None:
        controller = MotionController()
        for index in range(200):
            now = index * 0.01
            controller.set_targets(ArmTargets(None, ArmAngles(1.0, 0.8)), now, now)
            controller.step(0.01, now)
        self.assertAlmostEqual(controller.positions[0], NEUTRAL[0])
        self.assertAlmostEqual(controller.positions[1], 0.0)
        self.assertAlmostEqual(controller.positions[4], 1.0, places=5)

    def test_nonfinite_or_future_target_invalidates_tracking(self) -> None:
        for target, captured in ((ArmAngles(float("nan"), 0), 0), (ArmAngles(1, 1), 10),
                                 (ArmAngles(0, 0, float("nan"), 0), 0),
                                 (ArmAngles(0, 0, 0, float("inf")), 0)):
            with self.subTest(target=target, captured=captured):
                controller = MotionController()
                with self.assertRaises(ValueError):
                    controller.set_targets(ArmTargets(target, target), captured, 0)
                self.assertEqual(controller.tracking(0), (False, False))

    def test_old_packets_cannot_override_recent_target(self) -> None:
        controller = MotionController()
        controller.set_targets(ArmTargets(ArmAngles(1, 1)), 10, 10)
        with self.assertRaises(ValueError):
            controller.set_targets(ArmTargets(ArmAngles(2, 2)), 9, 10)

    def test_stale_packet_is_safe_even_when_it_arrives_now(self) -> None:
        controller = MotionController()
        controller.set_targets(ArmTargets(ArmAngles(2, 2)), 0, 1)
        self.assertEqual(controller.step(0.01, 1), NEUTRAL)

    def test_invalid_configuration_or_timestep_is_rejected(self) -> None:
        for options in ({"timeout": 0}, {"max_speed": float("nan")}, {"smoothing": -1}):
            with self.assertRaises(ValueError):
                MotionController(**options)
        for dt in (0, -1, 0.5, float("inf")):
            with self.assertRaises(ValueError):
                MotionController().step(dt, 0)
