"""Poses conocidas y variantes de escala, inclinación y oclusión."""

from dataclasses import replace
from math import cos, pi, radians, sin
import unittest

from proyectorobot.control.motion import NEUTRAL
from proyectorobot.control.retargeting import PoseRetargeter
from proyectorobot.data.types import Landmark, Observation


def frontal_pose(left=(0.0, 0.0), right=(0.0, 0.0), width=640, height=480):
    points = [Landmark(0.5, 0.5) for _ in range(33)]
    for side, angles, shoulder, elbow, wrist, hip in (
        (1, left, 11, 13, 15, 23), (-1, right, 12, 14, 16, 24),
    ):
        shoulder_x = 320 + side * 80
        shoulder_y = 180
        elevation, flexion = angles
        elbow_x = shoulder_x + side * 85 * sin(elevation)
        elbow_y = shoulder_y + 85 * cos(elevation)
        wrist_x = elbow_x + side * 75 * sin(elevation + flexion)
        wrist_y = elbow_y + 75 * cos(elevation + flexion)
        for index, horizontal, vertical in (
            (shoulder, shoulder_x, shoulder_y), (elbow, elbow_x, elbow_y),
            (wrist, wrist_x, wrist_y), (hip, 320 + side * 70, 350),
        ):
            points[index] = Landmark(horizontal / width, vertical / height)
    return tuple(points)


def calibrate(retargeter: PoseRetargeter, pose=None, width=640, height=480) -> None:
    pose = frontal_pose() if pose is None else pose
    for timestamp in range(0, 1201, 100):
        retargeter.update(Observation(timestamp, pose), width, height)


class RetargetingTests(unittest.TestCase):
    def test_upper_body_calibrates_and_tracks_with_hips_outside_image(self) -> None:
        def crop_hips(pose):
            points = list(pose)
            for index in (23, 24):
                points[index] = Landmark(0.5, 1.2, visibility=0.1)
            return tuple(points)

        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate(retargeter, crop_hips(frontal_pose()))
        self.assertTrue(retargeter.calibrated)
        result = retargeter.update(Observation(1300, crop_hips(frontal_pose(left=(pi / 2, 0)))), 640, 480)
        self.assertAlmostEqual(result.targets.left.shoulder, NEUTRAL[0] + pi / 2)

    def test_calibration_at_six_fps_tolerates_small_detection_jitter(self) -> None:
        retargeter = PoseRetargeter()
        for index in range(8):
            pose = frontal_pose(left=(radians(3 if index % 2 else -3), 0))
            retargeter.update(Observation(index * 170, pose), 640, 480)
        self.assertTrue(retargeter.calibrated)

    def test_calibration_requires_stable_relaxed_arms_and_time(self) -> None:
        retargeter = PoseRetargeter()
        for timestamp in range(0, 1000, 100):
            result = retargeter.update(Observation(timestamp, frontal_pose()), 640, 480)
            self.assertFalse(retargeter.calibrated)
            self.assertIsNone(result.targets.left)
        retargeter.update(Observation(1000, frontal_pose()), 640, 480)
        self.assertTrue(retargeter.calibrated)

    def test_raised_arms_cannot_be_used_as_neutral(self) -> None:
        retargeter = PoseRetargeter()
        for timestamp in range(0, 2000, 100):
            retargeter.update(Observation(timestamp, frontal_pose(left=(pi / 2, 0))), 640, 480)
        self.assertFalse(retargeter.calibrated)

    def test_exact_independent_elevation_and_anatomical_elbow_conversion(self) -> None:
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate(retargeter)
        result = retargeter.update(Observation(1300, frontal_pose(
            left=(radians(90), radians(60)), right=(radians(30), radians(-70)),
        )), 640, 480)
        self.assertAlmostEqual(result.targets.left.shoulder, NEUTRAL[0] + radians(90))
        self.assertAlmostEqual(result.targets.left.elbow, radians(60))
        self.assertAlmostEqual(result.targets.right.shoulder, NEUTRAL[4] + radians(30))
        self.assertAlmostEqual(result.targets.right.elbow, radians(70))

    def test_non_square_pixels_are_corrected(self) -> None:
        for width, height in ((640, 480), (1280, 720), (800, 600)):
            with self.subTest(width=width, height=height):
                retargeter = PoseRetargeter(recovery_ms=0)
                calibrate(retargeter, frontal_pose(width=width, height=height), width, height)
                pose = frontal_pose(left=(pi / 4, pi / 3), width=width, height=height)
                target = retargeter.update(Observation(1300, pose), width, height).targets.left
                self.assertAlmostEqual(target.shoulder, NEUTRAL[0] + pi / 4)
                self.assertAlmostEqual(target.elbow, pi / 3)

    def test_neutral_offsets_are_subtracted(self) -> None:
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate(retargeter, frontal_pose(left=(radians(12), radians(8))))
        result = retargeter.update(Observation(1300, frontal_pose(left=(radians(42), radians(38)))), 640, 480)
        self.assertAlmostEqual(result.targets.left.shoulder, NEUTRAL[0] + radians(30))
        self.assertAlmostEqual(result.targets.left.elbow, radians(30))

    def test_occluded_elbow_stops_only_that_arm_then_requires_recovery(self) -> None:
        retargeter = PoseRetargeter()
        calibrate(retargeter)
        points = list(frontal_pose())
        points[13] = replace(points[13], visibility=0.1)
        lost = retargeter.update(Observation(1300, tuple(points)), 640, 480)
        self.assertIsNone(lost.targets.left)
        self.assertIsNotNone(lost.targets.right)
        self.assertIsNone(retargeter.update(Observation(1400, frontal_pose()), 640, 480).targets.left)
        self.assertIsNone(retargeter.update(Observation(1500, frontal_pose()), 640, 480).targets.left)
        self.assertIsNotNone(retargeter.update(Observation(1600, frontal_pose()), 640, 480).targets.left)

    def test_missing_or_nonfinite_torso_never_produces_targets(self) -> None:
        for pose in ((), tuple([Landmark(float("nan"), 0.5)] * 33)):
            result = PoseRetargeter().update(Observation(0, pose), 640, 480)
            self.assertIsNone(result.targets.left)
            self.assertIsNone(result.targets.right)

    def test_foreshortened_forearm_is_rejected_instead_of_guessing_depth(self) -> None:
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate(retargeter)
        points = list(frontal_pose())
        points[15] = replace(points[13], y=points[13].y + 0.02)
        self.assertIsNone(retargeter.update(Observation(1300, tuple(points)), 640, 480).targets.left)

    def test_angle_wrap_near_overhead_does_not_reverse_shoulder(self) -> None:
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate(retargeter)
        for timestamp, angle in ((1300, 179), (1400, 181)):
            result = retargeter.update(Observation(timestamp, frontal_pose(left=(radians(angle), 0))), 640, 480)
            self.assertGreater(result.targets.left.shoulder, radians(170))

    def test_recalibration_clears_offsets_and_produces_no_old_commands(self) -> None:
        retargeter = PoseRetargeter()
        calibrate(retargeter)
        retargeter.recalibrate()
        self.assertFalse(retargeter.calibrated)
        self.assertIsNone(retargeter.update(Observation(1300, frontal_pose()), 640, 480).targets.left)

    def test_camera_translation_scaling_and_tilt_preserve_joint_angles(self) -> None:
        def transform(pose):
            transformed = []
            for point in pose:
                horizontal = point.x * 640 - 320
                vertical = point.y * 480 - 240
                rotation = radians(12)
                transformed.append(replace(
                    point,
                    x=(0.7 * (horizontal * cos(rotation) - vertical * sin(rotation)) + 330) / 640,
                    y=(0.7 * (horizontal * sin(rotation) + vertical * cos(rotation)) + 240) / 480,
                ))
            return tuple(transformed)

        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate(retargeter, transform(frontal_pose()))
        pose = transform(frontal_pose(left=(radians(60), radians(45))))
        result = retargeter.update(Observation(1300, pose), 640, 480)
        self.assertAlmostEqual(result.targets.left.shoulder, NEUTRAL[0] + radians(60))
        self.assertAlmostEqual(result.targets.left.elbow, radians(45))

    def test_integration_coordinates_drive_real_robot_then_loss_returns_safe(self) -> None:
        try:
            from proyectorobot.simulation.engine import RobotSimulation
        except (ImportError, OSError):
            self.skipTest("MuJoCo no disponible")
        simulation = RobotSimulation()
        initial_left = simulation.data.site("left_wrist").xpos.copy()
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate(retargeter)
        for timestamp in range(1300, 3301, 50):
            result = retargeter.update(Observation(timestamp, frontal_pose(left=(pi / 4, pi / 4))), 640, 480)
            simulation.controller.set_targets(result.targets, timestamp / 1000, timestamp / 1000)
            simulation.advance(0.05, timestamp / 1000)
        self.assertGreater(simulation.data.site("left_wrist").xpos[0], initial_left[0] + 0.2)
        self.assertGreater(simulation.data.site("left_wrist").xpos[2], initial_left[2] + 0.2)
        for actual, neutral in zip(simulation.positions[4:], NEUTRAL[4:]):
            self.assertAlmostEqual(actual, neutral, places=4)
        for timestamp in range(3350, 6351, 50):
            result = retargeter.update(Observation(timestamp), 640, 480)
            simulation.controller.set_targets(result.targets, timestamp / 1000, timestamp / 1000)
            simulation.advance(0.05, timestamp / 1000)
        for actual, neutral in zip(simulation.positions, NEUTRAL):
            self.assertAlmostEqual(actual, neutral, places=3)
