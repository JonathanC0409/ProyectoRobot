"""El arranque en 2D no bloquea referencias 3D válidas que llegan después."""

from dataclasses import replace
from math import radians
from types import SimpleNamespace
import unittest

from proyectorobot.control.pose_quality import PoseQualityFilter
from proyectorobot.control.retargeting import PoseRetargeter
from proyectorobot.control.spatial import SpatialArm, SpatialMapper, inspect_world
from test_spatial import MUJOCO_AVAILABLE, world_observation


class CalibrationWindowTests(unittest.TestCase):
    def test_pending_depth_calibration_window_spans_one_second_at_thirty_fps(self):
        mapper = SpatialMapper()
        neutral = SpatialArm((0, 0, -1), (0, 0, -1), 0.6, 0.5)
        for index in range(36):
            status = mapper.observe_neutral((neutral, neutral), index * 33)
        self.assertEqual(status, ("LISTA", "LISTA"))
        self.assertTrue(all(reference is not None for reference in mapper.reference))


@unittest.skipUnless(MUJOCO_AVAILABLE, "MuJoCo requerido para el oracle cinemático")
class LateDepthCalibrationTests(unittest.TestCase):
    def initial_2d(self):
        converter = PoseRetargeter(recovery_ms=0, depth_grace_ms=200)
        for timestamp in range(0, 1300, 100):
            result = converter.update(replace(world_observation(timestamp), pose_world=()), 640, 480)
        self.assertTrue(converter.calibrated)
        self.assertIn("SIN WORLD", result.depth_status)
        return converter

    def test_world_arriving_after_2d_calibration_can_activate_depth_without_restart(self):
        converter = self.initial_2d()
        for timestamp in range(1300, 2900, 100):
            result = converter.update(world_observation(timestamp), 640, 480)
        self.assertIn("I:3D D:3D", result.status)
        self.assertIn("I:ACTIVA", result.depth_status)
        for index in range(1, 13):
            result = converter.update(world_observation(2800 + index * 100,
                                                       (0, index * 5, 0, index * 3)), 640, 480)
        self.assertIn("I:3D", result.status)
        self.assertGreater(result.targets.left.flexion, radians(55))
        self.assertGreater(result.targets.left.elbow, radians(30))

    def test_each_arm_can_acquire_depth_while_other_is_not_visible(self):
        converter = self.initial_2d()
        for timestamp in range(1300, 2900, 100):
            observation = world_observation(timestamp)
            world = list(observation.pose_world)
            world[14] = replace(world[14], visibility=0.1)
            result = converter.update(replace(observation, pose_world=tuple(world)), 640, 480)
        self.assertIn("I:3D D:2D", result.status)
        self.assertIn("D:CONFIANZA BRAZO", result.depth_status)

    def test_raised_or_reaching_pose_does_not_become_a_false_neutral(self):
        converter = self.initial_2d()
        for timestamp in range(1300, 3300, 100):
            result = converter.update(world_observation(timestamp, (0, 60, 0, 45)), 640, 480)
        self.assertIsNone(converter._spatial.reference[0])
        self.assertIn("I:BAJAR BRAZO PARA 3D", result.depth_status)
        self.assertIsNotNone(converter._spatial.reference[1])

    def test_old_valid_reference_is_not_changed_when_other_side_calibrates_late(self):
        converter = self.initial_2d()
        for timestamp in range(1300, 2900, 100):
            observation = world_observation(timestamp)
            world = list(observation.pose_world)
            world[14] = replace(world[14], visibility=0.1)
            converter.update(replace(observation, pose_world=tuple(world)), 640, 480)
        original = converter._spatial.reference[0]
        for timestamp in range(2900, 4600, 100):
            converter.update(world_observation(timestamp, (35, 40, 0, 70)), 640, 480)
        self.assertEqual(converter._spatial.reference[0], original)
        self.assertIsNotNone(converter._spatial.reference[1])

    def test_optional_world_scores_and_filter_drive_forward_motion_after_late_calibration(self):
        from proyectorobot.vision.detector import convert_world_landmarks

        converter = PoseRetargeter(recovery_ms=0, depth_grace_ms=200)
        quality = PoseQualityFilter()
        for timestamp in range(0, 1300, 100):
            observation = replace(world_observation(timestamp), pose_world=())
            converter.update(quality.update(observation, 640, 480).observation, 640, 480)
        for timestamp in range(1300, 4500, 100):
            amount = min(60, max(0, (timestamp - 2900) / 100 * 4))
            observation = world_observation(timestamp, (0, amount, 0, amount / 2))
            raw = [SimpleNamespace(x=point.x, y=point.y, z=point.z, visibility=None, presence=None)
                   for point in observation.pose_world]
            observation = replace(observation, pose_world=convert_world_landmarks(raw, observation.pose))
            filtered = quality.update(observation, 640, 480).observation
            result = converter.update(filtered, 640, 480)
        self.assertIn("I:3D", result.status)
        self.assertGreater(result.targets.left.flexion, radians(55))

    def test_world_rejections_explain_why_depth_is_disabled(self):
        initial = world_observation(0)
        cases = [(replace(initial, pose_world=()), "SIN WORLD"),
                 (replace(initial, pose=()), "SIN POSE")]
        world = list(initial.pose_world)
        world[11] = replace(world[11], presence=0)
        cases.append((replace(initial, pose_world=tuple(world)), "CONFIANZA WORLD HOMBROS"))
        for observation, reason in cases:
            with self.subTest(reason=reason):
                measured, reasons = inspect_world(observation, 640, 480)
                self.assertEqual(measured, (None, None))
                self.assertEqual(reasons, (reason, reason))
