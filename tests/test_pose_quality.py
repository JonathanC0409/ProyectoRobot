"""Regresiones de cruces, saltos de pose y ambigüedad delante/detrás."""

from dataclasses import replace
from math import pi, radians
import unittest

from proyectorobot.control.pose_quality import PoseQualityFilter
from proyectorobot.control.retargeting import PoseRetargeter
from proyectorobot.data.types import Landmark, Observation
from test_retargeting import frontal_pose
from test_spatial import MUJOCO_AVAILABLE, calibrate_world, world_observation


def sample(timestamp, pose=None):
    return Observation(timestamp, frontal_pose() if pose is None else tuple(pose))


class PoseQualityTests(unittest.TestCase):
    def prepared(self):
        filtering = PoseQualityFilter()
        for timestamp in range(0, 600, 100):
            filtering.update(sample(timestamp), 640, 480)
        return filtering

    def test_single_frame_wrong_elbow_plane_is_rejected_and_not_drawn(self):
        filtering = self.prepared()
        result = filtering.update(sample(600, frontal_pose(left=(0, pi / 2))), 640, 480)
        self.assertEqual(result.left, "SALTO")
        self.assertFalse(result.observation.pose[13].visible(0.6))
        self.assertFalse(result.observation.pose[15].visible(0.6))
        self.assertTrue(result.observation.pose[14].visible(0.6))

    def test_swapped_elbows_and_wrists_do_not_silently_command_opposite_arms(self):
        filtering = self.prepared()
        pose = list(frontal_pose())
        for first, second in ((13, 14), (15, 16)):
            pose[first], pose[second] = pose[second], pose[first]
        result = filtering.update(sample(600, pose), 640, 480)
        self.assertIn(result.left, ("HUESO INCOHERENTE", "SALTO"))
        self.assertIn(result.right, ("HUESO INCOHERENTE", "SALTO"))
        self.assertFalse(result.observation.pose[15].visible(0.6))
        self.assertFalse(result.observation.pose[16].visible(0.6))

    def test_gradual_real_cross_body_motion_is_not_relabelled_or_blocked(self):
        filtering = self.prepared()
        for index in range(1, 19):
            result = filtering.update(sample(500 + index * 100, frontal_pose(
                left=(0, -radians(index * 10)), right=(0, radians(index * 10)),
            )), 640, 480)
            self.assertEqual((result.left, result.right), ("2D", "2D"))
        self.assertTrue(result.observation.pose[15].visible(0.6))

    def test_scale_and_body_translation_do_not_count_as_arm_jump(self):
        filtering = self.prepared()
        pose = tuple(replace(point, x=0.8 * point.x + 0.05, y=0.8 * point.y + 0.03)
                     for point in frontal_pose())
        result = filtering.update(sample(600, pose), 640, 480)
        self.assertEqual((result.left, result.right), ("2D", "2D"))

    def test_occlusion_requires_three_fresh_samples_and_preserves_other_arm(self):
        filtering = self.prepared()
        pose = list(frontal_pose())
        pose[13] = replace(pose[13], visibility=0.2)
        result = filtering.update(sample(600, pose), 640, 480)
        self.assertEqual((result.left, result.right), ("OCULTO", "2D"))
        for timestamp in (700, 800):
            result = filtering.update(sample(timestamp), 640, 480)
            self.assertEqual(result.left, "RECUPERANDO")
            self.assertEqual(result.right, "2D")
        self.assertEqual(filtering.update(sample(900), 640, 480).left, "2D")

    def test_foreshortened_forearm_without_world_does_not_guess_depth(self):
        filtering = self.prepared()
        pose = list(frontal_pose())
        pose[15] = replace(pose[15], y=pose[13].y + (pose[15].y - pose[13].y) * 0.3)
        result = filtering.update(sample(600, pose), 640, 480)
        self.assertEqual(result.left, "SIN PROFUNDIDAD")
        self.assertFalse(result.observation.pose[15].visible(0.6))

    def test_recalibration_forgets_bone_reference_and_timestamp(self):
        filtering = self.prepared()
        filtering.reset()
        result = filtering.update(sample(0), 640, 480)
        self.assertEqual(result.left, "2D")

    def test_bone_reference_is_not_learned_with_raised_arms(self):
        filtering = PoseQualityFilter()
        for timestamp in range(0, 800, 100):
            filtering.update(sample(timestamp, frontal_pose(left=(pi / 2, 0))), 640, 480)
        self.assertIsNone(filtering._lengths[0])
        for timestamp in range(800, 2300, 100):
            filtering.update(sample(timestamp), 640, 480)
        self.assertIsNotNone(filtering._lengths[0])

    def test_missing_torso_nan_and_stale_times_fail_safely(self):
        filtering = self.prepared()
        result = filtering.update(Observation(600), 640, 480)
        self.assertEqual(result.observation.pose, ())
        with self.assertRaises(ValueError):
            filtering.update(sample(600), 640, 480)
        pose = list(frontal_pose())
        pose[13] = Landmark(float("nan"), 0.5)
        result = filtering.update(sample(700, pose), 640, 480)
        self.assertEqual(result.left, "OCULTO")


@unittest.skipUnless(MUJOCO_AVAILABLE, "MuJoCo requerido para el oracle cinemático")
class DepthStabilityTests(unittest.TestCase):
    def test_mirrored_depth_with_identical_image_is_rejected(self):
        filtering = PoseQualityFilter()
        for timestamp in range(0, 1000, 100):
            filtering.update(world_observation(timestamp), 640, 480)
        for index in range(1, 8):
            original = world_observation(900 + index * 100, (0, 0, 0, index * 10))
            good = filtering.update(original, 640, 480)
        world = tuple(replace(point, z=-point.z) for point in original.pose_world)
        flipped = replace(original, timestamp_ms=1700, pose_world=world)
        result = filtering.update(flipped, 640, 480)
        self.assertEqual(result.left, "PROFUNDIDAD AMBIGUA")
        self.assertTrue(good.observation.pose[15].visible(0.6))
        self.assertFalse(result.observation.pose[15].visible(0.6))
        self.assertFalse(result.observation.pose_world[15].confident(0.6))
        self.assertTrue(result.observation.pose[16].visible(0.6))

    def test_reliable_forward_motion_survives_foreshortening(self):
        filtering = PoseQualityFilter()
        retargeter = PoseRetargeter(recovery_ms=0, depth_grace_ms=200)
        for timestamp in range(0, 1300, 100):
            quality = filtering.update(world_observation(timestamp), 640, 480)
            retargeter.update(quality.observation, 640, 480)
        for index in range(1, 16):
            quality = filtering.update(world_observation(1200 + index * 100,
                                                        (0, index * 4, 0, index * 3)), 640, 480)
            result = retargeter.update(quality.observation, 640, 480)
        self.assertEqual(quality.left, "3D ESTIMADA")
        self.assertIn("I:3D", result.status)
        self.assertIsNotNone(result.targets.left)
        self.assertGreater(result.targets.left.flexion, radians(50))

    def test_brief_3d_loss_holds_then_stops_flattening_during_recovery(self):
        retargeter = PoseRetargeter(recovery_ms=0, depth_grace_ms=200)
        calibrate_world(retargeter)
        for timestamp in range(1300, 2300, 100):
            result = retargeter.update(world_observation(timestamp, (30, 40, 45, 90)), 640, 480)
        previous = result.targets.left
        missing = replace(world_observation(2300, (30, 40, 45, 90)), pose_world=())
        held = retargeter.update(missing, 640, 480)
        self.assertEqual(held.targets.left, previous)
        self.assertIn("I:3D-PAUSA", held.status)
        retargeter.update(world_observation(2400, (30, 40, 45, 90)), 640, 480)
        recovering = retargeter.update(world_observation(2500, (30, 40, 45, 90)), 640, 480)
        self.assertIsNone(recovering.targets.left)
        self.assertIn("I:3D-RECUPERANDO", recovering.status)
        recovered = retargeter.update(world_observation(2600, (30, 40, 45, 90)), 640, 480)
        self.assertIn("I:3D D:3D", recovered.status)

    def test_depth_hold_is_bounded_and_cannot_reuse_occluded_arm(self):
        retargeter = PoseRetargeter(recovery_ms=0, depth_grace_ms=200)
        calibrate_world(retargeter)
        for timestamp in range(1300, 2300, 100):
            retargeter.update(world_observation(timestamp, (30, 40, 45, 90)), 640, 480)
        original = world_observation(2300, (30, 40, 45, 90))
        pose = list(original.pose)
        pose[13] = replace(pose[13], visibility=0)
        result = retargeter.update(replace(original, pose=tuple(pose)), 640, 480)
        self.assertIsNone(result.targets.left)
        self.assertIsNotNone(result.targets.right)
        with self.assertRaises(ValueError):
            PoseRetargeter(depth_grace_ms=350)
