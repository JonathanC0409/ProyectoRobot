"""Oracle cinemático MJCF, calidad monocular y retorno conservador por eje."""

from dataclasses import replace
from math import degrees, radians
import unittest

from proyectorobot.control.motion import ArmAngles, JOINTS, NEUTRAL, MotionController
from proyectorobot.control.retargeting import PoseRetargeter
from proyectorobot.control.spatial import measure_world, solve_arm
from proyectorobot.data.types import Landmark, Observation

try:
    import mujoco
    import numpy as np
    from proyectorobot.simulation.engine import RobotSimulation
    MUJOCO_AVAILABLE = True
except (ImportError, OSError):
    MUJOCO_AVAILABLE = False


def world_observation(timestamp, left=(0, 0, 0, 0), right=(0, 0, 0, 0)):
    """Proyecta segmentos del modelo real; las entradas están en grados y orden MJCF."""
    simulation = RobotSimulation()
    simulation.data.qpos[:] = [radians(angle) for angle in (*left, *right)]
    mujoco.mj_forward(simulation.model, simulation.data)
    image = [Landmark(0.5, 0.5, visibility=0, presence=0) for _ in range(33)]
    world = list(image)
    for side, indices in (("left", (11, 13, 15)), ("right", (12, 14, 16))):
        positions = (simulation.data.body(f"{side}_upper_arm").xpos,
                     simulation.data.body(f"{side}_forearm").xpos,
                     simulation.data.site(f"{side}_wrist").xpos)
        for index, position in zip(indices, positions):
            horizontal, depth, vertical = position
            image[index] = Landmark(0.5 + horizontal * 0.5, 0.3 + (1.39 - vertical) * 0.65)
            world[index] = Landmark(float(horizontal), float(1 - vertical), float(depth))
    return Observation(timestamp, tuple(image), pose_world=tuple(world))


def calibrate_world(retargeter, left=(0, 0, 0, 0), right=(0, 0, 0, 0)):
    for timestamp in range(0, 1201, 100):
        retargeter.update(world_observation(timestamp, left, right), 640, 480)


@unittest.skipUnless(MUJOCO_AVAILABLE, "MuJoCo no disponible para el oracle cinemático")
class SpatialTests(unittest.TestCase):
    def test_small_elbow_changes_do_not_switch_axial_rotation_by_ninety_degrees(self):
        rotations = []
        for elbow in (5, 10, 14.4, 14.6, 20, 25, 40):
            arm = measure_world(world_observation(0, (90, 0, -90, elbow)), 640, 480)[0]
            rotations.append(solve_arm(arm.upper, arm.forearm, 1).rotation)
        self.assertAlmostEqual(rotations[0], 0, places=6)
        self.assertAlmostEqual(rotations[-1], radians(-90))
        self.assertLess(abs(rotations[3] - rotations[2]), radians(3))
        self.assertTrue(all(later <= earlier + 1e-12
                            for earlier, later in zip(rotations, rotations[1:])))

    def test_rotation_and_abduction_are_continuous_near_front_shoulder_singularity(self):
        previous = ArmAngles(radians(60), radians(80), radians(80), radians(40))
        poses = []
        for flexion in (85.3, 85.5, 85.7):
            arm = measure_world(world_observation(0, (60, flexion, 40, 80)), 640, 480)[0]
            poses.append(solve_arm(arm.upper, arm.forearm, 1, previous))
        for first, second in zip(poses, poses[1:]):
            self.assertLess(abs(second.shoulder - first.shoulder), radians(5))
            self.assertLess(abs(second.rotation - first.rotation), radians(5))

    def test_unreachable_axial_angle_wrap_holds_last_safe_rotation(self):
        previous = ArmAngles(radians(40), radians(90), 0, radians(60))
        for rotation in (179, -179, 178, -178):
            arm = measure_world(world_observation(0, (40, 0, rotation, 90)), 640, 480)[0]
            angles = solve_arm(arm.upper, arm.forearm, 1, previous)
            self.assertAlmostEqual(angles.rotation, previous.rotation)

    def test_frontal_bend_threshold_and_length_jitter_do_not_flip_rotation(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        for timestamp in range(0, 1201, 100):
            retargeter.update(replace(world_observation(timestamp), pose_world=()), 640, 480)
        rotations = []
        for timestamp, elbow, ratio in ((1300, 14.9, 1), (1400, 15.1, 1),
                                         (1500, 90, 1), (1600, 90, 0.79),
                                         (1700, 90, 0.81), (1800, 90, 0.78)):
            observation = world_observation(timestamp, (90, 0, -90, elbow))
            image = list(observation.pose)
            image[15] = replace(image[15], x=image[13].x + (image[15].x - image[13].x) * ratio,
                                y=image[13].y + (image[15].y - image[13].y) * ratio)
            result = retargeter.update(replace(observation, pose=tuple(image), pose_world=()), 640, 480)
            self.assertIn('I:2D-F', result.status)
            rotations.append(result.targets.left.rotation)
        self.assertLess(abs(rotations[1] - rotations[0]), radians(3))
        np.testing.assert_allclose(rotations[2:], [radians(-90)] * 4)

    def test_fast_recovery_still_requires_three_valid_world_samples(self):
        retargeter = PoseRetargeter(recovery_ms=80)
        calibrate_world(retargeter)
        retargeter.lose_tracking()
        for timestamp in (1300, 1340):
            result = retargeter.update(world_observation(timestamp), 640, 480)
            self.assertNotIn('I:3D', result.status)
        result = retargeter.update(world_observation(1380), 640, 480)
        self.assertIn('I:3D D:3D', result.status)

    def test_image_only_goalpost_flexes_upward_in_frontal_plane_for_both_sides(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        for timestamp in range(0, 1201, 100):
            retargeter.update(replace(world_observation(timestamp), pose_world=()), 640, 480)
        simulation = RobotSimulation()
        for timestamp in range(1300, 4401, 100):
            observation = world_observation(timestamp, (90, 0, -90, 90), (90, 0, -90, 90))
            result = retargeter.update(replace(observation, pose_world=()), 640, 480)
            simulation.controller.set_targets(result.targets, timestamp / 1000, timestamp / 1000)
            simulation.advance(0.1, timestamp / 1000)
        for side, target in (("left", result.targets.left), ("right", result.targets.right)):
            self.assertAlmostEqual(target.elbow, radians(90))
            self.assertAlmostEqual(target.rotation, radians(-90))
            self.assertEqual(target.flexion, 0)
            elbow = simulation.data.body(f"{side}_forearm").xpos
            wrist = simulation.data.site(f"{side}_wrist").xpos
            self.assertGreater(wrist[2] - elbow[2], 0.24)
            self.assertLess(abs(wrist[1] - elbow[1]), 0.01)

    def test_invented_world_foreshortening_cannot_erase_visible_frontal_elbow(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate_world(retargeter)
        for timestamp in range(1300, 2301, 100):
            observation = world_observation(timestamp, (90, 0, -90, 90))
            world = list(observation.pose_world)
            shoulder = world[11]
            world[13] = replace(shoulder, x=shoulder.x + 0.05,
                                z=shoulder.z - (0.30 ** 2 - 0.05 ** 2) ** 0.5)
            world[15] = replace(world[13], y=world[13].y - 0.05,
                                z=world[13].z - (0.27 ** 2 - 0.05 ** 2) ** 0.5)
            result = retargeter.update(replace(observation, pose_world=tuple(world)), 640, 480)
        self.assertAlmostEqual(result.targets.left.elbow, radians(90))
        self.assertAlmostEqual(result.targets.left.rotation, radians(-90))
        self.assertIn('I:2D-F D:3D', result.status)

    def test_frontal_goalpost_with_valid_world_preserves_spatial_plane(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate_world(retargeter)
        for timestamp in range(1300, 2301, 100):
            result = retargeter.update(world_observation(timestamp, (90, 0, -90, 90),
                                                        (90, 0, 90, 90)), 640, 480)
        self.assertIn('I:3D D:3D', result.status)
        self.assertAlmostEqual(result.targets.left.rotation, radians(-90))
        self.assertAlmostEqual(result.targets.right.rotation, radians(90))
        self.assertAlmostEqual(result.targets.left.elbow, radians(90))
        self.assertAlmostEqual(result.targets.right.elbow, radians(90))

    def test_forward_depth_and_frontal_l_remain_independent(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate_world(retargeter)
        for timestamp in range(1300, 2401, 100):
            observation = world_observation(timestamp, (0, 45, 0, 60), (90, 0, -90, 90))
            world = list(observation.pose_world)
            world[14] = replace(world[14], visibility=0.1)
            result = retargeter.update(replace(observation, pose_world=tuple(world)), 640, 480)
        self.assertIn('I:3D D:2D-F', result.status)
        self.assertAlmostEqual(result.targets.left.flexion, radians(45))
        self.assertAlmostEqual(result.targets.left.elbow, radians(60))
        self.assertEqual(result.targets.left.rotation, 0)
        self.assertAlmostEqual(result.targets.right.elbow, radians(90))
        self.assertAlmostEqual(result.targets.right.rotation, radians(-90))

    def test_frontal_up_and_down_elbows_have_opposite_planes_and_return_safe(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        for timestamp in range(0, 1201, 100):
            retargeter.update(replace(world_observation(timestamp), pose_world=()), 640, 480)
        simulation = RobotSimulation()
        for timestamp in range(1300, 4401, 100):
            observation = world_observation(timestamp, (90, 0, -90, 90), (90, 0, 90, 90))
            result = retargeter.update(replace(observation, pose_world=()), 640, 480)
            simulation.controller.set_targets(result.targets, timestamp / 1000, timestamp / 1000)
            simulation.advance(0.1, timestamp / 1000)
        self.assertGreater(simulation.data.site('left_wrist').xpos[2],
                           simulation.data.body('left_forearm').xpos[2] + 0.24)
        self.assertLess(simulation.data.site('right_wrist').xpos[2],
                        simulation.data.body('right_forearm').xpos[2] - 0.24)
        for index in range(80):
            simulation.advance(0.1, 4.5 + index * 0.1)
        np.testing.assert_allclose(simulation.positions, NEUTRAL, atol=1e-4)

    def test_foreshortened_image_does_not_enable_frontal_plane_assumption(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        for timestamp in range(0, 1201, 100):
            retargeter.update(replace(world_observation(timestamp), pose_world=()), 640, 480)
        observation = world_observation(1300, (90, 0, -90, 90))
        image = list(observation.pose)
        image[15] = replace(image[15], y=image[13].y + (image[15].y - image[13].y) * 0.5)
        result = retargeter.update(replace(observation, pose=tuple(image), pose_world=()), 640, 480)
        self.assertEqual(result.targets.left.rotation, 0)
        self.assertEqual(result.targets.left.flexion, 0)
        self.assertIn('I:2D D:2D', result.status)

    def test_inverse_matches_real_mjcf_for_both_anatomical_sides(self):
        for values in ((0, 0, 0, 80), (60, 0, 0, 90), (0, 70, 30, 90),
                       (40, 40, -60, 100), (120, 20, 45, 75), (20, -20, 20, 50)):
            with self.subTest(values=values):
                arms = measure_world(world_observation(0, values, values), 640, 480)
                for side, arm in zip((1, -1), arms):
                    self.assertIsNotNone(arm)
                    measured = solve_arm(arm.upper, arm.forearm, side)
                    np.testing.assert_allclose([degrees(value) for value in measured.values], values, atol=1e-6)

    def test_forward_overhead_chooses_continuous_euler_branch(self):
        previous = ArmAngles(0, radians(60), radians(70), radians(20))
        for flexion in (75, 80, 85, 89, 90, 91, 95, 100, 120, 150):
            arm = measure_world(world_observation(0, (0, flexion, 20, 60)), 640, 480)[0]
            self.assertIsNotNone(arm)
            measured = solve_arm(arm.upper, arm.forearm, 1, previous)
            self.assertAlmostEqual(degrees(measured.flexion), flexion, places=5)
            self.assertLess(abs(measured.shoulder - previous.shoulder), radians(5))
            previous = measured

    def test_straight_arm_cannot_report_axial_rotation(self):
        arm = measure_world(world_observation(0, (30, 20, 80, 0)), 640, 480)[0]
        self.assertIsNotNone(arm)
        measured = solve_arm(arm.upper, arm.forearm, 1)
        self.assertAlmostEqual(measured.rotation, 0)
        self.assertAlmostEqual(measured.elbow, 0, places=6)

    def test_quality_rejects_missing_nonfinite_low_confidence_and_inconsistent_world(self):
        initial = world_observation(0)
        self.assertEqual(measure_world(replace(initial, pose_world=()), 640, 480), (None, None))
        for altered in (Landmark(float('nan'), 0, 0), Landmark(0, 0, visibility=0.1),
                        Landmark(10, 10, 10)):
            world = list(initial.pose_world)
            world[13] = altered
            left, right = measure_world(replace(initial, pose_world=tuple(world)), 640, 480)
            self.assertIsNone(left)
            self.assertIsNotNone(right)
        world = list(initial.pose_world)
        world[13] = replace(world[13], y=world[11].y - 0.3)
        self.assertIsNone(measure_world(replace(initial, pose_world=tuple(world)), 640, 480)[0])

    def test_calibration_and_independent_spatial_imitation(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate_world(retargeter)
        for timestamp in range(1300, 2301, 100):
            result = retargeter.update(world_observation(
                timestamp, (35, 40, 60, 90), (65, 20, -45, 70)), 640, 480)
        self.assertIn('I:3D D:3D', result.status)
        np.testing.assert_allclose(result.targets.left.values,
                                   [radians(value) for value in (45, 40, 60, 90)], atol=1e-6)
        np.testing.assert_allclose(result.targets.right.values,
                                   [radians(value) for value in (75, 20, -45, 70)], atol=1e-6)

    def test_world_calibration_subtracts_initial_shoulder_and_elbow_offsets(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate_world(retargeter, (10, 8, 0, 5))
        for timestamp in range(1300, 2201, 100):
            result = retargeter.update(world_observation(timestamp, (40, 38, 0, 65)), 640, 480)
        self.assertIn('I:3D', result.status)
        np.testing.assert_allclose(result.targets.left.values,
                                   [radians(value) for value in (40, 30, 0, 60)], atol=1e-6)

    def test_depth_loss_resets_only_depth_axes_then_recovery_requires_samples(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate_world(retargeter)
        for timestamp in range(1300, 2201, 100):
            result = retargeter.update(world_observation(timestamp, (30, 40, 45, 90)), 640, 480)
        self.assertNotEqual(result.targets.left.flexion, 0)
        missing = replace(world_observation(2300, (30, 40, 45, 90)), pose_world=())
        result = retargeter.update(missing, 640, 480)
        self.assertIn('I:2D', result.status)
        self.assertEqual(result.targets.left.flexion, 0)
        self.assertEqual(result.targets.left.rotation, 0)
        result = retargeter.update(world_observation(2400, (30, 40, 45, 90)), 640, 480)
        self.assertIn('I:2D', result.status)
        result = retargeter.update(world_observation(2500, (30, 40, 45, 90)), 640, 480)
        self.assertIn('I:2D', result.status)
        result = retargeter.update(world_observation(2600, (30, 40, 45, 90)), 640, 480)
        self.assertIn('I:3D', result.status)

    def test_foreshortened_image_uses_world_only_when_valid(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate_world(retargeter)
        for timestamp in range(1300, 2301, 100):
            observation = world_observation(timestamp, (0, 90, 0, 60))
            result = retargeter.update(observation, 640, 480)
        self.assertIn('I:3D', result.status)
        self.assertAlmostEqual(result.targets.left.flexion, radians(90))
        result = retargeter.update(replace(observation, timestamp_ms=2400, pose_world=()), 640, 480)
        self.assertIsNone(result.targets.left)

    def test_ambiguous_shoulder_axes_use_neutral_despite_calibration_offsets(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate_world(retargeter, (10, 8, 0, 5))
        for timestamp in range(1300, 2301, 100):
            result = retargeter.update(world_observation(timestamp, (0, 90, 40, 60)), 640, 480)
        self.assertIn('I:3D', result.status)
        self.assertAlmostEqual(result.targets.left.shoulder, NEUTRAL[0])
        self.assertEqual(result.targets.left.rotation, 0)

    def test_missing_world_during_calibration_never_enables_uncalibrated_depth(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        for timestamp in range(0, 1201, 100):
            retargeter.update(replace(world_observation(timestamp), pose_world=()), 640, 480)
        for timestamp in range(1300, 2301, 100):
            result = retargeter.update(world_observation(timestamp, (30, 40, 45, 90)), 640, 480)
        self.assertIn('I:2D', result.status)
        self.assertEqual(result.targets.left.flexion, 0)

    def test_depth_spike_and_occlusion_do_not_disable_other_arm(self):
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate_world(retargeter)
        result = retargeter.update(world_observation(1300, (0, 70, 50, 100)), 640, 480)
        self.assertIn('I:-- D:3D', result.status)
        self.assertIsNone(result.targets.left)
        image = list(world_observation(1400).pose)
        image[13] = replace(image[13], visibility=0.1)
        result = retargeter.update(replace(world_observation(1400), pose=tuple(image)), 640, 480)
        self.assertIsNone(result.targets.left)
        self.assertIsNotNone(result.targets.right)
        self.assertIn('D:3D', result.status)

    def test_translation_and_uniform_scale_leave_world_angles_unchanged(self):
        original = world_observation(0, (40, 35, -60, 90))
        moved = replace(original, pose_world=tuple(replace(point, x=point.x * 0.8 + 5,
                        y=point.y * 0.8 - 3, z=point.z * 0.8 + 2) for point in original.pose_world))
        first = measure_world(original, 640, 480)[0]
        second = measure_world(moved, 640, 480)[0]
        np.testing.assert_allclose(solve_arm(first.upper, first.forearm, 1).values,
                                   solve_arm(second.upper, second.forearm, 1).values, atol=1e-10)

    def test_world_commands_drive_physics_and_timeout_returns_all_eight_axes(self):
        simulation = RobotSimulation(MotionController(smoothing=0.08))
        retargeter = PoseRetargeter(recovery_ms=0)
        calibrate_world(retargeter)
        for timestamp in range(1300, 4301, 100):
            result = retargeter.update(world_observation(timestamp, (30, 40, 45, 90)), 640, 480)
            simulation.controller.set_targets(result.targets, timestamp / 1000, timestamp / 1000)
            simulation.advance(0.1, timestamp / 1000)
        self.assertGreater(simulation.positions[1], radians(35))
        self.assertGreater(simulation.positions[2], radians(40))
        self.assertGreater(simulation.positions[3], radians(85))
        for value, spec in zip(simulation.positions, JOINTS):
            self.assertTrue(spec.lower <= value <= spec.upper)
        for index in range(80):
            simulation.advance(0.1, 4.4 + index * 0.1)
        np.testing.assert_allclose(simulation.positions, NEUTRAL, atol=1e-4)
