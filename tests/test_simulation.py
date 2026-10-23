"""Modelo y dinámica reales, sin contexto gráfico ni webcam."""

from math import radians
import unittest

try:
    import mujoco
    import numpy as np
    from proyectorobot.simulation.engine import RobotSimulation
    MUJOCO_AVAILABLE = True
except (ImportError, OSError):
    MUJOCO_AVAILABLE = False

from proyectorobot.control.motion import ArmAngles, ArmTargets, JOINTS, NEUTRAL
from proyectorobot.simulation.demo import demo_targets


@unittest.skipUnless(MUJOCO_AVAILABLE, "MuJoCo no instalado; Fase 1 sigue siendo utilizable")
class SimulationTests(unittest.TestCase):
    def test_neutral_pose_is_stable(self) -> None:
        simulation = RobotSimulation()
        for index in range(500):
            simulation.advance(0.005, index * 0.005)
        np.testing.assert_allclose(simulation.positions, NEUTRAL, atol=1e-6)
        self.assertEqual(simulation.model.nq, 8)
        self.assertEqual(simulation.guard_count, 0)

    def test_anatomical_left_and_right_move_outward_and_up(self) -> None:
        simulation = RobotSimulation()
        simulation.data.qpos[:] = 0
        mujoco.mj_forward(simulation.model, simulation.data)
        initial = simulation.data.site_xpos.copy()
        simulation.data.qpos[:] = (radians(90), 0, 0, 0, radians(90), 0, 0, 0)
        mujoco.mj_forward(simulation.model, simulation.data)
        left = simulation.data.site("left_wrist").xpos
        right = simulation.data.site("right_wrist").xpos
        self.assertGreater(left[0], 0.7)
        self.assertLess(right[0], -0.7)
        self.assertGreater(left[2], initial[0, 2] + 0.4)
        self.assertGreater(right[2], initial[1, 2] + 0.4)

    def test_demo_exercises_all_eight_physical_joints_within_limits(self) -> None:
        simulation = RobotSimulation()
        minimum = np.array(simulation.positions)
        maximum = minimum.copy()
        for index in range(4800):
            now = index * 0.005
            simulation.controller.set_targets(demo_targets(now), now, now)
            previous_positions = np.array(simulation.positions)
            simulation.advance(0.005, now)
            positions = np.array(simulation.positions)
            self.assertTrue(np.all(np.abs(positions - previous_positions) <= 0.01 + 1e-12))
            minimum = np.minimum(minimum, positions)
            maximum = np.maximum(maximum, positions)
            for current, spec in zip(positions, JOINTS):
                self.assertGreaterEqual(current, spec.lower)
                self.assertLessEqual(current, spec.upper)
            self.assertLessEqual(float(np.abs(simulation.data.qvel).max()), 2.0 + 1e-12)
        self.assertTrue(np.all(maximum - minimum > radians(40)))
        simulation.settle_safe()
        np.testing.assert_allclose(simulation.positions, NEUTRAL, atol=radians(1))

    def test_independent_axes_and_connected_segments(self) -> None:
        simulation = RobotSimulation()
        for side, offset in (("left", 0), ("right", 4)):
            for joint in range(4):
                with self.subTest(side=side, joint=joint):
                    simulation.data.qpos[:] = 0
                    simulation.data.qpos[offset + 3] = radians(60)
                    mujoco.mj_forward(simulation.model, simulation.data)
                    initial = simulation.data.site(f"{side}_wrist").xpos.copy()
                    other = "right" if side == "left" else "left"
                    other_initial = simulation.data.site(f"{other}_wrist").xpos.copy()
                    simulation.data.qpos[offset + joint] += radians(30)
                    mujoco.mj_forward(simulation.model, simulation.data)
                    self.assertGreater(np.linalg.norm(simulation.data.site(f"{side}_wrist").xpos - initial), 0.05)
                    np.testing.assert_allclose(simulation.data.site(f"{other}_wrist").xpos, other_initial)
                    shoulder = simulation.data.body(f"{side}_upper_arm").xpos
                    elbow = simulation.data.body(f"{side}_forearm").xpos
                    wrist = simulation.data.site(f"{side}_wrist").xpos
                    self.assertAlmostEqual(float(np.linalg.norm(elbow - shoulder)), 0.30)
                    self.assertAlmostEqual(float(np.linalg.norm(wrist - elbow)), 0.27)
                    if joint == 1:
                        self.assertLess(wrist[1], initial[1])

    def test_demo_isolates_axes_with_bent_elbow_for_axial_rotation(self) -> None:
        for index in range(8):
            targets = demo_targets(index * 3 + 1.5)
            values = targets.left.values + targets.right.values
            changed = [axis for axis, (value, neutral) in enumerate(zip(values, NEUTRAL))
                       if abs(value - neutral) > 1e-9]
            self.assertEqual(changed, [index, index + 1] if index % 4 == 2 else [index])
            if index % 4 == 2:
                following = demo_targets(index * 3 + 1.7)
                following_values = following.left.values + following.right.values
                moving = [axis for axis, (before, after) in enumerate(zip(values, following_values))
                          if abs(before - after) > 1e-9]
                self.assertEqual(moving, [index])

    def test_excessive_wall_time_does_not_trigger_unbounded_catchup(self) -> None:
        simulation = RobotSimulation()
        self.assertEqual(simulation.advance(100, 100), 20)

    def test_direct_extreme_commands_cannot_escape_model_limits(self) -> None:
        simulation = RobotSimulation()
        for index in range(600):
            now = index * 0.005
            simulation.controller.set_targets(ArmTargets(ArmAngles(100, -100), ArmAngles(-100, 100)), now, now)
            simulation.advance(0.005, now)
            for value, spec in zip(simulation.positions, JOINTS):
                self.assertTrue(spec.lower <= value <= spec.upper)
