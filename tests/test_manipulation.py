"""Pruebas de física real: contactos opuestos, transporte y liberación sin cámara."""

from contextlib import redirect_stderr
import io
from dataclasses import replace
from math import isfinite
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from proyectorobot.control.hands import FIST, MAX_CURL, OPEN, HandCommand, HandTargets
from proyectorobot.control.motion import JOINTS, ArmTargets
from proyectorobot.simulation.grasp_demo import run_headless
from proyectorobot.simulation.hand_model import CONTROLS, FINGERS, scene_xml
from proyectorobot.simulation.hands_app import AssistedControl, hand_demo_targets, main, packet_usable
from proyectorobot.simulation.manipulation import ManipulationSimulation


class ManipulationPhysicsTests(unittest.TestCase):
    def test_original_arm_axes_and_limits_are_preserved_by_name(self):
        simulation = ManipulationSimulation()
        original = mujoco.MjModel.from_xml_path("proyectorobot/simulation/assets/humanoid.xml")
        for spec in JOINTS:
            np.testing.assert_allclose(simulation.model.joint(spec.name).axis, original.joint(spec.name).axis)
            np.testing.assert_allclose(simulation.model.joint(spec.name).range, (spec.lower, spec.upper))
        self.assertEqual(simulation.model.nu, 14)
        self.assertEqual(simulation.model.neq, 4)
        self.assertTrue(np.all(simulation.model.eq_type == mujoco.mjtEq.mjEQ_JOINT))
        for index in range(3):
            self.assertEqual(simulation.model.joint(f"cube_{index}_free").type, mujoco.mjtJoint.mjJNT_FREE)
        self.assertNotIn("weld", scene_xml())
        self.assertEqual(simulation.model.nmocap, 0)

    def test_each_finger_control_moves_independently_and_other_three_share_synergy(self):
        simulation = ManipulationSimulation()
        step = 0
        for side in ("left", "right"):
            for finger_index, finger in enumerate(CONTROLS):
                values = [0.0, 0.0, 0.0]
                values[finger_index] = 1
                targets = HandTargets(**{side: HandCommand(*values)})
                for _ in range(1200):
                    now = step * simulation.timestep
                    simulation.hands.set_targets(targets, now, now)
                    simulation.advance(simulation.timestep, now)
                    step += 1
                self.assertGreater(simulation.data.joint(f"{side}_{finger}_joint").qpos[0], 1.0)
                for other_side in ("left", "right"):
                    for other_finger in FINGERS:
                        position = simulation.data.joint(f"{other_side}_{other_finger}_joint").qpos[0]
                        spec = simulation.model.joint(f"{other_side}_{other_finger}_joint")
                        self.assertGreaterEqual(position, spec.range[0] - 0.005)
                        self.assertLessEqual(position, spec.range[1] + 0.005)
                        moved = other_side == side and (other_finger == finger or
                                                       finger == "middle" and other_finger in FINGERS[2:])
                        self.assertAlmostEqual(float(position), MAX_CURL if moved else 0.0, delta=0.015)

    def test_physical_grasp_lift_transfer_and_release_both_anatomical_sides(self):
        reports = [run_headless(side) for side in ("left", "right")]
        for report in reports:
            self.assertTrue(report.passed, report)
            self.assertGreater(report.lift_m, 0.12)
            self.assertGreater(report.transfer_m, 0.13)
        self.assertAlmostEqual(reports[0].lift_m, reports[1].lift_m, delta=0.001)
        self.assertAlmostEqual(reports[0].transfer_m, reports[1].transfer_m, delta=0.001)

    def test_disabling_finger_contacts_prevents_grasp(self):
        simulation = ManipulationSimulation()
        for index in range(simulation.model.ngeom):
            if simulation.model.geom_contype[index] == 2:
                simulation.model.geom_contype[index] = 0
                simulation.model.geom_conaffinity[index] = 0
        report = run_headless(simulation=simulation)
        self.assertFalse(report.passed)
        self.assertEqual(report.opposing_contact_steps, 0)
        self.assertLess(report.lift_m, 0.001)
        self.assertLess(report.transfer_m, 0.001)

    def test_stalled_vision_opens_both_hands_and_recovers_without_resetting_objects(self):
        simulation = ManipulationSimulation()
        for step in range(2000):
            now = step * simulation.timestep
            simulation.hands.set_targets(HandTargets(FIST, FIST), now, now)
            simulation.advance(simulation.timestep, now)
        self.assertGreater(min(simulation.hands.positions), 1.0)
        before = simulation.data.body("cube_0").xpos.copy()
        for step in range(2000):
            simulation.advance(simulation.timestep, 5 + step * simulation.timestep)
        self.assertLess(max(simulation.hands.positions), 0.001)
        np.testing.assert_allclose(simulation.data.body("cube_0").xpos, before, atol=0.001)
        simulation.hands.set_targets(HandTargets(right=FIST), 10, 10)
        simulation.advance(simulation.timestep, 10)
        self.assertLess(simulation.hands.positions[3], 0.005)

    def test_bad_times_are_rejected_without_mutating_physics(self):
        simulation = ManipulationSimulation()
        before = simulation.data.qpos.copy()
        for elapsed, now in ((-1, 0), (0, float("nan")), (float("inf"), 0)):
            with self.assertRaises(ValueError):
                simulation.advance(elapsed, now)
            np.testing.assert_equal(before, simulation.data.qpos)
        self.assertTrue(all(map(isfinite, simulation.positions)))


class HandsAppTests(unittest.TestCase):
    def test_assistance_cannot_override_stale_untracked_future_or_paused_packets(self):
        from proyectorobot.control.motion import ArmAngles
        from proyectorobot.data.types import Observation
        from proyectorobot.vision.worker import VisionPacket
        packet = VisionPacket(Observation(1000), 1.0, 10, ArmTargets(left=ArmAngles(0, 0)))
        self.assertTrue(packet_usable(packet, 0, 1.2))
        for sample, generation, now in ((packet, 0, 1.36), (packet, -1, 1.1),
                                        (packet, 1, 1.1), (None, 0, 1.1),
                                        (replace(packet, targets=ArmTargets()), 0, 1.1),
                                        (replace(packet, captured_at=2.0), 0, 1.1)):
            self.assertFalse(packet_usable(sample, generation, now))

    def test_camera_recalibration_resumes_after_robot_pause_without_old_keyboard_orders(self):
        control = AssistedControl("left", True)
        control.on_key(ord("R"))
        control.change_generation(-1)
        control.on_key(ord("F"))
        self.assertTrue(control.paused)
        control.change_generation(2)
        self.assertFalse(control.paused)
        self.assertEqual(control.hands(), HandTargets())

    def test_assisted_keys_change_only_joint_commands_and_pause_clears_override(self):
        control = AssistedControl("left")
        control.on_key(ord("F"))
        self.assertEqual(control.hands().left, FIST)
        self.assertIsNone(control.hands().right)
        control.on_key(ord("2"))
        control.on_key(ord("O"))
        self.assertEqual(control.hands().right, OPEN)
        control.on_key(ord("B"))
        self.assertTrue(control.assisted)
        self.assertIsNotNone(control.arm_targets.right)
        control.on_key(ord("R"))
        self.assertTrue(control.paused)
        self.assertEqual(control.hands(), HandTargets())
        control.on_key(ord("C"))
        self.assertFalse(control.paused)

    def test_headless_cannot_open_camera_and_incomplete_demos_are_rejected(self):
        for argv in (["--headless"], ["--grasp-demo", "--duration", "2"], ["--duration", "nan"]):
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                main(argv)
            self.assertEqual(error.exception.code, 2)

    def test_hand_demo_covers_each_control_without_coupling_other_hand(self):
        for index in range(6):
            targets = hand_demo_targets(index * 3 + 1)
            values = targets.left.values + targets.right.values
            self.assertAlmostEqual(values[index], 1.0)
            self.assertAlmostEqual(sum(values), 1.0)

    def test_live_simulation_errors_are_actionable_and_resources_close(self):
        from proyectorobot.simulation.hands_app import build_parser, run_graphical
        from proyectorobot.vision.worker import VisionWorkerError
        simulation = ManipulationSimulation()
        with patch("proyectorobot.simulation.viewer.RobotViewer") as viewer_type, patch(
            "proyectorobot.vision.worker.VisionStream"
        ) as stream_type:
            viewer = viewer_type.return_value.__enter__.return_value
            viewer.running.return_value = True
            viewer.key_events = []
            viewer.reset.is_set.return_value = False
            viewer.recalibrate.is_set.return_value = False
            stream = stream_type.return_value.__enter__.return_value
            stream.generation = 0
            stream.poll.side_effect = VisionWorkerError("Cámara desconectada")
            with self.assertRaisesRegex(VisionWorkerError, "Cámara desconectada"):
                run_graphical(build_parser().parse_args(["--no-hands"]), simulation)
            viewer_type.return_value.__exit__.assert_called_once()
            stream_type.return_value.__exit__.assert_called_once()
            self.assertEqual(simulation.controller.tracking(0), (False, False))
            self.assertEqual(simulation.hands.tracking(0), (False, False))
            config = stream_type.call_args.args[0]
            self.assertFalse(config.track_hands)
            self.assertFalse(config.recognize_hands)
