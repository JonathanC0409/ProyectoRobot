"""Secuencia verificable de agarre físico, sin webcam ni uniones al objeto."""

from dataclasses import dataclass
from math import cos, hypot, radians, sin
from typing import TYPE_CHECKING

from proyectorobot.control.hands import HandTargets, OPEN, PINCH
from proyectorobot.control.motion import ArmAngles, ArmTargets
from proyectorobot.control.spatial import solve_arm
from proyectorobot.simulation.hand_model import task_pose

if TYPE_CHECKING:
    from proyectorobot.simulation.manipulation import ManipulationSimulation


DURATION = 19.0


def transfer_pose(side: str) -> ArmAngles:
    angle, azimuth = radians(75), radians(30)
    sign = 1 if side == "left" else -1
    return solve_arm((sign * sin(angle) * cos(azimuth), sin(angle) * sin(azimuth),
                      -cos(angle)), (0, 0, -1), sign)


def command_at(elapsed: float, side: str) -> tuple[ArmTargets, HandTargets, str]:
    if elapsed < 1:
        pose, command, status = task_pose(65), OPEN, "1 ABRIR"
    elif elapsed < 4:
        pose, command, status = task_pose(45), OPEN, "2 ACERCAR"
    elif elapsed < 7:
        pose, command, status = task_pose(45), PINCH, "3 CERRAR"
    elif elapsed < 10:
        pose, command, status = task_pose(75), PINCH, "4 LEVANTAR"
    elif elapsed < 14:
        pose, command, status = transfer_pose(side), PINCH, "5 TRASLADAR"
    else:
        pose, command, status = transfer_pose(side), OPEN, "6 SOLTAR"
    return ArmTargets(**{side: pose}), HandTargets(**{side: command}), status


@dataclass(frozen=True)
class GraspReport:
    side: str
    lift_m: float
    transfer_m: float
    release_height_error_m: float
    opposing_contact_steps: int
    released: bool
    speed_at_rest: float

    @property
    def passed(self) -> bool:
        return (self.lift_m > 0.09 and self.transfer_m > 0.09
                and self.opposing_contact_steps > 100 and self.released
                and self.release_height_error_m < 0.005 and self.speed_at_rest < 0.02)


class GraspMonitor:
    def __init__(self, simulation: "ManipulationSimulation") -> None:
        self.initial = simulation.data.body("cube_0").xpos.copy()
        self.lift = 0.0
        self.transfer = 0.0
        self.contacts = 0

    def observe(self, simulation: "ManipulationSimulation", elapsed: float) -> None:
        position = simulation.data.body("cube_0").xpos
        if 8 < elapsed < 14:
            self.lift = max(self.lift, float(position[2] - self.initial[2]))
            self.transfer = max(self.transfer, hypot(*(position[:2] - self.initial[:2])))
            if {"thumb", "index"} <= simulation.finger_contacts():
                self.contacts += 1

    def report(self, simulation: "ManipulationSimulation") -> GraspReport:
        velocity = simulation.data.joint("cube_0_free").qvel
        return GraspReport(
            simulation.side, self.lift, self.transfer,
            abs(float(simulation.data.body("cube_0").xpos[2] - self.initial[2])),
            self.contacts, not simulation.finger_contacts(), float(sum(velocity ** 2) ** 0.5),
        )


def run_headless(side: str = "left", simulation: "ManipulationSimulation | None" = None
                 ) -> GraspReport:
    from proyectorobot.simulation.manipulation import ManipulationSimulation

    simulation = simulation if simulation is not None else ManipulationSimulation(side)
    monitor = GraspMonitor(simulation)
    for step in range(round(DURATION / simulation.timestep)):
        elapsed = step * simulation.timestep
        arms, hands, _ = command_at(elapsed, side)
        simulation.controller.set_targets(arms, elapsed, elapsed)
        simulation.hands.set_targets(hands, elapsed, elapsed)
        simulation.advance(simulation.timestep, elapsed)
        monitor.observe(simulation, elapsed)
    return monitor.report(simulation)
