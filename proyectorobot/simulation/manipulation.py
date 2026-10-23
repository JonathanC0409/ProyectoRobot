"""Dinámica de contacto: nunca corrige ni transporta objetos por coordenadas."""

from math import isfinite

import mujoco
import numpy as np

from proyectorobot.control.hands import HandController
from proyectorobot.control.motion import JOINTS, MotionController
from proyectorobot.simulation.engine import SimulationError
from proyectorobot.simulation.hand_model import CONTROLS, safe_posture, scene_xml


class ManipulationSimulation:
    def __init__(self, side: str = "left") -> None:
        self.side = side
        self.controller = MotionController(neutral=safe_posture(side), smoothing=0.08)
        self.hands = HandController()
        self.model = mujoco.MjModel.from_xml_string(scene_xml(side))
        self.data = mujoco.MjData(self.model)
        self.timestep = float(self.model.opt.timestep)
        self._accumulator = 0.0
        self.guard_count = 0
        self.arm_servos = [self.model.actuator(spec.name + "_servo").id for spec in JOINTS]
        self.hand_servos = [self.model.actuator(f"{hand}_{finger}_servo").id
                            for hand in ("left", "right") for finger in CONTROLS]
        for spec, value in zip(JOINTS, self.controller.neutral):
            self.data.joint(spec.name).qpos[0] = value
        self.data.ctrl[self.arm_servos] = self.controller.neutral
        mujoco.mj_forward(self.model, self.data)

    def advance(self, elapsed: float, now: float) -> int:
        if not isfinite(elapsed) or elapsed < 0 or not isfinite(now):
            raise ValueError("Tiempo de simulación inválido.")
        self._accumulator += min(elapsed, 0.1)
        steps = 0
        while self._accumulator + 1e-12 >= self.timestep:
            self.data.ctrl[self.arm_servos] = self.controller.step(self.timestep, now)
            self.data.ctrl[self.hand_servos] = self.hands.step(self.timestep, now)
            mujoco.mj_step(self.model, self.data)
            if not all(np.isfinite(values).all() for values in (self.data.qpos, self.data.qvel)):
                self.controller.invalidate()
                self.hands.invalidate()
                raise SimulationError("Simulación no finita; detén la ejecución y revisa contactos.")
            self._accumulator -= self.timestep
            steps += 1
        return steps

    def settle_safe(self) -> None:
        self.controller.invalidate()
        self.hands.invalidate()
        for index in range(1200):
            self.advance(self.timestep, index * self.timestep)

    @property
    def positions(self) -> tuple[float, ...]:
        return tuple(float(self.data.joint(spec.name).qpos[0]) for spec in JOINTS)

    def finger_contacts(self, cube: str = "cube_0", side: str | None = None) -> set[str]:
        """Devuelve dedos que ejercen fuerza normal sobre el objeto indicado."""
        side = side or self.side
        cube_geom = self.model.geom(cube + "_geom").id
        contacts = set()
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            if cube_geom not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == cube_geom else contact.geom1
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, other) or ""
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, index, force)
            if name.startswith(side + "_") and force[0] > 0.01:
                contacts.add(name.split("_")[1])
        return contacts
