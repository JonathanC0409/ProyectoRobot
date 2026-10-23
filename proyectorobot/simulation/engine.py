"""Motor físico sin dependencia de webcam ni necesidad de contexto gráfico."""

from math import isfinite
from pathlib import Path

import mujoco
import numpy as np

from proyectorobot.control.motion import JOINTS, NEUTRAL, MotionController


MODEL_PATH = Path(__file__).resolve().parent / "assets" / "humanoid.xml"


class SimulationError(RuntimeError):
    """Modelo incompatible, inestabilidad o error de simulación."""


class RobotSimulation:
    def __init__(self, controller: MotionController | None = None) -> None:
        self.controller = controller if controller is not None else MotionController()
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        if (self.model.nq, self.model.nv, self.model.nu) != (len(JOINTS),) * 3:
            raise SimulationError("El humanoide debe tener ocho bisagras y ocho servos.")
        for index, spec in enumerate(JOINTS):
            if self.model.joint(spec.name).id != index:
                raise SimulationError("El orden de las articulaciones no coincide con el controlador.")
            if not np.allclose(self.model.jnt_range[index], (spec.lower, spec.upper)):
                raise SimulationError(f"Límites inconsistentes: {spec.name}.")
            if not np.allclose(self.model.actuator_ctrlrange[index], (spec.lower, spec.upper)):
                raise SimulationError(f"Límites del servo inconsistentes: {spec.name}.")
        self.timestep = float(self.model.opt.timestep)
        self._accumulator = 0.0
        self.guard_count = 0
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("neutral").id)
        if not np.allclose(self.data.qpos, NEUTRAL):
            raise SimulationError("La postura neutral MJCF no coincide con el controlador.")
        mujoco.mj_forward(self.model, self.data)

    def advance(self, elapsed: float, now: float) -> int:
        if not isfinite(elapsed) or elapsed < 0 or not isfinite(now):
            raise ValueError("Tiempo de simulación inválido.")
        self._accumulator += min(elapsed, 0.1)
        steps = 0
        while self._accumulator + 1e-12 >= self.timestep:
            self.data.ctrl[:] = self.controller.step(self.timestep, now)
            previous_positions = self.data.qpos.copy()
            mujoco.mj_step(self.model, self.data)
            if not all(np.isfinite(values).all() for values in (self.data.qpos, self.data.qvel)):
                self.controller.invalidate()
                mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
                raise SimulationError("Estado físico no finito; robot reiniciado a postura segura.")
            lower = np.maximum(self.model.jnt_range[:, 0], previous_positions - 2 * self.timestep)
            upper = np.minimum(self.model.jnt_range[:, 1], previous_positions + 2 * self.timestep)
            clamped = np.clip(self.data.qpos, lower, upper)
            changed = clamped != self.data.qpos
            self.guard_count += int(np.count_nonzero(np.abs(clamped - self.data.qpos) > 1e-9))
            self.data.qvel[changed] = (clamped[changed] - previous_positions[changed]) / self.timestep
            self.data.qpos[:] = clamped
            np.clip(self.data.qvel, -2.0, 2.0, out=self.data.qvel)
            self._accumulator -= self.timestep
            steps += 1
        mujoco.mj_forward(self.model, self.data)
        return steps

    def settle_safe(self) -> None:
        self.controller.invalidate()
        for index in range(600):
            self.advance(self.timestep, index * self.timestep)

    @property
    def positions(self) -> tuple[float, ...]:
        return tuple(float(value) for value in self.data.qpos)
