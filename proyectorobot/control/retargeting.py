"""Calibración frontal con profundidad estimada opcional y respaldo 2D seguro."""

from collections import deque
from dataclasses import dataclass
from math import atan2, copysign, hypot, pi, radians

from proyectorobot.control.motion import ArmAngles, ArmTargets, NEUTRAL
from proyectorobot.data.types import Observation
from proyectorobot.control.spatial import SpatialArm, SpatialMapper, measure_world, smooth_confidence


def wrap_angle(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


@dataclass(frozen=True)
class MeasuredArm:
    angles: ArmAngles
    upper_ratio: float
    forearm_ratio: float


@dataclass(frozen=True)
class RetargetResult:
    targets: ArmTargets
    status: str
    progress: float = 0.0


class PoseRetargeter:
    """Neutral estable inicial; laterales anatómicos y escalas de imagen corregidas."""

    def __init__(self, calibration_ms: int = 1000, recovery_ms: int = 150) -> None:
        if calibration_ms < 200 or recovery_ms < 0:
            raise ValueError("Calibración >= 200 ms y recuperación >= 0 requeridas.")
        self.calibration_ms = calibration_ms
        self.recovery_ms = recovery_ms
        self._last_ms = -1
        self.recalibrate()

    def recalibrate(self) -> None:
        self._samples = deque(maxlen=90)
        self._zero: tuple[MeasuredArm, MeasuredArm] | None = None
        self._width_ratio: float | None = None
        self._recover_since: list[int | None] = [None, None]
        self._spatial = SpatialMapper(self.recovery_ms)
        self._frontal_active = [False, False]

    @property
    def calibrated(self) -> bool:
        return self._zero is not None

    def lose_tracking(self) -> None:
        self._recover_since = [None, None]
        self._spatial.reset_tracking()
        self._frontal_active = [False, False]
        if not self.calibrated:
            self._samples.clear()

    def _measure(self, observation: Observation, width: int, height: int):
        pose = observation.pose
        if len(pose) != 33 or min(width, height) <= 0:
            return None
        if not all(pose[index].visible(0.6) for index in (11, 12)):
            return None
        shoulders = tuple((pose[index].x * width, pose[index].y * height) for index in (11, 12))
        shoulder_difference = (shoulders[0][0] - shoulders[1][0], shoulders[0][1] - shoulders[1][1])
        torso = hypot(*shoulder_difference)
        if torso < 0.1 * width:
            return None
        outward = (shoulder_difference[0] / torso, shoulder_difference[1] / torso)
        down = (-outward[1], outward[0])
        if down[1] < 0:
            down = (-down[0], -down[1])
        if down[1] < 0.5:
            return None
        width_ratio = torso / height
        if self._width_ratio is not None and width_ratio < 0.6 * self._width_ratio:
            return None
        arms = []
        for side, (shoulder, elbow, wrist) in enumerate(((11, 13, 15), (12, 14, 16))):
            if not all(pose[index].visible(0.6) for index in (shoulder, elbow, wrist)):
                arms.append(None)
                continue
            upper = ((pose[elbow].x - pose[shoulder].x) * width,
                     (pose[elbow].y - pose[shoulder].y) * height)
            forearm = ((pose[wrist].x - pose[elbow].x) * width,
                       (pose[wrist].y - pose[elbow].y) * height)
            lengths = (hypot(*upper) / torso, hypot(*forearm) / torso)
            if min(lengths) < 0.08:
                arms.append(None)
                continue
            if self._zero is not None:
                baseline = self._zero[side]
                if any(current < 0.35 * initial or current > 2.5 * initial
                       for current, initial in zip(lengths, (baseline.upper_ratio, baseline.forearm_ratio))):
                    arms.append(None)
                    continue
            side_outward = outward if side == 0 else (-outward[0], -outward[1])

            def angle(vector: tuple[float, float]) -> float:
                return atan2(sum(value * axis for value, axis in zip(vector, side_outward)),
                             sum(value * axis for value, axis in zip(vector, down)))

            shoulder_angle = angle(upper)
            if shoulder_angle < -pi / 2:
                shoulder_angle += 2 * pi
            elbow_angle = wrap_angle(angle(forearm) - shoulder_angle)
            arms.append(MeasuredArm(ArmAngles(shoulder_angle, elbow_angle), *lengths))
        return tuple(arms), width_ratio

    def _frontal_target(self, side: int, arm: MeasuredArm | None) -> ArmAngles | None:
        """Orienta la bisagra según la flexión visible, sin reconstruir profundidad.

        Solo representa el plano frontal cuando ambos segmentos conservan una
        longitud proyectada cercana a la calibrada y el codo está claramente
        doblado. El signo perdido al tomar abs(codo) determina el plano de
        flexión; una bisagra positiva necesita giro opuesto de ±90 grados.
        """
        if arm is None:
            return None
        baseline = self._zero[side]
        ratios = (arm.upper_ratio / baseline.upper_ratio,
                  arm.forearm_ratio / baseline.forearm_ratio)
        elbow = wrap_angle(arm.angles.elbow - baseline.angles.elbow)
        lower, upper = (0.72, 1.4) if self._frontal_active[side] else (0.8, 1.3)
        minimum_bend = radians(3 if self._frontal_active[side] else 5)
        if abs(elbow) < minimum_bend or not all(lower <= ratio <= upper for ratio in ratios):
            return None
        return ArmAngles(
            NEUTRAL[side * 4] + arm.angles.shoulder - baseline.angles.shoulder,
            abs(elbow), rotation=-copysign(pi / 2, elbow) * smooth_confidence(
                abs(elbow), radians(5), radians(25)),
        )

    def _projection_conflicts(self, side: int, image: MeasuredArm | None,
                              world: SpatialArm | None) -> bool:
        """Rechaza profundidad que acorta segmentos claramente completos en imagen.

        La tolerancia contempla perspectiva y una calibración con los segmentos
        algo inclinados. No se aplica si falta un plano frontal bien observado.
        """
        reference = self._spatial.reference[side]
        if world is None or reference is None or self._frontal_target(side, image) is None:
            return False
        baseline = self._zero[side]
        for vector, ratio, world_zero, image_ratio, image_zero in (
            (world.upper, world.upper_ratio, reference[1], image.upper_ratio, baseline.upper_ratio),
            (world.forearm, world.forearm_ratio, reference[2], image.forearm_ratio, baseline.forearm_ratio),
        ):
            predicted = hypot(vector[0], vector[2]) * ratio / world_zero
            if image_ratio / image_zero > 1.6 * predicted + 0.1:
                return True
        return False

    def update(self, observation: Observation, width: int, height: int) -> RetargetResult:
        timestamp = observation.timestamp_ms
        if timestamp < 0 or timestamp <= self._last_ms:
            raise ValueError("La conversión requiere timestamps estrictamente crecientes.")
        if self._last_ms >= 0 and timestamp - self._last_ms > 350:
            self.lose_tracking()
        self._last_ms = timestamp
        measured = self._measure(observation, width, height)
        if measured is None:
            self.lose_tracking()
            return RetargetResult(ArmTargets(), "SEGURO: SIN TORSO")
        arms, width_ratio = measured
        spatial = measure_world(observation, width, height)
        if not self.calibrated:
            if any(arm is None for arm in arms):
                self._samples.clear()
                return RetargetResult(ArmTargets(), "CALIBRAR: AMBOS BRAZOS")
            if any(abs(arm.angles.shoulder) > radians(35) or abs(arm.angles.elbow) > radians(25)
                   for arm in arms):
                self._samples.clear()
                return RetargetResult(ArmTargets(), "CALIBRAR: BRAZOS ABAJO")
            values = tuple(angle for arm in arms for angle in (arm.angles.shoulder, arm.angles.elbow))
            if self._samples:
                first_angles = self._samples[0][1]
                if any(abs(angle - initial) > radians(8) for angle, initial in zip(values, first_angles)):
                    self._samples.clear()
            self._samples.append((timestamp, values, arms, width_ratio, spatial))
            elapsed = timestamp - self._samples[0][0]
            progress = min(1.0, elapsed / self.calibration_ms, len(self._samples) / 6)
            if progress < 1:
                return RetargetResult(ArmTargets(), f"CALIBRANDO: {progress:.0%}", progress)
            count = len(self._samples)
            self._zero = tuple(
                MeasuredArm(
                    ArmAngles(*(sum(sample[1][side * 2 + joint] for sample in self._samples) / count
                                for joint in range(2))),
                    sum(sample[2][side].upper_ratio for sample in self._samples) / count,
                    sum(sample[2][side].forearm_ratio for sample in self._samples) / count,
                )
                for side in range(2)
            )
            self._width_ratio = sum(sample[3] for sample in self._samples) / count
            self._spatial.calibrate([sample[4] for sample in self._samples])
            self._samples.clear()
        spatial = tuple(None if self._projection_conflicts(side, arms[side], arm) else arm
                        for side, arm in enumerate(spatial))
        spatial_targets = self._spatial.targets(spatial, timestamp)
        targets = []
        modes = []
        for side, arm in enumerate(arms):
            if spatial_targets[side] is not None:
                self._recover_since[side] = timestamp
                targets.append(spatial_targets[side])
                modes.append("3D")
                continue
            frontal = self._frontal_target(side, arm)
            self._frontal_active[side] = frontal is not None
            modes.append("2D-F" if frontal is not None else "2D" if arm is not None else "--")
            if arm is None:
                self._recover_since[side] = None
                targets.append(None)
                continue
            if self._recover_since[side] is None:
                self._recover_since[side] = timestamp
            if timestamp - self._recover_since[side] < self.recovery_ms:
                targets.append(None)
                continue
            zero = self._zero[side].angles
            targets.append(frontal if frontal is not None else ArmAngles(
                NEUTRAL[side * 4] + arm.angles.shoulder - zero.shoulder,
                abs(wrap_angle(arm.angles.elbow - zero.elbow)),
            ))
        tracked = sum(target is not None for target in targets)
        status = ("SIGUIENDO", "SEGURO: SEGUIMIENTO PARCIAL", "SEGURO: SIN SEGUIMIENTO")
        detail = f" | I:{modes[0]} D:{modes[1]}"
        return RetargetResult(ArmTargets(*targets), status[2 - tracked] + detail, 1.0)
