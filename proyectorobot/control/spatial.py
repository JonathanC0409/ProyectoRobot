"""Cinemática 3D conservadora, independiente de MediaPipe, NumPy y MuJoCo.

El robot usa X izquierda anatómica, Y hacia atrás y Z arriba. El hombro
compone Ry(-lado*abducción), Rx(-flexión), Rz(-lado*rotación), y el codo
Rx(-flexión). Los lados valen +1 izquierdo y -1 derecho. Las coordenadas
world son estimaciones monoculares; las comprobaciones son heurísticas,
no una medida de certeza de profundidad.
"""

from dataclasses import dataclass
from collections.abc import Sequence
from math import acos, asin, atan2, cos, hypot, pi, radians, sin, sqrt
from statistics import fmean

from proyectorobot.control.motion import ArmAngles
from proyectorobot.data.types import Landmark, Observation


Vector = tuple[float, float, float]


def dot(first: Vector, second: Vector) -> float:
    return sum(left * right for left, right in zip(first, second))


def subtract(first: Vector, second: Vector) -> Vector:
    return tuple(left - right for left, right in zip(first, second))


def norm(vector: Vector) -> float:
    return sqrt(dot(vector, vector))


def unit(vector: Vector) -> Vector:
    length = norm(vector)
    if length < 1e-8:
        raise ValueError("Vector degenerado; no define una orientación.")
    return tuple(value / length for value in vector)


def cross(first: Vector, second: Vector) -> Vector:
    return (first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0])


def coordinates(point: Landmark) -> Vector:
    return point.x, point.y, point.z


def wrap(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


def smooth_confidence(value: float, lower: float, upper: float) -> float:
    """Transición continua entre ausencia de observabilidad y confianza completa."""
    fraction = max(0.0, min(1.0, (value - lower) / (upper - lower)))
    return fraction * fraction * (3 - 2 * fraction)


def shoulder_confidence(upper: Vector) -> float:
    return smooth_confidence(hypot(upper[0], upper[2]), 0.04, 0.16)


def axial_rotation(forearm: Vector, abduction: float, flexion: float, side: int) -> float:
    horizontal = cos(abduction) * forearm[0] + side * sin(abduction) * forearm[2]
    vertical = -side * sin(abduction) * forearm[0] + cos(abduction) * forearm[2]
    depth = cos(flexion) * forearm[1] - sin(flexion) * vertical
    return atan2(-side * horizontal, -depth)


def solve_arm(upper: Vector, forearm: Vector, side: int,
              previous: ArmAngles = ArmAngles(0, 0)) -> ArmAngles:
    """Invierte los ejes MJCF; brazo recto no permite observar giro axial."""
    upper = unit(upper)
    forearm = unit(forearm)
    flexion = asin(max(-1.0, min(1.0, -upper[1])))
    if hypot(upper[0], upper[2]) < 1e-8:
        abduction = 0.0
        flexion = atan2(-upper[1], -upper[2])
    else:
        abduction = atan2(side * upper[0], -upper[2])
        alternative = (wrap(abduction + pi), pi - flexion if flexion >= 0 else -pi - flexion)
        candidates = [(abduction, flexion), alternative]
        plausible = [(angle, bend) for angle, bend in candidates
                     if radians(-30) <= angle <= radians(170)
                     and radians(-50) <= bend <= radians(160)]
        if plausible:
            feasible = [(angle, bend) for angle, bend in plausible
                        if abs(axial_rotation(forearm, angle, bend, side)) <= radians(100)
                        or abs(dot(upper, forearm)) > 0.97]
            plausible = feasible or plausible
            abduction, flexion = min(plausible, key=lambda pair:
                                    abs(pair[0] - previous.shoulder)
                                    + abs(pair[1] - previous.flexion))
    elbow = acos(max(-1.0, min(1.0, dot(upper, forearm))))
    confidence = shoulder_confidence(upper)
    bend_confidence = smooth_confidence(sin(elbow), sin(radians(5)), sin(radians(25)))
    rotation = axial_rotation(forearm, abduction, flexion, side)
    if abs(rotation) > radians(100):
        rotation = max(radians(-100), min(radians(100), previous.rotation))
    rotation *= confidence * bend_confidence
    return ArmAngles(abduction * confidence, elbow, flexion, rotation)


@dataclass(frozen=True)
class SpatialArm:
    upper: Vector
    forearm: Vector
    upper_ratio: float
    forearm_ratio: float


SpatialMeasurements = tuple[SpatialArm | None, SpatialArm | None]


def measure_world(observation: Observation, width: int, height: int
                  ) -> SpatialMeasurements:
    """Marco frontal obtenido de hombros y vertical óptica, sin exigir caderas.

    Solo se usan world landmarks con confianza, longitudes plausibles y
    direcciones proyectadas coherentes con la imagen. Se rechaza la vista
    lateral; no se sustituye world por el Z normalizado de la imagen.
    """
    image, world = observation.pose, observation.pose_world
    if len(image) != 33 or len(world) != 33 or min(width, height) <= 0:
        return None, None
    if not all(image[index].visible(0.65) and world[index].confident(0.65)
               for index in (11, 12)):
        return None, None
    shoulder_vector = subtract(coordinates(world[11]), coordinates(world[12]))
    shoulder_width = norm(shoulder_vector)
    if not 0.15 <= shoulder_width <= 0.65:
        return None, None
    left_axis = unit(shoulder_vector)
    if left_axis[0] < 0.5 or abs(left_axis[2]) > 0.5 or abs(left_axis[1]) > 0.5:
        return None, None
    optical_up = (0.0, -1.0, 0.0)
    up_axis = unit(tuple(value - dot(optical_up, left_axis) * axis
                         for value, axis in zip(optical_up, left_axis)))
    back_axis = cross(up_axis, left_axis)
    basis = (left_axis, back_axis, up_axis)
    arms = []
    for indices in ((11, 13, 15), (12, 14, 16)):
        if not all(image[index].visible(0.65) and world[index].confident(0.65)
                   for index in indices):
            arms.append(None)
            continue
        vectors = [subtract(coordinates(world[end]), coordinates(world[start]))
                   for start, end in zip(indices, indices[1:])]
        lengths = tuple(norm(vector) / shoulder_width for vector in vectors)
        if not all(0.3 <= length <= 1.5 for length in lengths):
            arms.append(None)
            continue
        if not 0.55 <= lengths[1] / lengths[0] <= 1.25:
            arms.append(None)
            continue
        coherent = True
        for vector, start, end in zip(vectors, indices, indices[1:]):
            projected = ((image[end].x - image[start].x) * width,
                         (image[end].y - image[start].y) * height)
            world_projection = hypot(vector[0], vector[1])
            image_projection = hypot(*projected)
            if world_projection > 0.15 * norm(vector):
                if image_projection < 2 or (vector[0] * projected[0] + vector[1] * projected[1]
                                            ) / (world_projection * image_projection) < 0.7:
                    coherent = False
        if not coherent:
            arms.append(None)
            continue
        transformed = [unit(tuple(dot(vector, axis) for axis in basis)) for vector in vectors]
        arms.append(SpatialArm(*transformed, *lengths))
    return arms[0], arms[1]


class SpatialMapper:
    """Referencia 3D calibrada por brazo y recuperación con tres muestras."""

    def __init__(self, recovery_ms: int = 150) -> None:
        if recovery_ms < 0:
            raise ValueError("La recuperación debe ser no negativa.")
        self.recovery_ms = recovery_ms
        self.reference: list[tuple[ArmAngles, float, float] | None] = [None, None]
        self.reset_tracking()

    def reset_tracking(self) -> None:
        self._previous = [ArmAngles(0, 0), ArmAngles(0, 0)]
        self._vectors: list[SpatialArm | None] = [None, None]
        self._since: list[int | None] = [None, None]
        self._count = [0, 0]
        self._last_ms: list[int | None] = [None, None]

    def calibrate(self, samples: Sequence[SpatialMeasurements]) -> None:
        for side in range(2):
            valid = [sample[side] for sample in samples if sample[side] is not None]
            if len(valid) < 6 or len(valid) < 0.8 * len(samples):
                continue
            angles = [solve_arm(arm.upper, arm.forearm, 1 if side == 0 else -1) for arm in valid]
            if any(abs(angle.flexion) > radians(25) or abs(angle.shoulder) > radians(35)
                   or angle.elbow > radians(25) for angle in angles):
                continue
            if any(max(getattr(angle, field) for angle in angles)
                   - min(getattr(angle, field) for angle in angles) > radians(10)
                   for field in ("shoulder", "flexion", "elbow")):
                continue
            upper = fmean(arm.upper_ratio for arm in valid)
            forearm = fmean(arm.forearm_ratio for arm in valid)
            if any(not 0.8 <= arm.upper_ratio / upper <= 1.2
                   or not 0.8 <= arm.forearm_ratio / forearm <= 1.2 for arm in valid):
                continue
            self.reference[side] = (
                ArmAngles(fmean(angle.shoulder for angle in angles),
                          fmean(angle.elbow for angle in angles),
                          fmean(angle.flexion for angle in angles)), upper, forearm,
            )

    def targets(self, measured: SpatialMeasurements, timestamp: int
                ) -> tuple[ArmAngles | None, ArmAngles | None]:
        targets = []
        for side, arm in enumerate(measured):
            reference = self.reference[side]
            valid = reference is not None and arm is not None
            if valid:
                valid = (0.7 <= arm.upper_ratio / reference[1] <= 1.3
                         and 0.7 <= arm.forearm_ratio / reference[2] <= 1.3)
            previous_vectors = self._vectors[side]
            if valid and previous_vectors is not None:
                elapsed = (timestamp - self._last_ms[side]) / 1000
                maximum = max(0.25, min(0.8, 6 * elapsed))
                valid = all(norm(subtract(current, previous)) <= maximum
                            for current, previous in ((arm.upper, previous_vectors.upper),
                                                      (arm.forearm, previous_vectors.forearm)))
            if not valid:
                self._vectors[side] = None
                self._since[side] = None
                self._count[side] = 0
                targets.append(None)
                continue
            angles = solve_arm(arm.upper, arm.forearm, 1 if side == 0 else -1,
                               self._previous[side])
            self._previous[side] = angles
            self._vectors[side] = arm
            self._last_ms[side] = timestamp
            if self._since[side] is None:
                self._since[side] = timestamp
            self._count[side] += 1
            if self._count[side] < 3 or timestamp - self._since[side] < self.recovery_ms:
                targets.append(None)
                continue
            zero = reference[0]
            abduction = radians(10) + angles.shoulder - zero.shoulder * shoulder_confidence(arm.upper)
            targets.append(ArmAngles(
                abduction,
                max(0.0, angles.elbow - zero.elbow),
                angles.flexion - zero.flexion, angles.rotation,
            ))
        return targets[0], targets[1]
