"""Continuidad de pose y rechazo de ambigüedades antes de mover el robot."""

from dataclasses import dataclass, replace
from math import dist, exp, hypot, isfinite

from proyectorobot.control.spatial import SpatialArm, measure_world
from proyectorobot.data.types import Landmark, Observation


ARM_INDICES = ((11, 13, 15), (12, 14, 16))
ImageVector = tuple[tuple[float, float], tuple[float, float]]


@dataclass(frozen=True)
class QualityResult:
    observation: Observation
    left: str = "SIN POSE"
    right: str = "SIN POSE"

    @property
    def status(self) -> str:
        return f"Calidad I:{self.left} D:{self.right}"


class PoseQualityFilter:
    """Comprueba huesos, continuidad y reflejos de profundidad sin reparar lados.

    Los cruces físicos graduales se permiten. Una pose rechazada no sustituye
    la última referencia aceptada; la recuperación exige muestras coherentes.
    No se convierte el Z de imagen en distancia ni se intercambian etiquetas.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._last_ms: int | None = None
        self._accepted: list[tuple[int, ImageVector] | None] = [None, None]
        self._world: list[SpatialArm | None] = [None, None]
        self._lengths: list[tuple[float, float] | None] = [None, None]
        self._samples: list[list[tuple[float, float]]] = [[], []]
        self._pending: list[tuple[int, ImageVector] | None] = [None, None]
        self._counts = [0, 0]
        self._recovering = [False, False]

    @staticmethod
    def _mask(points: list[Landmark], side: int) -> None:
        for index in ARM_INDICES[side][1:]:
            if index < len(points):
                points[index] = replace(points[index], visibility=0.0, presence=0.0)

    def _recover(self, side: int, vector: ImageVector, timestamp: int) -> bool:
        previous = self._pending[side]
        consistent = previous is not None and max(
            dist(current, old) for current, old in zip(vector, previous[1])
        ) < 0.3 and timestamp - previous[0] <= 350
        count = self._counts[side] + 1 if consistent else 1
        self._pending[side] = (timestamp, vector)
        self._counts[side] = count
        return count >= 3

    def update(self, observation: Observation, width: int, height: int) -> QualityResult:
        timestamp = observation.timestamp_ms
        if (not isfinite(timestamp) or timestamp < 0
                or self._last_ms is not None and timestamp <= self._last_ms):
            raise ValueError("El filtro de pose requiere tiempos crecientes y no negativos.")
        gap = self._last_ms is not None and timestamp - self._last_ms > 350
        self._last_ms = timestamp
        pose, world = list(observation.pose), list(observation.pose_world)
        if (len(pose) != 33 or min(width, height) <= 0
                or not all(pose[index].visible(0.7) for index in (11, 12))):
            self._recovering = [True, True]
            self._pending = [None, None]
            return QualityResult(replace(observation, pose=(), pose_world=()))
        xy = lambda point: (point.x * width, point.y * height)
        left, right = xy(pose[11]), xy(pose[12])
        span = dist(left, right)
        if span < 0.1 * width:
            self._recovering = [True, True]
            return QualityResult(replace(observation, pose=(), pose_world=()))
        center = tuple((first + second) / 2 for first, second in zip(left, right))
        down = (-(left[1] - right[1]) / span, (left[0] - right[0]) / span)
        if down[1] < 0:
            down = tuple(-value for value in down)
        spatial = measure_world(observation, width, height)
        statuses = []
        for side, indices in enumerate(ARM_INDICES):
            reason = "OK"
            vector = tuple(tuple((value - origin) / span for value, origin in zip(xy(pose[index]), center))
                           for index in indices[1:])
            lengths = tuple(dist(xy(pose[start]), xy(pose[end])) / span
                            for start, end in zip(indices, indices[1:]))
            baseline = self._lengths[side]
            previous = self._accepted[side]
            elapsed = (timestamp - previous[0]) / 1000 if previous is not None else 0.0
            jump = max(dist(current, old) for current, old in zip(vector, previous[1])) if previous else 0.0
            old_world, new_world = self._world[side], spatial[side]
            if not all(pose[index].visible(0.7) for index in indices):
                reason = "OCULTO"
            elif max(lengths) > 1.5 or min(lengths) < 0.015 and new_world is None:
                reason = "GEOMETRIA"
            elif baseline is not None and any(length > 1.45 * reference + 0.05
                                              for length, reference in zip(lengths, baseline)):
                reason = "HUESO INCOHERENTE"
            elif previous is not None and elapsed <= 0.35 and jump > 0.14 + 3.0 * elapsed:
                reason = "SALTO"
            elif (old_world is not None and new_world is not None and jump < 0.18
                  and previous is not None and elapsed <= 0.35 and any(
                      before[1] * after[1] < 0 and min(abs(before[1]), abs(after[1])) > 0.3
                      for before, after in ((old_world.upper, new_world.upper),
                                            (old_world.forearm, new_world.forearm)))):
                reason = "PROFUNDIDAD AMBIGUA"
            elif (baseline is not None and new_world is None
                  and any(length < 0.5 * reference for length, reference in zip(lengths, baseline))):
                reason = "SIN PROFUNDIDAD"
            if reason != "OK":
                self._mask(pose, side)
                self._mask(world, side)
                self._recovering[side] = True
                self._pending[side] = None
                statuses.append(reason)
                continue
            if gap:
                self._recovering[side] = True
            if self._recovering[side] and not self._recover(side, vector, timestamp):
                self._mask(pose, side)
                self._mask(world, side)
                statuses.append("RECUPERANDO")
                continue
            self._recovering[side] = False
            if baseline is None and new_world is not None:
                frontal = all(hypot(segment[0], segment[2]) > 0.85
                              for segment in (new_world.upper, new_world.forearm))
            else:
                frontal = True
            segments = tuple(tuple((end - start) / span for start, end in zip(xy(pose[first]), xy(pose[second])))
                             for first, second in zip(indices, indices[1:]))
            relaxed = all(sum(value * axis for value, axis in zip(segment, down)) > 0.82 * length
                          for segment, length in zip(segments, lengths)) and min(lengths) > 0.25
            if baseline is None and frontal and relaxed:
                self._samples[side].append(lengths)
                self._samples[side] = self._samples[side][-9:]
                if len(self._samples[side]) >= 6:
                    self._lengths[side] = tuple(max(sample[index] for sample in self._samples[side])
                                                  for index in range(2))
            elif baseline is None:
                self._samples[side].clear()
            if previous is not None and 0 < elapsed <= 0.35:
                alpha = 1 - exp(-elapsed / 0.025)
                filtered = tuple(tuple(old + alpha * (current - old) for current, old in zip(point, old_point))
                                 for point, old_point in zip(vector, previous[1]))
                for index, point in zip(indices[1:], filtered):
                    pose[index] = replace(pose[index], x=(point[0] * span + center[0]) / width,
                                          y=(point[1] * span + center[1]) / height)
                vector = filtered
            self._accepted[side] = (timestamp, vector)
            self._world[side] = new_world
            statuses.append("3D ESTIMADA" if new_world is not None else "2D")
        return QualityResult(replace(observation, pose=tuple(pose), pose_world=tuple(world)), *statuses)
