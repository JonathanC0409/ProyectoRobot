"""Datos independientes de los objetos internos de MediaPipe."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class Landmark:
    x: float
    y: float
    z: float = 0.0
    visibility: float = 1.0
    presence: float = 1.0

    def confident(self, threshold: float) -> bool:
        """Confianza y finitud; también válida para coordenadas métricas 3D."""
        return (
            all(isfinite(value) for value in (self.x, self.y, self.z,
                                              self.visibility, self.presence))
            and threshold <= self.visibility <= 1.0
            and threshold <= self.presence <= 1.0
        )

    def visible(self, threshold: float) -> bool:
        return self.confident(threshold) and 0 <= self.x <= 1 and 0 <= self.y <= 1


@dataclass(frozen=True)
class Observation:
    timestamp_ms: int
    pose: tuple[Landmark, ...] = ()
    hands: tuple[tuple[Landmark, ...], ...] = ()
    pose_world: tuple[Landmark, ...] = ()
