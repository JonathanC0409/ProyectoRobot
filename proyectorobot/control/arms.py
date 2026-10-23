"""Detecta muñecas elevadas respecto de sus hombros en una vista frontal."""

from dataclasses import dataclass
from enum import Enum
from math import hypot, isfinite

from proyectorobot.data.types import Landmark, Observation


class ArmState(str, Enum):
    UNKNOWN = "SIN SEGUIMIENTO"
    DOWN = "ABAJO"
    UP = "ARRIBA"


@dataclass(frozen=True)
class ArmStates:
    left: ArmState
    right: ArmState

    @property
    def movement(self) -> str:
        """Resume ambos lados sin inferir el estado de una extremidad oculta."""
        if self.left == self.right == ArmState.UNKNOWN:
            return "SIN SEGUIMIENTO"
        if ArmState.UNKNOWN in (self.left, self.right):
            return "SEGUIMIENTO PARCIAL"
        if self.left == self.right == ArmState.UP:
            return "AMBAS MANOS ARRIBA"
        if self.left == ArmState.UP:
            return "MANO IZQUIERDA ARRIBA"
        if self.right == ArmState.UP:
            return "MANO DERECHA ARRIBA"
        return "MANOS ABAJO"


class ArmTracker:
    """Histéresis espacial y permanencia temporal; reinicio al perder puntos."""

    def __init__(
        self, raise_ratio: float = 0.20, lower_ratio: float = 0.10,
        hold_ms: int = 150, visibility: float = 0.6,
    ) -> None:
        if not all(isfinite(value) for value in (raise_ratio, lower_ratio, visibility)):
            raise ValueError("Los umbrales deben ser finitos.")
        if not 0 <= lower_ratio < raise_ratio or hold_ms < 0:
            raise ValueError("Se requiere 0 <= lower_ratio < raise_ratio y hold_ms >= 0.")
        if not 0 <= visibility <= 1:
            raise ValueError("La visibilidad debe estar entre 0 y 1.")
        self.raise_ratio = raise_ratio
        self.lower_ratio = lower_ratio
        self.hold_ms = hold_ms
        self.visibility = visibility
        self.state = ArmState.UNKNOWN
        self._candidate = ArmState.UNKNOWN
        self._since_ms = 0

    def update(
        self, shoulder: Landmark, wrist: Landmark, hip: Landmark, timestamp_ms: int,
        body_scale: float | None = None,
    ) -> ArmState:
        required = (shoulder, wrist, hip) if body_scale is None else (shoulder, wrist)
        if not all(point.visible(self.visibility) for point in required):
            return self.reset()
        torso_height = hip.y - shoulder.y if body_scale is None else body_scale
        if not isfinite(torso_height) or torso_height < 0.05:
            return self.reset()
        elevation = (shoulder.y - wrist.y) / torso_height
        if elevation >= self.raise_ratio:
            target = ArmState.UP
        elif elevation <= self.lower_ratio:
            target = ArmState.DOWN
        else:
            self._candidate = self.state
            return self.state
        if target != self._candidate:
            self._candidate = target
            self._since_ms = timestamp_ms
        if timestamp_ms - self._since_ms >= self.hold_ms:
            self.state = target
        return self.state

    def reset(self) -> ArmState:
        self.state = ArmState.UNKNOWN
        self._candidate = ArmState.UNKNOWN
        return self.state


class ArmRecognizer:
    """Lados anatómicos: hombros 11/12, muñecas 15/16 y caderas 23/24."""

    def __init__(self, hold_ms: int = 150, visibility: float = 0.6,
                 allow_upper_body: bool = False) -> None:
        self.left = ArmTracker(hold_ms=hold_ms, visibility=visibility)
        self.right = ArmTracker(hold_ms=hold_ms, visibility=visibility)
        self._last_ms = -1
        self.allow_upper_body = allow_upper_body

    def update(self, observation: Observation) -> ArmStates:
        if observation.timestamp_ms < 0 or observation.timestamp_ms <= self._last_ms:
            raise ValueError("Los timestamps deben ser no negativos y crecientes.")
        self._last_ms = observation.timestamp_ms
        if len(observation.pose) != 33:
            return ArmStates(self.left.reset(), self.right.reset())
        pose = observation.pose
        body_scale = None
        if self.allow_upper_body:
            if not all(pose[index].visible(self.left.visibility) for index in (11, 12)):
                return ArmStates(self.left.reset(), self.right.reset())
            body_scale = hypot(pose[11].x - pose[12].x, pose[11].y - pose[12].y)
        return ArmStates(
            self.left.update(pose[11], pose[15], pose[23], observation.timestamp_ms, body_scale),
            self.right.update(pose[12], pose[16], pose[24], observation.timestamp_ms, body_scale),
        )
