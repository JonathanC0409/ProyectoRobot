"""Consignas de manos normalizadas y control limitado sin bibliotecas de visión."""

from dataclasses import dataclass
from math import exp, isfinite, radians


@dataclass(frozen=True)
class HandCommand:
    thumb: float = 0.0
    index: float = 0.0
    fingers: float = 0.0

    @property
    def values(self) -> tuple[float, float, float]:
        return self.thumb, self.index, self.fingers


@dataclass(frozen=True)
class HandTargets:
    left: HandCommand | None = None
    right: HandCommand | None = None


OPEN = HandCommand()
FIST = HandCommand(1, 1, 1)
PINCH = HandCommand(0.65, 0.65, 0)
MAX_CURL = radians(70)


class HandController:
    """Una mano ausente se abre progresivamente; las consignas nunca saltan."""

    def __init__(self, timeout: float = 0.35, max_speed: float = 2.0) -> None:
        if not all(isfinite(value) and value > 0 for value in (timeout, max_speed)):
            raise ValueError("Timeout y velocidad de manos deben ser positivos.")
        self.timeout = timeout
        self.max_speed = max_speed
        self.positions = (0.0,) * 6
        self.invalidate()

    def invalidate(self) -> None:
        self._targets = (0.0,) * 6
        self._valid = (False, False)
        self._captured_at: float | None = None

    def set_targets(self, targets: HandTargets, captured_at: float, now: float) -> None:
        try:
            if (not all(isfinite(value) for value in (captured_at, now))
                    or captured_at > now + 0.05
                    or (self._captured_at is not None and captured_at <= self._captured_at)):
                raise ValueError("Marca temporal de manos inválida o repetida.")
            values = tuple(value for command in (targets.left, targets.right)
                           for value in (command or OPEN).values)
            if not all(isfinite(value) for value in values):
                raise ValueError("Consigna de mano no finita.")
        except ValueError:
            self.invalidate()
            raise
        self._targets = tuple(max(0.0, min(1.0, value)) * MAX_CURL for value in values)
        self._valid = (targets.left is not None, targets.right is not None)
        self._captured_at = captured_at

    def tracking(self, now: float) -> tuple[bool, bool]:
        if self._captured_at is None or now - self._captured_at > self.timeout:
            return False, False
        return self._valid

    def step(self, dt: float, now: float) -> tuple[float, ...]:
        if not isfinite(dt) or not 0 < dt <= 0.1 or not isfinite(now):
            raise ValueError("Paso temporal de manos inválido.")
        valid = self.tracking(now)
        self.positions = tuple(
            max(0.0, min(MAX_CURL, current + max(-self.max_speed * dt, min(
                self.max_speed * dt,
                (target if valid[index // 3] else 0.0) - current,
            )) * (1 - exp(-dt / 0.006))))
            for index, (current, target) in enumerate(zip(self.positions, self._targets))
        )
        return self.positions
