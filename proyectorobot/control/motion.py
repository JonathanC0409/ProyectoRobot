"""Control articular independiente del motor físico y de las bibliotecas de visión."""

from dataclasses import dataclass
from math import exp, isfinite, radians


@dataclass(frozen=True)
class ArmAngles:
    shoulder: float
    elbow: float
    flexion: float = 0.0
    rotation: float = 0.0

    @property
    def values(self) -> tuple[float, ...]:
        """Orden MJCF: abducción, flexión, rotación de hombro y codo."""
        return self.shoulder, self.flexion, self.rotation, self.elbow


@dataclass(frozen=True)
class ArmTargets:
    left: ArmAngles | None = None
    right: ArmAngles | None = None


@dataclass(frozen=True)
class JointSpec:
    name: str
    lower: float
    upper: float
    neutral: float


JOINTS = (
    JointSpec("left_shoulder", radians(-30), radians(170), radians(10)),
    JointSpec("left_shoulder_flexion", radians(-50), radians(160), 0.0),
    JointSpec("left_shoulder_rotation", radians(-100), radians(100), 0.0),
    JointSpec("left_elbow", 0.0, radians(150), 0.0),
    JointSpec("right_shoulder", radians(-30), radians(170), radians(10)),
    JointSpec("right_shoulder_flexion", radians(-50), radians(160), 0.0),
    JointSpec("right_shoulder_rotation", radians(-100), radians(100), 0.0),
    JointSpec("right_elbow", 0.0, radians(150), 0.0),
)
NEUTRAL = tuple(joint.neutral for joint in JOINTS)


class MotionController:
    """Filtra consignas y vuelve a neutral si faltan observaciones recientes."""

    def __init__(
        self, timeout: float = 0.35, max_speed: float = 1.5, smoothing: float = 0.12,
        adaptive: bool = False, neutral: tuple[float, ...] = NEUTRAL,
    ) -> None:
        if not all(isfinite(value) and value > 0 for value in (timeout, max_speed, smoothing)):
            raise ValueError("Timeout, velocidad y suavizado deben ser positivos y finitos.")
        self.timeout = timeout
        self.max_speed = max_speed
        self.smoothing = smoothing
        self.adaptive = adaptive
        if len(neutral) != len(JOINTS) or any(
            not isfinite(value) or not spec.lower <= value <= spec.upper
            for value, spec in zip(neutral, JOINTS)
        ):
            raise ValueError("Postura segura incompatible con los límites articulares.")
        self.neutral = tuple(neutral)
        self.positions = self.neutral
        self._targets = self.neutral
        self._valid = (False, False)
        self._captured_at: float | None = None

    def invalidate(self) -> None:
        self._targets = self.neutral
        self._valid = (False, False)
        self._captured_at = None

    def set_targets(self, targets: ArmTargets, captured_at: float, now: float) -> None:
        try:
            if not all(isfinite(value) for value in (captured_at, now)):
                raise ValueError("Tiempos no finitos.")
            if captured_at > now + 0.05:
                raise ValueError("La observación procede del futuro.")
            if self._captured_at is not None and captured_at <= self._captured_at:
                raise ValueError("La observación debe ser posterior a la anterior.")
            values = []
            for offset, arm in ((0, targets.left), (4, targets.right)):
                specs = JOINTS[offset:offset + 4]
                angles = arm.values if arm is not None else self.neutral[offset:offset + 4]
                for value, spec in zip(angles, specs):
                    if not isfinite(value):
                        raise ValueError("Ángulo no finito; se activa postura segura.")
                    margin = radians(2)
                    lower = min(spec.lower + margin, spec.neutral)
                    values.append(max(lower, min(spec.upper - margin, value)))
        except ValueError:
            self.invalidate()
            raise
        self._targets = tuple(values)
        self._valid = (targets.left is not None, targets.right is not None)
        self._captured_at = captured_at

    def tracking(self, now: float) -> tuple[bool, bool]:
        if self._captured_at is None or now - self._captured_at > self.timeout:
            return False, False
        return self._valid

    def step(self, dt: float, now: float) -> tuple[float, ...]:
        if not isfinite(dt) or not 0 < dt <= 0.1 or not isfinite(now):
            raise ValueError("Paso de control inválido: se requiere 0 < dt <= 0.1.")
        valid = self.tracking(now)
        desired = tuple(
            target if valid[index // 4] else self.neutral[index]
            for index, target in enumerate(self._targets)
        )
        def filtered_delta(current: float, target: float) -> float:
            error = target - current
            responsiveness = min(1.0, abs(error) / radians(20)) if self.adaptive else 0.0
            smoothing = self.smoothing * (1 - 0.6 * responsiveness)
            return (1 - exp(-dt / smoothing)) * error
        maximum_delta = self.max_speed * dt
        self.positions = tuple(
            max(spec.lower, min(spec.upper, current + max(
                -maximum_delta, min(maximum_delta, filtered_delta(current, target)),
            )))
            for current, target, spec in zip(self.positions, desired, JOINTS)
        )
        return self.positions
