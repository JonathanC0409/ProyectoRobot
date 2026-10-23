"""Gestos geométricos calibrados y asociación anatómica mediante las muñecas."""

from dataclasses import dataclass
from math import acos, dist, exp, isfinite, pi
from statistics import median

from proyectorobot.control.hands import HandCommand, HandTargets
from proyectorobot.data.types import Landmark, Observation


CHAINS = ((1, 2, 3, 4), (5, 6, 7, 8), (9, 10, 11, 12),
          (13, 14, 15, 16), (17, 18, 19, 20))


@dataclass(frozen=True)
class HandFeatures:
    curls: tuple[float, ...]
    pinch_gap: float


@dataclass(frozen=True)
class GestureResult:
    targets: HandTargets = HandTargets()
    left: str = "SIN MANO"
    right: str = "SIN MANO"


def measure_hand(points: tuple[Landmark, ...], width: int, height: int
                 ) -> HandFeatures | None:
    """Ángulos y distancias relativos a la palma; Z local nunca posiciona objetos."""
    if min(width, height) <= 0 or len(points) != 21 or not all(
        point.visible(0.5) for point in points
    ):
        return None
    coordinates = [(point.x * width, point.y * height, point.z * width) for point in points]
    scale = (dist(coordinates[5], coordinates[17]) + dist(coordinates[0], coordinates[9])) / 2
    if scale < 12 or scale > min(width, height) * 0.65:
        return None
    if any(abs(point[2] - coordinates[0][2]) > 2 * scale for point in coordinates):
        return None
    curls = []
    for chain in CHAINS:
        segments = [tuple(coordinates[end][axis] - coordinates[start][axis] for axis in range(3))
                    for start, end in zip(chain, chain[1:])]
        lengths = [sum(value * value for value in segment) ** 0.5 for segment in segments]
        if any(not 0.035 * scale <= length <= 1.3 * scale for length in lengths):
            return None
        angles = [acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(first, second))
                                       / (lengths[index] * lengths[index + 1]))))
                  for index, (first, second) in enumerate(zip(segments, segments[1:]))]
        angle_curl = min(1.0, (0.8 * angles[0] + 0.2 * angles[1]) / (pi / 2))
        reach_curl = min(1.0, 1.5 * (1 - dist(coordinates[chain[0]], coordinates[chain[-1]])
                                             / sum(lengths)))
        curls.append(0.8 * angle_curl + 0.2 * reach_curl)
    return HandFeatures(tuple(curls), dist(coordinates[4], coordinates[8]) / scale)


def associate_hands(observation: Observation, width: int, height: int
                    ) -> tuple[tuple[Landmark, ...] | None, tuple[Landmark, ...] | None]:
    """No depende del espejo ni del orden/etiquetas que devuelve MediaPipe."""
    pose = observation.pose
    if len(pose) != 33 or min(width, height) <= 0 or not all(
        pose[index].visible(0.6) for index in (11, 12)
    ):
        return None, None
    xy = lambda point: (point.x * width, point.y * height)
    scale = dist(xy(pose[11]), xy(pose[12]))
    if scale < 20:
        return None, None
    wrists = [pose[15], pose[16]]
    if all(point.visible(0.6) for point in wrists) and dist(*map(xy, wrists)) < 0.18 * scale:
        return None, None
    choices = []
    for hand_index, hand in enumerate(observation.hands[:2]):
        if len(hand) != 21 or not hand[0].visible(0.5):
            continue
        distances = [dist(xy(hand[0]), xy(wrist)) if wrist.visible(0.6) else float("inf")
                     for wrist in wrists]
        side = min(range(2), key=distances.__getitem__)
        if (distances[side] <= max(18, 0.45 * scale)
                and abs(distances[0] - distances[1]) > 0.1 * scale):
            choices.append((distances[side], side, hand_index))
    assigned = [None, None]
    for _, side, hand_index in sorted(choices):
        if assigned[side] is None:
            assigned[side] = observation.hands[hand_index]
    return tuple(assigned)


def normalized_command(features: HandFeatures, opened: tuple[float, ...],
                       closed: tuple[float, ...], open_gap: float = 1.3
                       ) -> tuple[HandCommand, str]:
    curls = tuple(max(0.0, min(1.0, (value - low) / max(0.35, high - low)))
                  for value, low, high in zip(features.curls, opened, closed))
    others = median(curls[2:])
    pinch = max(0.0, min(1.0, (open_gap - features.pinch_gap) / max(0.3, open_gap - 0.18)))
    if curls[1] > 0.72 and others > 0.72:
        return HandCommand(max(0.8, curls[0]), curls[1], others), "PUNO"
    command = HandCommand(max(curls[0], pinch * 0.65), max(curls[1], pinch * 0.65), others)
    if features.pinch_gap < 0.35:
        status = "PINZA"
    elif max(curls) < 0.25 and features.pinch_gap > 0.8 * open_gap:
        status = "ABIERTA"
    else:
        status = "PARCIAL"
    return command, status


class _HandState:
    def __init__(self) -> None:
        self.opened: tuple[float, ...] | None = None
        self.closed: tuple[float, ...] | None = None
        self.open_gap = 1.3
        self.samples: list[tuple[int, tuple[float, ...], float]] = []
        self.lost()

    def lost(self) -> None:
        self.samples = []
        self.since: int | None = None
        self.count = 0
        self.previous: int | None = None
        self.filtered: HandCommand | None = None
        self.pending = ""
        self.pending_since = 0
        self.label = "RECUPERANDO"

    def update(self, features: HandFeatures, timestamp: int) -> tuple[HandCommand | None, str]:
        if self.previous is not None and timestamp - self.previous > 350:
            self.lost()
        dt = (timestamp - self.previous) / 1000 if self.previous is not None else 0.0
        self.previous = timestamp
        if self.closed is None:
            closing = self.opened is not None
            valid = (median(features.curls[1:]) > 0.6 if closing else
                     max(features.curls) < 0.23 and features.pinch_gap > 0.5)
            status = "CAL: CERRAR PUNO" if closing else "CAL: ABRIR MANO"
            if not valid:
                self.samples.clear()
                return None, status
            self.samples.append((timestamp, features.curls, features.pinch_gap))
            self.samples = self.samples[-40:]
            if timestamp - self.samples[0][0] >= 500 and len(self.samples) >= 5:
                columns = tuple(zip(*(sample[1] for sample in self.samples)))
                if any(max(column) - min(column) > 0.18 for column in columns):
                    self.samples = self.samples[-1:]
                    return None, status + " ESTABLE"
                reference = tuple(median(column) for column in columns)
                if closing:
                    if any(high - low < 0.35 for high, low in zip(reference[1:], self.opened[1:])):
                        self.samples.clear()
                        return None, "CAL: CERRAR MAS"
                    self.closed = reference
                    self.lost()
                    return None, "CALIBRADA"
                self.opened = reference
                self.open_gap = median(sample[2] for sample in self.samples)
                self.samples.clear()
            return None, status
        self.since = timestamp if self.since is None else self.since
        self.count += 1
        if self.count < 2 or timestamp - self.since < 80:
            return None, "RECUPERANDO"
        command, label = normalized_command(features, self.opened, self.closed, self.open_gap)
        alpha = 1 - exp(-max(0.0, min(dt, 0.1)) / 0.065)
        if self.filtered is None:
            self.filtered = command
        else:
            self.filtered = HandCommand(*(old + alpha * (new - old)
                                          for old, new in zip(self.filtered.values, command.values)))
        if label != self.pending:
            self.pending, self.pending_since = label, timestamp
        if timestamp - self.pending_since >= 80:
            self.label = label
        return self.filtered, self.label


class HandGestureRecognizer:
    def __init__(self) -> None:
        self.recalibrate()

    def recalibrate(self) -> None:
        self.states = [_HandState(), _HandState()]
        self.last_timestamp: int | None = None

    def update(self, observation: Observation, width: int, height: int) -> GestureResult:
        timestamp = observation.timestamp_ms
        if not isfinite(timestamp) or (self.last_timestamp is not None and timestamp <= self.last_timestamp):
            for state in self.states:
                state.lost()
            return GestureResult(left="TIEMPO INVALIDO", right="TIEMPO INVALIDO")
        self.last_timestamp = timestamp
        hands = associate_hands(observation, width, height)
        commands, statuses = [], []
        for state, hand in zip(self.states, hands):
            features = measure_hand(hand, width, height) if hand is not None else None
            if features is None:
                state.lost()
                command, status = None, "SIN MANO"
            else:
                command, status = state.update(features, timestamp)
            commands.append(command)
            statuses.append(status)
        return GestureResult(HandTargets(*commands), *statuses)
