"""Señales de prueba exclusivas del modo demo sin webcam."""

from math import cos, pi, radians

from proyectorobot.control.motion import ArmAngles, ArmTargets, JOINTS, NEUTRAL


DEMO_DURATION = 24.0


def demo_joint(seconds: float) -> int:
    return min(len(JOINTS) - 1, max(0, int(seconds / 3.0)))


def demo_targets(seconds: float) -> ArmTargets:
    """Prueba cada eje; el giro axial prepara un codo flexionado para hacerlo visible."""
    index = demo_joint(seconds)
    phase = max(0.0, seconds) % 3.0
    values = list(NEUTRAL)
    amplitude = radians(70 if index % 4 != 2 else 60)
    values[index] += amplitude * (1 - cos(2 * pi * phase / 3)) / 2
    if index % 4 == 2:
        values[index + 1] = radians(60)
    return ArmTargets(*(
        ArmAngles(values[offset], values[offset + 3], values[offset + 1], values[offset + 2])
        for offset in (0, 4)
    ))
