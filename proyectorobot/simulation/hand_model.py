"""Escena ligera de manipulación; conserva el MJCF y los ocho ejes originales."""

from math import radians
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

from proyectorobot.control.motion import ArmAngles, ArmTargets, JOINTS, NEUTRAL


ASSETS = Path(__file__).parent / "assets"
FINGERS = ("thumb", "index", "middle", "ring", "little")
CONTROLS = ("thumb", "index", "middle")


def task_pose(angle: float) -> ArmAngles:
    """Codo y hombro compensados para que la pinza apunte hacia abajo."""
    return ArmAngles(radians(angle), radians(angle), rotation=radians(90))


def task_targets(angle: float, side: str = "left") -> ArmTargets:
    return ArmTargets(**{side: task_pose(angle)})


def safe_posture(side: str) -> tuple[float, ...]:
    values = list(NEUTRAL)
    offset = 0 if side == "left" else 4
    values[offset:offset + 4] = task_pose(65).values
    return tuple(values)


def _hand(side: str) -> ET.Element:
    palm = ET.parse(ASSETS / "hand.xml").getroot()
    for element in palm.iter():
        if "name" in element.attrib:
            element.set("name", element.get("name").replace("left_", side + "_"))
        if side == "right":
            for attribute in ("pos", "fromto", "axis", "euler"):
                if attribute not in element.attrib:
                    continue
                values = [float(value) for value in element.get(attribute).split()]
                for index in range(len(values)):
                    if (attribute in ("pos", "fromto") and index % 3 == 0
                            or attribute in ("axis", "euler") and index % 3 != 0):
                        values[index] *= -1
                element.set(attribute, " ".join(map(str, values)))
    return palm


def scene_xml(side: str = "left") -> str:
    """Genera la escena; las posiciones iniciales se definen antes de simular."""
    if side not in ("left", "right"):
        raise ValueError("El lado debe ser left o right.")
    root = ET.parse(ASSETS / "humanoid.xml").getroot()
    root.remove(root.find("keyframe"))
    root.find("option").set("timestep", "0.002")
    root.find("option").set("iterations", "50")
    root.find("option").set("cone", "elliptic")
    geom_default = root.find("default/geom")
    geom_default.set("solref", "0.008 1")
    geom_default.set("solimp", "0.95 0.99 0.001")
    equality = ET.SubElement(root, "equality")
    actuator = root.find("actuator")
    for hand_side in ("left", "right"):
        forearm = root.find(f".//body[@name='{hand_side}_forearm']")
        forearm.remove(forearm.find(f"geom[@name='{hand_side}_hand']"))
        forearm.append(_hand(hand_side))
        for finger in CONTROLS:
            ET.SubElement(actuator, "position", name=f"{hand_side}_{finger}_servo",
                          joint=f"{hand_side}_{finger}_joint", kp="1.5", kv="0.06",
                          forcerange="-0.15 0.15")
        for finger in ("ring", "little"):
            ET.SubElement(equality, "joint", joint1=f"{hand_side}_{finger}_joint",
                          joint2=f"{hand_side}_middle_joint", polycoef="0 1 0 0 0")
    robot_only = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(robot_only)
    data = mujoco.MjData(model)
    for spec, value in zip(JOINTS,
                           task_pose(45).values + NEUTRAL[4:] if side == "left"
                           else NEUTRAL[:4] + task_pose(45).values):
        data.joint(spec.name).qpos[0] = value
    mujoco.mj_forward(model, data)
    center = data.site(f"{side}_grip").xpos.copy()
    table_z = center[2] - 0.02
    world = root.find("worldbody")
    floor = world.find("geom[@name='floor']")
    floor.set("contype", "1")
    floor.set("conaffinity", "3")
    sign = 1 if side == "left" else -1
    ET.SubElement(world, "geom", name="table", type="box",
                  pos=f"{sign * 0.60} 0.07 {table_z - 0.025}", size="0.27 0.36 0.025",
                  contype="1", conaffinity="3", rgba="0.38 0.28 0.2 1")
    for index, color in enumerate(("0.9 0.2 0.16 1", "0.15 0.45 0.95 1", "0.95 0.8 0.15 1")):
        position = (center[0] + sign * index * 0.10, center[1] + index * 0.12, center[2])
        body = ET.SubElement(world, "body", name=f"cube_{index}",
                             pos=" ".join(map(str, position)))
        ET.SubElement(body, "freejoint", name=f"cube_{index}_free")
        ET.SubElement(body, "geom", name=f"cube_{index}_geom", type="box",
                      size="0.02 0.02 0.02", mass="0.04", contype="1", conaffinity="3",
                      friction="1.5 0.03 0.001", condim="4", rgba=color)
    return ET.tostring(root, encoding="unicode")
