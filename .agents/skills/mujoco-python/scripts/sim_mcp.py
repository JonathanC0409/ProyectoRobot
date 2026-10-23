#!/usr/bin/env python3
"""
MuJoCo Simulation MCP Server.

Exposes MuJoCo simulation control as MCP tools so an agent can call them
directly (e.g., mcp__mujoco__step, mcp__mujoco__screenshot).

Requirements:
    pip install mcp mujoco pillow

Usage:
    python sim_mcp.py <scene.xml>

    Then add to your Claude Code settings.json:
    {
      "mcpServers": {
        "mujoco": {
          "command": "python3",
          "args": ["path/to/sim_mcp.py", "path/to/scene.xml"]
        }
      }
    }
"""

import argparse
import base64
import io
import json
import sys

import mujoco
import numpy as np

try:
    from mcp.server.fastmcp import FastMCP
except ImportError:
    print("ERROR: MCP package not installed. Run: pip install mcp", file=sys.stderr)
    sys.exit(1)

# ── Globals ──────────────────────────────────────────────────────────────

MODEL = None
DATA = None
CAMERA = None


def init_sim(scene_path):
    global MODEL, DATA, CAMERA
    MODEL = mujoco.MjModel.from_xml_path(scene_path)
    DATA = mujoco.MjData(MODEL)
    CAMERA = mujoco.MjvCamera()
    CAMERA.type = mujoco.mjtCamera.mjCAMERA_FREE
    CAMERA.lookat[:] = [0, 0, 1.0]
    CAMERA.distance = 3.0
    CAMERA.azimuth = 135
    CAMERA.elevation = -25
    mujoco.mj_forward(MODEL, DATA)


# ── MCP Server ───────────────────────────────────────────────────────────

mcp = FastMCP("mujoco")


@mcp.tool()
def status() -> str:
    """Get simulation status: time, model info, solver config."""
    return json.dumps({
        "time": float(DATA.time),
        "timestep": float(MODEL.opt.timestep),
        "nq": MODEL.nq, "nv": MODEL.nv, "nu": MODEL.nu,
        "nbody": MODEL.nbody, "njnt": MODEL.njnt, "nsensor": MODEL.nsensor,
        "solver": int(MODEL.opt.solver),
        "integrator": int(MODEL.opt.integrator),
    })


@mcp.tool()
def step(n: int = 1) -> str:
    """Advance the simulation by n timesteps."""
    for _ in range(n):
        mujoco.mj_step(MODEL, DATA)
    return json.dumps({"time": float(DATA.time), "steps": n})


@mcp.tool()
def reset(keyframe: int = -1) -> str:
    """Reset simulation to initial state, or to a specific keyframe index."""
    if keyframe >= 0:
        mujoco.mj_resetDataKeyframe(MODEL, DATA, keyframe)
    else:
        mujoco.mj_resetData(MODEL, DATA)
    mujoco.mj_forward(MODEL, DATA)
    return json.dumps({"time": float(DATA.time), "status": "reset"})


@mcp.tool()
def screenshot(
    output_path: str = "screenshot.png",
    width: int = 640,
    height: int = 480,
    azimuth: float = None,
    elevation: float = None,
    distance: float = None,
    lookat_x: float = None,
    lookat_y: float = None,
    lookat_z: float = None,
    camera_name: str = None,
) -> str:
    """Render a screenshot and save to file. Returns the file path."""
    from PIL import Image

    cam = mujoco.MjvCamera()
    cam.type = CAMERA.type
    cam.lookat[:] = CAMERA.lookat
    cam.distance = CAMERA.distance
    cam.azimuth = CAMERA.azimuth
    cam.elevation = CAMERA.elevation

    if azimuth is not None:
        cam.azimuth = azimuth
    if elevation is not None:
        cam.elevation = elevation
    if distance is not None:
        cam.distance = distance
    if lookat_x is not None:
        cam.lookat[0] = lookat_x
    if lookat_y is not None:
        cam.lookat[1] = lookat_y
    if lookat_z is not None:
        cam.lookat[2] = lookat_z
    if camera_name:
        cam_id = mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if cam_id >= 0:
            cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            cam.fixedcamid = cam_id

    renderer = mujoco.Renderer(MODEL, height=height, width=width)
    renderer.update_scene(DATA, camera=cam)
    pixels = renderer.render()
    renderer.close()

    Image.fromarray(pixels).save(output_path)
    return json.dumps({"path": output_path, "width": width, "height": height})


@mcp.tool()
def sensors() -> str:
    """Read all sensor values."""
    result = {}
    for i in range(MODEL.nsensor):
        name = MODEL.sensor(i).name
        result[name] = DATA.sensor(name).data.tolist()
    return json.dumps({"time": float(DATA.time), "sensors": result})


@mcp.tool()
def joints() -> str:
    """Read all joint positions and velocities."""
    result = {}
    for i in range(MODEL.njnt):
        name = MODEL.joint(i).name
        result[name] = {
            "qpos": DATA.joint(name).qpos.tolist(),
            "qvel": DATA.joint(name).qvel.tolist(),
        }
    return json.dumps({"time": float(DATA.time), "joints": result})


@mcp.tool()
def bodies() -> str:
    """Read all body positions and orientations."""
    result = {}
    for i in range(MODEL.nbody):
        name = MODEL.body(i).name
        result[name] = {
            "pos": DATA.xpos[i].tolist(),
            "quat": DATA.xquat[i].tolist(),
        }
    return json.dumps({"time": float(DATA.time), "bodies": result})


@mcp.tool()
def ctrl(values: str = None, name: str = None, value: float = None) -> str:
    """
    Set actuator controls.
    - values: JSON array of all control values, e.g. "[0.0, 1.0, ...]"
    - name + value: set a single actuator by name
    """
    if values:
        DATA.ctrl[:] = np.array(json.loads(values), dtype=np.float64)
    elif name and value is not None:
        act_id = mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if act_id < 0:
            return json.dumps({"error": f"Actuator '{name}' not found"})
        DATA.ctrl[act_id] = float(value)
    return json.dumps({"ctrl": DATA.ctrl.tolist()})


@mcp.tool()
def set_camera(
    azimuth: float = None,
    elevation: float = None,
    distance: float = None,
    lookat_x: float = None,
    lookat_y: float = None,
    lookat_z: float = None,
) -> str:
    """Configure the default camera for subsequent screenshots."""
    if azimuth is not None:
        CAMERA.azimuth = azimuth
    if elevation is not None:
        CAMERA.elevation = elevation
    if distance is not None:
        CAMERA.distance = distance
    if lookat_x is not None:
        CAMERA.lookat[0] = lookat_x
    if lookat_y is not None:
        CAMERA.lookat[1] = lookat_y
    if lookat_z is not None:
        CAMERA.lookat[2] = lookat_z
    return json.dumps({
        "azimuth": CAMERA.azimuth, "elevation": CAMERA.elevation,
        "distance": CAMERA.distance, "lookat": CAMERA.lookat.tolist(),
    })


@mcp.tool()
def configure_solver(
    solver: int = None,
    integrator: int = None,
    iterations: int = None,
    tolerance: float = None,
    timestep: float = None,
    gravity_x: float = None,
    gravity_y: float = None,
    gravity_z: float = None,
) -> str:
    """Configure physics solver/integrator. Solver: 0=PGS, 1=CG, 2=Newton. Integrator: 0=Euler, 1=RK4, 2=Implicit, 3=ImplicitFast."""
    if solver is not None:
        MODEL.opt.solver = solver
    if integrator is not None:
        MODEL.opt.integrator = integrator
    if iterations is not None:
        MODEL.opt.iterations = iterations
    if tolerance is not None:
        MODEL.opt.tolerance = tolerance
    if timestep is not None:
        MODEL.opt.timestep = timestep
    if gravity_x is not None:
        MODEL.opt.gravity[0] = gravity_x
    if gravity_y is not None:
        MODEL.opt.gravity[1] = gravity_y
    if gravity_z is not None:
        MODEL.opt.gravity[2] = gravity_z
    return json.dumps({
        "solver": int(MODEL.opt.solver),
        "integrator": int(MODEL.opt.integrator),
        "iterations": MODEL.opt.iterations,
        "tolerance": MODEL.opt.tolerance,
        "timestep": float(MODEL.opt.timestep),
        "gravity": MODEL.opt.gravity.tolist(),
    })


@mcp.tool()
def apply_force(body_name: str, fx: float = 0, fy: float = 0, fz: float = 0,
                tx: float = 0, ty: float = 0, tz: float = 0) -> str:
    """Apply external force and torque to a body."""
    body_id = mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        return json.dumps({"error": f"Body '{body_name}' not found"})
    DATA.xfrc_applied[body_id] = [fx, fy, fz, tx, ty, tz]
    return json.dumps({"body": body_name, "force": [fx, fy, fz, tx, ty, tz]})


# ── Main ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="MuJoCo MCP Server")
    parser.add_argument("scene", help="Path to MuJoCo XML scene file")
    args = parser.parse_args()

    init_sim(args.scene)
    print(f"MuJoCo MCP server loaded: {args.scene} (nq={MODEL.nq} nu={MODEL.nu})", file=sys.stderr)

    mcp.run(transport="stdio")


if __name__ == "__main__":
    main()
