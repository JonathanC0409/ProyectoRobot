#!/usr/bin/env python3
"""
MuJoCo Simulation Client — talk to sim_server.py from Python.

Use this client to control the simulation in a single script, avoiding
per-action tool confirmations when run by an agent.

Usage:
    from sim_client import SimClient

    sim = SimClient()                        # connects to localhost:8090
    sim = SimClient("http://localhost:8090") # explicit URL

    # Step and read state
    sim.step(100)
    joints = sim.joints()
    sensors = sim.sensors()

    # Take a screenshot
    sim.screenshot("output.png", azimuth=90, distance=2.0)

    # Control joints
    sim.ctrl([0.0] * 43)
    sim.ctrl_by_name("left_hip_pitch", 1.0)
    sim.ctrl_by_names({"left_hip_pitch": 1.0, "right_hip_pitch": -1.0})

    # Full workflow example
    sim.reset()
    sim.solver(timestep=0.001, iterations=100)
    sim.camera(azimuth=0, elevation=-30, distance=3.0, lookat=[0, 0, 0.8])
    for i in range(10):
        sim.step(50)
        sim.screenshot(f"frame_{i:03d}.png")
        print(f"t={sim.status()['time']:.3f}")
"""

import json
import os
import urllib.request
import urllib.parse
import sys


class SimClient:
    """Lightweight HTTP client for sim_server.py — no dependencies beyond stdlib."""

    def __init__(self, base_url=None):
        if base_url is None:
            port = os.environ.get("MUJOCO_SIM_PORT", "8090")
            base_url = f"http://localhost:{port}"
        self.base_url = base_url.rstrip("/")

    def _get(self, path, params=None):
        url = f"{self.base_url}{path}"
        if params:
            url += "?" + urllib.parse.urlencode(params)
        with urllib.request.urlopen(url) as resp:
            return json.loads(resp.read())

    def _get_bytes(self, path, params=None):
        url = f"{self.base_url}{path}"
        if params:
            url += "?" + urllib.parse.urlencode(params)
        with urllib.request.urlopen(url) as resp:
            return resp.read()

    def _post(self, path, data=None):
        url = f"{self.base_url}{path}"
        body = json.dumps(data or {}).encode()
        req = urllib.request.Request(url, data=body, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as resp:
            return json.loads(resp.read())

    # ── Core simulation ─────────────────────────────────────────────────

    def status(self):
        """Get model info and simulation time."""
        return self._get("/status")

    def step(self, n=1):
        """Advance simulation by n timesteps."""
        return self._post("/step", {"n": n})

    def reset(self, keyframe=None):
        """Reset simulation to initial state (or a keyframe)."""
        body = {"keyframe": keyframe} if keyframe is not None else {}
        return self._post("/reset", body)

    # ── State reading ────────────────────────────────────────────────────

    def joints(self):
        """Read all joint positions and velocities."""
        return self._get("/joints")

    def sensors(self):
        """Read all sensor values."""
        return self._get("/sensors")

    def bodies(self):
        """Read all body positions and orientations."""
        return self._get("/bodies")

    def actuators(self):
        """List all actuators and their current control values."""
        return self._get("/actuators")

    # ── Control ──────────────────────────────────────────────────────────

    def ctrl(self, values):
        """Set all actuator controls. values: list of floats (length = nu)."""
        return self._post("/ctrl", {"ctrl": values})

    def ctrl_by_name(self, name, value):
        """Set a single actuator by name."""
        return self._post("/ctrl", {"name": name, "value": value})

    def ctrl_by_names(self, values_dict):
        """Set multiple actuators by name. values_dict: {name: value, ...}."""
        return self._post("/ctrl", {"values": values_dict})

    def force(self, body_name, force):
        """Apply external force/torque to a body. force: [fx, fy, fz, tx, ty, tz]."""
        return self._post("/force", {"body": body_name, "force": force})

    # ── Camera ───────────────────────────────────────────────────────────

    def camera(self, **kwargs):
        """Configure the default camera. Accepts: azimuth, elevation, distance, lookat, camera_name."""
        return self._post("/camera", kwargs)

    # ── Rendering ────────────────────────────────────────────────────────

    def screenshot(self, output_path="screenshot.png", **kwargs):
        """
        Render a screenshot and save to file.

        Keyword args (all optional):
            camera:    name of model-defined camera
            azimuth:   horizontal angle (degrees)
            elevation: vertical angle (degrees)
            distance:  distance from lookat
            lookat_x, lookat_y, lookat_z: lookat point
            width, height: image dimensions
        """
        params = {k: v for k, v in kwargs.items() if v is not None}
        png_data = self._get_bytes("/screenshot", params)
        with open(output_path, "wb") as f:
            f.write(png_data)
        return output_path

    # ── Solver / Physics ─────────────────────────────────────────────────

    def solver(self, **kwargs):
        """
        Configure solver/integrator.

        Keyword args: solver, integrator, iterations, tolerance, timestep, gravity
        """
        return self._post("/solver", kwargs)

    # ── Convenience ──────────────────────────────────────────────────────

    def step_and_screenshot(self, n=1, output_path="screenshot.png", **cam_kwargs):
        """Step n times, then take a screenshot."""
        self.step(n)
        return self.screenshot(output_path, **cam_kwargs)


# ── CLI usage ────────────────────────────────────────────────────────────

def _cli():
    """
    CLI for controlling a running sim_server.py.

    Usage:
        python sim_client.py <command> [args...]

    Commands:
        status                              Show model info and sim time
        step [N]                            Step N times (default: 1)
        reset                               Reset to initial state
        joints                              Read all joint positions/velocities
        sensors                             Read all sensor values
        bodies                              Read all body positions
        actuators                           List all actuators
        screenshot [PATH] [--azimuth A]     Render screenshot
                   [--elevation E]
                   [--distance D]
                   [--camera NAME]
                   [--width W] [--height H]
        ctrl NAME VALUE                     Set single actuator by name
        ctrl-all V0 V1 V2 ...              Set all actuator controls
        camera [--azimuth A] [--elevation E] [--distance D]
               [--lookat X Y Z]            Configure default camera
        solver [--timestep DT]              Configure solver
               [--iterations N]
               [--gravity X Y Z]
        force BODY FX FY FZ [TX TY TZ]     Apply force to body

    Environment:
        MUJOCO_SIM_PORT    Server port (default: 8090)

    Examples:
        python sim_client.py status
        python sim_client.py step 500
        python sim_client.py screenshot front.png --azimuth 0 --distance 2.5
        python sim_client.py ctrl left_hip_pitch_joint 1.0
        python sim_client.py camera --azimuth 90 --elevation -30 --lookat 0 0 0.8
        python sim_client.py solver --timestep 0.001 --iterations 100
        python sim_client.py force torso 0 0 100
    """
    sim = SimClient()

    if len(sys.argv) < 2:
        print(_cli.__doc__)
        sys.exit(1)

    cmd = sys.argv[1]
    args = sys.argv[2:]

    def parse_flags(args, known_flags):
        """Parse --flag value pairs from args list."""
        result = {}
        positional = []
        i = 0
        while i < len(args):
            if args[i].startswith("--") and args[i][2:] in known_flags:
                flag = args[i][2:]
                if i + 1 < len(args):
                    result[flag] = args[i + 1]
                    i += 2
                else:
                    i += 1
            else:
                positional.append(args[i])
                i += 1
        return positional, result

    if cmd == "status":
        print(json.dumps(sim.status(), indent=2))

    elif cmd == "step":
        n = int(args[0]) if args else 1
        print(json.dumps(sim.step(n), indent=2))

    elif cmd == "reset":
        print(json.dumps(sim.reset(), indent=2))

    elif cmd == "joints":
        data = sim.joints()
        if "--name" in args:
            name = args[args.index("--name") + 1]
            j = data["joints"].get(name)
            if j:
                print(json.dumps({"name": name, **j}, indent=2))
            else:
                print(f"Joint '{name}' not found")
                sys.exit(1)
        else:
            print(json.dumps(data, indent=2))

    elif cmd == "sensors":
        print(json.dumps(sim.sensors(), indent=2))

    elif cmd == "bodies":
        print(json.dumps(sim.bodies(), indent=2))

    elif cmd == "actuators":
        print(json.dumps(sim.actuators(), indent=2))

    elif cmd == "screenshot":
        pos, flags = parse_flags(args, ["azimuth", "elevation", "distance", "camera", "width", "height"])
        path = pos[0] if pos else "screenshot.png"
        kwargs = {}
        for k in ["azimuth", "elevation", "distance", "width", "height"]:
            if k in flags:
                kwargs[k] = float(flags[k])
        if "camera" in flags:
            kwargs["camera"] = flags["camera"]
        sim.screenshot(path, **kwargs)
        print(f"Saved: {path}")

    elif cmd == "ctrl":
        if len(args) >= 2:
            sim.ctrl_by_name(args[0], float(args[1]))
            print(f"Set {args[0]} = {args[1]}")
        else:
            print("Usage: ctrl NAME VALUE")
            sys.exit(1)

    elif cmd == "ctrl-all":
        values = [float(v) for v in args]
        result = sim.ctrl(values)
        print(json.dumps(result, indent=2))

    elif cmd == "camera":
        pos, flags = parse_flags(args, ["azimuth", "elevation", "distance"])
        kwargs = {}
        for k in ["azimuth", "elevation", "distance"]:
            if k in flags:
                kwargs[k] = float(flags[k])
        # Handle --lookat X Y Z
        if "--lookat" in sys.argv:
            idx = sys.argv.index("--lookat")
            kwargs["lookat"] = [float(sys.argv[idx + 1]), float(sys.argv[idx + 2]), float(sys.argv[idx + 3])]
        print(json.dumps(sim.camera(**kwargs), indent=2))

    elif cmd == "solver":
        _, flags = parse_flags(args, ["timestep", "iterations", "tolerance"])
        kwargs = {}
        if "timestep" in flags:
            kwargs["timestep"] = float(flags["timestep"])
        if "iterations" in flags:
            kwargs["iterations"] = int(flags["iterations"])
        if "tolerance" in flags:
            kwargs["tolerance"] = float(flags["tolerance"])
        if "--gravity" in sys.argv:
            idx = sys.argv.index("--gravity")
            kwargs["gravity"] = [float(sys.argv[idx + 1]), float(sys.argv[idx + 2]), float(sys.argv[idx + 3])]
        print(json.dumps(sim.solver(**kwargs), indent=2))

    elif cmd == "force":
        if len(args) < 4:
            print("Usage: force BODY FX FY FZ [TX TY TZ]")
            sys.exit(1)
        body_name = args[0]
        force = [float(x) for x in args[1:7]]
        while len(force) < 6:
            force.append(0.0)
        print(json.dumps(sim.force(body_name, force), indent=2))

    else:
        print(f"Unknown command: {cmd}")
        print("Run without arguments to see usage.")
        sys.exit(1)


if __name__ == "__main__":
    _cli()
