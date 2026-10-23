# MuJoCo HTTP API

A zero-dependency HTTP server for persistent simulation control. The agent starts the server
in the background and then controls it via `sim_client.py` — one Python script per action batch,
avoiding per-action tool confirmations.

The server is built on two classes:

- **`SimApp`** — generic HTTP server with decorator-based GET/POST routing. Knows nothing about MuJoCo.
- **`MujocoSimApp(SimApp)`** — extends SimApp with built-in MuJoCo simulation routes.

## Quick Start

### 1. Start the server (background)

```bash
python scripts/sim_server.py path/to/scene.xml &
```

The port defaults to `8090`. Override via `--port` flag or `MUJOCO_SIM_PORT` env variable:

```bash
export MUJOCO_SIM_PORT=9000
python scripts/sim_server.py path/to/scene.xml &
```

Both `sim_server.py` and `sim_client.py` read `MUJOCO_SIM_PORT` automatically.

### 2. Control from Python (single script, single confirmation)

```python
import sys
sys.path.insert(0, "path/to/skills/mujoco-python/scripts")
from sim_client import SimClient

sim = SimClient()  # connects to localhost:8090

# Read initial state
print(sim.status())

# Step and observe
sim.step(500)
print(sim.joints())
print(sim.sensors())

# Control joints
sim.ctrl_by_names({"left_hip_pitch_joint": 0.5, "right_hip_pitch_joint": -0.5})
sim.step(200)

# Camera + screenshot
sim.camera(azimuth=0, elevation=-30, distance=2.5, lookat=[0, 0, 0.8])
sim.screenshot("front_view.png")
sim.screenshot("side_view.png", azimuth=90)

# Reset and reconfigure
sim.reset()
sim.solver(timestep=0.001, iterations=100)
```

### 3. CLI usage

`sim_client.py` also works as a CLI tool:

```bash
python sim_client.py status
python sim_client.py step 500
python sim_client.py screenshot front.png --azimuth 0 --distance 2.5
python sim_client.py ctrl left_hip_pitch_joint 1.0
python sim_client.py camera --azimuth 90 --elevation -30 --lookat 0 0 0.8
python sim_client.py solver --timestep 0.001 --iterations 100
python sim_client.py sensors
python sim_client.py joints
python sim_client.py bodies
python sim_client.py actuators
python sim_client.py reset
python sim_client.py force torso 0 0 100
```

### 4. Or use curl (one-off commands)

```bash
curl localhost:8090/status
curl -X POST localhost:8090/step -d '{"n": 100}'
curl -o screenshot.png "localhost:8090/screenshot?azimuth=90"
curl -X POST localhost:8090/ctrl -d '{"values": {"left_hip_pitch_joint": 0.5}}'
```

## Built-in Endpoints (MujocoSimApp)

| Method | Path | Body / Params | Returns |
|--------|------|---------------|---------|
| GET | `/status` | — | Model info, sim time |
| GET | `/routes` | — | All registered routes |
| POST | `/step` | `{"n": 100}` | New sim time |
| POST | `/reset` | `{"keyframe": 0}` (optional) | Confirmation |
| GET | `/screenshot` | `?azimuth=90&distance=2.5&camera=name` | PNG image |
| GET | `/sensors` | — | All sensor readings |
| GET | `/joints` | — | All joint qpos/qvel |
| GET | `/bodies` | — | All body positions |
| GET | `/actuators` | — | Actuator names + current ctrl |
| POST | `/ctrl` | `{"ctrl": [...]}` or `{"name": "x", "value": 1.0}` or `{"values": {"x": 1.0}}` | Updated ctrl |
| POST | `/camera` | `{"azimuth": 90, "distance": 2}` | Camera state |
| POST | `/solver` | `{"timestep": 0.001, "iterations": 100}` | Solver config |
| POST | `/force` | `{"body": "torso", "force": [0,0,100,0,0,0]}` | Confirmation |

## Extending and Customizing

### Add custom routes

```python
from sim_server import MujocoSimApp

app = MujocoSimApp("scene.xml")

@app.get("/reward")
def reward(handler, params):
    height = float(app.data.qpos[2])
    handler.json_response({"reward": -abs(height - 0.793)})

@app.post("/pd_step")
def pd_step(handler, body):
    target = body.get("target", [0.0] * app.model.nu)
    kp, kd = body.get("kp", 50.0), body.get("kd", 5.0)
    n = body.get("n", 1)
    free = app.model.nq > app.model.nu
    qoff, voff = (7, 6) if free else (0, 0)
    for _ in range(n):
        for i in range(app.model.nu):
            q = app.data.qpos[qoff + i]
            dq = app.data.qvel[voff + i]
            app.data.ctrl[i] = kp * (target[i] - q) + kd * (0.0 - dq)
        import mujoco
        mujoco.mj_step(app.model, app.data)
    handler.json_response({"time": float(app.data.time), "steps": n})

app.run()
```

### Override built-in routes

```python
app = MujocoSimApp("scene.xml")

@app.post("/step")
def custom_step(handler, body):
    """Override /step to add a pre-step hook."""
    n = body.get("n", 1)
    for _ in range(n):
        my_controller(app.model, app.data)  # your logic
        import mujoco
        mujoco.mj_step(app.model, app.data)
    handler.json_response({"time": float(app.data.time), "steps": n})

app.run()
```

### Wrap existing model/data (no reload)

If you already have `model` and `data` loaded in your script:

```python
from sim_server import MujocoSimApp

app = MujocoSimApp(model=model, data=data)
app.run()
```

`app.model` and `app.data` are the same objects — no copies, no sync issues.

### Run alongside your own control loop

```python
import threading

app = MujocoSimApp(model=model, data=data)
threading.Thread(target=lambda: app.run(port=8090), daemon=True).start()

# Your loop — agent can observe/intervene via HTTP
while True:
    data.ctrl[:] = my_policy(data)
    mujoco.mj_step(model, data)
```

### Use SimApp for non-MuJoCo servers

`SimApp` is a generic HTTP micro-framework you can use for anything:

```python
from sim_server import SimApp

app = SimApp()

@app.get("/hello")
def hello(handler, params):
    handler.json_response({"msg": "world"})

@app.post("/echo")
def echo(handler, body):
    handler.json_response(body)

app.run(port=3000)
```

## Dependencies

None beyond `mujoco` and `pillow` (for screenshots). Uses Python stdlib `http.server`.
