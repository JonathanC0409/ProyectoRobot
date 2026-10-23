# MuJoCo Python Setup

## Requirements

- Python >= 3.10

## Installation

The native Python bindings come pre-packaged with a copy of MuJoCo:

```bash
pip install mujoco
```

That's it. No separate MuJoCo download, no license key, no environment variables.

## Optional dependencies

```bash
pip install pillow    # for saving screenshots as PNG/JPEG
pip install numpy     # usually already installed as a mujoco dependency
pip install mediapy   # for rendering video in Jupyter notebooks
```

## Verify installation

```python
import mujoco
print(mujoco.__version__)

# Quick smoke test: load a minimal model and step
model = mujoco.MjModel.from_xml_string("<mujoco><worldbody><light pos='0 0 3'/><geom type='plane' size='1 1 .1'/><body pos='0 0 1'><freejoint/><geom type='sphere' size='.1' mass='1'/></body></worldbody></mujoco>")
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
print(f"Simulation time after one step: {data.time:.4f}s")
```

## Troubleshooting

- **macOS ARM (M1/M2/M3/M4)**: `pip install mujoco` works natively. No Rosetta needed.
- **Headless Linux servers**: Offscreen rendering via `mujoco.Renderer` works without a display. EGL is used automatically.
- **ImportError about GLFW**: Only affects the interactive viewer (`mujoco.viewer`). Offscreen rendering does not need GLFW.

## Getting models

Clone MuJoCo Menagerie for a collection of ready-made robot models:

```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```

Example model (Unitree G1 humanoid with hands):
```
mujoco_menagerie/unitree_g1/scene_with_hands.xml
```

For the most up-to-date documentation, see: https://mujoco.readthedocs.io/en/stable/python.html
