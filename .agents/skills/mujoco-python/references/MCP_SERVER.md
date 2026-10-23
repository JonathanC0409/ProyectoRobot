# MuJoCo MCP Server

Exposes MuJoCo simulation control as native MCP tools. Once configured, the agent
can call tools like `mcp__mujoco__step`, `mcp__mujoco__screenshot`, etc. directly
without writing scripts.

## Setup

### 1. Install dependencies

```bash
pip install mcp mujoco pillow
```

### 2. Add to Claude Code settings

In `~/.claude/settings.json` (or project `.claude/settings.json`):

```json
{
  "mcpServers": {
    "mujoco": {
      "command": "python3",
      "args": [
        "/path/to/skills/mujoco-python/scripts/sim_mcp.py",
        "/path/to/scene.xml"
      ]
    }
  }
}
```

### 3. Restart Claude Code

The following tools become available:

| Tool | Description |
|------|-------------|
| `status` | Model info, sim time, solver config |
| `step` | Advance N timesteps |
| `reset` | Reset to initial state or keyframe |
| `screenshot` | Render and save PNG |
| `sensors` | Read all sensor values |
| `joints` | Read all joint qpos/qvel |
| `bodies` | Read all body positions/orientations |
| `ctrl` | Set actuator controls (by array or name) |
| `set_camera` | Configure default camera |
| `configure_solver` | Change solver, integrator, timestep, gravity |
| `apply_force` | Apply external force/torque to a body |

## When to Use MCP vs HTTP

| | MCP | HTTP |
|---|-----|------|
| **Setup** | Requires config + restart | Start server in background |
| **Agent UX** | Native tool calls | Writes Python scripts |
| **Confirmations** | Per tool call | Per script (batched) |
| **Best for** | Persistent setups | Ad-hoc / exploratory |
| **Dependencies** | `mcp` package | stdlib only |
