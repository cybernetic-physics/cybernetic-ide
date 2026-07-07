---
name: robotics-debugging
description: Use when diagnosing Cybernetic IDE robotics runtime failures, lag, Docker/MuJoCo problems, viewer issues, protocol errors, Python control script failures, or Unitree SDK facade behavior.
---

# Robotics Debugging

Use this skill when the robot viewer is blank, the sim feels laggy, a Python control script fails, the Agent Panel cannot see robotics tools, or the Docker/MuJoCo runtime is unhealthy.

## First Checks

1. `sim_status`
2. `docker_logs`
3. `protocol_probe_http` with `/status`
4. `protocol_probe_ws` with `simulation_state`
5. `viewer_snapshot`

These separate Docker/runtime health from viewer/UI issues.

## Common Findings

- Simulator controls can acknowledge quickly while visible frame feedback is slow because the current server renders frames in Docker with software MuJoCo and cached JPEGs.
- `camera_frame_0` over WebSocket is not necessarily faster; the current compatible server renders PNG frames synchronously.
- If the Agent Panel does not list robotics tools, check that the `cybernetic-robotics` context server is active and that the active profile is `Robotics`.
- If Python cannot import `unitree_sdk2py`, run scripts through `python_control_run` or set `PYTHONPATH=overlays/unitree-g1-sdk-shim`.

## Useful Files

- `script/cyber-robotics-mcp.mjs`
- `cyber-robotics-mcp`
- `overlays/unitree-g1-mujoco-protocol/python/g1_protocol_sim.py`
- `overlays/unitree-g1-sdk-shim/`
- `examples/g1_raise_hand_sdk.py`
- `examples/control_g1_sim.py`
- `.runtime/unitree-g1-mujoco/compose.env`

## Debugging Order

Prefer live probes before code edits. Once the failing layer is known:

- Runtime/assets: use `sim_prepare_runtime`, `sim_start`, or Docker logs.
- Protocol: use HTTP/WS probes and compare endpoint results.
- Viewer: capture `viewer_snapshot` and check whether the sim frame itself is good.
- Python SDK: run the smallest script through `python_control_run`.
- Scene editing: validate MJCF before recreating the runtime.
