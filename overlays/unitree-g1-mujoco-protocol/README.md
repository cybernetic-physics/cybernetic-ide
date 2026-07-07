# Unitree G1 MuJoCo Protocol Container

This overlay is a separate, explicit runtime for a Unitree G1 MuJoCo scene.
It follows the Booster simulator client shape instead of inventing a Zed-only
API:

- MuJoCo runs inside Docker.
- The IDE/client connects to a physics WebSocket on `8788`.
- Subscriptions use the same `subscribe:<topic>` text convention as the
  reversed Booster simulator.
- Binary frames use the same 8-byte big-endian `{message_type, payload_len}`
  envelope followed by MessagePack.
- `38383` exposes `/health`, `/status`, `/visual_frame`, `/command`,
  `/camera`, `/camera_control`, `/camera_frame_0.png`, and
  `/camera_frame_0.jpg`.

Supported topics:

- `simulation_state` (`6`)
- `visual_scene` (`10`)
- `visual_frame` (`11`)
- `camera_frame_0` (`3`, MessagePack with PNG bytes)

Camera controls:

- `GET /camera` returns the current MuJoCo free-camera state.
- `POST /camera` accepts `{ "action": "orbit" | "pan" | "zoom" | "reset" }`
  plus `dx`/`dy` or `delta` values and returns the updated camera state.
- `POST /camera_frame_0.png` accepts the same camera command, applies it, and
  returns the freshly rendered PNG in one round trip.
- `GET /camera_frame_0.jpg` returns the latest cached JPEG from the background
  MuJoCo render loop (`UNITREE_G1_RENDER_HZ`, default `8`). This is the lower
  overhead path for embedded viewport refresh.
- `POST /camera_frame_0.jpg` still accepts a camera command and returns the
  latest cached JPEG, but interactive clients should prefer `POST /camera`
  plus `GET /camera_frame_0.jpg` so input is not coupled to image rendering.
  The Booster-like WebSocket camera topic remains PNG-encoded.
- WebSocket JSON commands with `{ "type": "camera", ... }` use the same action
  payload for clients that want to keep all control traffic on the reversed
  physics socket.

Simulator commands:

- WebSocket `{ "type": "command", "command": "pause" | "resume" | "reset" | "step" }`
  controls the MuJoCo lifecycle.
- WebSocket `{ "type": "command", "command": "pose", "pose": "raise_right_hand" }`
  applies a deterministic G1 arm pose directly in MuJoCo. This is an interim
  simulator command, not the official SDK2/DDS arm-control path.
- `POST /command` accepts the same command JSON over HTTP for simple probes.

The Unitree assets are not vendored in this repo. The prepare script fetches
the official `unitreerobotics/unitree_mujoco` repository at a pinned commit
into `.runtime/unitree-g1-mujoco/unitree_mujoco` and mounts it read-only.

Probe the live Booster-style envelope from this repo with:

```sh
node script/probe-unitree-g1-mujoco-protocol.mjs --topic simulation_state
node script/probe-unitree-g1-mujoco-protocol.mjs --topic visual_frame
```
