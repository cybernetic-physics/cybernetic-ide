# Unitree G1 MuJoCo Protocol Container

This overlay is Cybernetic IDE's separate, explicit runtime for a Unitree G1
MuJoCo scene. It follows the Booster simulator client shape instead of
inventing an editor-only API:

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
- `loco` commands with `action=set_arm_task` accept Unitree wave/shake task IDs
  plus the official G1 `G1ArmActionClient.action_map` IDs and map recognized
  values onto named poses such as `high_five`, `hug`, `heart`, and
  `raise_right_hand`. Unknown task IDs are recorded in locomotion state without
  applying an unexpected pose.
- `lowcmd` commands record `received_at`, `age_seconds`, `expires_at`,
  `active`, `stale`, and `watchdog_seconds` metadata. The watchdog timeout is
  controlled by `UNITREE_G1_LOWCMD_WATCHDOG_SECONDS` and defaults to `2.0`.
- `yoga_policy` commands start, stop, or inspect the optional LocoMuJoCo-trained
  policy runtime. Set `UNITREE_G1_POLICY_BUNDLE` to a packed `g1-yoga-pack`
  bundle path inside the container; when the file is absent the command returns
  `no policy bundle loaded`.
- `POST /command` accepts the same command JSON over HTTP for simple probes.

The default compose runtime mounts
`.runtime/unitree-g1-mujoco/policy/` to
`/opt/unitree-g1-mujoco-protocol/policy/`. Copy a deploy bundle to
`.runtime/unitree-g1-mujoco/policy/g1_yoga_policy.npz` and recreate the
container to enable `yoga_policy` mode.

Direct HTTP examples:

```sh
curl -sS http://127.0.0.1:38383/status

curl -sS \
  -H 'content-type: application/json' \
  -d '{"command":"pose","pose":"raise_right_hand"}' \
  http://127.0.0.1:38383/command

curl -sS \
  -H 'content-type: application/json' \
  -d '{"command":"yoga_policy","action":"start","loop":true}' \
  http://127.0.0.1:38383/command
```

The Unitree assets are not vendored in this repo. The prepare script fetches
the official `unitreerobotics/unitree_mujoco` repository at a pinned commit
into `.runtime/unitree-g1-mujoco/unitree_mujoco` and mounts it read-only.

Probe the live Booster-style envelope from this repo with:

```sh
node script/probe-unitree-g1-mujoco-protocol.mjs --topic simulation_state
node script/probe-unitree-g1-mujoco-protocol.mjs --topic visual_frame
```

The embedded Cybernetic Robot Viewer consumes the HTTP status/frame endpoints
for responsiveness and keeps the WebSocket protocol available for parity with
the reversed Booster Studio simulator contract.
