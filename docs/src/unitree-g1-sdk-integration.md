# Unitree G1 SDK Integration

This note records the current gap between Cybernetic IDE and Booster Studio,
then defines the next robotics architecture goal: control a Unitree G1 through
the official Unitree SDK2 stack.

## Current State {#current-state}

Cybernetic IDE currently has a useful G1 MuJoCo viewer and a Dockerized
Booster-shaped protocol harness. It can launch a MuJoCo scene, poll simulator
status, render camera frames, and move the viewport camera.

It is not yet a robot control IDE. The current G1 path does not expose the
official Unitree SDK2 control surface, does not publish or subscribe Unitree
DDS topics, and does not let a user send G1 locomotion, posture, arm, hand, or
low-level joint commands.

The implemented features are:

- `cyber: open robot viewer`: open the embedded Robot Viewer in the active
  workspace pane.
- `cyber: open robot viewer beside`: open or move the Robot Viewer beside the
  active code pane.
- Automatic local runtime preparation through
  `script/prepare-unitree-g1-mujoco-container.mjs`.
- Docker image build for `cyber/unitree-g1-mujoco-protocol:0.1.0`.
- Docker Compose lifecycle for the `unitree-g1-mujoco` container.
- HTTP status, scene, camera, frame, and command endpoints on `38383`.
- Booster-like physics WebSocket subscriptions on `8788`.
- Interactive camera orbit, zoom, pan, reset, pause/resume, refresh, and step
  controls in the embedded viewer.
- Python examples for protocol probing and simulator control.
- A Unitree SDK2-shaped Python facade for the first G1 arm-action demo.
- `packages/cybernetic-robotics/`, an installable Python package with a
  beginner `G1Robot` API, `cyber-g1` CLI, raw simulator clients, MJCF scene
  helpers, and packaged `unitree_sdk2py` compatibility modules.

## End User Guide {#end-user-guide}

Build Cybernetic IDE:

```sh
cargo build -p zed
```

Prepare the pinned public Unitree MuJoCo G1 assets:

```sh
node script/prepare-unitree-g1-mujoco-container.mjs
```

Build the simulator protocol image:

```sh
docker build -t cyber/unitree-g1-mujoco-protocol:0.1.0 overlays/unitree-g1-mujoco-protocol
```

Start the simulator:

```sh
docker compose \
  --env-file .runtime/unitree-g1-mujoco/compose.env \
  -f overlays/unitree-g1-mujoco-container/compose.yaml \
  up -d
```

Open an example in Cybernetic IDE:

```sh
./target/debug/zed examples/control_g1_sim.py
```

Open the Command Palette and run `cyber: open robot viewer beside`. The viewer
will place the G1 MuJoCo viewport next to the active code pane.

Run the general simulator probe:

```sh
python3 examples/control_g1_sim.py --steps 20 --run-seconds 1.2
```

Install the beginner-friendly package:

```sh
python3 -m pip install -e packages/cybernetic-robotics
```

Then drive the robot with less boilerplate:

```python
from cybernetic_robotics import G1Robot

with G1Robot.connect() as robot:
    robot.raise_right_hand()
    robot.snapshot(".runtime/g1-control-demo/right-hand-up.jpg")
```

Or use the CLI:

```sh
cyber-g1 status
cyber-g1 raise-hand --snapshot .runtime/g1-control-demo/right-hand-up.jpg
```

Run the first Unitree-shaped SDK demo:

```sh
python3 examples/g1_raise_hand_sdk.py
```

The task picker also contains:

- `run Unitree G1 sim control demo`
- `raise Unitree G1 hand`
- `raise Unitree G1 hand via Unitree SDK facade`

## Robot Viewer Controls {#robot-viewer-controls}

The Robot Viewer renders cached MuJoCo camera frames and sends lightweight
camera commands separately from rendering. This keeps mouse interaction
responsive even when MuJoCo frame generation is slower than the UI.

| Control | Behavior |
| --- | --- |
| Drag | Orbit the MuJoCo free camera around the G1. |
| Scroll | Zoom the camera in or out. |
| Refresh button | Reconnect and reprobe the simulator. |
| Cube button | Reset the camera to the default G1 framing. |
| Play/Pause button | Resume or pause simulation stepping. |
| Crosshair button | Step or recenter viewer state depending on current state. |

Default endpoints:

- `http://127.0.0.1:38383/status`
- `http://127.0.0.1:38383/visual_scene`
- `http://127.0.0.1:38383/visual_frame`
- `http://127.0.0.1:38383/camera`
- `http://127.0.0.1:38383/camera_frame_0.jpg`
- `ws://127.0.0.1:8788`

Agent workflows use the same viewer protocol through the default
`cybernetic-robotics` MCP server. `viewer_snapshot` returns the current camera
frame inline, `viewer_snapshot_file` writes one frame to the workspace, and
`viewer_snapshot_series` captures a small evidence packet of named views such
as `current`, `front`, `right`, and `three_quarter` into
`.runtime/robot-viewer-snapshots/`. This is the preferred way for agents to
show what changed after a Python control script or scene edit.

Runtime environment knobs:

- `CYBER_ROBOT_HARNESS_DIR`: repo root for the Docker harness.
- `CYBER_ROBOT_IMAGE`: simulator image, default
  `cyber/unitree-g1-mujoco-protocol:0.1.0`.
- `CYBER_ROBOT_MODEL_PATH`: mounted MJCF path, default
  `/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml`.
- `CYBER_ROBOT_VIEWER_OPEN_ON_STARTUP=1`: auto-open Robot Viewer in debug
  sessions.
- `CYBER_G1_GAME_CONTROL_URL`: GameControl base URL for Python examples.
- `CYBER_G1_WS_HOST` and `CYBER_G1_WS_PORT`: WebSocket host/port for
  `examples/control_g1_sim.py`.

## Developer Guide {#developer-guide}

Keep robotics changes behind the Cybernetic extension boundaries:

| Area | Code |
| --- | --- |
| Workspace item and UI | `crates/cyber_robot_viewer/` |
| MuJoCo renderer/control service | `overlays/unitree-g1-mujoco-protocol/` |
| Container lifecycle | `overlays/unitree-g1-mujoco-container/` and `script/prepare-unitree-g1-mujoco-container.mjs` |
| Unitree-shaped Python facade | `overlays/unitree-g1-sdk-shim/` |
| Installable Python package | `packages/cybernetic-robotics/` |
| End-user demos | `examples/` |
| Long-form docs | `docs/src/unitree-g1-sdk-integration.md` |

Focused validation:

```sh
python3 -m py_compile \
  examples/easy_g1_playground.py \
  examples/control_g1_sim.py \
  examples/g1_raise_hand_sdk.py \
  examples/g1_loco_sdk.py \
  examples/g1_lowcmd_sdk.py \
  examples/g1_joint_targets.py \
  packages/cybernetic-robotics/src/cybernetic_robotics/*.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/core/channel.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/idl/default.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/idl/unitree_hg/msg/dds_.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/g1/arm/*.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/g1/loco/*.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/comm/motion_switcher/*.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/utils/*.py \
  overlays/unitree-g1-mujoco-protocol/python/g1_protocol_sim.py \
  overlays/unitree-g1-sdk-shim/unitree_sdk2py/core/channel.py \
  overlays/unitree-g1-sdk-shim/unitree_sdk2py/g1/arm/g1_arm_action_api.py \
  overlays/unitree-g1-sdk-shim/unitree_sdk2py/g1/arm/g1_arm_action_client.py

cargo test -p cyber_robot_viewer
```

Manual runtime validation:

```sh
docker compose \
  --env-file .runtime/unitree-g1-mujoco/compose.env \
  -f overlays/unitree-g1-mujoco-container/compose.yaml \
  up -d --force-recreate unitree-g1-mujoco

node script/probe-unitree-g1-mujoco-protocol.mjs --topic simulation_state
python3 examples/g1_raise_hand_sdk.py
python3 examples/g1_loco_sdk.py
python3 examples/g1_lowcmd_sdk.py
python3 examples/g1_joint_targets.py
```

## Missing Booster Studio Features {#missing-booster-studio-features}

These are the largest missing feature groups compared with the real Booster
Studio product and the reverse-engineering docs in `dasm`.

| Priority | Missing feature                    | Why it matters                                                                                                                                                                                                                                   |
| -------- | ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 1        | Real robot SDK driver path         | Booster has native robot APIs and a driver-shaped control layer. Cybernetic needs the Unitree equivalent before it can control hardware.                                                                                                         |
| 2        | Robot control panels               | Booster has Teleop, Gauge, Table, Parameters, CallService, PieChart, PlaybackPerformance, SourceInfo, Tab, and TopicGraph-style panels. Cybernetic only has the viewer.                                                                          |
| 3        | Simulator control parity           | Booster exposes Docker lifecycle, physics WebSocket, camera relay, GameControl, scene switching, robot spawn/despawn, mode changes, reset, pause, speed, and body transforms. Cybernetic supports only a small camera/status/sim command subset. |
| 4        | Telemetry and diagnostics          | Booster surfaces robot status, process health, logs, callbacks, and service state. Cybernetic telemetry is limited to model path, pause/speed, robot labels, and visual frame summaries.                                                         |
| 5        | Robot inventory and profiles       | Booster has robot discovery, saved robots, activation, reconnect, shell channels, service restart, and log retrieval. Cybernetic is configured around one env-driven G1 harness.                                                                 |
| 6        | SDK and agent development workflow | Booster has sample-code, agent build/run/deploy, terminal, shell, and runtime workflows. Cybernetic has no G1 SDK project workflow yet.                                                                                                          |
| 7        | Real robotics runtime transport    | Booster's container runs a robotics runtime behind the IDE-facing simulator protocol. Cybernetic currently runs a pure Python MuJoCo/WebSocket renderer with no SDK2/DDS actuator path.                                                          |

The useful thing to copy from Booster is not its private native addon,
proprietary robot transport, or closed gait binaries. The useful thing is the
product shape: a stable IDE driver boundary, lifecycle management, telemetry,
panels, and a simulator that uses the same command path as the physical robot.

## Unitree Repos Reviewed {#unitree-repos-reviewed}

The official Unitree sources cloned under `/Users/cuboniks/wagmi` define the
G1 integration surface.

| Repo                  | Role                                                                                                                                                           |
| --------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `unitree_sdk2`        | Official C++ SDK2. G1 locomotion, arm action, DDS types, low-level examples, and CMake integration.                                                            |
| `unitree_sdk2_python` | Official Python SDK2 wrapper. Fastest route to high-level G1 actions, telemetry subscriptions, and prototype runtime services.                                 |
| `unitree_mujoco`      | Official MuJoCo simulator. It presents MuJoCo as a Unitree SDK2/DDS peer using `rt/lowcmd`, `rt/lowstate`, `rt/sportmodestate`, BMS, wireless, and IMU topics. |
| `unitree_ros2`        | ROS2 message and example surface. Useful for future ROS workflows, but not the first integration layer.                                                        |
| `unitree_rl_gym`      | RL training, MuJoCo sim2sim, and sim2real deployment examples. Useful for policy controllers, but its MuJoCo runner bypasses DDS.                              |
| `unitree_rl_lab`      | Isaac Lab training and G1 policy deployment examples. Useful later for behavior and policy workflows.                                                          |
| `unitree_rl_mjlab`    | MuJoCo-native RL training and deployment examples. Useful later for policy workflows.                                                                          |
| `unitree_IL_lerobot`  | LeRobot data conversion, dataset editing, G1/Dex hand evaluation, and imitation-learning tooling. Useful after base SDK control exists.                        |

## Key Findings {#key-findings}

Unitree SDK2 is the control plane. It uses CycloneDDS and exposes both
request/response services and typed topics. The SDK request/response services
use names like `rt/api/<service>/request` and `rt/api/<service>/response`.

For G1, the friendly first surface is high-level SDK control:

- `sport` / `LocoClient`: damping, zero torque, sit, stand, squat, high/low
  stand, balance, velocity movement, wave, shake hand, speed mode, and
  internal/user-control switching in the C++ client.
- `arm` / `G1ArmActionClient`: preset and custom arm actions, with a release
  action for actions that hold.
- `rt/lowstate`: motor state, IMU, mode fields, wireless state, and other low
  level telemetry.
- `rt/lowcmd`: expert-only motor commands with `mode_machine`, `mode_pr`,
  joint commands, gains, torque feedforward, and CRC.
- `rt/arm_sdk`, `rt/hand_sdk`, and Dex topics: arm and hand control surfaces
  for later milestones.

The official `unitree_mujoco` simulator is the right sim-to-real foundation.
It is not just an asset viewer. It runs MuJoCo behind the same SDK2/DDS topic
shape used by the real robot:

- Simulation: DDS domain `1`, interface `lo`, Cybernetic-managed simulator
  process.
- Physical G1: DDS domain `0`, user-selected robot network interface, no
  simulator process.

The main Cybernetic API should stay the same above that transport switch.
Whether the peer is MuJoCo or a real G1 should be a session configuration
detail.

## Recommended Architecture {#recommended-architecture}

Build a `UnitreeG1Provider` runtime sidecar and make the IDE talk to that
runtime, not directly to DDS from UI code. User code should keep the official
Unitree SDK2 Python shape:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map

ChannelFactoryInitialize(0, "cyber-sim")

arm = G1ArmActionClient()
arm.SetTimeout(10.0)
arm.Init()
arm.ExecuteAction(action_map["right hand up"])
```

In sim mode, Cybernetic can put its SDK compatibility overlay on `PYTHONPATH`
and route that call to the local simulator runtime. In real-robot mode, the
same import path should resolve to the official Unitree SDK2 package and talk
to CycloneDDS on the selected robot network interface.

```text
User Python code and Cybernetic IDE panels
  |
  | Unitree SDK2-shaped API plus local JSON/RPC or WebSocket API
  v
UnitreeG1Provider runtime
  |-- session lifecycle and selected network interface
  |-- SDK2 Python worker for high-level actions and telemetry
  |-- C++ daemon for low-level 2 ms control loops
  |-- safety supervisor and command watchdog
  |-- telemetry/log stream
  |
  | sim mode:  DDS domain 1 on lo
  | real mode: DDS domain 0 on selected robot NIC
  v
official Unitree SDK2 / CycloneDDS
  |
  | sim peer: official unitree_mujoco
  | real peer: physical Unitree G1
```

The first implementation should use the Python SDK2 worker because it is the
fastest path to real G1 connection, high-level commands, and telemetry. The
C++ SDK2 daemon should own low-level joint loops, policy deployment, arm SDK
streaming, hand control, CRC handling, and watchdog behavior.

The current Booster-style WebSocket viewer should remain the visual layer, but
it should stop being the control source of truth. For official G1 simulation,
the provider should launch `unitree_mujoco` and then control it over DDS, just
as it would control a real G1.

## Bootstrap Implemented {#bootstrap-implemented}

The current repo has the first narrow version of that API boundary:

- `overlays/unitree-g1-sdk-shim/` provides a local `unitree_sdk2py` compatibility
  package with `ChannelFactoryInitialize`, `G1ArmActionClient`, and Unitree's
  G1 arm `action_map`.
- `packages/cybernetic-robotics/` packages that same beginner experience into
  an installable Python project with `G1Robot`, `SimulatorClient`,
  `TinyWebSocket`, `SceneWorkspace`, `cyber-g1`, and simulator-backed
  `unitree_sdk2py` modules for high-level arm, locomotion, G1 audio intent, and
  low-level `rt/lowcmd` / `rt/lowstate` channel examples.
- `examples/g1_raise_hand_sdk.py` uses the Unitree-shaped imports and calls
  `ExecuteAction(action_map["right hand up"])`.
- `examples/g1_loco_sdk.py` uses the Unitree-shaped `LocoClient` surface.
- The shim exposes `unitree_sdk2py.g1.audio.g1_audio_client.AudioClient` for
  TTS, volume, LED, and stream method-shape compatibility. In MuJoCo this
  records intent only; real audio hardware still belongs to the future official
  SDK2/DDS provider.
- `examples/g1_lowcmd_sdk.py` uses Unitree-shaped `ChannelPublisher`,
  `ChannelSubscriber`, `LowCmd_`, `LowState_`, `CRC`, and
  `MotionSwitcherClient` imports.
- `examples/g1_joint_targets.py` demonstrates the named-joint layer that reads
  `/joint_state` and compiles joint-name targets back into simulator-backed
  lowcmd slots.
- In the current simulator backend, that action posts `{"command": "pose",
  "pose": "raise_right_hand"}` to the Dockerized G1 MuJoCo protocol harness.

This is intentionally a bootstrap bridge. It proves the invisible SDK-shaped
developer experience while keeping a clean path to replace the internals with
official `unitree_mujoco` + SDK2 DDS topics.

## First Milestone {#first-milestone}

Build the safe official-SDK runtime path before real hardware or learned
policies.

1. Add a `unitree-g1-runtime` image or sidecar with SDK2 Python, CycloneDDS,
   and the official Unitree G1 examples available.
2. Add session config for `mode=sim|real`, DDS domain, network interface, G1
   model variant, and safety profile.
3. Replace the current local HTTP approximation for `rt/lowcmd` and
   `rt/lowstate` with a CycloneDDS-compatible bridge.
4. Implement `connect`, `disconnect`, `getStatus`, `streamTelemetry`,
   `damp`, `zeroTorque`, `sit`, `stand`, `stopMove`, `moveVelocity`,
   `setStandHeight`, `balanceStand`, and `executeArmAction`.
5. In sim mode, launch official `unitree_mujoco` with `-r g1`, the selected
   G1 scene, domain `1`, and interface `lo`.
6. In real mode, connect SDK2 to domain `0` on the selected physical network
   interface.
7. Add Cybernetic panels for connection, status, safety stop, teleop,
   telemetry, and logs.
8. Route viewer telemetry and control state through the provider so the same
   panels work for sim and hardware.

## Safety Gates {#safety-gates}

Physical robot control must be explicit and gated. The runtime should require
separate unlocks for high-level actions, arm actions, and low-level commands.

Minimum gates:

- Confirm the user selected `real` mode and a concrete network interface.
- Confirm fresh `rt/lowstate` before enabling commands.
- Show current FSM, `mode_machine`, `mode_pr`, battery, IMU orientation, and
  command watchdog state before sending motion.
- Clamp velocity, stand height, arm action selection, joint targets, gains,
  and torque feedforward.
- Provide a visible stop path that sends `StopMove` or damping immediately.
- Require an expert unlock before releasing motion services or publishing
  `rt/lowcmd`.
- Disable invalid joints based on G1 variant. The official docs call out
  differences between 23DOF, 29DOF, and hand variants.
- Treat RL and low-level policy execution as a later milestone that must pass
  simulation first.

## Goal {#goal}

Cybernetic IDE should support a Unitree G1 through an official SDK2 provider
that works against both the official Unitree MuJoCo simulator and a physical
G1. The IDE should expose safe high-level control first, then add low-level
joint and policy control behind stricter runtime gates.
