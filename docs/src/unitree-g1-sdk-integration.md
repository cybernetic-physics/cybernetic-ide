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
- An opt-in SDK2 sidecar under `overlays/unitree-g1-sdk2-sidecar/` that mounts
  pinned official `unitree_sdk2_python`, `unitree_sdk2`, and `unitree_mujoco`
  checkouts, installs CycloneDDS, imports official Unitree HG IDL types,
  initializes the DDS domain, and creates probe `rt/lowcmd`/`rt/lowstate`
  channel objects.

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
    robot.safety_stop()
```

Or use the CLI:

```sh
cyber-g1 status
cyber-g1 raise-hand --snapshot .runtime/g1-control-demo/right-hand-up.jpg
cyber-g1 safety-check
cyber-g1 safety-stop
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
Agents can also persist exact camera angles with
`viewer_camera_bookmark_save`, inspect them with `viewer_camera_bookmark_list`,
restore them with `viewer_camera_bookmark_apply`, and clean them up with
`viewer_camera_bookmark_delete`. The bookmarks live at
`.runtime/robot-viewer-camera-bookmarks.json`, which makes repeatable
before/after screenshots possible without re-deriving orbit/pan/zoom deltas.

The same MCP server also exposes `unitree_prepare_sdk2_sidecar` and
`unitree_sdk2_sidecar_status`. These tools prepare pinned official Unitree SDK2
sources and run a diagnostic sidecar report. The report proves SDK2 Python
imports, CycloneDDS domain initialization, Unitree HG IDL imports, and
`rt/lowcmd`/`rt/lowstate` channel creation. The follow-up
`unitree_probe_official_mujoco_dds` tool now launches official
`unitree_mujoco` under Xvfb and proves that the same SDK2 Python stack can read
an `rt/lowstate` sample from the upstream G1 peer.

`unitree_official_mujoco_plan` exposes the next native gate directly to agents.
It reports whether the upstream C++ `simulate/build/unitree_mujoco` binary is
present, whether the G1 MJCF scene exists, whether the `simulate/mujoco`
symlink exists, and the launch command for the intended peer:

```sh
/opt/unitree_mujoco/simulate/build/unitree_mujoco -r g1 -s scene_29dof.xml -i 1 -n lo
```

The official C++ simulator is viewer-bound upstream. It starts the SDK2 bridge
thread internally and selects `G1Bridge` when the MuJoCo model has more than 20
actuators. A true headless Cybernetic runtime will need either an upstream patch
or a separate bridge process, so the current milestone is to first prove the
unmodified upstream peer can exchange `rt/lowstate` and `rt/lowcmd` samples.

`unitree_build_official_mujoco_peer` attempts the native build inside the
sidecar. The prep script downloads the pinned MuJoCo `3.3.6` Linux/aarch64
release into `.runtime/unitree-g1-sdk2/`, the sidecar installs native build
dependencies including Eigen for the SDK2 bridge headers, installs official
`unitree_sdk2` to `/opt/unitree_robotics`, links the MuJoCo release into
`simulate/mujoco`, and builds `simulate/build/unitree_mujoco`.

`unitree_probe_official_mujoco_launch` is the next gate after a successful
build. The first raw launch probe exposed two real runtime requirements: the
binary needs `LD_LIBRARY_PATH` pointed at Unitree SDK2's aarch64 DDS libraries
and MuJoCo's release libraries, and upstream `simulate` needs a display because
it initializes GLFW. The sidecar now installs Xvfb and runs a short
`xvfb-run` startup probe. Passing that probe only proves the official peer can
start headlessly.

`unitree_probe_official_mujoco_dds` is the next probe in that chain. It starts
the same upstream peer as a managed process, subscribes to `rt/lowstate` with
`unitree_sdk2py.idl.unitree_hg.msg.dds_.LowState_`, waits for a sample, and
returns a structured summary including motor count, `mode_machine`, IMU fields,
stdout/stderr tails, and whether MuJoCo plus the SDK2 bridge started. On the
current local runtime it has proven a 35-motor `LowState_` sample on DDS domain
`1` using interface `lo`. CycloneDDS still warns that `lo` is not
multicast-capable, so the warning is retained in the report.

`unitree_probe_official_mujoco_lowcmd` proves the corresponding write side
without intentionally moving the robot. It reads one official `LowState_`,
copies the current motor positions and `mode_machine` into an HG `LowCmd_`,
computes the official CRC, and publishes a short sequence of hold frames to
`rt/lowcmd`. The current verified result wrote 8 of 8 hold frames successfully
with a 35-motor `LowCmd_`. The next control milestone is promoting that writer
into a long-lived `cybernetic_robotics` DDS transport and then adding deliberate
arm-motion demos against the official peer.

`unitree_probe_official_mujoco_arm_motion` proves a first bounded motion
through the same official path. It reads initial `rt/lowstate`, builds a
CRC-valid `LowCmd_` that holds the sampled posture while applying a small target
to one arm joint, publishes the command sequence, then reads `rt/lowstate`
again and checks displacement. The MCP tool is now parameterized for the
official G1 arm joints, target delta, frame count, and PD gains so agents can
try bounded left/right arm motions without editing the sidecar. Verified local
runs include `right_shoulder_roll` from `0.0` to about `-0.289 rad` with a
`-0.25 rad` target and `left_elbow` from `0.0` to about `0.207 rad` with a
`0.18 rad` target over 120 successful `rt/lowcmd` writes, while keeping the
probe short-lived and explicitly simulator-only.

`unitree_probe_official_mujoco_arm_pose` is the first coordinated pose version
of that same path. It uses the official peer, publishes one bounded multi-joint
HG `LowCmd_`, then verifies per-joint movement through `rt/lowstate`. The
verified `raise_right_hand` preset moved five target joints
(`right_shoulder_pitch`, `right_shoulder_roll`, `right_shoulder_yaw`,
`right_elbow`, and `right_wrist_pitch`) with 180 of 180 successful
`rt/lowcmd` writes. This is still a diagnostic probe, but it is much closer to
the developer-facing "raise hand" SDK demo than isolated joint pokes.

The Python package now exposes that same official proof path through
`OfficialG1Sim`, `cyber-g1 official status`, and `cyber-g1 official
raise-hand`, so a developer can stay in the Cybernetic SDK instead of
hand-writing MCP or Docker Compose commands:

```python
from cybernetic_robotics import OfficialG1Sim

official = OfficialG1Sim.discover()
print(official.status()["ok"])
print(official.raise_right_hand()["moved_joints"])
```

`OfficialG1Sim.raise_right_hand()` still runs the short-lived sidecar probe.
For sustained sessions, `OfficialG1Sim.start_session()` starts the managed
`unitree-g1-sdk2-session` container, `session_status()` reads the Docker
inspect/log readiness state, `lowstate_session()` reads one official
`rt/lowstate` sample from the live peer, and
`raise_right_hand_session()` / `arm_pose_session()` command that already-running
official peer over SDK2/CycloneDDS without spawning a second MuJoCo process.
`OfficialG1Sim.stop_session()` removes the peer when the workflow is done.
Most Python scripts should use the context-managed wrapper so cleanup happens
automatically:

```python
with OfficialG1Sim.discover().session() as sim:
    print(sim.lowstate()["lowstate_summary"])
    print(sim.arm_pose("raise_right_hand")["moved_joints"])
    sim.arm_pose_evidence(output_path=".runtime/official-mujoco-evidence/latest.json")
```

```sh
cyber-g1 official start-session
cyber-g1 official session-status
cyber-g1 official lowstate-session
cyber-g1 official pose raise_right_hand --session
cyber-g1 official stop-session
python3 examples/g1_official_managed_session.py
```

The MCP server now also exposes the first managed official peer lifecycle:
`unitree_start_official_mujoco_session`,
`unitree_official_mujoco_session_status`, and
`unitree_stop_official_mujoco_session`. These tools run
`CYBER_UNITREE_ACTION=serve_official_mujoco` in a named Docker container
(`unitree-g1-sdk2-session`), parse its ready report from logs, and stop/remove
it cleanly. Agents can then call `unitree_read_official_mujoco_lowstate` to
read one official `rt/lowstate` sample from that sustained peer, or call
`unitree_command_official_mujoco_arm_pose` to send a bounded `raise_right_hand`
or custom arm pose to the live peer. In Python, setting
`CYBER_UNITREE_TRANSPORT=dds` routes
`G1ArmActionClient.ExecuteAction(action_map["right hand up"])` through the same
managed official session.

Runtime environment knobs:

- `CYBER_ROBOT_HARNESS_DIR`: repo root for the Docker harness.
- `CYBER_ROBOT_IMAGE`: simulator image, default
  `cyber/unitree-g1-mujoco-protocol:0.1.0`.
- `CYBER_ROBOT_MODEL_PATH`: mounted MJCF path, default
  `/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml`.
- `UNITREE_G1_LOWCMD_WATCHDOG_SECONDS`: simulator lowcmd freshness timeout,
  default `2.0`; status and lowstate telemetry report whether the most recent
  lowcmd is still active or has gone stale.
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
| Official SDK2 sidecar scaffold | `overlays/unitree-g1-sdk2-sidecar/` and `script/prepare-unitree-g1-sdk2-sidecar.mjs` |
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
  examples/g1_official_raise_hand.py \
  examples/g1_loco_sdk.py \
  examples/g1_wave_hand_sdk.py \
  examples/g1_walk_square_loco.py \
  examples/g1_lowcmd_sdk.py \
  examples/g1_joint_targets.py \
  examples/g1_scene_obstacle.py \
  examples/g1_safety_stop.py \
  examples/g1_agent_debug_loop.py \
  examples/g1_behavior_gallery.py \
  packages/cybernetic-robotics/src/cybernetic_robotics/*.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/core/channel.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/idl/default.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/idl/unitree_hg/msg/dds_.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/g1/arm/*.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/g1/loco/*.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/comm/motion_switcher/*.py \
  packages/cybernetic-robotics/src/unitree_sdk2py/utils/*.py \
  overlays/unitree-g1-mujoco-protocol/python/g1_protocol_sim.py \
  overlays/unitree-g1-mujoco-protocol/python/g1_policy_runtime.py \
  overlays/unitree-g1-sdk-shim/unitree_sdk2py/core/channel.py \
  overlays/unitree-g1-sdk-shim/unitree_sdk2py/g1/arm/g1_arm_action_api.py \
  overlays/unitree-g1-sdk-shim/unitree_sdk2py/g1/arm/g1_arm_action_client.py \
  packages/g1-yoga-rl/g1_yoga_rl/*.py

cargo test -p cyber_robot_viewer
node --check script/prepare-unitree-g1-mujoco-container.mjs
node --check script/prepare-unitree-g1-sdk2-sidecar.mjs
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

Cybernetic's current learned-policy research path lives in
`packages/g1-yoga-rl`. It now has a deploy gate for LocoMuJoCo-trained G1 yoga
policies: export a PPOJax agent to NumPy, evaluate it in LocoMuJoCo, pack a
29-DOF deploy bundle, validate observation parity against the training env, and
run local sim2sim through
`overlays/unitree-g1-mujoco-protocol/python/g1_policy_runtime.py`. That runtime
is also wired into the live Docker protocol server when
`UNITREE_G1_POLICY_BUNDLE` points at a packed bundle; `g1-yoga-sim2sim` remains
the pre-Docker proof harness for observation/action mapping. Agents can inspect
and control that optional runtime through `sim_policy_status`,
`sim_policy_start`, and `sim_policy_stop`.

## Key Findings {#key-findings}

Unitree SDK2 is the control plane. It uses CycloneDDS and exposes both
request/response services and typed topics. The SDK request/response services
use names like `rt/api/<service>/request` and `rt/api/<service>/response`.

For G1, the friendly first surface is high-level SDK control:

- `sport` / `LocoClient`: damping, zero torque, sit, stand, squat, high/low
  stand, balance, velocity movement, wave, shake hand, speed mode, and
  internal/user-control switching in the C++ client.
- `agv` / `AgvClient`: current G1 forward/yaw velocity and height-column
  commands. Unitree documents lateral `vy` as unsupported for this surface.
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
- `examples/g1_official_raise_hand.py` uses `OfficialG1Sim.raise_right_hand()`
  to run the official sidecar peer and verify the bounded multi-joint hand
  raise through real SDK2/CycloneDDS `rt/lowcmd` and `rt/lowstate`.
- `examples/g1_official_managed_session.py` starts the named official MuJoCo
  session, reads official `rt/lowstate`, commands an arm pose through the
  sustained `rt/lowcmd` peer, reads lowstate again, and stops the session unless
  `--keep-running` is passed.
- The simulator now maps Unitree's preset G1 arm actions to deterministic
  static poses for local development, including `high five`, `hands up`,
  `clap`, `hug`, `heart`, `face wave`, `high wave`, `shake hand`, kiss poses,
  `reject`, `x-ray`, and release.
- `examples/g1_loco_sdk.py` uses the Unitree-shaped `LocoClient` surface,
  including official-style FSM, balance, swing-height, and stand-height
  getters/setters. The shim also mirrors official C++ G1 methods such as
  `GetPhase`, `Squat`, `StandUp`, `ContinuousGait`, `SwitchMoveMode`,
  `SetSpeedMode`, `SwitchToUserCtrl`, and `SwitchToInternalCtrl`; in the local
  simulator these are stateful compatibility flags around the kinematic
  velocity path, not a whole-body balance controller.
- `examples/g1_agv_sdk.py` uses the newer Unitree-shaped
  `unitree_sdk2py.g1.agv.g1_agv_client.AgvClient` import path. `Move` is
  clamped to Unitree's documented AGV ranges and routes through the local
  kinematic velocity path; `HeightAdjust` is recorded as simulator intent until
  there is a modeled height-column actuator.
- `examples/g1_wave_hand_sdk.py` isolates the official-style
  `LocoClient.WaveHand()` call and writes before/after screenshots plus a
  manifest for agent review.
- `examples/g1_walk_square_loco.py` uses repeated `LocoClient.Move()` calls to
  trace a small square with final status, lowstate, snapshots, and a manifest.
- `examples/g1_lowstate_monitor.py` prints compact simulator-backed
  `rt/lowstate` and named-joint telemetry, with optional JSONL output for
  offline agent review.
- `examples/g1_scene_obstacle.py` demonstrates the scene-editing layer: it
  inserts a box obstacle into a copied MJCF, can activate that generated scene
  for the Docker harness, and records the generated host/container paths in a
  manifest for agents.
- `examples/g1_safety_stop.py` demonstrates the shared simulator stop path:
  release motion-switcher mode, damp locomotion, neutralize the arm pose, pause
  the simulator, and save an after-stop snapshot.
- `cyber-g1 safety-check`, `G1Robot.safety_check()`, and MCP
  `g1_safety_check` expose read-only Unitree G1-inspired termination checks
  from `common/terminations.hpp`: bad orientation, joint velocity, angular
  velocity, motor temperature, stale lowcmd, and fall state.
- `examples/g1_agent_debug_loop.py` runs one behavior, captures before/after
  screenshots, status, `rt/lowstate`, named joint state, and the safety-stop
  result into `.runtime/g1-agent-debug-loop/debug_bundle.json` for agent review.
- The shim exposes `unitree_sdk2py.g1.audio.g1_audio_client.AudioClient` for
  TTS, volume, LED, and stream method-shape compatibility. In MuJoCo this
  records intent only; real audio hardware still belongs to the future official
  SDK2/DDS provider.
- `examples/g1_lowcmd_sdk.py` uses Unitree-shaped `ChannelPublisher`,
  `ChannelSubscriber`, `LowCmd_`, `LowState_`, `CRC`, and
  `MotionSwitcherClient` imports. The motion switcher shim now supports
  `CheckMode`, `SelectMode`, `ReleaseMode`, `SetSilent`, and `GetSilent`;
  `ReleaseMode` clears the selected mode and puts the simulator into damp so
  official-style low-level setup scripts have a meaningful local equivalent.
- The channel shim also supports read-only `rt/sportmodestate` and
  `rt/wirelesscontroller` subscribers with lightweight `unitree_go` dataclasses,
  synthesized from local simulator status and lowstate.
- `examples/g1_joint_targets.py` demonstrates the named-joint layer that reads
  `/joint_state` and compiles joint-name targets back into simulator-backed
  lowcmd slots.
- `UnitreeSession.from_env().provider_status()`, `cyber-g1 provider`, and the
  `unitree_provider_status` MCP tool give agents and users the short answer for
  which backend is active: provider name, command path, telemetry path, motion
  surfaces, limitations, and next step.
- `cyber-g1 sdk-audit` and MCP `unitree_sdk_compatibility_audit` statically
  compare the cloned official Unitree G1 SDK2 Python examples with Cybernetic's
  current `unitree_sdk2py` shim. The current audit reports import/class/method
  coverage for all five official G1 examples under `example/g1/high_level/` and
  `example/g1/low_level/`; this is static compatibility, not proof of physical
  behavior parity.
- `cyber-g1 sdk-smoke` and MCP `unitree_sdk_behavior_smoke` run conservative
  behavior-level checks through the same official-style imports: arm action,
  locomotion method calls, and lowcmd/lowstate channel publish/read. This proves
  the local simulator facade responds to safe SDK-shaped calls, but it still
  does not prove whole-body balance or sim-to-real equivalence.
- `UnitreeSession.from_env().diagnostics()`, `cyber-g1 diagnostics`, and the
  `unitree_session_status` MCP tool expose the deeper transport boundary:
  `local_http` versus opt-in `dds`, sim/real mode, DDS domain/interface,
  simulator reachability, and topic freshness. With
  `CYBER_UNITREE_TRANSPORT=dds` in simulator mode, both Python diagnostics and
  the MCP tool now call the official sidecar status probe and report SDK2
  import, CycloneDDS domain, channel creation, and official MuJoCo peer
  readiness.
- `robotics_tool_reference` gives Agent-panel users a machine-readable safety
  map for the robotics tools: safety level, side effects, and expected
  simulator state.
- `unitree_sdk_compatibility_audit` gives Agent-panel users the same
  official-example compatibility report before porting an upstream script.
- `unitree_sdk_behavior_smoke` gives Agent-panel users a behavior-level smoke
  check for safe Unitree SDK-shaped calls after the simulator is running.
- `unitree_sdk_scaffold_python` now generates arm-action, locomotion,
  named-joint lowcmd, scene-edit, and telemetry-monitor scripts so agents can
  create editable Unitree-style starting points without guessing boilerplate.
- MCP now has managed official peer lifecycle tools for starting, inspecting,
  and stopping the `unitree-g1-sdk2-session` container that runs upstream
  `unitree_mujoco` under Xvfb as a sustained DDS peer.
- MCP now has `unitree_read_official_mujoco_lowstate` for reading one official
  `rt/lowstate` sample from that sustained peer without commanding motion.
- MCP now has `unitree_command_official_mujoco_arm_pose` for commanding that
  managed session, and the Python `G1ArmActionClient` routes `right hand up`
  through it when `CYBER_UNITREE_TRANSPORT=dds` is set in simulator mode.
- MCP now has `unitree_official_mujoco_evidence_bundle` for the agent-native
  version of that workflow: ensure the managed official peer is available, read
  official `rt/lowstate` before the pose, command a bounded arm pose over
  `rt/lowcmd`, read `rt/lowstate` again, and write a JSON evidence artifact
  under `.runtime/official-mujoco-evidence/`.
- In the current simulator backend, that action posts `{"command": "pose",
  "pose": "raise_right_hand"}` to the Dockerized G1 MuJoCo protocol harness.

This is intentionally a bootstrap bridge. It proves the invisible SDK-shaped
developer experience while keeping a clean path to replace the internals with
official `unitree_mujoco` + SDK2 DDS topics.

## First Milestone {#first-milestone}

Build the safe official-SDK runtime path before real hardware or learned
policies.

1. Partially done: add a `unitree-g1-runtime` image or sidecar with SDK2
   Python, CycloneDDS, and the official Unitree G1 examples available. The
   current sidecar prepares pinned official SDK2 Python/C++ and MuJoCo sources,
   builds the CycloneDDS Python binding, initializes a DDS domain, imports
   Unitree HG IDL, creates `rt/lowcmd`/`rt/lowstate` probe channels, launches
   official `unitree_mujoco`, proves real `rt/lowstate` sample reads, and
   proves safe CRC-valid `rt/lowcmd` hold-frame writes plus a bounded single
   arm-joint motion against that peer. Remaining work is turning the short-lived
   probes into the default long-lived DDS-backed session provider.
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
