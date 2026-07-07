# Cybernetic Robotics Python

The `cybernetic-robotics` Python package is the easiest way to control the
local Unitree G1 MuJoCo simulator from Cybernetic IDE.

It replaces repetitive demo-script boilerplate with a small layered API:

- `G1Robot` for first-time users who want to make the robot move.
- `SimulatorClient` for direct GameControl HTTP calls.
- `TinyWebSocket` for the Booster-style physics WebSocket.
- `SceneWorkspace` for safe MJCF scene-copy edits.
- `unitree_sdk2py` compatibility modules for Unitree SDK2-shaped examples.
- `cyber-g1` for terminal-driven status, pose, camera, snapshot, and lifecycle
  commands.

The package has no required third-party Python dependencies. It talks to the
Dockerized simulator that Cybernetic IDE already runs.

## Install

From the Cybernetic IDE repo root:

```sh
python3 -m pip install -e packages/cybernetic-robotics
```

Prepare and start the G1 simulator:

```sh
node script/prepare-unitree-g1-mujoco-container.mjs
docker build -t cyber/unitree-g1-mujoco-protocol:0.1.0 overlays/unitree-g1-mujoco-protocol
docker compose \
  --env-file .runtime/unitree-g1-mujoco/compose.env \
  -f overlays/unitree-g1-mujoco-container/compose.yaml \
  up -d
```

## First Script

Create a file like this:

```python
from cybernetic_robotics import G1Robot

with G1Robot.connect() as robot:
    robot.reset()
    robot.raise_right_hand()
    robot.snapshot(".runtime/g1-control-demo/right-hand-up.jpg")
    robot.safety_stop()
    print(robot.status().pose)
```

Run it:

```sh
python3 my_g1_script.py
```

The context manager pauses the simulator when the block exits. That keeps
beginner experiments from leaving a control script running by accident.

The repo also includes a ready-to-run version that exercises both the beginner
API and the Unitree SDK-shaped shim:

```sh
python3 examples/use_cybernetic_robotics_lib.py
python3 examples/use_cybernetic_robotics_lib.py --mode unitree
python3 examples/g1_official_raise_hand.py
python3 examples/g1_official_managed_session.py
python3 examples/g1_loco_sdk.py
python3 examples/g1_wave_hand_sdk.py
python3 examples/g1_walk_square_loco.py
python3 examples/g1_lowcmd_sdk.py
python3 examples/g1_lowstate_monitor.py --samples 3
python3 examples/g1_joint_targets.py
python3 examples/g1_scene_obstacle.py --activate
python3 examples/g1_safety_stop.py
python3 examples/g1_agent_debug_loop.py --behavior raise_hand
```

## CLI

The package installs `cyber-g1`:

```sh
cyber-g1 status
cyber-g1 raise-hand --snapshot .runtime/g1-control-demo/right-hand-up.jpg
cyber-g1 safety-stop
cyber-g1 official status
cyber-g1 official raise-hand
cyber-g1 camera orbit --dx 40 --dy -10
cyber-g1 step --count 20
cyber-g1 demo
```

Harness helpers are available too:

```sh
cyber-g1 prepare
cyber-g1 start
cyber-g1 logs --tail 80
cyber-g1 stop
```

These commands assume `CYBER_ROBOTICS_ROOT` points at the Cybernetic IDE repo,
or that the current working directory is inside the repo.

## Official Unitree MuJoCo Probe

The beginner API above talks to Cybernetic's lightweight local viewer harness.
For the official Unitree MuJoCo + SDK2/CycloneDDS path, use `OfficialG1Sim`:

```python
from cybernetic_robotics import OfficialG1Sim

official = OfficialG1Sim.discover()
result = official.raise_right_hand()
print(result["ok"], result["moved_joints"])
```

`official.status()` and `cyber-g1 official status` are read-only checks for
SDK2 imports, CycloneDDS domain initialization, channel creation, source
revisions, and the official MuJoCo peer plan. `raise_right_hand()` and
`cyber-g1 official raise-hand` launch the official peer, publish a bounded
multi-joint HG `LowCmd_` pose over `rt/lowcmd`, and verify moved joints through
official `rt/lowstate`. These calls are simulator-only and short-lived by
design.

For the long-running official simulator peer, Python can now manage the same
named session as the Agent-panel MCP:

```python
official = OfficialG1Sim.discover()

with official.session() as sim:
    print(sim.lowstate()["lowstate_summary"])
    print(sim.loco_rpc()["probe"])
    print(sim.raise_right_hand()["moved_joints"])
    sim.arm_pose_evidence(output_path=".runtime/official-mujoco-evidence/latest.json")
```

The matching CLI flow is:

```sh
cyber-g1 official start-session
cyber-g1 official lowstate-session
cyber-g1 official raise-hand --session
cyber-g1 official stop-session
```

`examples/g1_official_managed_session.py` wraps that whole lifecycle, writes
the evidence bundle, and leaves the session running only when `--keep-running`
is passed. If a script needs
manual control instead of the context manager, use `official.start_session()`,
`official.session_status()`, `official.lowstate_session()`, and
`official.rpc_discovery_session()`, `official.loco_rpc_session()`, and
`official.stop_session()`.
`official.rpc_discovery_session()` is a read-only DDS preflight that reports
whether official Unitree RPC request topics such as `rt/api/sport/request` and
`rt/api/agv/request` have matched service-side readers before any RPC command
is sent.
`official.rpc_bridge_smoke()` starts temporary Unitree-shaped `sport`, `agv`,
and `arm` RPC servers inside the SDK2 sidecar and calls them with SDK clients.
This does not command hardware; it proves the server/client bridge shape needed
for a future long-running service bridge.
`official.start_rpc_bridge()` promotes that shape into a named
`unitree-g1-rpc-bridge` container, while `official.rpc_bridge_client()` calls
the running bridge with official SDK clients and `official.stop_rpc_bridge()`
removes it. The managed bridge keeps `sport`/`agv`/`arm` state and now forwards
safe read/write RPCs to the local simulator provider when
`CYBER_SIMULATOR_GAME_CONTROL_URL` is reachable. Getter RPCs such as
`sport.GetFsmId`, `sport.GetFsmMode`, `sport.GetBalanceMode`,
`sport.GetSwingHeight`, and `sport.GetStandHeight` read back from the simulator
and include `simulator_readback` evidence in raw debug responses. Setter RPCs
currently forwarded include `sport.SetFsmId`, `sport.SetBalanceMode`,
`sport.SetSwingHeight`, `sport.SetStandHeight`, `sport.SetVelocity`,
`sport.SetTaskId`, `arm.ExecuteAction`, and `agv.Move`. `agv.HeightAdjust` is
accepted for SDK compatibility but reported as `bridge_state_only` until the
local simulator has a modeled height-column actuator. That covers common
`LocoClient` shortcuts such as `Damp`, `StopMove`, `WaveHand`, and
`ShakeHand`, plus normal `G1ArmActionClient.ExecuteAction(...)`; unreachable
or unsupported simulator calls are reported as `bridge_state_only` in the RPC
JSON response instead of being hidden.
`official.verify_rpc_bridge()` is the preferred Python evidence check for this
managed bridge: it can start the bridge if needed, call the official SDK
clients, and return a compact summary with call counts, `RPC_OK` counts,
simulator forward evidence, simulator readback evidence, and any
`bridge_state_only` fallbacks.
Use `official.rpc_bridge_command()` when an agent or script wants one explicit
SDK-shaped action instead of the full verifier sequence:

```python
official = OfficialG1Sim.discover()
print(official.rpc_bridge_command(
    service="sport",
    method="move",
    params={"vx": 0.05, "vy": 0.0, "omega": 0.0, "duration": 0.5},
    start_if_needed=True,
)["summary"])
print(official.rpc_bridge_command(service="sport", method="get_fsm_id")["summary"])
print(official.rpc_bridge_command(service="agv", method="height_adjust", params={"vz": 0.0})["summary"])
print(official.rpc_bridge_command(service="arm", method="execute_action", params={"action_id": 23})["summary"])
```

The Agent-panel MCP mirror is `unitree_command_rpc_bridge`; it accepts the same
`service`, `method`, `params`, `timeout_seconds`, `start_if_needed`, and
`stop_after` fields. Supported aliases include `get_fsm_id`, `move`,
`stop_move`, `damp`, `stand_up`, `set_stand_height`, `set_swing_height`,
`set_balance_mode`, `wave_hand`, `shake_hand`, `agv.height_adjust`,
`arm.execute_action`, and `arm.get_action_list`.
`official.loco_rpc_session()` probes whether the managed official peer answers
G1 `LocoClient` sport RPC calls on `rt/api/sport/request` and
`rt/api/sport/response`; use that evidence before promoting local locomotion
facade calls to official DDS routing. Session status includes
`ready_report`, `last_report`, `exit_report`, and
`lifecycle_reports_seen`, so scripts and agents can distinguish a live ready
peer from a peer that printed a startup report and later exited. Current live
evidence is conservative: the official `LocoClient` initializes, but
`GetFsmId` can return `[3102, null]` after the configured timeout while
CycloneDDS reports loopback UDP write failures, so locomotion remains on the
explicit local compatibility route until the sport RPC peer is proven. With
`CYBER_UNITREE_TRANSPORT=dds`, `G1ArmActionClient.ExecuteAction()` routes
`right hand up` through that managed official session.
With `CYBER_UNITREE_TRANSPORT=rpc_bridge`, high-level `LocoClient`,
`AgvClient`, and `G1ArmActionClient` methods use the managed
`unitree-g1-rpc-bridge` service instead of direct local HTTP. That means normal
Unitree-shaped calls such as `loco.Move(...)`, `loco.GetFsmId()`,
`loco.WaveHand()`, `agv.Move(...)`, `agv.HeightAdjust(...)`, and
`arm.ExecuteAction(action_map["right hand up"])` cross the same SDK2-shaped
RPC boundary as the MCP `unitree_command_rpc_bridge` tool, while still
exposing simulator forward/readback evidence in `last_response`.

## Unitree SDK2-Shaped Code

Installing `cybernetic-robotics` also exposes a simulator-backed subset of
`unitree_sdk2py`:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map

ChannelFactoryInitialize(0, "cyber-sim")

arm = G1ArmActionClient()
arm.SetTimeout(10.0)
arm.Init()
arm.ExecuteAction(action_map["right hand up"])
```

For the local HTTP transport, that call maps to the lightweight Cybernetic
viewer harness. With `CYBER_UNITREE_TRANSPORT=dds` in simulator mode, it uses
the managed official MuJoCo + SDK2/CycloneDDS session, so the user script keeps
the Unitree SDK shape while the backend talks to `rt/lowcmd` and verifies
motion from `rt/lowstate`.

For high-level locomotion and AGV work, opt into the managed RPC bridge:

```sh
CYBER_UNITREE_TRANSPORT=rpc_bridge python3 examples/g1_loco_sdk.py
```

In that mode the shim keeps the same `LocoClient`/`AgvClient` method names, but
the call path is `unitree_sdk2py` facade -> `UnitreeSession` ->
`OfficialG1Sim.rpc_bridge_command()` -> managed SDK2 `sport`/`agv`/`arm` bridge ->
local MuJoCo simulator provider. It is still simulator-only and still a mapped
subset, but it is much closer to the official Unitree request/response shape
than direct local HTTP.

Use `UnitreeSession.from_env().provider_status()` or `cyber-g1 provider` when
you need the short provider answer: active backend, command path, telemetry
path, implemented motion surfaces, limitations, and next step. Use
`UnitreeSession.from_env().diagnostics()` or `cyber-g1 diagnostics` when you
need the deeper transport, topic, and simulator health report.

The compatibility package currently implements high-level arm actions,
locomotion actions, G1 audio intent, and the low-level channels needed by the
local simulator:

| Action name | Action ID | Simulator pose |
| --- | ---: | --- |
| `two-hand kiss` | `11` | `two_hand_kiss` |
| `left kiss` | `12` | `left_kiss` |
| `right kiss` | `13` | `right_kiss` |
| `hands up` | `15` | `hands_up` |
| `clap` | `17` | `clap` |
| `high five` | `18` | `high_five` |
| `hug` | `19` | `hug` |
| `heart` | `20` | `heart` |
| `right heart` | `21` | `right_heart` |
| `reject` | `22` | `reject` |
| `right hand up` | `23` | `raise_right_hand` |
| `x-ray` | `24` | `x_ray` |
| `face wave` | `25` | `face_wave` |
| `high wave` | `26` | `high_wave` |
| `shake hand` | `27` | `shake_hand` |
| `release arm` | `99` | `neutral` |

These simulator actions are deterministic static MuJoCo poses, not Unitree's
full arm-action controller. They intentionally keep the official SDK2 method
shape while making the shipped G1 arm-action examples useful in local sim.

## Unitree G1 LocoClient-Shaped Code

Unitree's official Python SDK exposes G1 locomotion through
`unitree_sdk2py.g1.loco.g1_loco_client.LocoClient`. Cybernetic mirrors the
method names against the local simulator:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

ChannelFactoryInitialize(0, "cyber-sim")

loco = LocoClient()
loco.SetTimeout(10.0)
loco.Init()

loco.Start()
print(loco.GetFsmId())
print(loco.GetFsmMode())
loco.SetSwingHeight(0.08)
print(loco.GetSwingHeight())
loco.SwitchMoveMode(True)
loco.Move(0.25, 0.0, 0.0)
loco.StopMove()
loco.WaveHand()
```

Supported methods include `GetFsmId`, `GetFsmMode`, `GetBalanceMode`,
`GetSwingHeight`, `GetStandHeight`, `SetFsmId`, `SetBalanceMode`,
`GetPhase`, `SetSwingHeight`, `SetStandHeight`, `Damp`, `Start`, `Squat`,
`Sit`, `StandUp`, `ZeroTorque`, `Move`, `StopMove`, `LowStand`, `HighStand`,
`BalanceStand`, `ContinuousGait`, `SwitchMoveMode`, `SetSpeedMode`,
`SwitchToUserCtrl`, `SwitchToInternalCtrl`, `WaveHand`, and `ShakeHand`.
`Move` is currently simulated with simple kinematic base motion; continuous
move, speed mode, and control-owner calls are recorded as simulator state flags
for SDK compatibility, not as Unitree's full whole-body balance controller.

Two focused examples are useful when teaching agents or new users the
locomotion surface:

```sh
python3 examples/g1_wave_hand_sdk.py
python3 examples/g1_walk_square_loco.py
```

Both scripts call the Unitree-shaped `LocoClient`, write screenshots, and save
a JSON manifest under `.runtime/` so an AI agent can inspect return codes,
status, lowstate, and visual evidence after the behavior runs.

## Unitree G1 AgvClient-Shaped Code

The upstream C++ G1 SDK also exposes `unitree::robot::g1::AgvClient` for
AGV-style forward/yaw movement and height-column velocity. Cybernetic exposes a
matching Python import path for code that is following the current G1 SDK
headers:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.agv.g1_agv_client import AgvClient

ChannelFactoryInitialize(0, "cyber-sim")

agv = AgvClient()
agv.SetTimeout(10.0)
agv.Init()
agv.Move(0.3, 0.0, 0.2)
agv.HeightAdjust(0.25)
```

`Move` clamps `vx` to `[-1.5, 1.5]` m/s and `vyaw` to `[-0.6, 0.6]` rad/s,
matching Unitree's documented AGV limits. The `vy` argument is accepted because
the official method includes it, but the AGV surface documents lateral motion
as unsupported, so Cybernetic records and ignores it. `HeightAdjust` records a
clamped `[-1.0, 1.0]` simulator intent; the local G1 MuJoCo harness does not
yet have a physical height-column actuator.

## Unitree G1 AudioClient-Shaped Code

Unitree's official G1 SDK also exposes
`unitree_sdk2py.g1.audio.g1_audio_client.AudioClient`. Cybernetic mirrors the
method names for code compatibility:

```python
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

audio = AudioClient()
audio.Init()
audio.TtsMaker("hello from the simulator", 0)
audio.SetVolume(40)
audio.LedControl(0, 80, 255)
```

The MuJoCo simulator has no audio or LED hardware, so this records intent only.
It keeps examples import-compatible until the provider can route the same calls
through Unitree's official SDK2/DDS stack.

## Low-Level Unitree Channels

For developers adapting Unitree's official low-level examples, the package also
implements the core `rt/lowcmd` and `rt/lowstate` channel shape:

```python
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

ChannelFactoryInitialize(0, "cyber-sim")

motion = MotionSwitcherClient()
motion.SetTimeout(5.0)
motion.Init()
motion.ReleaseMode()

lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
lowstate_sub.Init()
low_state = lowstate_sub.Read()

low_cmd = unitree_hg_msg_dds__LowCmd_()
low_cmd.mode_machine = low_state.mode_machine
low_cmd.motor_cmd[22].mode = 1
low_cmd.motor_cmd[22].q = -1.0
low_cmd.motor_cmd[22].kp = 30.0
low_cmd.motor_cmd[22].kd = 1.0
low_cmd.crc = CRC().Crc(low_cmd)

lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
lowcmd_pub.Init()
lowcmd_pub.Write(low_cmd)
```

`ChannelSubscriber("rt/lowstate", LowState_)` reads synthesized MuJoCo motor
position, velocity, estimated torque, IMU quaternion, and mode-machine fields
from the local simulator. `ChannelPublisher("rt/lowcmd", LowCmd_)` applies
commanded joint position targets into a held MuJoCo frame and records torque
estimates for telemetry. The simulator records `mode_pr`, `mode_machine`, CRC,
accepted command count, applied position target count, clamped joint targets,
and ignored motor slots in the `lowcmd` field of `/status` and `/lowstate`.
The bridge intentionally preserves Unitree method names and import paths, but
it is still simulator-only and does not replace Unitree's full CycloneDDS
transport or a real whole-body balance controller.

Transport selection lives in `UnitreeSession`. The high-level
`G1ArmActionClient` delegates `ExecuteAction()` to
`UnitreeSession.execute_arm_action()`: by default that uses the local HTTP
simulator, while `CYBER_UNITREE_TRANSPORT=dds` in simulator mode routes the
currently supported bounded hand-raise poses to the managed official Unitree
MuJoCo + SDK2/CycloneDDS session. `LocoClient` and `AgvClient` delegate to
`UnitreeSession.execute_loco_command()` / `execute_agv_command()`: by default
they use local HTTP, while `CYBER_UNITREE_TRANSPORT=rpc_bridge` routes the
supported high-level sport/agv/arm subset through the managed Unitree RPC
bridge.
`ChannelPublisher("rt/lowcmd")` and `ChannelSubscriber("rt/lowstate")` also
cross the same session boundary. Until generic lowcmd streaming is promoted
into the managed official provider, DDS-mode locomotion and lowcmd calls are
clearly marked as local simulator compatibility fallbacks instead of being
presented as official CycloneDDS control.

Run the full example:

```sh
python3 examples/g1_lowcmd_sdk.py
```

## Named Joint Targets

Most developers should start with named joints before writing raw motor-index
commands. The simulator exposes `/joint_state`, which maps Unitree G1 joint
names to motor indices, ranges, positions, velocities, and torque estimates.

```python
from cybernetic_robotics import G1Robot

with G1Robot.connect() as robot:
    state = robot.joint_state()
    print(state["by_name"]["right_elbow_joint"])

    robot.apply_joint_targets(
        {
            "right_shoulder_pitch_joint": -1.45,
            "right_elbow_joint": 0.95,
        },
        kp=34.0,
        kd=1.2,
    )
```

This still routes through the simulator-backed lowcmd path, but it avoids
hard-coded motor indices in beginner and agent-authored scripts. Run:

```sh
python3 examples/g1_joint_targets.py
```

Low-level commands also carry freshness metadata. The simulator marks the last
lowcmd stale after `UNITREE_G1_LOWCMD_WATCHDOG_SECONDS` seconds, default `2.0`.
Beginner code can read `robot.status().lowcmd_active`,
`robot.status().lowcmd_stale`, and `robot.status().lowcmd_age_seconds`; SDK2
style code can read the same values from `LowState_`.

## Official Example Compatibility Audit

Use `cyber-g1 sdk-audit` to compare the cloned official Unitree G1 SDK2 Python
examples against Cybernetic's current `unitree_sdk2py` shim:

```sh
cyber-g1 sdk-audit --upstream-root /Users/cuboniks/wagmi/unitree_sdk2_python
```

The audit statically checks official G1 example imports, SDK client classes,
and method calls. It currently reports static import/call coverage for the five
official G1 examples under `example/g1/high_level/` and
`example/g1/low_level/`. That does not mean every behavior is physically
equivalent; locomotion and low-level balance are still simulator
approximations.

Then use `cyber-g1 sdk-smoke` for a conservative behavior-level check against
the local simulator facade:

```sh
cyber-g1 sdk-smoke --kind all
cyber-g1 sdk-smoke --kind arm
cyber-g1 sdk-smoke --kind loco
cyber-g1 sdk-smoke --kind lowcmd
cyber-g1 sdk-smoke --kind all --output .runtime/sdk-smoke/latest.json
cyber-g1 sdk-smoke --kind loco --transport rpc_bridge
```

The smoke runner executes safe official-style `G1ArmActionClient`,
`LocoClient`, and `ChannelPublisher` / `ChannelSubscriber` calls and returns
status plus safety-check evidence. It intentionally avoids free-running
official loops. Pass `--output` when you want a durable JSON artifact that an
agent, bug report, or follow-up prompt can inspect without rerunning motion.
Pass `--transport rpc_bridge` when the proof you want is specifically that
normal Unitree-shaped `LocoClient` / `AgvClient` code crosses the managed
`sport`/`agv` RPC bridge.

## Agent MCP Tools

The default Cybernetic IDE robotics MCP exposes viewer and simulator tools to
the Agent panel. The most useful camera tools are:

```text
viewer_camera_control
viewer_snapshot
viewer_snapshot_file
viewer_snapshot_series
sim_validate_behavior
robot_evidence_bundle
unitree_provider_status
unitree_session_status
unitree_sdk_compatibility_audit
unitree_sdk_behavior_smoke
unitree_read_official_mujoco_lowstate
unitree_probe_official_mujoco_rpc_discovery
unitree_probe_rpc_bridge_smoke
unitree_start_rpc_bridge
unitree_rpc_bridge_status
unitree_probe_rpc_bridge_client
unitree_verify_rpc_bridge
unitree_stop_rpc_bridge
unitree_probe_official_mujoco_loco_rpc
unitree_official_mujoco_evidence_bundle
robotics_tool_reference
unitree_sdk_scaffold_python
scene_add_object
scene_list_objects
scene_remove_object
g1_agv_command
g1_safety_check
g1_lowstate
g1_joint_state
g1_apply_joint_targets
g1_lowcmd
```

`viewer_snapshot` returns an MCP image result. `viewer_snapshot_file` writes the
current camera frame to a workspace file, which is better when the agent needs
to compare before/after views across multiple steps.
`sim_validate_behavior` is the quick post-run health check: it verifies the
simulator is ready, the robot is not fallen, the render cache is healthy,
lowstate telemetry is available, recent lowcmds are fresh when applicable, and
optionally writes a snapshot for visual evidence.
`robot_evidence_bundle` is the broader Agent-panel evidence capture path. It
writes a workspace JSON manifest containing `/status`, `/lowstate`,
`/joint_state`, visual scene metadata, provider diagnostics, check results, and
optional current/series screenshots. Use it after scene edits, SDK scripts, or
pose commands when the agent needs durable evidence instead of a transient
viewer glance.
`g1_safety_check` is the quick pre-motion check: it reads `/status` and
`/lowstate`, then applies Unitree G1-inspired termination predicates for bad
orientation, high joint velocity, high angular velocity, motor temperature,
stale lowcmd, and simulator fall state. It is read-only; use `safety_stop` to
actually damp, neutralize, and pause the simulator.
`unitree_session_status` answers the broader connection question: which
transport is selected, which DDS domain/interface would be used, whether the
current path is implemented, whether the local simulator is reachable, and how
fresh the `rt/lowcmd` / `rt/lowstate` surfaces are. With
`CYBER_UNITREE_TRANSPORT=dds` in simulator mode, it also runs the official
sidecar status probe and reports SDK2 import, CycloneDDS domain, channel
creation, and official MuJoCo peer readiness to Agent-panel MCP clients.
`robotics_tool_reference` returns a compact safety map for the default robotics
tools: safety level, side effects, and expected simulator state. Agents should
read it before choosing between read-only inspection, scene edits, deliberate
robot motion, script execution, and safety-stop workflows.
`scene_add_object`, `scene_list_objects`, and `scene_remove_object` let agents
iterate on simple MuJoCo scene objects. Scene edits write new MJCF copies under
`.runtime/unitree-g1-mujoco/unitree_mujoco/cybernetic_scenes/`; the simulator
container is only recreated when the tool call sets `activate` to true.
`scene_remove_object` defaults to the active scene, or can remove from a
generated-but-not-activated scene by passing its `scene_path`.
`unitree_sdk_scaffold_python` can return or write beginner-friendly scripts for
`raise_hand`, `release_arm`, `arm_action`, `locomotion`,
`lowcmd_joint_target`, `scene_edit`, and `telemetry_monitor`. The generated
scripts keep Unitree-style imports where possible, then use the local simulator
bridge for evidence, snapshots, and safety stops.
`unitree_sdk_behavior_smoke` runs the same SDK-shaped smoke path from the Agent
panel and writes `.runtime/sdk-smoke/latest.json` by default, or a caller-chosen
workspace-relative JSON path via `output_path`. It also accepts
`transport=rpc_bridge` so an Agent-panel assistant can prove that ordinary
Unitree facade calls are using the managed bridge.

## Power User API

Use `SimulatorClient` when you want the raw HTTP endpoint:

```python
from cybernetic_robotics import SimulatorClient

sim = SimulatorClient.from_env()
print(sim.status().simulation["mujoco"])
sim.command("pose", pose="raise_right_hand")
sim.orbit(dx=20, dy=-5)
sim.snapshot(".runtime/g1-control-demo/frame.jpg")
```

Use `TinyWebSocket` when you want the Booster-style physics socket:

```python
from cybernetic_robotics import TinyWebSocket

with TinyWebSocket.from_env() as ws:
    print(ws.request_json({"type": "command", "command": "pause"}))
    message_type, frame = ws.subscribe_once("simulation_state")
    print(message_type, len(frame))
```

Use `SceneWorkspace` for safe MJCF edits:

```python
from cybernetic_robotics import SceneWorkspace

scene = SceneWorkspace.discover()
host_path, container_path = scene.add_box(
    "blue_test_box",
    position=(0.8, 0.0, 0.15),
    size=(0.12, 0.12, 0.12),
    rgba=(0.1, 0.45, 1.0, 1.0),
    activate=True,
)
print(host_path)
print(container_path)
```

Scene helpers write a copy under `.runtime/.../cybernetic_scenes/`. They do
not overwrite the pinned upstream Unitree MJCF asset.

The matching runnable example is:

```sh
python3 examples/g1_scene_obstacle.py --activate
```

It writes `.runtime/g1-control-demo/scene-obstacle-manifest.json` with the
generated host path, the container path, the object parameters, and an optional
viewer screenshot result. If the simulator is not running, scene generation
still succeeds and the manifest records the snapshot error instead of hiding it.

## Environment

| Variable | Purpose |
| --- | --- |
| `CYBER_G1_GAME_CONTROL_URL` | GameControl HTTP URL. Default: `http://127.0.0.1:38383`. |
| `CYBER_G1_PHYSICS_URL` | Physics WebSocket URL. Default: `ws://127.0.0.1:8788`. |
| `CYBER_G1_WS_HOST` | Fallback WebSocket host when `CYBER_G1_PHYSICS_URL` is not set. |
| `CYBER_G1_WS_PORT` | Fallback WebSocket port when `CYBER_G1_PHYSICS_URL` is not set. |
| `CYBER_ROBOTICS_ROOT` | Cybernetic IDE checkout for Docker and scene helpers. |

## Design Notes

The package is deliberately split into layers:

- Beginner scripts should import `G1Robot` and avoid transport details.
- SDK-shaped demos should import `unitree_sdk2py` so future official SDK2/DDS
  work can keep the same user code.
- Advanced tools should use `SimulatorClient`, `TinyWebSocket`, and
  `SceneWorkspace` directly.

The current backend still maps high-level actions onto a direct MuJoCo pose
command. Physical robot control must remain behind explicit real-robot mode
and safety gates.
