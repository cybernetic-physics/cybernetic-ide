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
design. If the MCP has started the managed `unitree-g1-sdk2-session`,
`official.raise_right_hand_session()` sends the same bounded pose to that
already-running peer instead of launching another one. With
`CYBER_UNITREE_TRANSPORT=dds`, `G1ArmActionClient.ExecuteAction()` routes
`right hand up` through that managed official session.

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

## Agent MCP Tools

The default Cybernetic IDE robotics MCP exposes viewer and simulator tools to
the Agent panel. The most useful camera tools are:

```text
viewer_camera_control
viewer_snapshot
viewer_snapshot_file
viewer_snapshot_series
sim_validate_behavior
unitree_provider_status
unitree_session_status
robotics_tool_reference
unitree_sdk_scaffold_python
scene_add_object
scene_list_objects
scene_remove_object
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
