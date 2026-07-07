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
python3 examples/g1_loco_sdk.py
python3 examples/g1_lowcmd_sdk.py
```

## CLI

The package installs `cyber-g1`:

```sh
cyber-g1 status
cyber-g1 raise-hand --snapshot .runtime/g1-control-demo/right-hand-up.jpg
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

The compatibility package currently implements only high-level arm actions
and locomotion actions needed by the local simulator:

| Action name | Action ID | Simulator pose |
| --- | ---: | --- |
| `right hand up` | `23` | `raise_right_hand` |
| `release arm` | `99` | `neutral` |

Unsupported actions return a non-zero result and list the action map. That
makes missing simulator coverage obvious while keeping the official SDK2 method
shape.

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
loco.Move(0.25, 0.0, 0.0)
loco.StopMove()
loco.WaveHand()
```

Supported methods include `GetFsmId`, `SetFsmId`, `Damp`, `Start`,
`ZeroTorque`, `Move`, `StopMove`, `LowStand`, `HighStand`, `WaveHand`, and
`ShakeHand`. `Move` is currently simulated with simple kinematic base motion;
it is not yet Unitree's full locomotion controller.

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
from the local simulator. `ChannelPublisher("rt/lowcmd", LowCmd_)` converts
motor commands to MuJoCo actuator torques. The bridge intentionally preserves
Unitree method names and import paths, but it is still simulator-only and does
not replace Unitree's full CycloneDDS transport or a real whole-body balance
controller.

Run the full example:

```sh
python3 examples/g1_lowcmd_sdk.py
```

## Agent MCP Tools

The default Cybernetic IDE robotics MCP exposes viewer and simulator tools to
the Agent panel. The most useful camera tools are:

```text
viewer_camera_control
viewer_snapshot
viewer_snapshot_file
g1_lowstate
g1_lowcmd
```

`viewer_snapshot` returns an MCP image result. `viewer_snapshot_file` writes the
current camera frame to a workspace file, which is better when the agent needs
to compare before/after views across multiple steps.

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
from cybernetic_robotics.scene import SceneWorkspace

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
