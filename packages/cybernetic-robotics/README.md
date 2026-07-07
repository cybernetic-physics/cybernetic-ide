# Cybernetic Robotics Python

`cybernetic-robotics` is the small Python layer that makes Cybernetic IDE's
Unitree G1 MuJoCo harness feel like a robot you can play with, not a protocol
you have to memorize.

It has three layers:

- `G1Robot`: beginner-friendly controls such as `raise_right_hand()`,
  `snapshot()`, `pause()`, and `orbit()`.
- `SimulatorClient` and `TinyWebSocket`: direct HTTP/WebSocket access for
  power users who want the Booster-style simulator protocol.
- `unitree_sdk2py` compatibility modules: a simulator-backed subset of the
  official Unitree SDK2 Python import shape, including G1 arm actions, G1
  locomotion client methods, G1 audio intent, plus low-level `rt/lowcmd` /
  `rt/lowstate` channels.

The package intentionally uses only the Python standard library. The simulator
it talks to is still the Dockerized Cybernetic IDE G1 MuJoCo harness.

## Install

From the Cybernetic IDE repo root:

```sh
python3 -m pip install -e packages/cybernetic-robotics
```

Prepare and start the simulator:

```sh
node script/prepare-unitree-g1-mujoco-container.mjs
docker build -t cyber/unitree-g1-mujoco-protocol:0.1.0 overlays/unitree-g1-mujoco-protocol
docker compose \
  --env-file .runtime/unitree-g1-mujoco/compose.env \
  -f overlays/unitree-g1-mujoco-container/compose.yaml \
  up -d
```

## Beginner API

```python
from cybernetic_robotics import G1Robot

with G1Robot.connect() as robot:
    robot.reset()
    robot.raise_right_hand()
    robot.snapshot(".runtime/g1-control-demo/right-hand-up.jpg")
    robot.safety_stop()
    print(robot.status().pose)
```

## CLI

```sh
cyber-g1 status
cyber-g1 diagnostics
cyber-g1 raise-hand --snapshot .runtime/g1-control-demo/right-hand-up.jpg
cyber-g1 safety-check
cyber-g1 safety-stop
cyber-g1 official status
cyber-g1 official raise-hand
cyber-g1 camera orbit --dx 40 --dy -10
cyber-g1 step --count 20
cyber-g1 demo
```

`cyber-g1 diagnostics` is the quickest way to see what the SDK-shaped bridge is
actually using. It reports `transport=local_http|dds`, `mode=sim|real`, DDS
domain/interface, simulator reachability, and `rt/lowcmd` / `rt/lowstate`
freshness. With `CYBER_UNITREE_TRANSPORT=dds` in simulator mode it also runs
the official sidecar status probe and reports SDK2 import, CycloneDDS domain,
channel creation, and official MuJoCo peer readiness.
`cyber-g1 safety-check` is read-only and applies Unitree G1-inspired
termination checks to the current simulator lowstate before you issue more
motion. Use `cyber-g1 safety-stop` when you want to actually damp, neutralize,
and pause.

The same data is available from Python:

```python
from cybernetic_robotics import UnitreeSession

print(UnitreeSession.from_env().diagnostics())
```

## Official Unitree MuJoCo + SDK2 Probe

When you want to prove the SDK2/CycloneDDS path instead of the lightweight
local viewer harness, use the opt-in official sidecar bridge:

```python
from cybernetic_robotics import OfficialG1Sim

official = OfficialG1Sim.discover()
result = official.raise_right_hand()
print(result["ok"], result["moved_joints"])
```

Or from the CLI:

```sh
cyber-g1 official status
cyber-g1 official raise-hand
cyber-g1 official start-session
cyber-g1 official lowstate-session
cyber-g1 official raise-hand --session
cyber-g1 official stop-session
python3 examples/g1_official_raise_hand.py
python3 examples/g1_official_managed_session.py
```

`official status` is read-only: it checks the sidecar setup, SDK2 imports,
CycloneDDS domain initialization, channel creation, source revisions, and
official MuJoCo peer plan. `official raise-hand` launches the official
`unitree_mujoco` G1 peer in the sidecar, publishes a bounded multi-joint HG
`LowCmd_` pose over `rt/lowcmd`, and verifies moved joints through official
`rt/lowstate`. Both are deliberately short-lived and simulator-only.
For sustained simulator sessions, `official.start_session()` starts the same
managed `unitree-g1-sdk2-session` container used by the MCP, `session_status()`
parses the ready report from Docker logs, `lowstate_session()` reads one
official `rt/lowstate` sample, and `raise_right_hand_session()` sends a bounded
pose to that already-running peer instead of launching another one. Use
`official.stop_session()` when the session should be removed. The friendlier
form is a context manager:

```python
official = OfficialG1Sim.discover()

with official.session() as sim:
    print(sim.lowstate()["lowstate_summary"])
    print(sim.raise_right_hand()["moved_joints"])
    sim.arm_pose_evidence(output_path=".runtime/official-mujoco-evidence/latest.json")
```

Pass `keep_running=True` to leave the Docker peer up for follow-up MCP or CLI
commands. `arm_pose_evidence()` writes a reviewable JSON bundle with before and
after official `rt/lowstate` summaries, the bounded command parameters, moved
joints, and agent hints.

## Unitree SDK2-Shaped API

Installing this package also exposes a simulator-backed `unitree_sdk2py`
compatibility package:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map

ChannelFactoryInitialize(0, "cyber-sim")

arm = G1ArmActionClient()
arm.SetTimeout(10.0)
arm.Init()
arm.ExecuteAction(action_map["right hand up"])
arm.ExecuteAction(action_map["high five"])
```

The simulator maps Unitree's preset G1 arm actions to deterministic static
poses. This includes `right hand up`, `high five`, `hands up`, `clap`, `hug`,
`heart`, `face wave`, `high wave`, `shake hand`, and release. The method shape
matches the official SDK; the local behavior is a visual approximation.
With `CYBER_UNITREE_TRANSPORT=dds` in simulator mode, `right hand up` routes to
the managed official MuJoCo + SDK2/CycloneDDS session and verifies movement via
official `rt/lowstate`.

G1 locomotion examples can use Unitree's `LocoClient` shape:

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
loco.Move(0.25, 0.0, 0.0)
loco.StopMove()
```

Current G1 AGV examples can use the newer Unitree `AgvClient` shape:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.agv.g1_agv_client import AgvClient

ChannelFactoryInitialize(0, "cyber-sim")

agv = AgvClient()
agv.Init()
agv.Move(0.3, 0.0, 0.2)
agv.HeightAdjust(0.25)
```

In the local simulator, AGV movement routes through the same kinematic velocity
path as `LocoClient`. The lateral `vy` argument is accepted for SDK
compatibility but ignored, matching Unitree's AGV documentation.

Low-level examples that follow Unitree's official pattern can also use the
motion switcher before publishing `rt/lowcmd`:

```python
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

switcher = MotionSwitcherClient()
switcher.Init()
print(switcher.CheckMode())
switcher.ReleaseMode()
```

In simulator mode this state is exposed through the local GameControl bridge.
`ReleaseMode()` clears the selected mode and moves the simulator into damp,
which mirrors the setup step in Unitree's low-level examples.

G1 audio examples can use Unitree's `AudioClient` import shape. In the local
MuJoCo simulator this records intent only; real speakers, microphone, and LEDs
are still a future SDK2/DDS backend concern:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

ChannelFactoryInitialize(0, "cyber-sim")

audio = AudioClient()
audio.Init()
audio.TtsMaker("hello from Cybernetic IDE", 0)
audio.SetVolume(40)
audio.LedControl(0, 80, 255)
```

Low-level SDK2-shaped examples can publish `LowCmd_` and read `LowState_`:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

ChannelFactoryInitialize(0, "cyber-sim")

subscriber = ChannelSubscriber("rt/lowstate", LowState_)
subscriber.Init()
state = subscriber.Read()

command = unitree_hg_msg_dds__LowCmd_()
command.mode_machine = state.mode_machine
command.motor_cmd[22].mode = 1
command.motor_cmd[22].q = -1.0
command.motor_cmd[22].kp = 30.0
command.motor_cmd[22].kd = 1.0
command.crc = CRC().Crc(command)

publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
publisher.Init()
publisher.Write(command)
```

See `examples/g1_lowcmd_sdk.py` for a conservative right-arm motion that uses
the same channel shape.

The simulator shim also exposes read-only approximations of Unitree MuJoCo's
Go-family telemetry topics:

```python
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_, WirelessController_

sport = ChannelSubscriber("rt/sportmodestate", SportModeState_)
sport.Init()
print(sport.Read().velocity)

wireless = ChannelSubscriber("rt/wirelesscontroller", WirelessController_)
wireless.Init()
print(wireless.Read().keys)
```

These are synthesized from the local simulator status/lowstate endpoints. They
are useful for import and topic-shape parity, not a replacement for full
CycloneDDS telemetry.

For named-joint control, use the higher-level package API:

```python
from cybernetic_robotics import G1Robot

with G1Robot.connect() as robot:
    print(robot.joint_state()["by_name"]["right_elbow_joint"])
    robot.apply_joint_targets({"right_elbow_joint": 0.9})
```

See `examples/g1_joint_targets.py` for a complete script that inspects the
joint map, applies a right-arm target, and saves a snapshot.

This is a bootstrap bridge. In sim mode it posts to Cybernetic's local
GameControl endpoint. Future real-robot mode should let the same user code
resolve to Unitree's official SDK2 package and CycloneDDS transport.

## Power User API

```python
from cybernetic_robotics import SimulatorClient, TinyWebSocket

sim = SimulatorClient.from_env()
print(sim.status().simulation["mujoco"])
sim.command("pose", pose="raise_right_hand")

with TinyWebSocket.from_env() as ws:
    print(ws.request_json({"type": "command", "command": "pause"}))
```

Scene helpers live in `cybernetic_robotics.scene`. They edit copies of the
active MJCF scene under `.runtime/unitree-g1-mujoco/unitree_mujoco` and avoid
overwriting the pinned upstream Unitree assets.

## Environment Variables

- `CYBER_G1_GAME_CONTROL_URL`: default `http://127.0.0.1:38383`.
- `CYBER_G1_PHYSICS_URL`: default `ws://127.0.0.1:8788`.
- `CYBER_G1_WS_HOST`: fallback WebSocket host, default `127.0.0.1`.
- `CYBER_G1_WS_PORT`: fallback WebSocket port, default `8788`.
- `CYBER_ROBOTICS_ROOT`: Cybernetic IDE repo root for harness and scene helpers.
- `CYBER_UNITREE_TRANSPORT`: `local_http` for the lightweight viewer harness,
  or `dds` in simulator mode for official SDK2/CycloneDDS diagnostics and the
  managed `right hand up` arm action path.
- `CYBER_UNITREE_MODE`: `sim` or `real`; defaults to `sim`.
- `CYBER_UNITREE_DDS_DOMAIN`: defaults to `1` in sim mode and `0` in real mode.
- `CYBER_UNITREE_NETWORK_INTERFACE`: defaults to `lo` in sim mode and must be
  explicit in real mode.
- `CYBER_UNITREE_REAL_UNLOCK`: real mode stays locked unless set to
  `I_UNDERSTAND_THIS_CONTROLS_REAL_HARDWARE`.
- `UNITREE_G1_LOWCMD_WATCHDOG_SECONDS`: simulator lowcmd freshness timeout,
  default `2.0`. `G1Robot.status()` exposes `lowcmd_active`,
  `lowcmd_stale`, and `lowcmd_age_seconds`; SDK-style `LowState_` exposes the
  same freshness metadata plus `lowcmd_watchdog_seconds`.
