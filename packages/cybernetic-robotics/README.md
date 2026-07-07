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
    print(robot.status().pose)
```

## CLI

```sh
cyber-g1 status
cyber-g1 raise-hand --snapshot .runtime/g1-control-demo/right-hand-up.jpg
cyber-g1 camera orbit --dx 40 --dy -10
cyber-g1 step --count 20
cyber-g1 demo
```

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
```

G1 locomotion examples can use Unitree's `LocoClient` shape:

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
```

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
