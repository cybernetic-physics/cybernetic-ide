# Upstream Robotics Audit

This note records the upstream robotics sources cloned into `~/wagmi` and how
they inform Cybernetic IDE's current G1 simulator and SDK facade.

## Cloned Sources

| Repo | Local path | Why it matters |
| --- | --- | --- |
| `robfiras/loco-mujoco` | `~/wagmi/loco-mujoco` | Research environment stack for humanoid locomotion, imitation datasets, retargeting, domain randomization, and MuJoCo/MJX tasks. It includes Unitree G1 environments and examples such as `ImitationFactory.make("MjxUnitreeG1", ...)`. |
| `unitreerobotics/unitree_mujoco` | `~/wagmi/unitree_mujoco` | Official Unitree MuJoCo bridge. Its README states the simulator is meant to run `unitree_sdk2`, `unitree_ros2`, and `unitree_sdk2_python` control programs. It currently emphasizes low-level sim-to-real messages: `LowCmd`, `LowState`, `SportModeState`, G1 `IMUState`, and `unitree_hg` IDL. |
| `unitreerobotics/unitree_sdk2_python` | `~/wagmi/unitree_sdk2_python` | Official Python SDK shape. The key G1 surfaces are `G1ArmActionClient` and `LocoClient`, plus low-level DDS `ChannelPublisher`/`ChannelSubscriber` APIs for `rt/lowcmd` and `rt/lowstate`. |
| `unitreerobotics/unitree_sdk2` | `~/wagmi/unitree_sdk2` | Official C++ SDK and examples. Useful for matching API names, DDS topics, and high-level G1 examples. |
| `unitreerobotics/unitree_rl_gym` | `~/wagmi/unitree_rl_gym` | Policy training/deployment reference for Unitree robots, including G1 deployment material. Useful for future locomotion policy work, not required for the first SDK facade. |

## What We Implemented

Cybernetic's Python package now mirrors the two official high-level G1 SDK
surfaces a developer is likely to reach for first:

- `unitree_sdk2py.g1.arm.g1_arm_action_client.G1ArmActionClient`
- `unitree_sdk2py.g1.loco.g1_loco_client.LocoClient`
- `unitree_sdk2py.core.channel.ChannelPublisher("rt/lowcmd", LowCmd_)`
- `unitree_sdk2py.core.channel.ChannelSubscriber("rt/lowstate", LowState_)`

The arm action shim supports `ExecuteAction(action_map["right hand up"])` and
`ExecuteAction(action_map["release arm"])`.

The locomotion shim supports the official method names:

- `GetFsmId`
- `SetFsmId`
- `Damp`
- `Start`
- `ZeroTorque`
- `Move`
- `StopMove`
- `LowStand`
- `HighStand`
- `WaveHand`
- `ShakeHand`

In simulator mode, these methods post to Cybernetic's local GameControl API.
`Move` is represented as simple kinematic base motion in MuJoCo. Stand and arm
task commands map to local named poses. This makes official-shaped user code
runnable now while keeping the deeper DDS backend boundary explicit.

The low-level channel shim synthesizes `LowState_` from MuJoCo joint and IMU
state, applies `LowCmd_` joint targets into a held MuJoCo frame, and records
torque estimates for telemetry. It reports `mode_pr`, `mode_machine`, CRC,
accepted commands, clamped joint targets, and ignored motor slots so agents can
debug command quality. It also provides the compatibility imports used by
Unitree's official low-level example: `unitree_hg` IDL dataclasses, `CRC`,
`RecurrentThread`, and `MotionSwitcherClient`.

## What Is Still Not Full SDK2 Parity

Unitree's official MuJoCo bridge is low-level DDS-first. Cybernetic now has a
simulator-backed local approximation for `rt/lowcmd` and `rt/lowstate`, but for
full parity it still needs a real bridge for:

- CycloneDDS-compatible `rt/lowcmd` motor command publishing.
- CycloneDDS-compatible `rt/lowstate` motor/IMU state subscription.
- `rt/sportmodestate` high-level body pose and velocity telemetry.
- `rt/wirelesscontroller` simulated joystick input.
- Complete G1 `unitree_hg` IDL coverage and official CRC byte packing.
- A mode switcher / safety gate that prevents low-level commands from fighting
  high-level services.

The current `LocoClient` and low-level command implementation are intentionally
simulator facades, not whole-body balance controllers.

## Agent MCP Additions

The robotics MCP now gives agents both inline and file-based visual inspection:

- `viewer_camera_control`: orbit, pan, zoom, reset, or read the MuJoCo camera.
- `viewer_snapshot`: return the current camera frame as an MCP image result.
- `viewer_snapshot_file`: write the current camera frame to a workspace file.

It also exposes `g1_loco_command` for common `LocoClient`-style commands:
`start`, `move`, `stop_move`, `damp`, `zero_torque`, `low_stand`,
`high_stand`, `wave_hand`, and `shake_hand`.

For lower-level inspection and control, agents can use:

- `g1_lowstate`: read synthesized `rt/lowstate` telemetry.
- `g1_joint_state`: read named joint state, motor indices, limits, and lowcmd
  bookkeeping.
- `g1_apply_joint_targets`: apply joint targets by Unitree G1 joint name while
  still routing through the simulator-backed lowcmd path.
- `g1_lowcmd`: publish a list of simulator-backed motor commands.

## Example Scripts

Use these as the living examples for developer behavior:

```sh
python3 examples/use_cybernetic_robotics_lib.py
python3 examples/g1_raise_hand_sdk.py
python3 examples/g1_loco_sdk.py
python3 examples/g1_lowcmd_sdk.py
python3 examples/g1_joint_targets.py
python3 examples/g1_behavior_gallery.py
python3 examples/yoga_teacher.py
```

They cover the beginner `G1Robot` API, Unitree-shaped arm actions, Unitree
`LocoClient`-shaped locomotion, low-level `rt/lowcmd`/`rt/lowstate` control,
named-joint target control, a behavior gallery with snapshots, and a
higher-level scripted motion flow.

## LocoMuJoCo Policy-Training Package

`packages/g1-yoga-rl/` is the first committed bridge from this audit into a
training workflow. It does not train a policy yet; it makes the prerequisites
explicit and reproducible:

- `g1-yoga-project-poses`: reads Cybernetic's `NAMED_POSES` registry and
  projects those 29-DOF G1 targets onto LocoMuJoCo's reduced Unitree G1 joint
  set.
- `g1-yoga-make-trajectory`: creates a LocoMuJoCo `Trajectory` NPZ from that
  projected yoga sequence when LocoMuJoCo is installed.
- `g1-yoga-bench-env`: benchmarks local CPU/MJX stepping speed to decide
  whether to use JAX mimic training or a simpler CPU fallback.

This package is intentionally dependency-light at import time. Scripts import
LocoMuJoCo, JAX, NumPy, and MuJoCo only when the relevant command is run inside
the dedicated training environment.
