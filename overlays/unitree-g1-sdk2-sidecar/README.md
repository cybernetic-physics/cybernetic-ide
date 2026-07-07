# Unitree G1 SDK2 Sidecar

This overlay is the explicit boundary for the future official Unitree
SDK2/CycloneDDS provider. It is intentionally separate from the current
`unitree-g1-mujoco-protocol` container so the working local HTTP viewer bridge
continues to run while we build the real DDS path.

The sidecar currently:

- mounts pinned official `unitree_sdk2_python`, `unitree_sdk2`, and
  `unitree_mujoco` checkouts prepared under `.runtime/unitree-g1-sdk2/`;
- installs Debian CycloneDDS plus the official Python `cyclonedds==0.10.2`
  binding expected by `unitree_sdk2_python`;
- carries Cybernetic's selected `CYBER_UNITREE_MODE`,
  `CYBER_UNITREE_TRANSPORT`, `CYBER_UNITREE_DDS_DOMAIN`, and
  `CYBER_UNITREE_NETWORK_INTERFACE`;
- prints a structured diagnostic report with source revisions, expected Unitree
  topics/services, official SDK2 import status, DDS domain initialization, and
  probe channel creation for `rt/lowcmd` and `rt/lowstate`;
- reports the official C++ `unitree_mujoco` peer launch plan, including the G1
  scene, native binary path, MuJoCo symlink, native dependencies, and DDS topics
  to probe after launch.
- when `CYBER_UNITREE_ACTION=build_official_mujoco`, installs/builds official
  `unitree_sdk2`, links the pinned MuJoCo release, and builds the upstream C++
  `simulate/build/unitree_mujoco` binary in the runtime cache. The native image
  includes the SDK2 bridge headers' C++ dependencies, including Eigen.
- when `CYBER_UNITREE_ACTION=launch_probe_official_mujoco`, launches the
  upstream peer briefly under Xvfb with the required `LD_LIBRARY_PATH` so the
  next gate can distinguish loader/display problems from DDS problems.
- when `CYBER_UNITREE_ACTION=serve_official_mujoco`, launches the upstream G1
  peer under Xvfb and keeps it running until the container receives SIGTERM.
  The MCP tools use this action for a named managed session container.
- when `CYBER_UNITREE_ACTION=probe_official_mujoco_dds`, launches the upstream
  G1 peer under Xvfb, subscribes to `rt/lowstate` with official Unitree HG IDL
  types, and reports whether a sample was received from the simulator bridge.
- when `CYBER_UNITREE_ACTION=probe_official_mujoco_lowcmd`, launches the same
  peer, reads official `rt/lowstate`, builds a CRC-valid hold-position
  `LowCmd_`, and reports how many `rt/lowcmd` writes matched the peer.
- when `CYBER_UNITREE_ACTION=probe_official_mujoco_arm_motion`, launches the
  same peer, sends a bounded single-arm-joint `rt/lowcmd` target, and verifies
  motion by reading the changed joint position from `rt/lowstate`.
- when `CYBER_UNITREE_ACTION=command_official_mujoco_arm_pose`, connects to an
  already-running managed official peer, sends a bounded multi-joint arm pose
  over `rt/lowcmd`, and verifies movement from `rt/lowstate`.

It still does **not** replace the local HTTP viewer/control bridge as the
default transport. The sidecar has now proven official `rt/lowstate` sample
exchange, safe official `rt/lowcmd` hold-frame publishing, bounded arm motion,
and a managed-session command path for the Unitree-shaped Python facade.

Prepare sources:

```sh
node script/prepare-unitree-g1-sdk2-sidecar.mjs
```

Run the diagnostic sidecar:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm unitree-g1-sdk2-sidecar
```

Build the official C++ MuJoCo peer:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm -e CYBER_UNITREE_ACTION=build_official_mujoco \
  unitree-g1-sdk2-sidecar
```

Probe whether the official peer can start headlessly:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm -e CYBER_UNITREE_ACTION=launch_probe_official_mujoco \
  unitree-g1-sdk2-sidecar
```

Start a managed long-running official peer session:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run -d --name unitree-g1-sdk2-session \
  -e CYBER_UNITREE_ACTION=serve_official_mujoco \
  unitree-g1-sdk2-sidecar
```

The MCP wrappers `unitree_start_official_mujoco_session`,
`unitree_official_mujoco_session_status`, and
`unitree_stop_official_mujoco_session` manage that container and parse the
initial ready report from Docker logs. This is the durable official G1 DDS peer
lifecycle used by the managed arm-pose command path.

Command the managed official session with a bounded arm pose:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm \
  -e CYBER_UNITREE_ACTION=command_official_mujoco_arm_pose \
  -e CYBER_UNITREE_ARM_POSE_PRESET=raise_right_hand \
  -e CYBER_UNITREE_ARM_POSE_FRAMES=180 \
  unitree-g1-sdk2-sidecar
```

The MCP wrapper is `unitree_command_official_mujoco_arm_pose`. Python users can
hit the same path with `OfficialG1Sim.raise_right_hand_session()` or, with
`CYBER_UNITREE_TRANSPORT=dds`, the familiar
`G1ArmActionClient.ExecuteAction(action_map["right hand up"])` call.

Probe whether the official peer publishes SDK2 lowstate samples:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm -e CYBER_UNITREE_ACTION=probe_official_mujoco_dds \
  unitree-g1-sdk2-sidecar
```

The current verified local result receives a `unitree_hg.msg.dds_.LowState_`
sample from `rt/lowstate` with `motor_count=35` and `mode_machine=5`. The
report intentionally keeps CycloneDDS loopback multicast warnings visible
because they may matter when moving from local read probes to sustained
publisher/subscriber control.

Probe whether the official peer accepts safe SDK2 lowcmd writes:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm -e CYBER_UNITREE_ACTION=probe_official_mujoco_lowcmd \
  unitree-g1-sdk2-sidecar
```

The current verified local result reads the same `LowState_`, creates a
35-motor `unitree_hg.msg.dds_.LowCmd_` with `mode_machine=5`, computes CRC, and
successfully writes 8 of 8 hold frames to `rt/lowcmd`.

Probe whether a bounded official lowcmd changes a G1 arm joint:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm \
  -e CYBER_UNITREE_ACTION=probe_official_mujoco_arm_motion \
  -e CYBER_UNITREE_ARM_MOTION_JOINT=left_elbow \
  -e CYBER_UNITREE_ARM_MOTION_DELTA=0.18 \
  -e CYBER_UNITREE_ARM_MOTION_FRAMES=120 \
  unitree-g1-sdk2-sidecar
```

The MCP wrapper exposes the same knobs as bounded parameters: arm joint, target
delta, frame count, moving-joint `kp`/`kd`, and hold-joint `kp`/`kd`. Verified
local results include the default `right_shoulder_roll` probe writing 220 of
220 frames and moving from `0.0` to about `-0.289 rad`, plus a parameterized
`left_elbow` probe writing 120 of 120 frames and moving from `0.0` to about
`0.207 rad` through official `rt/lowstate`.

Probe a bounded multi-joint arm pose through the same official lowcmd path:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm \
  -e CYBER_UNITREE_ACTION=probe_official_mujoco_arm_pose \
  -e CYBER_UNITREE_ARM_POSE_PRESET=raise_right_hand \
  -e CYBER_UNITREE_ARM_POSE_FRAMES=180 \
  unitree-g1-sdk2-sidecar
```

The MCP wrapper also accepts custom bounded `joint_deltas` for official G1 arm
joints. The verified `raise_right_hand` preset wrote 180 of 180 frames and
moved five right-arm joints through official `rt/lowstate`: shoulder pitch,
shoulder roll, shoulder yaw, elbow, and wrist pitch.
