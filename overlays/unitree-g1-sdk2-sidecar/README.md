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
- when `CYBER_UNITREE_ACTION=read_official_mujoco_lowstate`, connects to an
  already-running managed official peer and reads one `rt/lowstate` sample
  without commanding motion.

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

Read one official lowstate sample from the managed session:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm -e CYBER_UNITREE_ACTION=read_official_mujoco_lowstate \
  unitree-g1-sdk2-sidecar
```

The MCP wrapper is `unitree_read_official_mujoco_lowstate`. Python users can
call `OfficialG1Sim.lowstate_session()`.

Inspect which official Unitree RPC request topics have a matched service-side
reader:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm \
  -e CYBER_UNITREE_ACTION=probe_official_mujoco_rpc_discovery \
  -e CYBER_UNITREE_RPC_DISCOVERY_WAIT=1.0 \
  unitree-g1-sdk2-sidecar
```

The MCP wrapper is `unitree_probe_official_mujoco_rpc_discovery`. Python users
can call `OfficialG1Sim.rpc_discovery_session()`. This is a read-only DDS
discovery check: it creates Unitree-typed request writers for `sport`, `agv`,
`arm`, and `voice`, waits for publication matching, and reports whether a
service-side reader exists before any RPC call is sent.

Smoke-test a temporary Unitree-shaped RPC bridge:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm \
  -e CYBER_UNITREE_ACTION=probe_unitree_rpc_bridge_smoke \
  -e CYBER_UNITREE_RPC_BRIDGE_TIMEOUT=1.0 \
  unitree-g1-sdk2-sidecar
```

The MCP wrapper is `unitree_probe_rpc_bridge_smoke`. Python users can call
`OfficialG1Sim.rpc_bridge_smoke()`. This starts temporary `sport`, `agv`, and
`arm` Unitree RPC servers inside the sidecar and calls them with SDK clients.
It does not command hardware; it proves the request/response service bridge
shape before a long-running bridge maps those APIs onto simulator providers.

Start a managed Unitree-shaped RPC bridge:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run -d --name unitree-g1-rpc-bridge \
  -e CYBER_UNITREE_ACTION=serve_unitree_rpc_bridge \
  unitree-g1-sdk2-sidecar
```

The MCP lifecycle is `unitree_start_rpc_bridge`,
`unitree_rpc_bridge_status`, `unitree_probe_rpc_bridge_client`, and
`unitree_verify_rpc_bridge`, plus the per-action `unitree_command_rpc_bridge`,
then `unitree_stop_rpc_bridge`. Python users can call
`OfficialG1Sim.start_rpc_bridge()`, `rpc_bridge_status()`,
`rpc_bridge_client()`, `verify_rpc_bridge()`, `rpc_bridge_command()`, and
`stop_rpc_bridge()`. The
verifier is the best agent-facing sanity check because it summarizes `RPC_OK`,
simulator readback, simulator forwarding, and bridge-state-only fallback counts.
For one command, use:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm \
  -e CYBER_UNITREE_ACTION=command_unitree_rpc_bridge \
  -e CYBER_UNITREE_RPC_BRIDGE_SERVICE=sport \
  -e CYBER_UNITREE_RPC_BRIDGE_METHOD=move \
  -e 'CYBER_UNITREE_RPC_BRIDGE_PARAMS={"vx":0.05,"duration":0.5}' \
  unitree-g1-sdk2-sidecar
```

This first managed bridge keeps `sport`, `agv`, and `arm` SDK2 servers alive
for external SDK2 clients and uses Cybernetic's simulator provider at
`CYBER_SIMULATOR_GAME_CONTROL_URL` for safe read/write calls.
`arm.ExecuteAction` maps known official G1 action IDs such as `right hand up`
to simulator poses, while unknown action IDs are recorded as bridge-state-only
intent. Getter RPCs
such as `sport.GetFsmId`, `sport.GetFsmMode`, `sport.GetBalanceMode`,
`sport.GetSwingHeight`, and `sport.GetStandHeight` read back from the simulator
when it is reachable and fall back to bridge state otherwise. Setter RPCs
forward:
`sport.SetFsmId`, `sport.SetBalanceMode`, `sport.SetSwingHeight`,
`sport.SetStandHeight`, `sport.SetVelocity`, `sport.SetTaskId`,
`arm.ExecuteAction`, `agv.Move`, and `agv.HeightAdjust`. `HeightAdjust` is
accepted for SDK compatibility but is reported as `bridge_state_only` until the
local simulator has a modeled height-column actuator. `sport.SetTaskId` keeps
Unitree's wave/shake task IDs and also accepts official G1 arm action IDs from
`G1ArmActionClient.action_map` as a Cybernetic simulator convenience; recognized
IDs are forwarded with a pose hint, and unknown IDs are recorded as state-only
intent. This also covers common `LocoClient` shortcuts including `Damp`,
`StopMove`, `WaveHand`, and `ShakeHand`, plus Cybernetic's simulator bridge
extensions for `GetPhase`, `SwitchMoveMode`, `SetSpeedMode`,
`SwitchToUserCtrl`, and
`SwitchToInternalCtrl`. If the simulator
HTTP bridge is unavailable or the operation is unsupported, the RPC still
returns `RPC_OK` for SDK compatibility, but the JSON response marks
`simulator_forward.provider` or
`simulator_readback.provider` as `bridge_state_only` so agents know the MuJoCo
state was not updated or queried. The managed client probe includes raw getter
debug calls plus
`sport.RawSetFsmIdDebug`, `sport.RawSetBalanceModeDebug`,
`sport.RawSetSwingHeightDebug`, `sport.RawSetVelocityDebug`, and
`sport.RawSetArmTaskDebug` because Unitree's high-level `LocoClient` setter
methods return only the status code; the raw SDK calls expose the handler JSON
bodies for diagnostics.

Probe whether the managed official peer answers G1 sport/LocoClient RPCs:

```sh
docker compose \
  --env-file .runtime/unitree-g1-sdk2/compose.env \
  -f overlays/unitree-g1-sdk2-sidecar/compose.yaml \
  run --rm \
  -e CYBER_UNITREE_ACTION=probe_official_mujoco_loco_rpc \
  -e CYBER_UNITREE_LOCO_RPC_TIMEOUT=2.0 \
  unitree-g1-sdk2-sidecar
```

The MCP wrapper is `unitree_probe_official_mujoco_loco_rpc`. Python users can
call `OfficialG1Sim.loco_rpc_session()`. This is deliberately a probe before a
provider: it proves whether the official G1 peer serves
`rt/api/sport/request` / `rt/api/sport/response` before the local `LocoClient`
facade routes locomotion there. Each call includes a Unitree `rpc_status`
annotation when the SDK returns a numeric code. `RPC_ERR_CLIENT_SEND` (`3102`)
means the request could not be written to DDS, typically because the
`rt/api/sport/request` writer did not match a service-side reader before the
timeout or because the DDS write failed.

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
