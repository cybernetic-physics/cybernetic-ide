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

It does **not** yet launch official `unitree_mujoco` or prove data exchange
with a simulator peer. That pub/sub probe is the next provider milestone.
Keeping this scaffold honest is better than pretending the local HTTP shim is
already the official transport.

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
