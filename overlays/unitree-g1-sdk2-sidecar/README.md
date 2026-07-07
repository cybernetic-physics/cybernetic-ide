# Unitree G1 SDK2 Sidecar

This overlay is the explicit boundary for the future official Unitree
SDK2/CycloneDDS provider. It is intentionally separate from the current
`unitree-g1-mujoco-protocol` container so the working local HTTP viewer bridge
continues to run while we build the real DDS path.

The sidecar currently:

- mounts pinned official `unitree_sdk2_python`, `unitree_sdk2`, and
  `unitree_mujoco` checkouts prepared under `.runtime/unitree-g1-sdk2/`;
- carries Cybernetic's selected `CYBER_UNITREE_MODE`,
  `CYBER_UNITREE_TRANSPORT`, `CYBER_UNITREE_DDS_DOMAIN`, and
  `CYBER_UNITREE_NETWORK_INTERFACE`;
- prints a structured diagnostic report with source revisions and expected
  Unitree topics/services.

It does **not** yet install CycloneDDS or publish DDS samples. That is the next
provider milestone. Keeping this scaffold honest is better than pretending the
local HTTP shim is already the official transport.

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
