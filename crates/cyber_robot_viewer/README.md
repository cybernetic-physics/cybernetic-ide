# Cyber Robot Viewer

Embedded Zed workspace item for viewing and lightly controlling the Unitree G1
MuJoCo simulator through the Booster-style virtual robot harness.

The viewer expects the Docker/MuJoCo harness to exist locally. By default it
uses:

- `CYBER_ROBOT_HARNESS_DIR=/Users/cuboniks/dasm/booster-studio-real-fork`
- `CYBER_ROBOT_IMAGE=cyber/unitree-g1-mujoco-protocol:0.1.0`
- `CYBER_ROBOT_MODEL_PATH=/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml`

The harness provides a Booster-compatible WebSocket surface on port `8788` and
HTTP GameControl endpoints on port `38383`. The current Zed view uses the HTTP
status and camera-frame endpoints for lower overhead while keeping the wire
protocol aligned with the reversed Booster simulator contract.
