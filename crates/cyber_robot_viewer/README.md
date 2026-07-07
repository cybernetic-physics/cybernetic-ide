# Cyber Robot Viewer

Embedded Zed workspace item for viewing and lightly controlling the Unitree G1
MuJoCo simulator through the Booster-style virtual robot harness.

The viewer expects the Docker/MuJoCo harness to exist locally. By default it
uses:

- `CYBER_ROBOT_HARNESS_DIR=<repo root>` (auto-detected when unset)
- `CYBER_ROBOT_IMAGE=cyber/unitree-g1-mujoco-protocol:0.1.0`
- `CYBER_ROBOT_MODEL_PATH=/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml`

The harness provides a Booster-compatible WebSocket surface on port `8788` and
HTTP GameControl endpoints on port `38383`. The current Zed view uses HTTP
status probes plus cached JPEG camera frames for the embedded viewport, and
sends camera-control updates separately so mouse input is not blocked on a
fresh MuJoCo render.

The checked-in harness lives under `overlays/unitree-g1-mujoco-*`. It fetches
the public Unitree G1 MuJoCo scene into `.runtime/unitree-g1-mujoco` and keeps
those assets out of git.

For debug screenshot runs, set `CYBER_ROBOT_VIEWER_OPEN_ON_STARTUP=1` before
launching a debug Zed binary. The first workspace window will open the robot
viewer automatically inside the active editor pane.
