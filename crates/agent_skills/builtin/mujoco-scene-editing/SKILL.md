---
name: mujoco-scene-editing
description: Use when editing the Unitree G1 MuJoCo scene, adding objects, validating MJCF, changing scene XML, or asking the simulator to load a modified scene in Cybernetic IDE.
---

# MuJoCo Scene Editing

Use this skill when the user asks to add objects, change the world, inspect MJCF, validate a scene, or make the G1 simulator environment more realistic.

## Current Scene Model

The default scene comes from the pinned public Unitree MuJoCo assets prepared under:

```text
.runtime/unitree-g1-mujoco/unitree_mujoco/
```

Inside the container the same tree is mounted at:

```text
/opt/unitree_mujoco/
```

The active model path is stored in:

```text
.runtime/unitree-g1-mujoco/compose.env
```

## Preferred Tool Path

- Use `scene_get` for a live visual scene summary.
- Use `scene_read_mjcf` before changing MJCF.
- Use `scene_validate_mjcf` after changing or activating a scene.
- Use `scene_add_box` for simple object insertion. With `activate: true`, it writes a scene copy, updates `compose.env`, and recreates the simulator container.

## MJCF Safety Rules

- Prefer creating a scene copy under `.runtime/unitree-g1-mujoco/unitree_mujoco/cybernetic_scenes/`.
- Do not overwrite the pinned upstream Unitree scene unless the user explicitly asks.
- Keep added object names alphanumeric plus `_` or `-`.
- Validate with MuJoCo inside the container before saying a scene works.
- If the simulator is running, expect scene activation to recreate the Docker container.

## Example Request

To add a box at the robot's side:

```json
{
  "name": "blue_test_box",
  "position": [0.8, 0.0, 0.15],
  "size": [0.12, 0.12, 0.12],
  "rgba": [0.1, 0.45, 1.0, 1.0],
  "activate": true
}
```

After activation, call `sim_status`, `scene_validate_mjcf`, and `viewer_snapshot`.
