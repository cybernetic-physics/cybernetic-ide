---
name: robot-safety
description: Use when controlling, stopping, resetting, or planning motion for simulated or real robots in Cybernetic IDE, especially Unitree G1 actions, low-level control, watchdogs, or safety-sensitive commands.
---

# Robot Safety

Use this skill whenever an agent is about to move a robot, run control code, change runtime mode, disable a watchdog, or bridge from simulation toward real hardware.

## Current Policy

The current Cybernetic IDE robotics tools are simulation-first. The default Unitree G1 path controls the local MuJoCo container, not a physical robot.

Safe by default:

- Read status, logs, visual scene, and snapshots.
- Generate Python SDK code.
- Run bounded simulator demos.
- Pause the simulator.

Confirm before doing:

- Starting, stopping, or recreating Docker runtime.
- Resetting the simulator.
- Applying robot poses.
- Editing or activating MJCF scenes.
- Starting long-running Python control jobs.

Deny unless the user explicitly configures a real-robot mode:

- Low-level torque or joint commands.
- Real robot network connection.
- Watchdog disablement.
- Raw DDS writes.
- Motions near people, obstacles, or hardware limits.

## Emergency Tooling

Use `safety_stop` when the user asks to stop motion or when a control script behaves unexpectedly. It pauses the simulator and returns the current G1 arm pose to neutral.

For managed Python jobs:

- `python_control_list`
- `python_control_logs`
- `python_control_pause`
- `python_control_stop`

## Response Style

When safety matters, be direct and concrete. Say what mode is active, what tool will run, whether it affects only simulation, and what the rollback/stop command is.
