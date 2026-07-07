#!/usr/bin/env python3
"""
Apply Unitree G1 joint targets by name in the local Cybernetic IDE simulator.

This is the ergonomic layer above raw `rt/lowcmd`: you can inspect the model's
joint names, send a small target dictionary, and let the simulator compile that
to the low-level motor slots used by Unitree's HG messages.
"""

from __future__ import annotations

from pathlib import Path

from cybernetic_robotics import G1Robot


TARGETS = {
    "right_shoulder_pitch_joint": -1.45,
    "right_shoulder_roll_joint": -0.25,
    "right_elbow_joint": 0.95,
    "right_wrist_roll_joint": 0.1,
}


def main():
    with G1Robot.connect() as robot:
        before = robot.joint_state()
        print("Available actuated joints:", before["actuator_count"])
        print("First five joints:", list(before["by_name"])[:5])

        result = robot.apply_joint_targets(TARGETS, kp=34.0, kd=1.2)
        print("Command:", result["lowcmd"])

        after = robot.joint_state()
        for name in TARGETS:
            joint = after["by_name"][name]
            print(f"{name}: q={joint['q']:.3f} motor_index={joint['motor_index']}")

        output = Path(".runtime/g1-control-demo/joint-targets.jpg")
        robot.snapshot(output)
        print(f"Snapshot: {output}")


if __name__ == "__main__":
    main()
