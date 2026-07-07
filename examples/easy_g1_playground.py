#!/usr/bin/env python3
"""Beginner-friendly Cybernetic Robotics package demo."""

from cybernetic_robotics import G1Robot


def main():
    with G1Robot.connect() as robot:
        robot.reset()
        robot.raise_right_hand()
        robot.orbit(dx=45, dy=-10)
        frame = robot.snapshot(".runtime/g1-control-demo/easy-playground.jpg")
        status = robot.status()
        print(f"G1 pose={status.pose} paused={status.paused} frame={frame}")


if __name__ == "__main__":
    main()
