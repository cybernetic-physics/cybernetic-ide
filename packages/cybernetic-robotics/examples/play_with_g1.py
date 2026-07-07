#!/usr/bin/env python3
"""Small beginner script for playing with the local Cybernetic G1 sim."""

from cybernetic_robotics import G1Robot


with G1Robot.connect() as robot:
    print("Connected:", robot.status())
    robot.reset()
    robot.raise_right_hand()
    robot.orbit(dx=35, dy=-8)
    frame = robot.snapshot(".runtime/g1-control-demo/play-with-g1.jpg")
    print("Saved", frame)
