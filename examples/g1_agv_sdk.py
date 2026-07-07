#!/usr/bin/env python3
"""Drive the local G1 sim through Unitree's G1 AgvClient-shaped API."""

from __future__ import annotations

from pathlib import Path
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "packages" / "cybernetic-robotics" / "src"

try:
    from cybernetic_robotics import G1Robot
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.agv.g1_agv_client import AgvClient
except ModuleNotFoundError:
    sys.path.insert(0, str(PACKAGE_SRC))
    from cybernetic_robotics import G1Robot
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.agv.g1_agv_client import AgvClient


def main() -> int:
    ChannelFactoryInitialize(0, "cyber-sim")

    agv = AgvClient()
    agv.SetTimeout(10.0)
    agv.Init()

    with G1Robot.connect() as robot:
        robot.reset()
        robot.reset_camera()
        before = REPO_ROOT / ".runtime" / "g1-control-demo" / "agv-before.jpg"
        after = REPO_ROOT / ".runtime" / "g1-control-demo" / "agv-after.jpg"
        robot.snapshot(before)

        print("Move(vx=0.3, vy=0.2, vyaw=0.2) ->", agv.Move(0.3, 0.2, 0.2))
        print("last_response:", agv.last_response)
        time.sleep(0.75)
        print("HeightAdjust(vz=0.25) ->", agv.HeightAdjust(0.25))
        print("last_response:", agv.last_response)
        time.sleep(0.25)
        print("Stop ->", agv.Move(0.0, 0.0, 0.0))

        robot.snapshot(after)
        robot.safety_stop()
        print("saved:", before)
        print("saved:", after)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
