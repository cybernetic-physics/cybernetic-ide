#!/usr/bin/env python3
"""Drive the local G1 sim through Unitree's G1 LocoClient-shaped API."""

from __future__ import annotations

import time
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "packages" / "cybernetic-robotics" / "src"

try:
    from cybernetic_robotics import G1Robot
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
except ModuleNotFoundError:
    sys.path.insert(0, str(PACKAGE_SRC))
    from cybernetic_robotics import G1Robot
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


def main() -> int:
    ChannelFactoryInitialize(0, "cyber-sim")

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()

    print("Start ->", loco.Start())
    print("Move forward ->", loco.Move(0.25, 0.0, 0.0))
    time.sleep(0.8)
    print("StopMove ->", loco.StopMove())
    print("WaveHand ->", loco.WaveHand())

    out = REPO_ROOT / ".runtime" / "g1-control-demo" / "loco-client.jpg"
    with G1Robot.connect() as robot:
        robot.snapshot(out)
        print("status:", robot.status())
    print("saved:", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
