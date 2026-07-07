#!/usr/bin/env python3
"""Show the simulator safety-stop path for a local Unitree G1."""

from __future__ import annotations

from pathlib import Path
import sys
import time


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
    out_dir = REPO_ROOT / ".runtime" / "g1-control-demo"
    out_dir.mkdir(parents=True, exist_ok=True)

    loco = LocoClient()
    loco.SetTimeout(5.0)
    loco.Init()

    with G1Robot.connect() as robot:
        robot.reset()
        robot.snapshot(out_dir / "safety-stop-before.jpg")

        loco.Start()
        loco.Move(0.2, 0.0, 0.0)
        time.sleep(0.6)

        result = robot.safety_stop()
        robot.snapshot(out_dir / "safety-stop-after.jpg")

    print("safety_stop:", result)
    print("saved:", out_dir / "safety-stop-after.jpg")
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
