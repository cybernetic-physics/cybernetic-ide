#!/usr/bin/env python3
"""Small behavior gallery for the local Cybernetic G1 simulator."""

from __future__ import annotations

from pathlib import Path
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "packages" / "cybernetic-robotics" / "src"

try:
    from cybernetic_robotics import G1Robot
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
except ModuleNotFoundError:
    sys.path.insert(0, str(PACKAGE_SRC))
    from cybernetic_robotics import G1Robot
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


def main() -> int:
    out_dir = REPO_ROOT / ".runtime" / "g1-behavior-gallery"
    out_dir.mkdir(parents=True, exist_ok=True)

    ChannelFactoryInitialize(0, "cyber-sim")

    with G1Robot.connect() as robot:
        robot.reset()
        robot.reset_camera()
        robot.snapshot(out_dir / "00-neutral.jpg")

        arm = G1ArmActionClient()
        arm.SetTimeout(10.0)
        arm.Init()
        arm.ExecuteAction(action_map["right hand up"])
        robot.snapshot(out_dir / "01-arm-action-right-hand-up.jpg")

        loco = LocoClient()
        loco.SetTimeout(10.0)
        loco.Init()
        loco.Start()
        loco.Move(0.25, 0.0, 0.0)
        time.sleep(0.8)
        loco.StopMove()
        robot.snapshot(out_dir / "02-loco-move-forward.jpg")

        loco.WaveHand()
        robot.orbit(dx=32, dy=-6)
        robot.snapshot(out_dir / "03-loco-wave-hand.jpg")

        loco.LowStand()
        robot.snapshot(out_dir / "04-low-stand.jpg")

        loco.HighStand()
        robot.snapshot(out_dir / "05-high-stand.jpg")

        loco.Damp()
        robot.neutral()
        robot.snapshot(out_dir / "06-safe-neutral.jpg")

    print(f"Saved behavior gallery frames under {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
