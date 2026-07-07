#!/usr/bin/env python3
"""Run the local G1 simulator through the Cybernetic Robotics Python package.

This is the replacement for the older dependency-free WebSocket probe script
when you want normal user code:

    python3 examples/use_cybernetic_robotics_lib.py
    python3 examples/use_cybernetic_robotics_lib.py --mode unitree

Install the package once with:

    python3 -m pip install -e packages/cybernetic-robotics
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "packages" / "cybernetic-robotics" / "src"


try:
    from cybernetic_robotics import G1Robot, SimulatorUnavailable
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
except ModuleNotFoundError:
    if PACKAGE_SRC.exists():
        sys.path.insert(0, str(PACKAGE_SRC))
        from cybernetic_robotics import G1Robot, SimulatorUnavailable
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
    else:
        raise


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Raise the local MuJoCo G1 hand through the Cybernetic Robotics Python package."
    )
    parser.add_argument(
        "--mode",
        choices=["friendly", "unitree", "both"],
        default="both",
        help="Which package surface to exercise.",
    )
    parser.add_argument(
        "--snapshot-dir",
        type=Path,
        default=REPO_ROOT / ".runtime" / "g1-control-demo",
        help="Directory for before/after camera frames.",
    )
    parser.add_argument(
        "--run-seconds",
        type=float,
        default=0.0,
        help="Optionally let physics run after saving the hand-raise snapshot.",
    )
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    args.snapshot_dir.mkdir(parents=True, exist_ok=True)

    try:
        if args.mode in {"friendly", "both"}:
            run_friendly_api(args)
        if args.mode in {"unitree", "both"}:
            run_unitree_style_api(args)
    except SimulatorUnavailable as error:
        print(f"Simulator is not reachable: {error}", file=sys.stderr)
        print("Install helpers with: python3 -m pip install -e packages/cybernetic-robotics", file=sys.stderr)
        print("Then start it with: cyber-g1 prepare && cyber-g1 start", file=sys.stderr)
        return 1

    print(f"Saved camera frames in {args.snapshot_dir}")
    return 0


def run_friendly_api(args: argparse.Namespace) -> None:
    print("\n== cybernetic_robotics.G1Robot ==")
    with G1Robot.connect(timeout=args.timeout) as robot:
        print_status("connected", robot)
        robot.reset()
        robot.reset_camera()
        robot.snapshot(args.snapshot_dir / "friendly-before.jpg")

        response = robot.raise_right_hand()
        print_json("raise_right_hand", response)
        robot.orbit(dx=30, dy=-8)
        robot.snapshot(args.snapshot_dir / "friendly-right-hand-up.jpg")

        if args.run_seconds > 0:
            robot.run_for(args.run_seconds)
            robot.snapshot(args.snapshot_dir / "friendly-after-run.jpg")

        print_status("after friendly API", robot)


def run_unitree_style_api(args: argparse.Namespace) -> None:
    print("\n== unitree_sdk2py-shaped G1ArmActionClient ==")
    ChannelFactoryInitialize(0, "cyber-sim")

    arm = G1ArmActionClient()
    arm.SetTimeout(args.timeout)
    arm.Init()

    code, actions = arm.GetActionList()
    simulated = [action for action in actions if action["simulated"]]
    print_json("available simulated actions", {"code": code, "actions": simulated})

    result = arm.ExecuteAction(action_map["right hand up"])
    print_json("ExecuteAction(action_map['right hand up'])", {"code": result, "response": arm.last_response})
    if result != 0:
        raise RuntimeError(f"Unitree-style action failed: {arm.last_response}")

    with G1Robot.connect(timeout=args.timeout) as robot:
        time.sleep(0.15)
        robot.snapshot(args.snapshot_dir / "unitree-right-hand-up.jpg")
        print_status("after Unitree-style API", robot)


def print_status(label: str, robot: G1Robot) -> None:
    status = robot.status()
    print_json(
        label,
        {
            "ready": status.ready,
            "pose": status.pose,
            "paused": status.paused,
            "speed": status.speed,
            "model_path": status.model_path,
        },
    )


def print_json(label: str, value: object) -> None:
    print(f"{label}:")
    print(json.dumps(value, indent=2, default=str))


if __name__ == "__main__":
    raise SystemExit(main())
