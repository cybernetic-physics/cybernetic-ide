#!/usr/bin/env python3
"""Walk a small square through the Unitree G1 LocoClient-shaped API."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any


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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=REPO_ROOT / ".runtime" / "g1-walk-square-loco",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.22,
        help="Linear speed for each side.",
    )
    parser.add_argument(
        "--turn-rate",
        type=float,
        default=0.55,
        help="Yaw speed for each corner.",
    )
    parser.add_argument("--side-seconds", type=float, default=0.7)
    parser.add_argument("--turn-seconds", type=float, default=0.45)
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    ChannelFactoryInitialize(0, "cyber-sim")

    commands: list[dict[str, Any]] = []
    with G1Robot.connect() as robot:
        robot.reset()
        robot.reset_camera()
        robot.snapshot(args.out_dir / "00-start.jpg")

        loco = LocoClient()
        loco.SetTimeout(10.0)
        loco.Init()
        commands.append(record_call("Start", loco.Start(), loco.last_response))

        for side in range(4):
            commands.append(
                record_call(
                    f"side_{side + 1}_move",
                    loco.Move(args.speed, 0.0, 0.0),
                    loco.last_response,
                )
            )
            time.sleep(max(0.0, args.side_seconds))
            commands.append(
                record_call(
                    f"corner_{side + 1}_turn",
                    loco.Move(0.0, 0.0, args.turn_rate),
                    loco.last_response,
                )
            )
            time.sleep(max(0.0, args.turn_seconds))
            robot.snapshot(args.out_dir / f"{side + 1:02d}-corner.jpg")

        commands.append(record_call("StopMove", loco.StopMove(), loco.last_response))
        final = collect_final_state(robot, args.out_dir)
        stop = robot.safety_stop()

    manifest: dict[str, Any] = {
        "ok": all(item["return_code"] == 0 for item in commands) and bool(stop.get("ok")),
        "sdk": "unitree_sdk2py.g1.loco.g1_loco_client.LocoClient",
        "pattern": "square",
        "parameters": {
            "speed": args.speed,
            "turn_rate": args.turn_rate,
            "side_seconds": args.side_seconds,
            "turn_seconds": args.turn_seconds,
        },
        "commands": commands,
        "final": final,
        "safety_stop": stop,
    }
    manifest_path = args.out_dir / "walk_square_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, default=str) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {"ok": manifest["ok"], "manifest": str(manifest_path)},
            indent=2,
        )
    )
    return 0 if manifest["ok"] else 1


def record_call(name: str, return_code: int, response: dict[str, Any] | None) -> dict[str, Any]:
    return {"name": name, "return_code": return_code, "response": response}


def collect_final_state(robot: G1Robot, out_dir: Path) -> dict[str, Any]:
    screenshot = out_dir / "final.jpg"
    robot.snapshot(screenshot)
    return {
        "screenshot": str(screenshot),
        "status": robot.status().raw,
        "lowstate": robot.lowstate(),
    }


if __name__ == "__main__":
    raise SystemExit(main())
