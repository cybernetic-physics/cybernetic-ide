#!/usr/bin/env python3
"""Wave the local Unitree G1 through the Unitree LocoClient-shaped API."""

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
        default=REPO_ROOT / ".runtime" / "g1-wave-hand-sdk",
    )
    parser.add_argument("--turn", action="store_true", help="Pass turn_flag=True to WaveHand.")
    parser.add_argument(
        "--settle",
        type=float,
        default=0.6,
        help="Seconds to wait before the after snapshot.",
    )
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    ChannelFactoryInitialize(0, "cyber-sim")

    with G1Robot.connect() as robot:
        robot.reset()
        robot.reset_camera()
        before = collect_evidence(robot, args.out_dir, "before")

        loco = LocoClient()
        loco.SetTimeout(10.0)
        loco.Init()
        code = loco.WaveHand(args.turn)
        time.sleep(max(0.0, args.settle))

        after = collect_evidence(robot, args.out_dir, "after")
        stop = robot.safety_stop()

    manifest: dict[str, Any] = {
        "ok": code == 0 and bool(stop.get("ok")),
        "sdk": "unitree_sdk2py.g1.loco.g1_loco_client.LocoClient",
        "call": "WaveHand",
        "turn_flag": args.turn,
        "return_code": code,
        "last_response": loco.last_response,
        "before": before,
        "after": after,
        "safety_stop": stop,
    }
    manifest_path = args.out_dir / "wave_hand_manifest.json"
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


def collect_evidence(robot: G1Robot, out_dir: Path, label: str) -> dict[str, Any]:
    screenshot = out_dir / f"{label}.jpg"
    robot.snapshot(screenshot)
    return {
        "screenshot": str(screenshot),
        "status": robot.status().raw,
        "joint_state": robot.joint_state(),
    }


if __name__ == "__main__":
    raise SystemExit(main())
