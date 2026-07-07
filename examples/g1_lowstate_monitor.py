#!/usr/bin/env python3
"""Monitor simulator-backed Unitree G1 lowstate and named joints."""

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
except ModuleNotFoundError:
    sys.path.insert(0, str(PACKAGE_SRC))
    from cybernetic_robotics import G1Robot


DEFAULT_JOINTS = [
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "right_shoulder_pitch_joint",
    "right_elbow_joint",
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--joint", action="append", dest="joints", help="Joint name to include. Repeatable.")
    parser.add_argument("--jsonl", type=Path, help="Optional path for raw JSONL samples.")
    args = parser.parse_args()

    joints = args.joints or DEFAULT_JOINTS
    jsonl_file = None
    if args.jsonl:
        args.jsonl.parent.mkdir(parents=True, exist_ok=True)
        jsonl_file = args.jsonl.open("w", encoding="utf-8")

    try:
        with G1Robot.connect() as robot:
            for index in range(max(1, args.samples)):
                sample = collect_sample(robot, joints)
                print_sample(index, sample)
                if jsonl_file is not None:
                    jsonl_file.write(json.dumps(sample, default=str) + "\n")
                    jsonl_file.flush()
                if index + 1 < max(1, args.samples):
                    time.sleep(max(0.0, args.interval))
    finally:
        if jsonl_file is not None:
            jsonl_file.close()

    return 0


def collect_sample(robot: G1Robot, joints: list[str]) -> dict[str, Any]:
    status = robot.sim.status().raw
    lowstate = robot.sim.lowstate()
    joint_state = robot.sim.joint_state()
    by_name = joint_state.get("by_name") if isinstance(joint_state.get("by_name"), dict) else {}
    return {
        "captured_at": time.time(),
        "status": {
            "pose": status.get("simulation", {}).get("pose"),
            "paused": status.get("simulation", {}).get("paused"),
            "fallen": status.get("simulation", {}).get("fallen"),
            "pelvis_height": status.get("simulation", {}).get("pelvis_height"),
        },
        "lowstate": {
            "mode_machine": lowstate.get("mode_machine"),
            "mode_pr": lowstate.get("mode_pr"),
            "crc": lowstate.get("crc"),
            "motor_count": len(lowstate.get("motor_state", [])),
            "lowcmd": lowstate.get("lowcmd", {}),
        },
        "joints": {name: summarize_joint(by_name.get(name)) for name in joints},
    }


def summarize_joint(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    return {
        "motor_index": value.get("motor_index"),
        "q": value.get("q"),
        "dq": value.get("dq"),
        "tau_est": value.get("tau_est"),
        "range": value.get("range"),
    }


def print_sample(index: int, sample: dict[str, Any]) -> None:
    status = sample["status"]
    lowstate = sample["lowstate"]
    lowcmd = lowstate.get("lowcmd") or {}
    print(
        f"[{index}] pose={status.get('pose')} paused={status.get('paused')} "
        f"fallen={status.get('fallen')} pelvis={status.get('pelvis_height')} "
        f"motors={lowstate.get('motor_count')} mode={lowstate.get('mode_machine')}/"
        f"{lowstate.get('mode_pr')} lowcmd_active={lowcmd.get('active')} "
        f"stale={lowcmd.get('stale')}"
    )
    for name, joint in sample["joints"].items():
        if joint is None:
            print(f"  {name:<32} unavailable")
        else:
            print(
                f"  {name:<32} idx={joint.get('motor_index')} "
                f"q={fmt(joint.get('q'))} dq={fmt(joint.get('dq'))} "
                f"tau={fmt(joint.get('tau_est'))}"
            )


def fmt(value: Any) -> str:
    try:
        return f"{float(value): .4f}"
    except (TypeError, ValueError):
        return "   n/a"


if __name__ == "__main__":
    raise SystemExit(main())
