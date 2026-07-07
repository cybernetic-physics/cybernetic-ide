#!/usr/bin/env python3
"""Run a G1 behavior and write a compact debugging bundle for an AI agent."""

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
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
except ModuleNotFoundError:
    sys.path.insert(0, str(PACKAGE_SRC))
    from cybernetic_robotics import G1Robot
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--behavior", choices=["raise_hand", "wave", "move"], default="raise_hand")
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / ".runtime" / "g1-agent-debug-loop")
    parser.add_argument("--sleep", type=float, default=0.6, help="Seconds to let motion settle before collecting after-state.")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, "cyber-sim")
    args.out_dir.mkdir(parents=True, exist_ok=True)

    with G1Robot.connect() as robot:
        robot.reset()
        robot.reset_camera()
        before = collect_evidence(robot, args.out_dir, "before")

        behavior_result = run_behavior(args.behavior)
        time.sleep(max(0.0, args.sleep))

        after = collect_evidence(robot, args.out_dir, "after")
        safety = robot.safety_stop()
        stopped = collect_evidence(robot, args.out_dir, "stopped")

    bundle = {
        "behavior": args.behavior,
        "ok": bool(behavior_result.get("ok", True)) and bool(safety.get("ok")),
        "behavior_result": behavior_result,
        "before": before,
        "after": after,
        "safety_stop": safety,
        "stopped": stopped,
        "agent_hints": [
            "Compare before.status.simulation.pose with after.status.simulation.pose.",
            "Check after.status.simulation.fallen before recommending stronger commands.",
            "Use joint_state.by_name for named joint positions and limits.",
            "Use screenshots for visual confirmation, not as the only source of truth.",
        ],
    }
    manifest = args.out_dir / "debug_bundle.json"
    manifest.write_text(json.dumps(bundle, indent=2, default=str) + "\n", encoding="utf-8")
    print(json.dumps({"ok": bundle["ok"], "bundle": str(manifest), "out_dir": str(args.out_dir)}, indent=2))
    return 0 if bundle["ok"] else 1


def run_behavior(name: str) -> dict[str, Any]:
    if name == "raise_hand":
        arm = G1ArmActionClient()
        arm.SetTimeout(10.0)
        arm.Init()
        code = arm.ExecuteAction(action_map["right hand up"])
        return {"ok": code == 0, "sdk": "G1ArmActionClient", "action": "right hand up", "last_response": arm.last_response}

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()
    if name == "wave":
        code = loco.WaveHand()
        return {"ok": code == 0, "sdk": "LocoClient", "action": "WaveHand", "last_response": loco.last_response}
    if name == "move":
        start_code = loco.Start()
        move_code = loco.Move(0.2, 0.0, 0.0)
        return {
            "ok": start_code == 0 and move_code == 0,
            "sdk": "LocoClient",
            "action": "Start + Move",
            "start_code": start_code,
            "move_code": move_code,
            "last_response": loco.last_response,
        }
    raise AssertionError(name)


def collect_evidence(robot: G1Robot, out_dir: Path, label: str) -> dict[str, Any]:
    screenshot = out_dir / f"{label}.jpg"
    robot.snapshot(screenshot)
    return {
        "label": label,
        "screenshot": str(screenshot),
        "status": robot.sim.status().raw,
        "lowstate": robot.sim.lowstate(),
        "joint_state": robot.sim.joint_state(),
    }


if __name__ == "__main__":
    raise SystemExit(main())
