#!/usr/bin/env python3
"""Drive the official Unitree MuJoCo G1 peer as a managed DDS session.

This is the Python package version of the Agent-panel MCP lifecycle:

1. Start the named `unitree-g1-sdk2-session` Docker container.
2. Read one official SDK2/CycloneDDS `rt/lowstate` sample.
3. Send a bounded G1 arm pose through official `rt/lowcmd`.
4. Read another `rt/lowstate` sample from the same sustained peer.

The example is simulator-only. It uses Unitree's official MuJoCo peer and SDK2
topics, but it does not target physical hardware.
"""

from __future__ import annotations

import argparse
import json

from cybernetic_robotics import OfficialG1Sim


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--preset", choices=["raise_right_hand", "raise_left_hand"], default="raise_right_hand")
    parser.add_argument("--frames", type=int, default=180)
    parser.add_argument("--out", default=".runtime/official-mujoco-evidence/latest.json")
    parser.add_argument("--keep-running", action="store_true")
    args = parser.parse_args()

    official = OfficialG1Sim.discover()
    stopped = None
    with official.session(keep_running=args.keep_running) as session:
        report = session.arm_pose_evidence(args.preset, frames=args.frames, output_path=args.out)
    stopped = session.stopped

    report = {**report, "kept_running": args.keep_running, "stopped": stopped}
    print(json.dumps(report, indent=2, default=str))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
