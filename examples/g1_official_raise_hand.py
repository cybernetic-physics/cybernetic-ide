#!/usr/bin/env python3
"""Raise the G1 hand through the official Unitree MuJoCo + SDK2 sidecar.

This launches the opt-in official simulator peer, publishes a bounded
multi-joint HG LowCmd_ pose over rt/lowcmd, and verifies motion through
official rt/lowstate. It is a short-lived diagnostic bridge, not the final
long-lived DDS transport.
"""

from __future__ import annotations

import json

from cybernetic_robotics import OfficialG1Sim


def main() -> int:
    official = OfficialG1Sim.discover()
    result = official.raise_right_hand(frames=180, min_moved_joints=2)
    print(
        json.dumps(
            {
                "ok": result["ok"],
                "preset": result["preset"],
                "moved_joints": result["moved_joints"],
                "lowcmd_write_successes": result["lowcmd_write_successes"],
                "parameters": result["parameters"],
            },
            indent=2,
        )
    )
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
