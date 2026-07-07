#!/usr/bin/env python3
"""Run a lease-limited official Unitree SDK2 LowCmd stream in MuJoCo.

This example targets the managed official Unitree MuJoCo G1 DDS peer, not
physical hardware. It starts or reuses `unitree-g1-sdk2-session`, reads
official `rt/lowstate`, fills unspecified motors with hold targets inside the
sidecar, and publishes a bounded `LowCmd_` stream with explicit frame-rate and
lease caps.
"""

from __future__ import annotations

import argparse
import json

from cybernetic_robotics import OfficialG1Sim


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", choices=["rt/lowcmd", "rt/arm_sdk", "rt/user_lowcmd"], default="rt/lowcmd")
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--lease-seconds", type=float, default=2.0)
    parser.add_argument("--keep-running", action="store_true")
    args = parser.parse_args()

    official = OfficialG1Sim.discover()
    stopped = None
    with official.session(keep_running=args.keep_running) as session:
        result = session.lowcmd_stream(
            topic=args.topic,
            motor_cmd=[
                {
                    "mode": 1,
                    "q": 0.0,
                    "dq": 0.0,
                    "tau": 0.0,
                    "kp": 8.0,
                    "kd": 0.4,
                }
            ],
            frames=args.frames,
            hz=args.hz,
            lease_seconds=args.lease_seconds,
            max_duration_seconds=min(5.0, max(0.1, args.lease_seconds)),
        )
    stopped = session.stopped

    print(
        json.dumps(
            {
                "ok": result["ok"],
                "topic": result["topic"],
                "effective_frames": result["effective_frames"],
                "stream_hz": result["stream_hz"],
                "lease_seconds": result["lease_seconds"],
                "lowcmd_write_successes": result["lowcmd_write_successes"],
                "lowcmd_summary": result["lowcmd_summary"],
                "kept_running": args.keep_running,
                "stopped": stopped,
            },
            indent=2,
            default=str,
        )
    )
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
