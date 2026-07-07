from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from .g1 import G1Robot
from .harness import DockerHarness
from .simulator import SimulatorClient


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="cyber-g1")
    subcommands = parser.add_subparsers(dest="command", required=True)

    subcommands.add_parser("status")
    subcommands.add_parser("pause")
    subcommands.add_parser("resume")
    subcommands.add_parser("reset")

    step = subcommands.add_parser("step")
    step.add_argument("--count", type=int, default=1)

    pose = subcommands.add_parser("pose")
    pose.add_argument("name", choices=["raise_right_hand", "neutral"])

    raise_hand = subcommands.add_parser("raise-hand")
    raise_hand.add_argument("--snapshot", type=Path)

    snapshot = subcommands.add_parser("snapshot")
    snapshot.add_argument("path", type=Path)

    camera = subcommands.add_parser("camera")
    camera_subcommands = camera.add_subparsers(dest="camera_command", required=True)
    camera_subcommands.add_parser("reset")
    orbit = camera_subcommands.add_parser("orbit")
    orbit.add_argument("--dx", type=float, required=True)
    orbit.add_argument("--dy", type=float, default=0.0)
    pan = camera_subcommands.add_parser("pan")
    pan.add_argument("--dx", type=float, required=True)
    pan.add_argument("--dy", type=float, default=0.0)
    zoom = camera_subcommands.add_parser("zoom")
    zoom.add_argument("--delta", type=float, required=True)

    demo = subcommands.add_parser("demo")
    demo.add_argument("--snapshot-dir", type=Path, default=Path(".runtime/g1-control-demo"))

    subcommands.add_parser("prepare")
    subcommands.add_parser("start")
    subcommands.add_parser("stop")
    logs = subcommands.add_parser("logs")
    logs.add_argument("--tail", type=int, default=120)

    args = parser.parse_args(argv)

    if args.command in {"prepare", "start", "stop", "logs"}:
        return _harness_command(args)

    robot = G1Robot.connect(wait=args.command != "snapshot")
    if args.command == "status":
        status = robot.status()
        return _print({"ready": status.ready, "pose": status.pose, "paused": status.paused, "speed": status.speed})
    if args.command == "pause":
        return _print(robot.pause())
    if args.command == "resume":
        return _print(robot.resume())
    if args.command == "reset":
        return _print(robot.reset())
    if args.command == "step":
        return _print({"steps": robot.step(args.count)})
    if args.command == "pose":
        return _print(robot.pose(args.name))
    if args.command == "raise-hand":
        result = robot.raise_right_hand()
        if args.snapshot:
            result = {**result, "snapshot": str(robot.snapshot(args.snapshot))}
        return _print(result)
    if args.command == "snapshot":
        return _print({"snapshot": str(robot.snapshot(args.path))})
    if args.command == "camera":
        return _camera_command(SimulatorClient.from_env(), args)
    if args.command == "demo":
        result = robot.demo(args.snapshot_dir)
        return _print(
            {
                "pose": result["status"].pose,
                "before": str(result["before"]),
                "after": str(result["after"]),
            }
        )

    parser.error(f"unknown command {args.command}")
    return 2


def _camera_command(sim: SimulatorClient, args: argparse.Namespace) -> int:
    if args.camera_command == "reset":
        return _print(sim.reset_camera().raw)
    if args.camera_command == "orbit":
        return _print(sim.orbit(args.dx, args.dy).raw)
    if args.camera_command == "pan":
        return _print(sim.pan(args.dx, args.dy).raw)
    if args.camera_command == "zoom":
        return _print(sim.zoom(args.delta).raw)
    raise AssertionError(args.camera_command)


def _harness_command(args: argparse.Namespace) -> int:
    harness = DockerHarness.discover()
    if args.command == "prepare":
        result = harness.prepare()
    elif args.command == "start":
        result = harness.start()
    elif args.command == "stop":
        result = harness.stop()
    elif args.command == "logs":
        result = harness.logs(args.tail)
    else:
        raise AssertionError(args.command)
    sys.stdout.write(result.stdout)
    sys.stderr.write(result.stderr)
    return result.returncode


def _print(value: object) -> int:
    print(json.dumps(value, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
