#!/usr/bin/env python3
"""Let the Unitree G1 lead you through a short yoga flow.

Two modes:

* physics (default): the motors are given torque and PD-hold each pose while
  MuJoCo physics runs, with gravity compensation. Statically stable poses
  (mountain, upward salute, namaste) are genuinely held up against gravity;
  balance-heavy poses (deep squats, one-legged balances) topple, because there
  is no whole-body balance controller. When the robot falls it is reset to
  standing before the next pose is attempted.

* --posed: kinematic posing with smooth glides between poses. Physics is paused,
  so the robot never falls -- good as a clean pose reference / choreography.

* --policy: the trained LocoMuJoCo mimic policy drives the whole flow with
  physics on -- smooth glides AND balance come from the network at 100 Hz.
  Requires the sim image to be built with a policy bundle.

    python3 examples/yoga_teacher.py
    python3 examples/yoga_teacher.py --posed
    python3 examples/yoga_teacher.py --policy --rounds 2
    python3 examples/yoga_teacher.py --hold-seconds 4 --rounds 2

Install the package once with:

    python3 -m pip install -e packages/cybernetic-robotics
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "packages" / "cybernetic-robotics" / "src"


try:
    from cybernetic_robotics import G1Robot, SimulatorUnavailable
except ModuleNotFoundError:
    if PACKAGE_SRC.exists():
        sys.path.insert(0, str(PACKAGE_SRC))
        from cybernetic_robotics import G1Robot, SimulatorUnavailable
    else:
        raise


@dataclass(frozen=True)
class Asana:
    """One pose in the flow."""

    pose: str
    name: str
    cue: str


FLOW: list[Asana] = [
    Asana("mountain", "Mountain / Tadasana",
          "Stand tall. Feet grounded, crown lifting. Take a slow breath in."),
    Asana("upward_salute", "Upward Salute / Urdhva Hastasana",
          "Sweep both arms overhead. Reach through the fingertips."),
    Asana("forward_fold", "Forward Fold / Uttanasana",
          "Exhale, hinge at the hips and fold forward. Let the head hang heavy."),
    Asana("chair", "Chair / Utkatasana",
          "Inhale, sit the hips back, knees bent, arms rising. Strong and steady."),
    Asana("warrior_one", "Warrior I / Virabhadrasana I",
          "Step the left foot forward, bend the knee, arms overhead. Find your focus."),
    Asana("warrior_two", "Warrior II / Virabhadrasana II",
          "Open the arms wide, gaze over the front hand. Ground down through both feet."),
    Asana("goddess", "Goddess / Utkata Konasana",
          "Wide stance, knees bowing out, arms in cactus. Breathe into the hips."),
    Asana("tree", "Tree / Vrksasana",
          "Root down through the standing leg, foot to the inner thigh, hands rise. Balance."),
    Asana("namaste", "Namaste",
          "Hands to heart center. Thank you for practicing."),
]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Guide the local MuJoCo G1 through a scripted yoga flow."
    )
    parser.add_argument(
        "--hold-seconds", type=float, default=3.0,
        help="How long to hold each pose (and let physics settle or topple).",
    )
    parser.add_argument(
        "--posed", action="store_true",
        help="Kinematic posing with smooth glides (physics paused; never falls).",
    )
    parser.add_argument(
        "--policy", action="store_true",
        help="Drive the flow with the trained balance policy (physics on).",
    )
    parser.add_argument(
        "--transition-seconds", type=float, default=1.5,
        help="Glide duration between poses in --posed mode.",
    )
    parser.add_argument(
        "--fps", type=int, default=30,
        help="Frames per second rendered during --posed glides.",
    )
    parser.add_argument(
        "--rounds", type=int, default=1,
        help="How many times to repeat the full flow.",
    )
    parser.add_argument(
        "--snapshot-dir", type=Path, default=REPO_ROOT / ".runtime" / "g1-yoga",
        help="Directory for per-pose camera frames.",
    )
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    args.snapshot_dir.mkdir(parents=True, exist_ok=True)

    try:
        if args.policy:
            held, fell = teach_policy(args)
        else:
            held, fell = teach(args)
    except SimulatorUnavailable as error:
        print(f"Simulator is not reachable: {error}", file=sys.stderr)
        print("Install helpers with: python3 -m pip install -e packages/cybernetic-robotics", file=sys.stderr)
        print("Then start it with: cyber-g1 prepare && cyber-g1 start", file=sys.stderr)
        return 1

    if not args.posed:
        print(f"\nSession complete: {held} poses held, {fell} toppled (and reset).")
    print(f"Namaste. Snapshots of your practice saved in {args.snapshot_dir}")
    return 0


def setup_scene(robot: G1Robot) -> None:
    robot.reset_camera()
    robot.orbit(dx=-8, dy=6)
    robot.zoom(140)


def teach_policy(args: argparse.Namespace) -> tuple[int, int]:
    """Run the flow under the trained mimic policy and narrate it live.

    The sim advances a reference trajectory (1 s settle, then per pose a 1.5 s
    glide + 3 s hold at 100 Hz) while the policy balances the robot on real
    torques. /status reports the active reference frame, pose label, and a
    fall counter (falls teleport the robot back onto the reference).
    """

    print("\n== G1 Yoga -- trained balance policy (physics on). Let's begin ==")

    held = 0
    fell = 0
    with G1Robot.connect(timeout=args.timeout) as robot:
        robot.reset()
        setup_scene(robot)

        response = robot.sim.command("yoga_policy", action="start", loop=True)
        if not response.get("ok", False):
            raise RuntimeError(
                f"Policy mode unavailable: {response}. "
                "Rebuild the sim image with a policy bundle to use --policy."
            )
        frames_total = int(response["policy"].get("frames_total", 0))

        def policy_status() -> dict:
            payload = robot.sim.command("yoga_policy", action="status")
            return payload.get("policy", {})

        frequency = 100.0
        settle_frames = int(1.0 * frequency)
        segment_frames = int(4.5 * frequency)

        for round_index in range(1, max(1, args.rounds) + 1):
            if args.rounds > 1:
                print(f"\n-- Round {round_index} of {args.rounds} --")
            round_offset = (round_index - 1) * frames_total
            for step_index, asana in enumerate(FLOW, start=1):
                hold_mid = (round_offset + settle_frames
                            + (step_index - 1) * segment_frames
                            + int(3.0 * frequency))
                falls_before = None
                while True:
                    status = policy_status()
                    if not status.get("active", False):
                        raise RuntimeError("policy mode stopped unexpectedly")
                    progressed = int(status.get("cycles", 0)) * frames_total + int(status["frame"])
                    if falls_before is None:
                        falls_before = int(status.get("falls", 0))
                    if progressed >= hold_mid:
                        break
                    time.sleep(0.25)

                frame = args.snapshot_dir / f"{round_index:02d}-{step_index:02d}-{asana.pose}-policy.jpg"
                robot.snapshot(frame)
                status = policy_status()
                falls_after = int(status.get("falls", 0))
                print(f"\n[{step_index}/{len(FLOW)}] {asana.name}")
                print(f"    {asana.cue}")
                if falls_after > (falls_before or 0):
                    fell += 1
                    print(f"    \U0001f4c9 lost balance ({falls_after - falls_before} recovery resets)")
                else:
                    held += 1
                    print("    \U0001f4aa balanced by the trained policy")

        robot.sim.command("yoga_policy", action="stop")
    return held, fell


def teach(args: argparse.Namespace) -> tuple[int, int]:
    mode = "posed choreography (physics paused)" if args.posed else "physics (motors on, gravity live)"
    print(f"\n== G1 Yoga -- {mode}. Let's begin ==")

    held = 0
    fell = 0
    with G1Robot.connect(timeout=args.timeout) as robot:
        # Smooth glides and physics holds are served synchronously, so allow the
        # request enough time to run the whole motion.
        robot.sim.timeout = max(robot.sim.timeout, args.transition_seconds + 10.0, args.hold_seconds + 10.0)

        robot.reset()
        setup_scene(robot)

        for round_index in range(1, max(1, args.rounds) + 1):
            if args.rounds > 1:
                print(f"\n-- Round {round_index} of {args.rounds} --")
            for step_index, asana in enumerate(FLOW, start=1):
                # If the last pose knocked the robot down, help it back up before
                # attempting the next one.
                if not args.posed and robot.is_fallen():
                    print("   ↺ robot is down -- resetting to standing")
                    robot.reset()
                    setup_scene(robot)

                if args.posed:
                    response = robot.pose(
                        asana.pose, smooth=True,
                        duration=args.transition_seconds, fps=args.fps,
                    )
                else:
                    response = robot.hold(asana.pose)
                if not response.get("ok", False):
                    raise RuntimeError(f"Pose '{asana.pose}' was rejected: {response}")

                time.sleep(max(0.0, args.hold_seconds))

                frame = args.snapshot_dir / f"{round_index:02d}-{step_index:02d}-{asana.pose}.jpg"
                robot.snapshot(frame)

                print(f"\n[{step_index}/{len(FLOW)}] {asana.name}")
                print(f"    {asana.cue}")
                if not args.posed:
                    status = robot.status()
                    if status.fallen:
                        fell += 1
                        print(f"    \U0001f4c9 lost balance and tumbled (pelvis {status.pelvis_height:.2f} m)")
                    else:
                        held += 1
                        print(f"    \U0001f4aa held on real torque (pelvis {status.pelvis_height:.2f} m)")

        # Close in Mountain.
        if not args.posed and robot.is_fallen():
            robot.reset()
            setup_scene(robot)
        if args.posed:
            robot.pose("mountain", smooth=True, duration=args.transition_seconds, fps=args.fps)
        else:
            robot.hold("mountain")
        robot.snapshot(args.snapshot_dir / "closing-mountain.jpg")

    return held, fell


if __name__ == "__main__":
    raise SystemExit(main())
