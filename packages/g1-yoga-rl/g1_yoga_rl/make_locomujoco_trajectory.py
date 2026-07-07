from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Create a LocoMuJoCo UnitreeG1 yoga trajectory from Cybernetic poses.")
    parser.add_argument("--output", type=Path, default=Path(".runtime/g1-yoga-rl/yoga_trajectory.npz"))
    parser.add_argument("--flow", choices=["full", "stable"], default="full")
    parser.add_argument("--poses", nargs="*", help="Explicit pose list, overriding --flow.")
    parser.add_argument("--glide", type=float, default=1.5, help="Glide seconds between poses.")
    parser.add_argument("--hold", type=float, default=3.0, help="Hold seconds per pose.")
    parser.add_argument("--replay", action="store_true", help="Replay the trajectory in LocoMuJoCo after saving.")
    args = parser.parse_args()

    from loco_mujoco.environments.humanoids.unitreeG1_mjx import MjxUnitreeG1

    from .yoga_traj import FULL_FLOW, STABLE_FLOW, build_yoga_trajectory

    pose_names = args.poses or (FULL_FLOW if args.flow == "full" else STABLE_FLOW)
    env = MjxUnitreeG1(headless=not args.replay)
    trajectory = build_yoga_trajectory(env, pose_names, glide_seconds=args.glide, hold_seconds=args.hold)
    frame_count = int(trajectory.data.qpos.shape[0])

    args.output.parent.mkdir(parents=True, exist_ok=True)
    trajectory.save(str(args.output))
    print(f"Wrote {args.output} ({frame_count} frames, {frame_count / trajectory.info.frequency:.1f}s)")
    if args.replay:
        env.load_trajectory(trajectory)
        env.play_trajectory(n_steps_per_episode=frame_count)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
