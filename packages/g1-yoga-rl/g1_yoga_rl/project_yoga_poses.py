from __future__ import annotations

import argparse
import json
from pathlib import Path

from .poses import DEFAULT_YOGA_SEQUENCE, build_pose_frames, project_named_poses, repo_root


def main() -> int:
    parser = argparse.ArgumentParser(description="Project Cybernetic G1 yoga poses onto LocoMuJoCo's Unitree G1 joint set.")
    parser.add_argument("--output", type=Path, default=Path(".runtime/g1-yoga-rl/yoga_pose_projection.json"))
    parser.add_argument("--transition-steps", type=int, default=30)
    parser.add_argument("--hold-steps", type=int, default=60)
    parser.add_argument("--poses", nargs="*", default=list(DEFAULT_YOGA_SEQUENCE))
    args = parser.parse_args()

    root = repo_root()
    projections = project_named_poses(tuple(args.poses), root=root)
    frames = build_pose_frames(projections, transition_steps=args.transition_steps, hold_steps=args.hold_steps)
    payload = {
        "source": str(root / "overlays/unitree-g1-mujoco-protocol/python/g1_protocol_sim.py"),
        "joint_set": "loco_mujoco.environments.humanoids.UnitreeG1 reduced joint set",
        "pose_count": len(projections),
        "frame_count": len(frames),
        "transition_steps": args.transition_steps,
        "hold_steps": args.hold_steps,
        "poses": [
            {
                "name": projection.name,
                "targets": projection.targets,
                "dropped": projection.dropped,
            }
            for projection in projections
        ],
        "frames": frames,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True))
    print(f"Wrote {args.output}")
    print(f"Projected {payload['pose_count']} poses into {payload['frame_count']} frames")
    dropped = {projection.name: sorted(projection.dropped) for projection in projections if projection.dropped}
    if dropped:
        print("Dropped joints:", json.dumps(dropped, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

