from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Render one frame per pose hold from a G1 yoga trajectory.")
    parser.add_argument("--traj", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--glide", type=float, default=1.5)
    parser.add_argument("--hold", type=float, default=3.0)
    args = parser.parse_args()

    import mujoco
    import numpy as np
    from PIL import Image
    from loco_mujoco.environments.humanoids.unitreeG1_mjx import MjxUnitreeG1
    from loco_mujoco.trajectory import Trajectory

    from .yoga_traj import FULL_FLOW

    trajectory = Trajectory.load(str(args.traj))
    qpos = np.asarray(trajectory.data.qpos)
    frequency = trajectory.info.frequency

    env = MjxUnitreeG1(headless=True)
    model, data = env.get_model(), env.get_data()
    renderer = mujoco.Renderer(model, height=480, width=640)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.lookat[:] = [0.0, 0.0, 0.6]
    camera.distance = 2.8
    camera.azimuth = 135.0
    camera.elevation = -15.0

    args.out_dir.mkdir(parents=True, exist_ok=True)
    settle_frames = int(round(args.settle * frequency))
    segment_frames = int(round((args.glide + args.hold) * frequency))
    for index, pose in enumerate(FULL_FLOW):
        frame = settle_frames + index * segment_frames + int(round((args.glide + args.hold / 2) * frequency))
        frame = min(frame, len(qpos) - 1)
        data.qpos[:] = qpos[frame]
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera)
        path = args.out_dir / f"{index:02d}_{pose}.png"
        Image.fromarray(renderer.render()).save(path)
        pelvis_z = float(qpos[frame][2])
        print(f"{pose:14s} frame={frame:5d} pelvis_z={pelvis_z:.3f} -> {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

