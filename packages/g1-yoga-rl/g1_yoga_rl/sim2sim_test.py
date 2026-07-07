"""Closed-loop sim2sim test: run the deploy bundle on the 29-DOF sim scene.

This is the exact control loop the Docker sim's policy mode should run: same
runtime module, same model, same two-rate cadence (policy at the trajectory
frequency over physics at 0.002 s). It is executed locally so the sim2sim dynamics
gap is measured BEFORE any container rebuild.

Reports per-pose survival like eval_policy, and optionally renders the pose
holds for visual QA.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from .eval_policy import GLIDE_S, HOLD_S, MIN_PELVIS_Z, pose_windows
from .validate_deploy import load_runtime

REPO_ROOT = Path(__file__).resolve().parents[3]

SCENE_29DOF_XML = (REPO_ROOT / ".runtime/unitree-g1-mujoco/unitree_mujoco/"
                   "unitree_robots/g1/scene_29dof.xml")
PHYSICS_DT = 0.002


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--render-dir", type=Path, default=None)
    parser.add_argument("--full-flow", action="store_true",
                        help="also run one uninterrupted rollout from frame 0")
    args = parser.parse_args()

    import mujoco
    import numpy as np

    runtime = load_runtime()
    bundle = dict(np.load(args.bundle, allow_pickle=True))

    spec = mujoco.MjSpec.from_file(str(SCENE_29DOF_XML))
    runtime.inject_mimic_sites(spec, bundle)
    model = spec.compile()
    model.opt.timestep = PHYSICS_DT
    data = mujoco.MjData(model)
    controller = runtime.YogaPolicyController(model, bundle)

    frequency = float(bundle["frequency"])
    substeps = int(round(1.0 / frequency / PHYSICS_DT))
    windows = pose_windows(frequency)

    renderer = camera = None
    if args.render_dir is not None:
        args.render_dir.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(model, height=480, width=640)
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.lookat[:] = [0.0, 0.0, 0.55]
        camera.distance = 2.8
        camera.azimuth = 135.0
        camera.elevation = -12.0

    def run(start_frame: int, num_ticks: int, render_at=None, render_path=None) -> int:
        controller.reset_to_frame(data, start_frame)
        for tick in range(num_ticks):
            controller.apply(data)
            for _ in range(substeps):
                controller.hold_extra_actuators(data)
                mujoco.mj_step(model, data)
            if renderer is not None and render_at is not None and tick == render_at:
                renderer.update_scene(data, camera=camera)
                from PIL import Image
                Image.fromarray(renderer.render()).save(render_path)
            if float(data.qpos[2]) < MIN_PELVIS_Z:
                return tick + 1
        return num_ticks

    print(f"{'pose':15s} {'survived':>9s} {'ticks':>11s}")
    held = 0
    for index, (pose, start, end) in enumerate(windows):
        ticks = end - start
        render_at = int((GLIDE_S + HOLD_S / 2) * frequency) if renderer else None
        render_path = (args.render_dir / f"sim2sim_{index:02d}_{pose}.png") if renderer else None
        survived_ticks = run(start, ticks, render_at, render_path)
        survived = survived_ticks == ticks
        held += int(survived)
        print(f"{pose:15s} {str(survived):>9s} {survived_ticks:6d}/{ticks:4d}")
    print(f"[sim2sim] per-pose: {held}/{len(windows)} held")

    if args.full_flow:
        total = int(bundle["goal_ref"].shape[0]) - 2
        survived_ticks = run(0, total)
        print(f"[sim2sim] full flow: {survived_ticks}/{total} ticks "
              f"({survived_ticks / frequency:.1f}s of {total / frequency:.1f}s)")


if __name__ == "__main__":
    main()
