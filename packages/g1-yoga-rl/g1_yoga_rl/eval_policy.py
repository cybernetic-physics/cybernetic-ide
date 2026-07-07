"""Evaluate an exported (numpy) yoga policy in the LocoMuJoCo CPU env.

Two modes, both using the exact numpy inference path the sim deployment uses:

- per-pose: start each episode at the beginning of a pose's glide window and
  run through the end of its hold; report survived / fell + mean reward.
- full flow: one rollout from frame 0 through the whole trajectory.

"Fell" means the episode terminated early (RootPoseTrajTerminalStateHandler
kills the episode when the root deviates from the reference) or the pelvis
dropped below 0.35 m.
"""

from __future__ import annotations

import argparse
from pathlib import Path

SETTLE_S = 1.0
GLIDE_S = 1.5
HOLD_S = 3.0
MIN_PELVIS_Z = 0.35


def pose_windows(frequency: float, full_flow: list[str] | None = None) -> list:
    """(pose, start_frame, end_frame) for each pose's glide+hold segment."""
    if full_flow is None:
        from .yoga_traj import FULL_FLOW
        full_flow = FULL_FLOW
    settle = int(round(SETTLE_S * frequency))
    segment = int(round((GLIDE_S + HOLD_S) * frequency))
    return [(pose, settle + i * segment, settle + (i + 1) * segment) for i, pose in enumerate(full_flow)]


def fix_start(env, frame: int) -> None:
    env.th.random_start = False
    env.th.fixed_start_conf = (0, int(frame))
    env.th.use_fixed_start = True


def rollout(env, policy, start_frame: int, num_steps: int, renderer=None, camera=None,
            render_at=None, render_path=None):
    """Run the policy for num_steps from start_frame; returns (steps_survived, rewards)."""
    import jax
    import numpy as np

    from .export_policy import numpy_policy_action

    fix_start(env, start_frame)
    obs = env.reset(jax.random.PRNGKey(0))
    rewards = []
    data = env.get_data()
    for step in range(num_steps):
        action = numpy_policy_action(policy, obs)
        obs, reward, absorbing, done, info = env.step(np.asarray(action))
        rewards.append(float(reward))
        pelvis_z = float(data.qpos[2])
        if renderer is not None and render_at is not None and step == render_at:
            renderer.update_scene(data, camera=camera)
            from PIL import Image
            Image.fromarray(renderer.render()).save(render_path)
        if absorbing or done or pelvis_z < MIN_PELVIS_Z:
            if step < num_steps - 1:
                return step + 1, rewards
    return num_steps, rewards


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=Path, required=True, help="exported policy npz")
    parser.add_argument("--traj", type=Path, required=True)
    parser.add_argument("--render-dir", type=Path, default=None)
    parser.add_argument("--skip-full-flow", action="store_true")
    args = parser.parse_args()

    import mujoco
    import numpy as np

    from .train_mimic import build_env

    policy = dict(np.load(args.policy, allow_pickle=True))
    env = build_env(args.traj, horizon=100000)
    frequency = 1.0 / env.dt
    windows = pose_windows(frequency)
    total_frames = int(env.th.traj.data.qpos.shape[0])

    renderer = camera = None
    if args.render_dir is not None:
        args.render_dir.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(env.get_model(), height=480, width=640)
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.lookat[:] = [0.0, 0.0, 0.55]
        camera.distance = 2.8
        camera.azimuth = 135.0
        camera.elevation = -12.0

    print(f"{'pose':15s} {'survived':>9s} {'steps':>11s} {'mean_reward':>11s}")
    held = 0
    for index, (pose, start, end) in enumerate(windows):
        num_steps = end - start
        render_at = int((GLIDE_S + HOLD_S / 2) * frequency) if renderer else None
        render_path = (args.render_dir / f"pose_{index:02d}_{pose}.png") if renderer else None
        steps, rewards = rollout(env, policy, start, num_steps,
                                 renderer, camera, render_at, render_path)
        survived = steps == num_steps
        held += int(survived)
        print(f"{pose:15s} {str(survived):>9s} {steps:6d}/{num_steps:4d} {np.mean(rewards):11.3f}")
    print(f"[eval] per-pose: {held}/{len(windows)} held")

    if not args.skip_full_flow:
        num_steps = total_frames - 2
        steps, rewards = rollout(env, policy, 0, num_steps)
        print(f"[eval] full flow: {steps}/{num_steps} steps "
              f"({steps / frequency:.1f}s of {num_steps / frequency:.1f}s), "
              f"mean reward {np.mean(rewards):.3f}")


if __name__ == "__main__":
    main()
