from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Sanity-check a generated G1 yoga trajectory in LocoMuJoCo's imitation env.")
    parser.add_argument("--traj", type=Path, required=True)
    parser.add_argument("--mjx", action="store_true", help="Also step the MJX path.")
    args = parser.parse_args()

    import jax
    import jax.numpy as jnp
    import numpy as np
    from loco_mujoco.task_factories import CustomDatasetConf, ImitationFactory
    from loco_mujoco.trajectory import Trajectory

    trajectory = Trajectory.load(str(args.traj))
    env = ImitationFactory.make(
        "MjxUnitreeG1",
        custom_dataset_conf=CustomDatasetConf(trajectory),
        headless=True,
        goal_type="GoalTrajMimic",
        reward_type="MimicReward",
        reward_params=dict(
            qpos_w_sum=0.4,
            qvel_w_sum=0.2,
            rpos_w_sum=0.5,
            rquat_w_sum=0.3,
            rvel_w_sum=0.1,
        ),
    )
    print(f"obs dim: {env.info.observation_space.shape}, action dim: {env.info.action_space.shape}")

    obs = env.reset(jax.random.PRNGKey(1))
    action = np.zeros(env.info.action_space.shape[0])
    rewards = []
    for _ in range(50):
        obs, reward, absorbing, done, _info = env.step(action)
        rewards.append(float(reward))
        if done or absorbing:
            break
    print(
        f"CPU zero-action: steps={len(rewards)} reward mean={np.mean(rewards):.3f} "
        f"min={np.min(rewards):.3f} max={np.max(rewards):.3f} finite={np.all(np.isfinite(rewards))}"
    )
    if not np.all(np.isfinite(obs)):
        raise RuntimeError("non-finite observation")

    if args.mjx:
        env.th.to_jax()
        vreset = jax.jit(jax.vmap(env.mjx_reset))
        vstep = jax.jit(jax.vmap(env.mjx_step))
        keys = jax.random.split(jax.random.PRNGKey(2), 4)
        state = vreset(keys)
        actions = jnp.zeros((4, env.info.action_space.shape[0]))
        for _ in range(10):
            state = vstep(state, actions)
        jax.block_until_ready(state.observation)
        print(
            f"MJX zero-action: reward={np.asarray(state.reward)} "
            f"obs finite={bool(jnp.all(jnp.isfinite(state.observation)))}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

