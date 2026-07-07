"""Train the G1 yoga mimic policy with LocoMuJoCo's PPOJax on the MJX CPU path.

Lean version of examples/training_examples/jax_rl_mimic/experiment.py: no
wandb, no hydra — an inline OmegaConf config sized for the M5's CPU (moderate
env counts; the whole train loop is jitted, ~5k env-steps/s).

Usage:
    g1-yoga-train --traj .runtime/g1-yoga-rl/yoga_trajectory.npz \
        --out .runtime/g1-yoga-rl/run1 --total-timesteps 10e6
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

# MimicReward weights from the jax_rl_mimic example conf.
REWARD_PARAMS = dict(
    qpos_w_sum=0.4,
    qvel_w_sum=0.2,
    rpos_w_sum=0.5,
    rquat_w_sum=0.3,
    rvel_w_sum=0.1,
)


def build_env(traj_path: Path, horizon: int = 1000):
    from loco_mujoco.task_factories import CustomDatasetConf, ImitationFactory
    from loco_mujoco.trajectory import Trajectory

    trajectory = Trajectory.load(str(traj_path))
    env = ImitationFactory.make(
        "MjxUnitreeG1",
        custom_dataset_conf=CustomDatasetConf(trajectory),
        headless=True,
        horizon=horizon,
        goal_type="GoalTrajMimic",
        goal_params=dict(visualize_goal=False),
        reward_type="MimicReward",
        reward_params=dict(REWARD_PARAMS),
    )
    # custom trajectories load as numpy; the MJX training path needs jax arrays
    env.th.to_jax()
    return env


def build_config(args):
    from omegaconf import OmegaConf

    return OmegaConf.create(
        {
            "experiment": {
                "task_factory": {"name": "ImitationFactory"},
                "env_params": {"env_name": "MjxUnitreeG1"},
                "hidden_layers": [512, 256],
                "lr": args.lr,
                "num_envs": args.num_envs,
                "num_steps": args.num_steps,
                "total_timesteps": args.total_timesteps,
                "update_epochs": 4,
                "proportion_env_reward": 0.0,
                "num_minibatches": args.num_minibatches,
                "gamma": 0.99,
                "gae_lambda": 0.95,
                "clip_eps": 0.2,
                "init_std": 0.2,
                "learnable_std": False,
                "ent_coef": 0.0,
                "vf_coef": 0.5,
                "max_grad_norm": 0.5,
                "activation": "tanh",
                "anneal_lr": False,
                "weight_decay": 0.0,
                "normalize_env": True,
                "debug": False,
                "n_seeds": 1,
                "vmap_across_seeds": False,
                "validation": {
                    "active": False,
                    "num": 1,
                    "num_steps": 100,
                    "num_envs": 10,
                },
            },
        }
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Train an experimental LocoMuJoCo PPOJax mimic policy for G1 yoga trajectories.")
    parser.add_argument("--traj", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--total-timesteps", type=float, default=10e6)
    parser.add_argument("--chunks", type=int, default=10,
                        help="train in N chunks, saving after each; the best-return chunk is kept "
                             "so a late PPO collapse cannot destroy the run")
    parser.add_argument("--num-envs", type=int, default=64)
    parser.add_argument("--num-steps", type=int, default=100)
    parser.add_argument("--num-minibatches", type=int, default=8)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--horizon", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    args.total_timesteps = int(args.total_timesteps) // args.chunks

    import jax
    import numpy as np

    from loco_mujoco.algorithms import PPOJax

    print(f"[train] jax backend={jax.default_backend()} devices={jax.devices()}")
    env = build_env(args.traj, horizon=args.horizon)
    print(f"[train] obs={env.info.observation_space.shape} act={env.info.action_space.shape}")

    config = build_config(args)
    agent_conf = PPOJax.init_agent_conf(env, config)
    train_fn = jax.jit(PPOJax.build_train_fn(env, agent_conf, mh=None))
    resume_fn = jax.jit(PPOJax.build_resume_train_fn(env, agent_conf, mh=None))

    updates = config.experiment.num_updates
    samples_per_update = args.num_envs * args.num_steps
    print(f"[train] {args.chunks} chunks x {args.total_timesteps:,} steps "
          f"({updates} updates x {samples_per_update} samples each)")

    args.out.mkdir(parents=True, exist_ok=True)
    rng = jax.random.PRNGKey(args.seed)
    agent_state = None
    best_return = -np.inf
    start = time.perf_counter()
    for chunk in range(args.chunks):
        rng, chunk_rng = jax.random.split(rng)
        out = train_fn(chunk_rng) if agent_state is None else resume_fn(chunk_rng, agent_state)
        agent_state = out["agent_state"]
        jax.block_until_ready(agent_state.train_state.params)

        metrics = out["training_metrics"]
        returns = np.asarray(metrics.mean_episode_return)
        lengths = np.asarray(metrics.mean_episode_length)
        tail = max(1, len(returns) // 5)
        tail_return = float(np.mean(returns[-tail:]))
        elapsed = time.perf_counter() - start
        print(f"[chunk {chunk + 1}/{args.chunks}] tail return={tail_return:8.2f} "
              f"length={float(np.mean(lengths[-tail:])):6.1f} "
              f"(elapsed {elapsed / 60.0:.1f} min)", flush=True)

        if tail_return > best_return:
            best_return = tail_return
            save_path = PPOJax.save_agent(str(args.out), agent_conf, agent_state)
            print(f"[chunk {chunk + 1}] new best -> {save_path}", flush=True)

    print(f"[train] done in {(time.perf_counter() - start) / 60.0:.1f} min; "
          f"best tail return {best_return:.2f}")


if __name__ == "__main__":
    main()
