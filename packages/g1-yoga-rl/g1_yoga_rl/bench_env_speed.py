from __future__ import annotations

import argparse
import time


def bench_mjx(num_envs: int, num_steps: int) -> float:
    import jax
    import jax.numpy as jnp
    from loco_mujoco.environments.humanoids.unitreeG1_mjx import MjxUnitreeG1

    env = MjxUnitreeG1(headless=True)
    action_dim = env.info.action_space.shape[0]
    rng = jax.random.PRNGKey(0)
    reset_keys = jax.random.split(rng, num_envs)
    vreset = jax.jit(jax.vmap(env.mjx_reset))
    vstep = jax.jit(jax.vmap(env.mjx_step))
    state = vreset(reset_keys)
    action = jnp.zeros((num_envs, action_dim))
    state = vstep(state, action)
    jax.block_until_ready(state.observation)

    start = time.perf_counter()
    for _ in range(num_steps):
        state = vstep(state, action)
    jax.block_until_ready(state.observation)
    elapsed = time.perf_counter() - start
    total = num_envs * num_steps
    print(f"MJX  envs={num_envs:4d} steps={num_steps} -> {total / elapsed:9.0f} env-steps/s (dt={env.dt:.4f})")
    return total / elapsed


def bench_cpu(num_steps: int) -> float:
    import jax
    import numpy as np
    from loco_mujoco.environments.humanoids.unitreeG1 import UnitreeG1

    env = UnitreeG1(headless=True)
    action_dim = env.info.action_space.shape[0]
    env.reset(jax.random.PRNGKey(0))
    action = np.zeros(action_dim)

    start = time.perf_counter()
    for _ in range(num_steps):
        _obs, _reward, absorbing, done, _info = env.step(action)
        if done or absorbing:
            env.reset(jax.random.PRNGKey(0))
    elapsed = time.perf_counter() - start
    print(f"CPU  single env steps={num_steps} -> {num_steps / elapsed:9.0f} env-steps/s (dt={env.dt:.4f})")
    return num_steps / elapsed


def main() -> int:
    parser = argparse.ArgumentParser(description="Benchmark LocoMuJoCo UnitreeG1 stepping on CPU/MJX.")
    parser.add_argument("--mode", choices=["mjx", "cpu", "both"], default="both")
    parser.add_argument("--num-envs", type=int, default=16)
    parser.add_argument("--num-steps", type=int, default=100)
    args = parser.parse_args()

    if args.mode in ("cpu", "both"):
        bench_cpu(max(args.num_steps, 300))
    if args.mode in ("mjx", "both"):
        bench_mjx(args.num_envs, args.num_steps)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

