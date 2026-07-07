"""Validate the sim-side (29-DOF) obs builder against training-side truth.

The Isaac->MuJoCo transfer failures documented in the research notes are
obs/action ordering and scaling mismatches. This harness catches them before
the Docker rebuild: for sampled trajectory frames it places BOTH models on the
reference state and compares

- deploy obs: 29-DOF model + injected mimic sites + g1_policy_runtime
- truth obs:  the LocoMuJoCo training env's own reset() observation

section by section (plain / current-site / reference-goal).
"""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path

from .eval_policy import fix_start
from .train_mimic import build_env

REPO_ROOT = Path(__file__).resolve().parents[3]

SIM_29DOF_XML = (REPO_ROOT / ".runtime/unitree-g1-mujoco/unitree_mujoco/"
                 "unitree_robots/g1/g1_29dof.xml")
RUNTIME_PATH = (REPO_ROOT / "overlays/unitree-g1-mujoco-protocol/python/"
                "g1_policy_runtime.py")

SECTIONS = [("plain", 0, 57), ("current-site", 57, 225), ("reference-goal", 225, 450)]


def load_runtime():
    spec = importlib.util.spec_from_file_location("g1_policy_runtime", RUNTIME_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--traj", type=Path, required=True)
    parser.add_argument("--frames", type=int, nargs="*", default=None)
    args = parser.parse_args()

    import jax
    import mujoco
    import numpy as np

    runtime = load_runtime()
    bundle = dict(np.load(args.bundle, allow_pickle=True))

    sim_spec = mujoco.MjSpec.from_file(str(SIM_29DOF_XML))
    runtime.inject_mimic_sites(sim_spec, bundle)
    sim_model = sim_spec.compile()
    sim_data = mujoco.MjData(sim_model)
    controller = runtime.YogaPolicyController(sim_model, bundle)

    env = build_env(args.traj, horizon=100000)
    n_frames = int(bundle["goal_ref"].shape[0])
    frames = args.frames or [0, 150, 400, 700, 1300, 1750, 2200, 2650, 3100, 3550, 4000]
    frames = [f for f in frames if f < n_frames]

    print(f"{'frame':>6s} " + " ".join(f"{name:>16s}" for name, _, _ in SECTIONS))
    worst = np.zeros(len(SECTIONS))
    for frame in frames:
        fix_start(env, frame)
        env.reset(jax.random.PRNGKey(0))
        # loco_mujoco's CPU reset computes the obs BEFORE mj_forward has seen
        # the trajectory state (site kinematics are stale); recompute the obs
        # on freshly-forwarded data for a valid comparison
        truth_model, truth_data = env.get_model(), env.get_data()
        mujoco.mj_forward(truth_model, truth_data)
        truth, _ = env._create_observation(truth_model, truth_data, env._additional_carry)
        truth = np.asarray(truth)

        controller.reset_to_frame(sim_data, frame)
        deploy = controller.build_obs(sim_data)

        diffs = []
        for i, (name, lo, hi) in enumerate(SECTIONS):
            diff = float(np.max(np.abs(truth[lo:hi] - deploy[lo:hi])))
            worst[i] = max(worst[i], diff)
            diffs.append(diff)
        print(f"{frame:6d} " + " ".join(f"{d:16.2e}" for d in diffs))

    print("worst  " + " ".join(f"{d:16.2e}" for d in worst))
    limits = {"plain": 1e-4, "current-site": 5e-3, "reference-goal": 1e-5}
    failed = [name for (name, _, _), d in zip(SECTIONS, worst) if d > limits[name]]
    if failed:
        raise SystemExit(f"OBS MISMATCH beyond tolerance in sections: {failed}")
    print("[validate] obs builder matches training-side truth within tolerance")


if __name__ == "__main__":
    main()
