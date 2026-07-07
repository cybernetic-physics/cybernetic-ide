"""Export a trained PPOJax agent to a plain-numpy npz for sim deployment.

The Docker sim image has numpy but no JAX/flax, so the deterministic actor is
exported as raw arrays:

- obs normalization: run_stats mean/var (frozen at export)
- actor MLP: obs_norm -> Dense(512) tanh -> Dense(256) tanh -> Dense(23)
- action scaling: ctrl = act_mean + action * act_delta per actuator
  (DefaultControl's [-1, 1] normalization over the actuator ctrlrange)

Also embeds the actuator name order, plain-obs layout, and mimic site names so
the deploy-side obs builder can be checked against training-side truth.

Includes a parity test: numpy forward pass vs the flax network on random obs.
"""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import numpy as np


def numpy_policy_action(policy: dict, obs: np.ndarray) -> np.ndarray:
    """Deterministic actor forward pass using exported arrays ([-1,1] output)."""
    x = (obs - policy["obs_mean"]) / np.sqrt(policy["obs_var"] + 1e-8)
    for i in range(int(policy["n_layers"])):
        x = x @ policy[f"w{i}"] + policy[f"b{i}"]
        if i < int(policy["n_layers"]) - 1:
            x = np.tanh(x)
    return x


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--agent", type=Path, required=True, help="PPOJax_saved.pkl")
    parser.add_argument("--out", type=Path, required=True, help="output npz path")
    args = parser.parse_args()

    import jax
    import jax.numpy as jnp
    import mujoco

    from loco_mujoco.environments.humanoids.unitreeG1_mjx import MjxUnitreeG1

    with open(args.agent, "rb") as f:
        saved = pickle.load(f)

    train_state = saved["agent_state"]["train_state"]
    params = train_state["params"]
    run_stats = train_state["run_stats"]["RunningMeanStd_0"]
    actor = params["FullyConnectedNet_0"]

    layers = sorted(actor.keys())  # Dense_0, Dense_1, Dense_2
    export = {
        "obs_mean": np.asarray(run_stats["mean"]),
        "obs_var": np.asarray(run_stats["var"]),
        "n_layers": np.asarray(len(layers)),
    }
    for i, layer in enumerate(layers):
        export[f"w{i}"] = np.asarray(actor[layer]["kernel"])
        export[f"b{i}"] = np.asarray(actor[layer]["bias"])

    # action scaling + name orders from the training env's model
    env = MjxUnitreeG1(headless=True)
    model = env.get_model()
    action_indices = np.asarray(env._action_indices)
    low = model.actuator_ctrlrange[action_indices, 0]
    high = model.actuator_ctrlrange[action_indices, 1]
    export["act_mean"] = (high + low) / 2.0
    export["act_delta"] = (high - low) / 2.0
    export["actuator_names"] = np.array(
        [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(i)) for i in action_indices]
    )
    export["obs_joint_names"] = np.array([
        obs_type.xml_name for obs_type in env.obs_container.list_all()
        if type(obs_type).__name__ == "JointPos"
    ])
    export["sites_for_mimic"] = np.array(list(env.sites_for_mimic))
    export["control_dt"] = np.asarray(env.dt)

    # parity check: flax apply vs numpy forward
    network = saved["agent_conf"]["network"]
    rng = np.random.default_rng(0)
    obs_dim = export["obs_mean"].shape[0]
    test_obs = rng.normal(size=(16, obs_dim)).astype(np.float32)
    (pi, _), _ = network.apply(
        {"params": params, "run_stats": {"RunningMeanStd_0": run_stats}},
        jnp.asarray(test_obs), mutable=["run_stats"],
    )
    jax_actions = np.asarray(pi.mode())
    numpy_actions = np.stack([numpy_policy_action(export, o) for o in test_obs])
    worst = float(np.max(np.abs(jax_actions - numpy_actions)))
    print(f"[export] parity max|jax - numpy| = {worst:.2e}")
    if worst > 5e-4:
        raise SystemExit("parity FAILED: numpy forward does not match flax network")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.out, **export)
    print(f"[export] wrote {args.out} "
          f"({export['obs_mean'].shape[0]}-d obs, {export['act_mean'].shape[0]}-d action)")


if __name__ == "__main__":
    main()
