"""Pack a deploy bundle for the Docker sim's yoga policy mode.

The bundle contains everything the sim-side runtime needs, with the entire
reference (trajectory) half of the GoalTrajMimic observation precomputed here
using loco_mujoco's own functions. The sim only ever computes the
current-state site quantities. This kills the largest class of cross-runtime
obs mismatches by construction.

Bundle contents (npz):
- policy: w0/b0..w2/b2, obs_mean/obs_var (frozen), act_mean/act_delta
- goal_ref[N, 225]: per-frame [ref_qpos(28), ref_qvel(29), site_rpos(42),
  site_rangles(42), site_rvel(84)] - the traj_goal_obs layout
- ref_qpos[N, 30], ref_qvel[N, 29]: raw trajectory state (for teleport resets)
- joint_names[24], actuator_names[23], sites: names/parent bodies/pos/quat
- control_dt
"""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=Path, required=True, help="exported policy npz")
    parser.add_argument("--traj", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    import mujoco
    import numpy as np

    from loco_mujoco.core.utils.math import calculate_relative_site_quatities
    from loco_mujoco.core.utils.mujoco import mj_jntid2qposid, mj_jntid2qvelid
    from loco_mujoco.trajectory import Trajectory

    from .cyber_env import make_cyber_env

    policy = dict(np.load(args.policy, allow_pickle=True))
    trajectory = Trajectory.load(str(args.traj))
    traj_data = trajectory.data

    env = make_cyber_env(headless=True)
    model = env.get_model()

    # site table for injection into the 29-DOF sim model
    site_names = list(env.sites_for_mimic)
    site_ids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, n) for n in site_names])
    site_body_ids = model.site_bodyid[site_ids]
    site_parent_bodies = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(b)) for b in site_body_ids]
    site_pos = model.site_pos[site_ids].copy()
    site_quat = model.site_quat[site_ids].copy()

    # GoalTrajMimic's qpos/qvel reorderings
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, env.root_free_joint_xml_name)
    qpos_ind = np.concatenate(
        [mj_jntid2qposid(i, model)[2:] for i in range(model.njnt) if i == root_id]
        + [mj_jntid2qposid(i, model) for i in range(model.njnt) if i != root_id]
    )
    qvel_ind = np.concatenate([mj_jntid2qvelid(i, model) for i in range(model.njnt)])

    n_frames = int(traj_data.qpos.shape[0])
    rel_body_ids = model.site_bodyid[site_ids]
    n_hinge = model.njnt - 1
    n_rel_sites = len(site_names) - 1
    goal_dim = (5 + n_hinge) + (6 + n_hinge) + 12 * n_rel_sites
    goal_ref = np.zeros((n_frames, goal_dim), dtype=np.float32)
    for k in range(n_frames):
        frame = traj_data.get(0, k, np)
        rpos, rangles, rvel = calculate_relative_site_quatities(
            frame, site_ids, rel_body_ids, model.body_rootid, np)
        goal_ref[k] = np.concatenate([
            np.asarray(frame.qpos)[qpos_ind],
            np.asarray(frame.qvel)[qvel_ind],
            np.ravel(rpos), np.ravel(rangles), np.ravel(rvel),
        ])

    joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]

    # joint driven by each actuator, in action order; the sim maps actuators
    # across models by joint name, not actuator name
    action_indices = np.asarray(env._action_indices)
    act_joint_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(model.actuator_trnid[int(i), 0]))
        for i in action_indices
    ]

    bundle = {
        "goal_ref": goal_ref,
        "ref_qpos": np.asarray(traj_data.qpos, dtype=np.float32),
        "ref_qvel": np.asarray(traj_data.qvel, dtype=np.float32),
        "joint_names": np.array(joint_names),
        "site_names": np.array(site_names),
        "site_parent_bodies": np.array(site_parent_bodies),
        "site_pos": site_pos,
        "site_quat": site_quat,
        "act_joint_names": np.array(act_joint_names),
        "frequency": np.asarray(trajectory.info.frequency),
    }
    for key in ("obs_mean", "obs_var", "n_layers", "w0", "b0", "w1", "b1", "w2", "b2",
                "act_mean", "act_delta", "actuator_names", "obs_joint_names", "control_dt"):
        bundle[key] = policy[key]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, **bundle)
    size_mb = args.out.stat().st_size / 1e6
    print(f"[pack] wrote {args.out} ({size_mb:.1f} MB, {n_frames} frames, "
          f"{len(site_names)} sites)")


if __name__ == "__main__":
    main()
