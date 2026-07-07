"""Numerically solve ankle pitch/roll (and lunge leg reach) for the yoga poses.

For each pose, projects it onto the training model, then Newton-iterates
directly against the model:
- ankle_pitch / ankle_roll per planted foot, zeroing the sole's tilt
- one reach joint for lunges (rear/straight knee), zeroing the clearance gap
  between the two feet

Prints the solved joint values to copy back into NAMED_POSES. Sign conventions
come from measured numeric derivatives, not assumptions.
"""

from __future__ import annotations

import mujoco
import numpy as np

from .yoga_traj import FULL_FLOW, ground_qpos, load_named_poses, project_pose

# pose -> joint used to equalize the two feet's floor clearance
REACH_JOINTS = {
    "warrior_one": "right_knee_joint",
    "warrior_two": "left_knee_joint",
}
PLANTED_CLEARANCE = 0.06  # feet starting within 6 cm of the floor are meant to plant
SOLVE_POSES = ["forward_fold", "chair", "warrior_one", "warrior_two", "goddess"]


def qadr(model, joint_name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise KeyError(joint_name)
    return int(model.jnt_qposadr[joint_id])


def foot_metrics(model, data, qpos, side: str):
    """(pitch_tilt, roll_tilt, clearance) of a foot for a candidate qpos."""
    data.qpos[:] = qpos
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_ankle_roll_link")
    rotation = data.xmat[body].reshape(3, 3)
    pitch = float(np.arctan2(rotation[0, 2], rotation[2, 2]))
    roll = float(np.arctan2(-rotation[1, 2], rotation[2, 2]))
    bottoms = []
    for i in range(1, 5):
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_foot_{i}_col")
        bottoms.append(data.geom_xpos[geom][2] - model.geom_rbound[geom])
    return pitch, roll, float(np.min(bottoms))


def newton_zero(qpos, adr: int, model, data, measure, iterations: int = 4, step: float = 0.05):
    """Adjust qpos[adr] until measure(qpos) ~ 0 using numeric derivatives."""
    for _ in range(iterations):
        value = measure(qpos)
        if abs(value) < 1e-3:
            break
        perturbed = qpos.copy()
        perturbed[adr] += step
        gradient = (measure(perturbed) - value) / step
        if abs(gradient) < 1e-6:
            break
        low, high = model.jnt_range[np.searchsorted(model.jnt_qposadr, adr)]
        qpos[adr] = float(np.clip(qpos[adr] - value / gradient, low, high))
    return qpos


def main() -> None:
    from loco_mujoco.environments.humanoids.unitreeG1_mjx import MjxUnitreeG1

    env = MjxUnitreeG1(headless=True)
    model, data = env.get_model(), env.get_data()

    named_poses = load_named_poses()
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    base_qpos = data.qpos.copy()

    for pose_name in SOLVE_POSES:
        projection = project_pose(model, base_qpos, pose_name, named_poses[pose_name])
        qpos = projection.qpos.copy()
        base_z = int(model.jnt_qposadr[0]) + 2

        def grounded(candidate):
            candidate = candidate.copy()
            candidate[base_z] = ground_qpos(model, data, candidate)
            return candidate

        qpos = grounded(qpos)
        planted = []
        for side in ("left", "right"):
            _, _, clearance = foot_metrics(model, data, qpos, side)
            if clearance < PLANTED_CLEARANCE:
                planted.append(side)

        # equalize reach first so both feet can plant, then flatten soles;
        # repeat once since the corrections interact
        for _ in range(2):
            reach_joint = REACH_JOINTS.get(pose_name)
            if reach_joint is not None:
                adr = qadr(model, reach_joint)

                def clearance_gap(candidate):
                    candidate = grounded(candidate)
                    _, _, left_clr = foot_metrics(model, data, candidate, "left")
                    _, _, right_clr = foot_metrics(model, data, candidate, "right")
                    return left_clr - right_clr

                qpos = newton_zero(qpos, adr, model, data, clearance_gap)
                qpos = grounded(qpos)
                planted = ["left", "right"]

            for side in planted:
                for axis, index in (("pitch", 0), ("roll", 1)):
                    joint_name = f"{side}_ankle_{axis}_joint"
                    adr = qadr(model, joint_name)

                    def tilt(candidate, side=side, index=index):
                        candidate = grounded(candidate)
                        metrics = foot_metrics(model, data, candidate, side)
                        return metrics[index]

                    qpos = newton_zero(qpos, adr, model, data, tilt)
            qpos = grounded(qpos)

        print(f"\n[{pose_name}] solved joint values:")
        for side in ("left", "right"):
            pitch, roll, clearance = foot_metrics(model, data, qpos, side)
            print(
                f"  {side}: tilt=({np.degrees(pitch):.1f},{np.degrees(roll):.1f})deg "
                f"clearance={clearance * 100:.1f}cm"
            )
        tuned_joints = [f"{s}_ankle_{a}_joint" for s in ("left", "right") for a in ("pitch", "roll")]
        tuned_joints += list(filter(None, [REACH_JOINTS.get(pose_name)]))
        for joint in sorted(set(tuned_joints)):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if joint_id >= 0:
                print(f"    \"{joint}\": {qpos[model.jnt_qposadr[joint_id]]:.2f},")


if __name__ == "__main__":
    main()
