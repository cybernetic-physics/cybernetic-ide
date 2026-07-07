"""Build a LocoMuJoCo Trajectory for the G1 yoga flow from the sim's NAMED_POSES.

The poses live in the Docker sim script (overlays/.../g1_protocol_sim.py) as
{joint_name: radians} dicts for the 29-DOF G1. Here they are projected onto the
LocoMuJoCo UnitreeG1 (23-DOF) joint set, glued into a glide+hold sequence with
smoothstep interpolation (the same easing the sim's animate_to_pose uses), and
extended with body/site kinematics so GoalTrajMimic / MimicReward can consume
the result.
"""

import ast
from dataclasses import dataclass
from pathlib import Path

import jax.numpy as jnp
import mujoco
import numpy as np

from loco_mujoco.trajectory import (
    Trajectory,
    TrajectoryData,
    TrajectoryInfo,
    TrajectoryModel,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
SIM_SCRIPT = REPO_ROOT / "overlays/unitree-g1-mujoco-protocol/python/g1_protocol_sim.py"

# The sim's yoga flow order (examples/yoga_teacher.py); mountain acts as the
# neutral anchor the flow starts from.
FULL_FLOW = [
    "mountain",
    "upward_salute",
    "forward_fold",
    "chair",
    "warrior_one",
    "warrior_two",
    "goddess",
    "tree",
    "namaste",
]
STABLE_FLOW = ["mountain", "upward_salute", "namaste"]

# Training-side reference adjustments for poses that don't survive projection
# onto the 23-DOF joint set. forward_fold relies on waist_pitch (absent here):
# with an upright base, hip flexion 1.4 grounds as a seated pose that only-feet
# contacts can never support. Instead the base itself pitches forward while the
# hips flex to keep the legs vertical and the ankles keep the feet flat
# (foot pitch ~= base_pitch - hip_pitch + knee + ankle).
POSE_OVERRIDES = {
    "forward_fold": {
        "base_pitch": 1.15,
        "joints": {
            "left_hip_pitch_joint": 1.3,
            "right_hip_pitch_joint": 1.3,
            "left_knee_joint": 0.25,
            "right_knee_joint": 0.25,
            "left_ankle_pitch_joint": -0.10,
            "right_ankle_pitch_joint": -0.10,
            "left_shoulder_pitch_joint": 0.25,
            "right_shoulder_pitch_joint": 0.25,
        },
    },
}


def load_named_poses(sim_script: Path = SIM_SCRIPT) -> dict:
    """Parse NAMED_POSES out of the sim script without importing it.

    The sim script imports msgpack/websockets/PIL at module scope, which are
    not installed in the training venv, so the dict is read via ast instead.
    """
    tree = ast.parse(sim_script.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "NAMED_POSES":
                    return ast.literal_eval(node.value)
    raise ValueError(f"NAMED_POSES not found in {sim_script}")


@dataclass
class PoseProjection:
    """A named pose projected onto the training model's joint set."""

    name: str
    qpos: np.ndarray
    skipped_joints: list


def smoothstep(fraction: np.ndarray) -> np.ndarray:
    return fraction * fraction * (3.0 - 2.0 * fraction)


def project_pose(model, base_qpos: np.ndarray, name: str, targets: dict) -> PoseProjection:
    """Set the pose's joint angles on a copy of base_qpos, clamped to jnt_range.

    Joints missing from the training model (waist pitch/roll, wrist pitch/yaw)
    are recorded as skipped rather than raising.
    """
    qpos = base_qpos.copy()
    skipped = []
    override = POSE_OVERRIDES.get(name)
    if override is not None:
        targets = {**targets, **override["joints"]}
        base_pitch = override.get("base_pitch", 0.0)
        if base_pitch:
            quat_adr = int(model.jnt_qposadr[0]) + 3
            half = 0.5 * base_pitch
            qpos[quat_adr:quat_adr + 4] = [np.cos(half), 0.0, np.sin(half), 0.0]
    for joint_name, value in targets.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            skipped.append(joint_name)
            continue
        low, high = model.jnt_range[joint_id]
        qpos[model.jnt_qposadr[joint_id]] = float(np.clip(value, low, high))
    return PoseProjection(name=name, qpos=qpos, skipped_joints=skipped)


def ground_qpos(model, data, qpos: np.ndarray, penetration_tol: float = 1e-4) -> float:
    """Return the root z that seats the pose's lowest colliding geom on the floor.

    Binary search on the free-joint z: contacts (foot<->floor pairs in the MJX
    spec) report negative dist when penetrating, and disappear when clear, so
    the boundary is the touching height.
    """
    base_z = int(model.jnt_qposadr[0]) + 2

    def penetration(z: float) -> float:
        data.qpos[:] = qpos
        data.qpos[base_z] = z
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        if data.ncon == 0:
            return np.inf
        return float(np.min(data.contact.dist[: data.ncon]))

    low, high = qpos[base_z] - 1.0, qpos[base_z] + 0.5
    if penetration(low) > 0:
        raise RuntimeError("floor not found below pose; model has no foot contacts")
    for _ in range(40):
        mid = 0.5 * (low + high)
        if penetration(mid) < -penetration_tol:
            low = mid
        else:
            high = mid
    return low


def build_yoga_trajectory(
    env,
    pose_names: list,
    glide_seconds: float = 1.5,
    hold_seconds: float = 3.0,
    settle_seconds: float = 1.0,
) -> Trajectory:
    """Assemble the glide+hold qpos sequence and extend it with kinematics.

    The sequence starts at the env default (standing) pose, holds it for
    settle_seconds, then glides into each pose in order and holds it.
    """
    model = env.get_model()
    data = env.get_data()
    dt = env.dt

    named_poses = load_named_poses()
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    base_qpos = data.qpos.copy()

    projections = []
    for name in pose_names:
        if name not in named_poses:
            raise KeyError(f"pose '{name}' not in NAMED_POSES ({sorted(named_poses)})")
        projections.append(project_pose(model, base_qpos, name, named_poses[name]))
    for projection in projections:
        if projection.skipped_joints:
            print(f"[traj] {projection.name}: skipped joints missing from model: "
                  f"{projection.skipped_joints}")

    glide_frames = max(1, int(round(glide_seconds / dt)))
    hold_frames = max(1, int(round(hold_seconds / dt)))
    settle_frames = max(1, int(round(settle_seconds / dt)))

    keyframes = [base_qpos] + [p.qpos for p in projections]
    segments = [settle_frames] + [glide_frames + hold_frames] * len(projections)

    qpos_frames = []
    current = keyframes[0]
    qpos_frames.extend([current.copy() for _ in range(settle_frames)])
    for target in keyframes[1:]:
        fractions = smoothstep(np.linspace(0.0, 1.0, glide_frames, endpoint=False))
        for fraction in fractions:
            qpos_frames.append(current + (target - current) * fraction)
        qpos_frames.extend([target.copy() for _ in range(hold_frames)])
        current = target
    qpos = np.stack(qpos_frames)
    assert len(qpos) == sum(segments)

    base_z = int(model.jnt_qposadr[0]) + 2
    for i in range(len(qpos)):
        qpos[i, base_z] = ground_qpos(model, data, qpos[i])

    # qvel by central differences; boundary frames copy their neighbors. The
    # root quat is identity throughout, so angular velocity stays zero.
    nv = model.nv
    qvel = np.zeros((len(qpos), nv))
    dq = (qpos[2:] - qpos[:-2]) / (2.0 * dt)
    # free joint: linear velocity from xyz differences, angular stays zero
    qvel[1:-1, 0:3] = dq[:, 0:3]
    # hinge joints map 1:1 behind the free joint (qpos 7:, qvel 6:)
    qvel[1:-1, 6:] = dq[:, 7:]
    qvel[0] = qvel[1]
    qvel[-1] = qvel[-2]

    return extend_and_pack(model, data, qpos, qvel, frequency=1.0 / dt)


def extend_and_pack(model, data, qpos: np.ndarray, qvel: np.ndarray, frequency: float) -> Trajectory:
    """Run FK over every frame and pack the complete Trajectory.

    Mirrors loco_mujoco's ExtendTrajData recorder: GoalTrajMimic and
    MimicReward need per-frame xpos/xquat/cvel/subtree_com/site_xpos/site_xmat
    plus the model constants in TrajectoryInfo.
    """
    n_frames = len(qpos)
    n_bodies, n_sites = model.nbody, model.nsite

    xpos = np.zeros((n_frames, n_bodies, 3))
    xquat = np.zeros((n_frames, n_bodies, 4))
    cvel = np.zeros((n_frames, n_bodies, 6))
    subtree_com = np.zeros((n_frames, n_bodies, 3))
    site_xpos = np.zeros((n_frames, n_sites, 3))
    site_xmat = np.zeros((n_frames, n_sites, 9))

    for i in range(n_frames):
        data.qpos[:] = qpos[i]
        data.qvel[:] = qvel[i]
        mujoco.mj_forward(model, data)
        xpos[i] = data.xpos
        xquat[i] = data.xquat
        cvel[i] = data.cvel
        subtree_com[i] = data.subtree_com
        site_xpos[i] = data.site_xpos
        site_xmat[i] = data.site_xmat.reshape(n_sites, 9)

    jnt_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    body_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(n_bodies)]
    site_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i) for i in range(n_sites)]

    traj_model = TrajectoryModel(
        njnt=model.njnt,
        jnt_type=jnp.array(model.jnt_type),
        nbody=n_bodies,
        body_rootid=jnp.array(model.body_rootid),
        body_weldid=jnp.array(model.body_weldid),
        body_mocapid=jnp.array(model.body_mocapid),
        body_pos=jnp.array(model.body_pos),
        body_quat=jnp.array(model.body_quat),
        body_ipos=jnp.array(model.body_ipos),
        body_iquat=jnp.array(model.body_iquat),
        nsite=n_sites,
        site_bodyid=jnp.array(model.site_bodyid),
        site_pos=jnp.array(model.site_pos),
        site_quat=jnp.array(model.site_quat),
    )
    traj_info = TrajectoryInfo(
        joint_names=jnt_names,
        model=traj_model,
        frequency=frequency,
        body_names=body_names,
        site_names=site_names,
    )
    traj_data = TrajectoryData(
        qpos=jnp.array(qpos),
        qvel=jnp.array(qvel),
        xpos=jnp.array(xpos),
        xquat=jnp.array(xquat),
        cvel=jnp.array(cvel),
        subtree_com=jnp.array(subtree_com),
        site_xpos=jnp.array(site_xpos),
        site_xmat=jnp.array(site_xmat),
        split_points=jnp.array([0, n_frames]),
    )
    return Trajectory(traj_info, traj_data)
