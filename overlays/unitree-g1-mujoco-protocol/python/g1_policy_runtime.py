"""Yoga policy runtime for the G1 MuJoCo protocol sim.

Runs a LocoMuJoCo-trained mimic policy (exported as plain numpy arrays) inside
the 29-DOF sim. The policy was trained on the 23-DOF LocoMuJoCo G1 model, so:

- the bundle's reference half of the observation (trajectory qpos/qvel and
  relative site quantities) is precomputed at pack time with LocoMuJoCo's own
  functions and simply indexed here by frame;
- the current-state half is computed from this model's data using the mimic
  sites injected into the 29-DOF model at load (same names/positions as the
  training XML);
- actuators are mapped across models by the joint they drive; the six
  29-DOF-only actuators (wrist pitch/yaw, waist roll/pitch) are PD-held at 0.

The relative-site math mirrors loco_mujoco.core.utils.math (numpy backend)
exactly, including its frame conventions, so the policy sees the same
observation distribution it was trained on.
"""

import mujoco
import numpy as np


# The 23-DOF training model lumps the whole wrist+hand into one body; in the
# 29-DOF model the same frame is reached from the wrist_yaw link after the
# wrist pitch (+0.038 x) and yaw (+0.046 x) link offsets. Valid as long as
# wrist pitch/yaw stay at 0, which the policy's held actuators enforce.
FALLBACK_SITE_PARENTS = {
    "left_wrist_roll_rubber_hand": ("left_wrist_yaw_link", np.array([-0.084, 0.0, 0.0])),
    "right_wrist_roll_rubber_hand": ("right_wrist_yaw_link", np.array([-0.084, 0.0, 0.0])),
}


def inject_mimic_sites(spec, bundle):
    """Add the training model's mimic sites to a 29-DOF MjSpec (before compile)."""
    bodies = {body.name: body for body in spec.bodies}
    missing = []
    for name, parent, pos, quat in zip(
        bundle["site_names"], bundle["site_parent_bodies"],
        bundle["site_pos"], bundle["site_quat"],
    ):
        parent = str(parent)
        pos = np.asarray(pos, dtype=float)
        if parent not in bodies and parent in FALLBACK_SITE_PARENTS:
            parent, offset = FALLBACK_SITE_PARENTS[parent]
            pos = pos + offset
        body = bodies.get(parent)
        if body is None:
            missing.append((str(name), parent))
            continue
        site = body.add_site()
        site.name = str(name)
        site.pos = pos
        site.quat = np.asarray(quat, dtype=float)
        site.size = np.array([0.01, 0.01, 0.01])
        site.group = 4
    if missing:
        raise ValueError(f"mimic site parent bodies missing from model: {missing}")
    return spec


FOOT_CAPSULE_NAMES = [
    f"{side}_foot_{i}_col" for side in ("left", "right") for i in range(1, 5)
]


def name_foot_capsules(spec):
    """Name the 29-DOF model's foot collision capsules like the training model.

    The stock XML leaves them unnamed; explicit contact pairs need names.
    """
    for side in ("left", "right"):
        ankle = None
        for body in spec.bodies:
            if body.name == f"{side}_ankle_roll_link":
                ankle = body
        if ankle is None:
            raise ValueError(f"missing body {side}_ankle_roll_link")
        capsules = [g for g in ankle.geoms
                    if g.type in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_SPHERE)]
        if len(capsules) != 4:
            raise ValueError(f"{side} foot: expected 4 contact geoms, found {len(capsules)}")
        for i, geom in enumerate(capsules, start=1):
            if not geom.name:
                geom.name = f"{side}_foot_{i}_col"
    return spec


def reduce_robot_contacts(spec):
    """Restrict robot collisions to explicit foot-capsule<->floor pairs.

    Mirrors MjxUnitreeG1._modify_spec_for_mjx on the 29-DOF model: robot geoms
    stop colliding by contype/conaffinity and only the eight foot capsules
    contact the floor. Non-robot geoms (floor, scene objects) keep their
    default contype behavior.
    """
    floor = None
    for geom in spec.geoms:
        if geom.name == "floor":
            floor = geom
    if floor is None:
        raise ValueError("scene has no 'floor' geom")

    name_foot_capsules(spec)

    # every non-world body in the stock scene belongs to the robot
    for body in spec.bodies:
        if body.name == "world":
            continue
        for geom in body.geoms:
            geom.contype = 0
            geom.conaffinity = 0

    for name in FOOT_CAPSULE_NAMES:
        spec.add_pair(geomname1="floor", geomname2=name)
    return spec


def apply_training_solver_options(model):
    """Match the MJX training solver settings (iterations, EULERDAMP off)."""
    model.opt.iterations = 2
    model.opt.ls_iterations = 4
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_EULERDAMP


def _rotmat_to_rotvec(mat):
    """Rotation matrix (3,3) -> rotation vector, matching scipy's as_rotvec."""
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, mat.reshape(9))
    if quat[0] < 0:
        quat = -quat
    sin_half = float(np.linalg.norm(quat[1:]))
    if sin_half < 1e-12:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(sin_half, float(quat[0]))
    return quat[1:] / sin_half * angle


def relative_site_quantities(data, site_ids, parent_body_ids, body_rootid):
    """Current-state site rpos/rangles/rvel, mirroring loco_mujoco's math.

    Site world velocities come from the parent body cvel translated to the
    site position (world frame); relative quantities are expressed against the
    first site (upper-body main site), with the same einsum conventions as
    calculate_relative_site_quatities.
    """
    site_xpos = data.site_xpos[site_ids]
    site_xmat = data.site_xmat[site_ids].reshape(-1, 3, 3)

    cvel = data.cvel[parent_body_ids]
    root_com = data.subtree_com[body_rootid[parent_body_ids]]
    rot_vel = cvel[:, :3]
    lin_vel = cvel[:, 3:]
    rpos_to_com = site_xpos - root_com
    site_lin = lin_vel - np.cross(rpos_to_com, rot_vel, axis=-1)
    site_xvel = np.hstack([rot_vel, site_lin])

    main_xvel = site_xvel[0]
    other_xvel = site_xvel[1:]
    main_mat = site_xmat[0]
    other_mat = site_xmat[1:]

    rel_rot = np.einsum("ik,nkj->nij", main_mat.T, other_mat)
    site_rpos = site_xpos[1:] - site_xpos[0]
    site_rangles = np.stack([_rotmat_to_rotvec(m) for m in rel_rot])

    ang_a, lin_a = main_xvel[:3], main_xvel[3:]
    ang_b, lin_b = other_xvel[:, :3], other_xvel[:, 3:]
    relative_lin = np.einsum("jk,ik->ij", main_mat, lin_a - lin_b)
    ang_b_in_a = np.einsum("ikj,ik->ij", rel_rot, ang_b)
    site_rvel = np.hstack([ang_b_in_a - ang_a, relative_lin])

    return site_rpos, site_rangles, site_rvel


class YogaPolicyController:
    """Time-indexed mimic policy playback over the sim's torque actuators."""

    def __init__(self, model, bundle):
        self.model = model
        self.bundle = bundle
        self.n_layers = int(bundle["n_layers"])
        self.weights = [(bundle[f"w{i}"], bundle[f"b{i}"]) for i in range(self.n_layers)]
        self.obs_mean = bundle["obs_mean"]
        self.obs_var = bundle["obs_var"]
        self.act_mean = bundle["act_mean"]
        self.act_delta = bundle["act_delta"]
        self.goal_ref = bundle["goal_ref"]
        self.ref_qpos = bundle["ref_qpos"]
        self.ref_qvel = bundle["ref_qvel"]
        self.n_frames = int(self.goal_ref.shape[0])
        self.control_dt = float(bundle["control_dt"])

        self.site_ids = np.array([
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, str(n))
            for n in bundle["site_names"]
        ])
        if np.any(self.site_ids < 0):
            raise ValueError("mimic sites not present in model; inject_mimic_sites first")
        self.site_parent_bodies = model.site_bodyid[self.site_ids]

        # plain-obs joint maps (23 joints, obs-spec order)
        self.obs_qpos_adr = []
        self.obs_dof_adr = []
        for name in bundle["obs_joint_names"]:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
            if joint_id < 0:
                raise ValueError(f"joint {name} missing from sim model")
            self.obs_qpos_adr.append(int(model.jnt_qposadr[joint_id]))
            self.obs_dof_adr.append(int(model.jnt_dofadr[joint_id]))
        self.obs_qpos_adr = np.array(self.obs_qpos_adr)
        self.obs_dof_adr = np.array(self.obs_dof_adr)

        # actuators mapped by driven joint; leftover sim actuators get PD-held
        joint_to_actuator = {}
        for i in range(model.nu):
            joint_id = int(model.actuator_trnid[i, 0])
            joint_to_actuator[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)] = i
        self.policy_actuators = []
        for name in bundle["act_joint_names"]:
            actuator = joint_to_actuator.get(str(name))
            if actuator is None:
                raise ValueError(f"no sim actuator drives joint {name}")
            self.policy_actuators.append(actuator)
        self.policy_actuators = np.array(self.policy_actuators)
        self.held_actuators = np.array(
            [i for i in range(model.nu) if i not in set(self.policy_actuators.tolist())],
            dtype=int,
        )
        self.held_qpos_adr = np.array(
            [int(model.jnt_qposadr[model.actuator_trnid[i, 0]]) for i in self.held_actuators],
            dtype=int,
        )
        self.held_dof_adr = np.array(
            [int(model.jnt_dofadr[model.actuator_trnid[i, 0]]) for i in self.held_actuators],
            dtype=int,
        )

        # 23-dof reference qpos/qvel -> 29-dof state maps (for teleport resets)
        self.ref_joint_qpos_cols = []
        self.ref_joint_sim_qpos = []
        self.ref_joint_qvel_cols = []
        self.ref_joint_sim_dof = []
        ref_names = [str(n) for n in bundle["joint_names"]]
        qpos_col, qvel_col = 0, 0
        for name in ref_names:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if name == "root" or (joint_id >= 0 and model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE):
                qpos_col += 7
                qvel_col += 6
                continue
            if joint_id >= 0:
                self.ref_joint_qpos_cols.append(qpos_col)
                self.ref_joint_sim_qpos.append(int(model.jnt_qposadr[joint_id]))
                self.ref_joint_qvel_cols.append(qvel_col)
                self.ref_joint_sim_dof.append(int(model.jnt_dofadr[joint_id]))
            qpos_col += 1
            qvel_col += 1
        self.ref_joint_qpos_cols = np.array(self.ref_joint_qpos_cols)
        self.ref_joint_sim_qpos = np.array(self.ref_joint_sim_qpos)
        self.ref_joint_qvel_cols = np.array(self.ref_joint_qvel_cols)
        self.ref_joint_sim_dof = np.array(self.ref_joint_sim_dof)

        self.frame = 0

    def reset_to_frame(self, data, frame):
        """Teleport the sim state onto the reference trajectory at `frame`."""
        frame = int(frame) % self.n_frames
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.qpos[3] = 1.0
        data.qpos[0:7] = self.ref_qpos[frame, 0:7]
        data.qpos[self.ref_joint_sim_qpos] = self.ref_qpos[frame, self.ref_joint_qpos_cols]
        data.qvel[0:6] = self.ref_qvel[frame, 0:6]
        data.qvel[self.ref_joint_sim_dof] = self.ref_qvel[frame, self.ref_joint_qvel_cols]
        data.ctrl[:] = 0.0
        self.frame = frame
        mujoco.mj_forward(self.model, data)

    def build_obs(self, data):
        plain = np.concatenate([
            data.qpos[2:7],
            data.qpos[self.obs_qpos_adr],
            data.qvel[0:6],
            data.qvel[self.obs_dof_adr],
        ])
        rpos, rangles, rvel = relative_site_quantities(
            data, self.site_ids, self.site_parent_bodies, self.model.body_rootid)
        return np.concatenate([
            plain,
            np.ravel(rpos), np.ravel(rangles), np.ravel(rvel),
            self.goal_ref[self.frame],
        ])

    def policy_action(self, obs):
        x = (obs - self.obs_mean) / np.sqrt(self.obs_var + 1e-8)
        for i, (w, b) in enumerate(self.weights):
            x = x @ w + b
            if i < self.n_layers - 1:
                x = np.tanh(x)
        return x

    def apply(self, data):
        """One policy tick: obs -> action -> ctrl for the mapped actuators."""
        obs = self.build_obs(data)
        action = np.clip(self.policy_action(obs), -1.0, 1.0)
        ctrl = self.act_mean + action * self.act_delta
        low = self.model.actuator_ctrlrange[self.policy_actuators, 0]
        high = self.model.actuator_ctrlrange[self.policy_actuators, 1]
        data.ctrl[self.policy_actuators] = np.clip(ctrl, low, high)
        self.frame = (self.frame + 1) % self.n_frames

    def hold_extra_actuators(self, data, kp=90.0, kd=3.0):
        """Gravity-compensated PD to zero for any actuators the policy does not drive."""
        if len(self.held_actuators) == 0:
            return
        q = data.qpos[self.held_qpos_adr]
        qd = data.qvel[self.held_dof_adr]
        bias = data.qfrc_bias[self.held_dof_adr]
        tau = bias + kp * (0.0 - q) - kd * qd
        low = self.model.actuator_ctrlrange[self.held_actuators, 0]
        high = self.model.actuator_ctrlrange[self.held_actuators, 1]
        data.ctrl[self.held_actuators] = np.clip(tau, low, high)
