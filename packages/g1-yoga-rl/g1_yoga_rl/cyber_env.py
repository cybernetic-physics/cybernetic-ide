"""LocoMuJoCo training env built on OUR 29-DOF deploy model.

The policy trained on LocoMuJoCo's 23-DOF G1 did not transfer: obs matched to
machine precision but the model gap (lumped wrist masses, welded-vs-PD waist,
~1 kg total) broke the trained closed loop (0/9 sim2sim vs 6/9 in-env). The
fix is to train on the exact model we deploy to: the sim's scene_29dof.xml
with the training mimic sites injected and the same MJX contact reduction the
23-DOF env used (foot capsules <-> floor only), via the env's `spec` argument.

Everything downstream (trajectory generation, training, export, pack,
validation, deploy) is name-driven, so the wider joint set flows through.
"""

from __future__ import annotations

import importlib.util

import mujoco
import numpy as np

from .yoga_traj import REPO_ROOT

SCENE_29DOF_XML = (REPO_ROOT / ".runtime/unitree-g1-mujoco/unitree_mujoco/"
                   "unitree_robots/g1/scene_29dof.xml")
RUNTIME_PATH = (REPO_ROOT / "overlays/unitree-g1-mujoco-protocol/python/"
                "g1_policy_runtime.py")


def load_runtime():
    spec = importlib.util.spec_from_file_location("g1_policy_runtime", RUNTIME_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def mimic_site_table() -> dict:
    """Site name/parent/pos/quat table from the 23-DOF training model."""
    from loco_mujoco.environments.humanoids.unitreeG1_mjx import MjxUnitreeG1

    env = MjxUnitreeG1(headless=True)
    model = env.get_model()
    names = list(env.sites_for_mimic)
    site_ids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, n) for n in names])
    body_ids = model.site_bodyid[site_ids]
    return {
        "site_names": np.array(names),
        "site_parent_bodies": np.array(
            [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(b)) for b in body_ids]
        ),
        "site_pos": model.site_pos[site_ids].copy(),
        "site_quat": model.site_quat[site_ids].copy(),
    }


def build_cyber_spec() -> "mujoco.MjSpec":
    """29-DOF scene spec prepared for the LocoMuJoCo env.

    Injects the mimic sites, names the foot capsules (the env's own
    _modify_spec_for_mjx applies the contact reduction using those names),
    and renames the free joint to "root" (loco_mujoco's expected root name).
    """
    runtime = load_runtime()
    spec = mujoco.MjSpec.from_file(str(SCENE_29DOF_XML))
    runtime.inject_mimic_sites(spec, mimic_site_table())
    runtime.name_foot_capsules(spec)
    for joint in spec.joints:
        if joint.type == mujoco.mjtJoint.mjJNT_FREE:
            joint.name = "root"
    return spec


def make_cyber_env(**kwargs):
    """Construct the MJX-enabled UnitreeG1 env on the 29-DOF spec.

    Observation/actuation specs are generated from the model: root free joint
    (FreeJointPosNoXY + FreeJointVel) plus JointPos/JointVel for every hinge
    joint in model order — the same layout pattern as the stock env, extended
    to all 29 joints.
    """
    from . import recovery_randomizer  # noqa: F401
    from loco_mujoco.core import ObservationType
    from loco_mujoco.environments.humanoids.unitreeG1_mjx import MjxUnitreeG1

    spec = build_cyber_spec()
    compiled = spec.compile()
    joint_names = [
        mujoco.mj_id2name(compiled, mujoco.mjtObj.mjOBJ_JOINT, i)
        for i in range(compiled.njnt)
    ]
    hinge_joints = [
        name for i, name in enumerate(joint_names)
        if compiled.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE
    ]
    root_name = joint_names[0]

    observation_spec = (
        [ObservationType.FreeJointPosNoXY("q_root", xml_name=root_name)]
        + [ObservationType.JointPos(f"q_{n}", xml_name=n) for n in hinge_joints]
        + [ObservationType.FreeJointVel("dq_root", xml_name=root_name)]
        + [ObservationType.JointVel(f"dq_{n}", xml_name=n) for n in hinge_joints]
    )
    actuation_spec = [actuator.name for actuator in spec.actuators]

    return MjxUnitreeG1(
        spec=spec,
        observation_spec=observation_spec,
        actuation_spec=actuation_spec,
        **kwargs,
    )
