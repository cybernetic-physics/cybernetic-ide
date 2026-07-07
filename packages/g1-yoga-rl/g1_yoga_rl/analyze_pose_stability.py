"""Static-stability report and optional renders for projected yoga references."""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

from .yoga_traj import FULL_FLOW, ground_qpos, load_named_poses, project_pose


FOOT_GEOMS = {
    "left": ["left_foot_1_col", "left_foot_2_col", "left_foot_3_col", "left_foot_4_col"],
    "right": ["right_foot_1_col", "right_foot_2_col", "right_foot_3_col", "right_foot_4_col"],
}
CONTACT_EPS = 0.01


def foot_report(model, data, side: str) -> dict:
    """Return floor-clearance, sole-tilt, and support points for one foot."""

    bottoms = []
    centers = []
    for name in FOOT_GEOMS[side]:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            continue
        center = data.geom_xpos[geom_id]
        bottoms.append(center[2] - model.geom_rbound[geom_id])
        centers.append(center.copy())

    if not bottoms:
        return {"clearance": float("nan"), "tilt_deg": float("nan"), "support_xy": np.empty((0, 2))}

    bottoms = np.array(bottoms)
    centers = np.array(centers)
    ankle_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_ankle_roll_link")
    if ankle_body >= 0:
        rotation = data.xmat[ankle_body].reshape(3, 3)
        tilt = float(np.degrees(np.arccos(np.clip(rotation[2, 2], -1.0, 1.0))))
    else:
        tilt = float("nan")
    supporting = bottoms < CONTACT_EPS
    return {
        "clearance": float(np.min(bottoms)),
        "tilt_deg": tilt,
        "support_xy": centers[supporting, :2] if np.any(supporting) else np.empty((0, 2)),
    }


def polygon_margin(com_xy: np.ndarray, points: np.ndarray) -> float:
    """Signed distance from ``com_xy`` to support region; positive means inside."""

    if len(points) == 0:
        return -np.inf
    if len(points) == 1:
        return -float(np.linalg.norm(com_xy - points[0]))
    if len(points) == 2:
        p, q = points
        denominator = max(np.dot(q - p, q - p), 1e-12)
        t = np.clip(np.dot(com_xy - p, q - p) / denominator, 0.0, 1.0)
        return -float(np.linalg.norm(com_xy - (p + t * (q - p))))

    hull = _convex_hull(points)
    distances = []
    for index in range(len(hull)):
        p = hull[index]
        q = hull[(index + 1) % len(hull)]
        edge = q - p
        normal = np.array([edge[1], -edge[0]])
        normal /= max(np.linalg.norm(normal), 1e-12)
        distances.append(np.dot(com_xy - p, normal))
    outside = [distance for distance in distances if distance > 0]
    if outside:
        return float(-max(outside))
    return float(-max(distances))


def _convex_hull(points: np.ndarray) -> np.ndarray:
    """Return a clockwise monotone-chain hull."""

    pts = sorted(map(tuple, points))
    if len(pts) <= 2:
        return np.array(pts)

    def cross(origin, a, b):
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower = []
    upper = []
    for point in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return np.array(lower[:-1] + upper[:-1])


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze static support margins for the Cybernetic G1 yoga pose registry.")
    parser.add_argument("--render-dir", type=Path, default=None, help="Optional directory for one PNG render per pose.")
    args = parser.parse_args()

    from .cyber_env import make_cyber_env

    env = make_cyber_env(headless=True)
    model = env.get_model()
    data = env.get_data()

    renderer = None
    camera = None
    if args.render_dir is not None:
        from PIL import Image

        args.render_dir.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(model, height=480, width=640)
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.lookat[:] = [0.0, 0.0, 0.55]
        camera.distance = 2.6
        camera.azimuth = 135.0
        camera.elevation = -12.0

    named_poses = load_named_poses()
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    base_qpos = data.qpos.copy()

    header = (
        f"{'pose':15s} {'pelvis_z':>8s} {'L_clr_cm':>8s} {'R_clr_cm':>8s} "
        f"{'L_tilt':>6s} {'R_tilt':>6s} {'margin_cm':>9s}  notes"
    )
    print(header)
    for index, pose_name in enumerate(FULL_FLOW):
        projection = project_pose(model, base_qpos, pose_name, named_poses[pose_name])
        qpos = projection.qpos.copy()
        base_z = int(model.jnt_qposadr[0]) + 2
        qpos[base_z] = ground_qpos(model, data, qpos)
        data.qpos[:] = qpos
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)

        left = foot_report(model, data, "left")
        right = foot_report(model, data, "right")
        total_mass = float(np.sum(model.body_mass))
        com = np.sum(data.xipos * model.body_mass[:, None], axis=0) / total_mass
        support = np.vstack([left["support_xy"], right["support_xy"]])
        margin = polygon_margin(com[:2], support)

        notes = []
        if len(left["support_xy"]) == 0 or len(right["support_xy"]) == 0:
            notes.append("single-support")
        if margin < 0:
            notes.append("CoM OUTSIDE")
        print(
            f"{pose_name:15s} {qpos[base_z]:8.3f} {left['clearance'] * 100:8.1f} "
            f"{right['clearance'] * 100:8.1f} {left['tilt_deg']:6.1f} {right['tilt_deg']:6.1f} "
            f"{margin * 100:9.1f}  {'; '.join(notes)}"
        )

        if renderer is not None:
            renderer.update_scene(data, camera=camera)
            Image.fromarray(renderer.render()).save(args.render_dir / f"{index:02d}_{pose_name}.png")


if __name__ == "__main__":
    main()
