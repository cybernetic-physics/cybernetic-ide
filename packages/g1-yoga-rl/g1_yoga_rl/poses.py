from __future__ import annotations

import ast
from dataclasses import dataclass
from pathlib import Path
from typing import Any


LOCOMUJOCO_G1_JOINTS: tuple[str, ...] = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
)


DEFAULT_YOGA_SEQUENCE: tuple[str, ...] = (
    "mountain",
    "upward_salute",
    "forward_fold",
    "chair",
    "warrior_one",
    "warrior_two",
    "goddess",
    "tree",
    "namaste",
)


@dataclass(frozen=True)
class PoseProjection:
    name: str
    targets: dict[str, float]
    dropped: dict[str, float]


def repo_root(start: Path | None = None) -> Path:
    current = (start or Path.cwd()).resolve()
    for candidate in (current, *current.parents):
        if (candidate / "script" / "cyber-robotics-mcp.mjs").exists():
            return candidate
    raise FileNotFoundError("Could not find Cybernetic IDE repo root")


def load_named_poses(root: Path | None = None) -> dict[str, dict[str, float]]:
    root = root or repo_root()
    protocol_path = root / "overlays/unitree-g1-mujoco-protocol/python/g1_protocol_sim.py"
    module = ast.parse(protocol_path.read_text())
    for node in module.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "NAMED_POSES":
                    value = ast.literal_eval(node.value)
                    return {
                        str(name): {str(joint): float(angle) for joint, angle in targets.items()}
                        for name, targets in value.items()
                    }
    raise ValueError(f"NAMED_POSES not found in {protocol_path}")


def project_pose(name: str, targets: dict[str, float], joint_names: tuple[str, ...] = LOCOMUJOCO_G1_JOINTS) -> PoseProjection:
    allowed = set(joint_names)
    projected = {joint: float(value) for joint, value in targets.items() if joint in allowed}
    dropped = {joint: float(value) for joint, value in targets.items() if joint not in allowed}
    return PoseProjection(name=name, targets=projected, dropped=dropped)


def project_named_poses(
    names: tuple[str, ...] = DEFAULT_YOGA_SEQUENCE,
    *,
    root: Path | None = None,
    joint_names: tuple[str, ...] = LOCOMUJOCO_G1_JOINTS,
) -> list[PoseProjection]:
    poses = load_named_poses(root)
    missing = [name for name in names if name not in poses]
    if missing:
        raise KeyError(f"Unknown poses: {', '.join(missing)}")
    return [project_pose(name, poses[name], joint_names) for name in names]


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, float(value)))
    return value * value * (3.0 - 2.0 * value)


def build_pose_frames(
    projections: list[PoseProjection],
    *,
    transition_steps: int = 30,
    hold_steps: int = 60,
    joint_names: tuple[str, ...] = LOCOMUJOCO_G1_JOINTS,
) -> list[dict[str, Any]]:
    current = {joint: 0.0 for joint in joint_names}
    frames: list[dict[str, Any]] = []
    for projection in projections:
        target = {**current, **projection.targets}
        for step in range(max(1, transition_steps)):
            alpha = smoothstep((step + 1) / max(1, transition_steps))
            joints = {
                joint: current[joint] + (target[joint] - current[joint]) * alpha
                for joint in joint_names
            }
            frames.append({"pose": projection.name, "phase": "transition", "joints": joints})
        for _ in range(max(0, hold_steps)):
            frames.append({"pose": projection.name, "phase": "hold", "joints": dict(target)})
        current = target
    return frames

