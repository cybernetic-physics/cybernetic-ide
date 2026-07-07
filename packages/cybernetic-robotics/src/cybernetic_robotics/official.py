from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
import subprocess
from typing import Any, Callable

from .config import find_robotics_root


ARM_POSE_PRESETS = {"raise_right_hand", "raise_left_hand"}
ARM_JOINTS = {
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
}

Runner = Callable[[list[str], Path, int], subprocess.CompletedProcess[str]]


def _subprocess_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=True,
    )


@dataclass
class OfficialG1Sim:
    """Opt-in bridge to the official Unitree MuJoCo + SDK2/CycloneDDS sidecar.

    This is intentionally a short-lived diagnostic bridge, not the final
    long-lived DDS transport. It gives Python users one clean call that reaches
    the official `unitree_mujoco` peer and verifies motion through `rt/lowcmd`
    and `rt/lowstate`.
    """

    root: Path
    timeout: int = 300
    _runner: Runner = field(default=_subprocess_runner, repr=False)

    @classmethod
    def discover(cls, start: str | Path | None = None, *, timeout: int = 300) -> "OfficialG1Sim":
        return cls(find_robotics_root(start), timeout=timeout)

    @property
    def compose_env(self) -> Path:
        return self.root / ".runtime/unitree-g1-sdk2/compose.env"

    @property
    def compose_file(self) -> Path:
        return self.root / "overlays/unitree-g1-sdk2-sidecar/compose.yaml"

    def raise_right_hand(self, **kwargs: Any) -> dict[str, Any]:
        return self.arm_pose("raise_right_hand", **kwargs)

    def raise_left_hand(self, **kwargs: Any) -> dict[str, Any]:
        return self.arm_pose("raise_left_hand", **kwargs)

    def raise_right_hand_session(self, **kwargs: Any) -> dict[str, Any]:
        return self.arm_pose_session("raise_right_hand", **kwargs)

    def raise_left_hand_session(self, **kwargs: Any) -> dict[str, Any]:
        return self.arm_pose_session("raise_left_hand", **kwargs)

    def lowstate_session(self) -> dict[str, Any]:
        """Read one official rt/lowstate sample from a managed MuJoCo DDS session."""

        env = {"CYBER_UNITREE_ACTION": "read_official_mujoco_lowstate"}
        completed = self._run_sidecar(env)
        report = _parse_json_report(completed.stdout)
        lowstate_read = report.get("lowstate_read") if isinstance(report, dict) else None
        return {
            "ok": bool(isinstance(lowstate_read, dict) and lowstate_read.get("ok")),
            "source": "official_unitree_mujoco_managed_session",
            "lowstate": lowstate_read,
            "lowstate_summary": lowstate_read.get("lowstate_summary") if isinstance(lowstate_read, dict) else None,
            "report": report,
            "command": " ".join(_sidecar_command(self.compose_env, self.compose_file, env)),
            "stdout_tail": completed.stdout[-12000:],
            "stderr_tail": completed.stderr[-12000:],
        }

    def status(self) -> dict[str, Any]:
        """Inspect the official SDK2 sidecar without commanding motion."""

        completed = self._run_sidecar({})
        report = _parse_json_report(completed.stdout)
        sdk2_probe = report.get("sdk2_probe") if isinstance(report, dict) else None
        peer = report.get("official_mujoco_peer") if isinstance(report, dict) else None
        return {
            "ok": bool(isinstance(sdk2_probe, dict) and sdk2_probe.get("domain_initialized")),
            "source": "official_unitree_mujoco_sdk2_sidecar",
            "sdk2_probe": sdk2_probe,
            "official_mujoco_peer": peer,
            "expected_topics": report.get("expected_topics", []) if isinstance(report, dict) else [],
            "sources": report.get("sources", {}) if isinstance(report, dict) else {},
            "next_step": report.get("next_step") if isinstance(report, dict) else None,
            "command": " ".join(_sidecar_command(self.compose_env, self.compose_file, {})),
            "stdout_tail": completed.stdout[-12000:],
            "stderr_tail": completed.stderr[-12000:],
        }

    def arm_pose(
        self,
        preset: str = "raise_right_hand",
        *,
        joint_deltas: dict[str, float] | None = None,
        frames: int = 180,
        kp: float = 30.0,
        kd: float = 1.0,
        hold_kp: float = 18.0,
        hold_kd: float = 0.8,
        min_moved_joints: int = 2,
    ) -> dict[str, Any]:
        if preset not in ARM_POSE_PRESETS:
            raise ValueError(f"unknown official G1 arm pose preset: {preset}")

        normalized_deltas = _normalize_joint_deltas(joint_deltas)
        env = {
            "CYBER_UNITREE_ACTION": "probe_official_mujoco_arm_pose",
            "CYBER_UNITREE_ARM_POSE_PRESET": preset,
            "CYBER_UNITREE_ARM_POSE_FRAMES": str(_clamp_int(frames, 20, 600)),
            "CYBER_UNITREE_ARM_POSE_KP": str(_clamp_float(kp, 0.0, 80.0)),
            "CYBER_UNITREE_ARM_POSE_KD": str(_clamp_float(kd, 0.0, 5.0)),
            "CYBER_UNITREE_ARM_POSE_HOLD_KP": str(_clamp_float(hold_kp, 0.0, 80.0)),
            "CYBER_UNITREE_ARM_POSE_HOLD_KD": str(_clamp_float(hold_kd, 0.0, 5.0)),
            "CYBER_UNITREE_ARM_POSE_MIN_MOVED_JOINTS": str(_clamp_int(min_moved_joints, 1, 8)),
        }
        if normalized_deltas:
            env["CYBER_UNITREE_ARM_POSE_DELTAS"] = json.dumps(normalized_deltas, sort_keys=True)

        completed = self._run_sidecar(env)
        report = _parse_json_report(completed.stdout)
        probe = report.get("arm_pose_probe") if isinstance(report, dict) else None
        return {
            "ok": bool(isinstance(probe, dict) and probe.get("ok")),
            "preset": preset,
            "parameters": {
                "joint_deltas": normalized_deltas,
                "frames": int(env["CYBER_UNITREE_ARM_POSE_FRAMES"]),
                "kp": float(env["CYBER_UNITREE_ARM_POSE_KP"]),
                "kd": float(env["CYBER_UNITREE_ARM_POSE_KD"]),
                "hold_kp": float(env["CYBER_UNITREE_ARM_POSE_HOLD_KP"]),
                "hold_kd": float(env["CYBER_UNITREE_ARM_POSE_HOLD_KD"]),
                "min_moved_joints": int(env["CYBER_UNITREE_ARM_POSE_MIN_MOVED_JOINTS"]),
            },
            "moved_joints": list(probe.get("moved_joints", [])) if isinstance(probe, dict) else [],
            "lowcmd_write_successes": probe.get("lowcmd_write_successes") if isinstance(probe, dict) else None,
            "probe": probe,
            "report": report,
            "command": " ".join(_sidecar_command(self.compose_env, self.compose_file, env)),
            "stdout_tail": completed.stdout[-12000:],
            "stderr_tail": completed.stderr[-12000:],
        }

    def arm_pose_session(
        self,
        preset: str = "raise_right_hand",
        *,
        joint_deltas: dict[str, float] | None = None,
        frames: int = 180,
        kp: float = 30.0,
        kd: float = 1.0,
        hold_kp: float = 18.0,
        hold_kd: float = 0.8,
        min_moved_joints: int = 2,
    ) -> dict[str, Any]:
        """Command an already-running official MuJoCo DDS session.

        This targets the managed `unitree-g1-sdk2-session` container started by
        the Cybernetic MCP instead of launching a second short-lived MuJoCo peer.
        """

        if preset not in ARM_POSE_PRESETS:
            raise ValueError(f"unknown official G1 arm pose preset: {preset}")

        normalized_deltas = _normalize_joint_deltas(joint_deltas)
        env = {
            "CYBER_UNITREE_ACTION": "command_official_mujoco_arm_pose",
            "CYBER_UNITREE_ARM_POSE_PRESET": preset,
            "CYBER_UNITREE_ARM_POSE_FRAMES": str(_clamp_int(frames, 20, 600)),
            "CYBER_UNITREE_ARM_POSE_KP": str(_clamp_float(kp, 0.0, 80.0)),
            "CYBER_UNITREE_ARM_POSE_KD": str(_clamp_float(kd, 0.0, 5.0)),
            "CYBER_UNITREE_ARM_POSE_HOLD_KP": str(_clamp_float(hold_kp, 0.0, 80.0)),
            "CYBER_UNITREE_ARM_POSE_HOLD_KD": str(_clamp_float(hold_kd, 0.0, 5.0)),
            "CYBER_UNITREE_ARM_POSE_MIN_MOVED_JOINTS": str(_clamp_int(min_moved_joints, 1, 8)),
        }
        if normalized_deltas:
            env["CYBER_UNITREE_ARM_POSE_DELTAS"] = json.dumps(normalized_deltas, sort_keys=True)

        completed = self._run_sidecar(env)
        report = _parse_json_report(completed.stdout)
        command = report.get("arm_pose_command") if isinstance(report, dict) else None
        return {
            "ok": bool(isinstance(command, dict) and command.get("ok")),
            "preset": preset,
            "source": "official_unitree_mujoco_managed_session",
            "parameters": {
                "joint_deltas": normalized_deltas,
                "frames": int(env["CYBER_UNITREE_ARM_POSE_FRAMES"]),
                "kp": float(env["CYBER_UNITREE_ARM_POSE_KP"]),
                "kd": float(env["CYBER_UNITREE_ARM_POSE_KD"]),
                "hold_kp": float(env["CYBER_UNITREE_ARM_POSE_HOLD_KP"]),
                "hold_kd": float(env["CYBER_UNITREE_ARM_POSE_HOLD_KD"]),
                "min_moved_joints": int(env["CYBER_UNITREE_ARM_POSE_MIN_MOVED_JOINTS"]),
            },
            "moved_joints": list(command.get("moved_joints", [])) if isinstance(command, dict) else [],
            "lowcmd_write_successes": command.get("lowcmd_write_successes") if isinstance(command, dict) else None,
            "command_result": command,
            "report": report,
            "command": " ".join(_sidecar_command(self.compose_env, self.compose_file, env)),
            "stdout_tail": completed.stdout[-12000:],
            "stderr_tail": completed.stderr[-12000:],
        }

    def _run_sidecar(self, env: dict[str, str]) -> subprocess.CompletedProcess[str]:
        if not self.compose_env.exists():
            raise FileNotFoundError(
                f"missing {self.compose_env}; run `node script/prepare-unitree-g1-sdk2-sidecar.mjs` first"
            )
        return self._runner(_sidecar_command(self.compose_env, self.compose_file, env), self.root, self.timeout)


def _sidecar_command(compose_env: Path, compose_file: Path, env: dict[str, str]) -> list[str]:
    args = ["docker", "compose", "--env-file", str(compose_env), "-f", str(compose_file), "run", "--rm"]
    for name, value in env.items():
        args.extend(["-e", f"{name}={value}"])
    args.append("unitree-g1-sdk2-sidecar")
    return args


def _parse_json_report(stdout: str) -> dict[str, Any]:
    start = stdout.find("{")
    end = stdout.rfind("}")
    if start == -1 or end <= start:
        return {}
    try:
        value = json.loads(stdout[start : end + 1])
    except json.JSONDecodeError:
        return {}
    return value if isinstance(value, dict) else {}


def _normalize_joint_deltas(joint_deltas: dict[str, float] | None) -> dict[str, float]:
    if not joint_deltas:
        return {}
    unknown = sorted(set(joint_deltas) - ARM_JOINTS)
    if unknown:
        raise ValueError(f"unknown official G1 arm joint(s): {', '.join(unknown)}")
    return {joint: _clamp_float(delta, -0.5, 0.5) for joint, delta in joint_deltas.items()}


def _clamp_float(value: Any, minimum: float, maximum: float) -> float:
    number = float(value)
    return min(max(number, minimum), maximum)


def _clamp_int(value: Any, minimum: int, maximum: int) -> int:
    number = int(round(float(value)))
    return min(max(number, minimum), maximum)
