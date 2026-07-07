from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
import subprocess
import time
from typing import Any, Callable

from .config import find_robotics_root


ARM_POSE_PRESETS = {"raise_right_hand", "raise_left_hand"}
OFFICIAL_MUJOCO_SESSION_CONTAINER = "unitree-g1-sdk2-session"
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


def _subprocess_unchecked_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
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
    _unchecked_runner: Runner = field(default=_subprocess_unchecked_runner, repr=False)

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

    def session(self, *, keep_running: bool = False, wait_timeout: float = 12.0) -> "OfficialG1ManagedSession":
        """Create a context-managed official MuJoCo DDS session helper."""

        return OfficialG1ManagedSession(self, keep_running=keep_running, wait_timeout=wait_timeout)

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

    def loco_rpc_session(self, *, include_stop: bool = False, timeout: float = 2.0) -> dict[str, Any]:
        """Probe official G1 LocoClient sport RPCs against a managed session."""

        env = {
            "CYBER_UNITREE_ACTION": "probe_official_mujoco_loco_rpc",
            "CYBER_UNITREE_LOCO_RPC_TIMEOUT": str(_clamp_float(timeout, 0.2, 10.0)),
            "CYBER_UNITREE_LOCO_RPC_STOP_MOVE": "1" if include_stop else "0",
        }
        completed = self._run_sidecar(env)
        report = _parse_json_report(completed.stdout)
        probe = report.get("loco_rpc_probe") if isinstance(report, dict) else None
        return {
            "ok": bool(isinstance(probe, dict) and probe.get("ok")),
            "source": "official_unitree_mujoco_managed_session",
            "request_topic": probe.get("request_topic") if isinstance(probe, dict) else "rt/api/sport/request",
            "response_topic": probe.get("response_topic") if isinstance(probe, dict) else "rt/api/sport/response",
            "probe": probe,
            "report": report,
            "command": " ".join(_sidecar_command(self.compose_env, self.compose_file, env)),
            "stdout_tail": completed.stdout[-12000:],
            "stderr_tail": completed.stderr[-12000:],
        }

    def rpc_discovery_session(self, *, wait: float = 1.0) -> dict[str, Any]:
        """Inspect official Unitree RPC service discovery against a managed session."""

        env = {
            "CYBER_UNITREE_ACTION": "probe_official_mujoco_rpc_discovery",
            "CYBER_UNITREE_RPC_DISCOVERY_WAIT": str(_clamp_float(wait, 0.1, 10.0)),
        }
        completed = self._run_sidecar(env)
        report = _parse_json_report(completed.stdout)
        probe = report.get("rpc_discovery_probe") if isinstance(report, dict) else None
        services = probe.get("services", []) if isinstance(probe, dict) else []
        return {
            "ok": bool(isinstance(probe, dict) and probe.get("ok")),
            "source": "official_unitree_mujoco_managed_session",
            "matched_services": probe.get("matched_services", []) if isinstance(probe, dict) else [],
            "missing_request_readers": probe.get("missing_request_readers", []) if isinstance(probe, dict) else [],
            "services": services,
            "probe": probe,
            "report": report,
            "command": " ".join(_sidecar_command(self.compose_env, self.compose_file, env)),
            "stdout_tail": completed.stdout[-12000:],
            "stderr_tail": completed.stderr[-12000:],
        }

    def start_session(self, *, wait: bool = True, wait_timeout: float = 12.0) -> dict[str, Any]:
        """Start the managed official Unitree MuJoCo DDS peer session.

        The session is the same named Docker container used by the robotics MCP:
        `unitree-g1-sdk2-session`. Once ready, `lowstate_session()` and
        `arm_pose_session()` connect SDK2 Python clients to this sustained peer
        instead of launching a second short-lived simulator.
        """

        if not self.compose_env.exists():
            raise FileNotFoundError(
                f"missing {self.compose_env}; run `node script/prepare-unitree-g1-sdk2-sidecar.mjs` first"
            )
        removed = self._unchecked_runner(
            ["docker", "rm", "-f", OFFICIAL_MUJOCO_SESSION_CONTAINER],
            self.root,
            min(self.timeout, 60),
        )
        env = {"CYBER_UNITREE_ACTION": "serve_official_mujoco"}
        args = [
            "docker",
            "compose",
            "--env-file",
            str(self.compose_env),
            "-f",
            str(self.compose_file),
            "run",
            "-d",
            "--name",
            OFFICIAL_MUJOCO_SESSION_CONTAINER,
        ]
        for name, value in env.items():
            args.extend(["-e", f"{name}={value}"])
        args.append("unitree-g1-sdk2-sidecar")
        started = self._runner(args, self.root, self.timeout)
        status = self.wait_for_session_ready(wait_timeout) if wait else self.session_status()
        return {
            "ok": bool(status.get("running") and status.get("ready")),
            "container": OFFICIAL_MUJOCO_SESSION_CONTAINER,
            "command": " ".join(args),
            "removed_existing": {
                "attempted": True,
                "status": removed.returncode,
                "stdout": removed.stdout,
                "stderr": removed.stderr,
            },
            "started": {
                "returncode": started.returncode,
                "stdout": started.stdout,
                "stderr": started.stderr,
            },
            "status": status,
        }

    def wait_for_session_ready(self, timeout_seconds: float = 12.0) -> dict[str, Any]:
        deadline = time.monotonic() + max(0.0, timeout_seconds)
        status = self.session_status()
        while status.get("running") and not status.get("ready") and time.monotonic() < deadline:
            time.sleep(0.5)
            status = self.session_status()
        return status

    def session_status(self, *, log_tail: int = 2000) -> dict[str, Any]:
        """Inspect the managed official MuJoCo session container."""

        inspect = self._unchecked_runner(
            ["docker", "inspect", OFFICIAL_MUJOCO_SESSION_CONTAINER, "--format", "{{json .State}}"],
            self.root,
            min(self.timeout, 30),
        )
        logs = self._unchecked_runner(
            ["docker", "logs", "--tail", str(_clamp_int(log_tail, 1, 1000)), OFFICIAL_MUJOCO_SESSION_CONTAINER],
            self.root,
            min(self.timeout, 30),
        )
        state = _parse_json_report(inspect.stdout.strip()) if inspect.returncode == 0 else {}
        reports = _parse_json_objects(logs.stdout) if logs.returncode == 0 else []
        lifecycle_source = "tail"
        if state.get("Running") is True and not reports and logs.returncode == 0:
            full_logs = self._unchecked_runner(
                ["docker", "logs", OFFICIAL_MUJOCO_SESSION_CONTAINER],
                self.root,
                min(self.timeout, 30),
            )
            if full_logs.returncode == 0:
                reports = _parse_json_objects(full_logs.stdout)
                lifecycle_source = "full_logs_fallback"
        ready_report = next((report for report in reports if report.get("action") == "serve_official_mujoco"), None)
        if ready_report is None and reports:
            ready_report = reports[0]
        exit_report = next(
            (report for report in reversed(reports) if report.get("action") == "serve_official_mujoco_exit"),
            None,
        )
        last_report = reports[-1] if reports else None
        running = state.get("Running") is True
        return {
            "container": OFFICIAL_MUJOCO_SESSION_CONTAINER,
            "exists": inspect.returncode == 0,
            "running": running,
            "status": state.get("Status"),
            "exit_code": state.get("ExitCode"),
            "started_at": state.get("StartedAt"),
            "finished_at": state.get("FinishedAt"),
            "inspect_error": None if inspect.returncode == 0 else inspect.stderr.strip(),
            "ready_report": ready_report,
            "last_report": last_report,
            "exit_report": exit_report,
            "lifecycle_reports_seen": len(reports),
            "lifecycle_report_source": lifecycle_source,
            "ready": running and isinstance(ready_report, dict) and ready_report.get("ok") is True and exit_report is None,
            "logs_tail": logs.stdout if logs.returncode == 0 else None,
            "logs_error": None if logs.returncode == 0 else logs.stderr.strip(),
        }

    def stop_session(self) -> dict[str, Any]:
        """Stop and remove the managed official Unitree MuJoCo DDS peer."""

        result = self._unchecked_runner(
            ["docker", "rm", "-f", OFFICIAL_MUJOCO_SESSION_CONTAINER],
            self.root,
            min(self.timeout, 60),
        )
        return {
            "ok": result.returncode == 0,
            "container": OFFICIAL_MUJOCO_SESSION_CONTAINER,
            "removed": result.returncode == 0,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "status": self.session_status(),
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


@dataclass
class OfficialG1ManagedSession:
    """Context manager for the managed official Unitree MuJoCo DDS peer."""

    sim: OfficialG1Sim
    keep_running: bool = False
    wait_timeout: float = 12.0
    started: dict[str, Any] | None = None
    stopped: dict[str, Any] | None = None

    def __enter__(self) -> "OfficialG1ManagedSession":
        self.started = self.sim.start_session(wait_timeout=self.wait_timeout)
        if not self.started.get("ok"):
            if not self.keep_running:
                self.stopped = self.sim.stop_session()
            raise RuntimeError("official Unitree MuJoCo session did not become ready")
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        if not self.keep_running:
            self.stopped = self.sim.stop_session()

    def status(self) -> dict[str, Any]:
        return self.sim.session_status()

    def lowstate(self) -> dict[str, Any]:
        return self.sim.lowstate_session()

    def loco_rpc(self, *, include_stop: bool = False, timeout: float = 2.0) -> dict[str, Any]:
        return self.sim.loco_rpc_session(include_stop=include_stop, timeout=timeout)

    def rpc_discovery(self, *, wait: float = 1.0) -> dict[str, Any]:
        return self.sim.rpc_discovery_session(wait=wait)

    def arm_pose(self, preset: str = "raise_right_hand", **kwargs: Any) -> dict[str, Any]:
        return self.sim.arm_pose_session(preset, **kwargs)

    def raise_right_hand(self, **kwargs: Any) -> dict[str, Any]:
        return self.sim.raise_right_hand_session(**kwargs)

    def raise_left_hand(self, **kwargs: Any) -> dict[str, Any]:
        return self.sim.raise_left_hand_session(**kwargs)

    def arm_pose_evidence(
        self,
        preset: str = "raise_right_hand",
        *,
        output_path: str | Path | None = None,
        **kwargs: Any,
    ) -> dict[str, Any]:
        """Run a bounded arm pose and collect before/after official lowstate."""

        before = self.lowstate()
        command = self.arm_pose(preset, **kwargs)
        after = self.lowstate()
        bundle = {
            "ok": bool(before.get("ok") and command.get("ok") and after.get("ok")),
            "source": "official_unitree_mujoco_managed_session",
            "preset": preset,
            "started": self.started,
            "before": before.get("lowstate_summary"),
            "command": {
                "ok": command.get("ok"),
                "parameters": command.get("parameters"),
                "moved_joints": command.get("moved_joints", []),
                "lowcmd_write_successes": command.get("lowcmd_write_successes"),
            },
            "after": after.get("lowstate_summary"),
            "agent_hints": [
                "Use before/after lowstate as the source of truth for official SDK2 motion verification.",
                "The managed official session is simulator-only; it is not a real hardware unlock.",
                "If ok is false, inspect command.command_result and the sidecar stdout/stderr tails.",
            ],
        }
        if output_path is not None:
            path = Path(output_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(bundle, indent=2, default=str) + "\n", encoding="utf-8")
            bundle["output_path"] = str(path)
        return bundle

    def stop(self) -> dict[str, Any]:
        self.stopped = self.sim.stop_session()
        return self.stopped


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


def _parse_first_json_object(text: str) -> dict[str, Any] | None:
    decoder = json.JSONDecoder()
    index = text.find("{")
    while index != -1:
        try:
            value, _end = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            index = text.find("{", index + 1)
            continue
        return value if isinstance(value, dict) else None
    return None


def _parse_json_objects(text: str) -> list[dict[str, Any]]:
    decoder = json.JSONDecoder()
    reports: list[dict[str, Any]] = []
    index = text.find("{")
    while index != -1:
        try:
            value, end = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            index = text.find("{", index + 1)
            continue
        if isinstance(value, dict):
            reports.append(value)
        next_index = text.find("{", index + max(end, 1))
        index = next_index
    return reports


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
