from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from .config import RobotEndpoints
from .errors import ProtocolError, SimulatorUnavailable
from .safety import evaluate_lowstate_safety


JsonObject = dict[str, Any]


@dataclass(frozen=True)
class CameraState:
    """MuJoCo free-camera state exposed by the simulator."""

    raw: JsonObject

    @property
    def distance(self) -> float | None:
        return _optional_float(self.raw.get("distance"))

    @property
    def azimuth(self) -> float | None:
        return _optional_float(self.raw.get("azimuth"))

    @property
    def elevation(self) -> float | None:
        return _optional_float(self.raw.get("elevation"))


@dataclass(frozen=True)
class SimulatorStatus:
    """Structured `/status` response from the G1 simulator."""

    raw: JsonObject

    @property
    def ready(self) -> bool:
        return bool(self.raw.get("ready"))

    @property
    def simulation(self) -> JsonObject:
        value = self.raw.get("simulation")
        return value if isinstance(value, dict) else {}

    @property
    def pose(self) -> str | None:
        pose = self.simulation.get("pose")
        return str(pose) if pose is not None else None

    @property
    def paused(self) -> bool:
        return bool(self.simulation.get("paused"))

    @property
    def speed(self) -> float:
        return float(self.simulation.get("actual_speed_factor") or 0.0)

    @property
    def model_path(self) -> str | None:
        model_path = self.simulation.get("model_path")
        return str(model_path) if model_path is not None else None

    @property
    def fallen(self) -> bool:
        return bool(self.simulation.get("fallen"))

    @property
    def pelvis_height(self) -> float | None:
        return _optional_float(self.simulation.get("pelvis_height"))

    @property
    def lowcmd(self) -> JsonObject:
        value = self.simulation.get("lowcmd")
        return value if isinstance(value, dict) else {}

    @property
    def dex3(self) -> JsonObject:
        value = self.simulation.get("dex3")
        return value if isinstance(value, dict) else {}

    @property
    def lowcmd_active(self) -> bool:
        return bool(self.lowcmd.get("active"))

    @property
    def lowcmd_stale(self) -> bool:
        return bool(self.lowcmd.get("stale"))

    @property
    def lowcmd_age_seconds(self) -> float | None:
        return _optional_float(self.lowcmd.get("age_seconds"))

    @property
    def lowcmd_watchdog_seconds(self) -> float | None:
        return _optional_float(self.lowcmd.get("watchdog_seconds"))


class SimulatorClient:
    """HTTP client for Cybernetic's GameControl-style simulator API."""

    def __init__(self, endpoints: RobotEndpoints | None = None, timeout: float = 5.0):
        self.endpoints = endpoints or RobotEndpoints.from_env()
        self.timeout = float(timeout)

    @classmethod
    def from_env(cls, timeout: float = 5.0) -> "SimulatorClient":
        return cls(RobotEndpoints.from_env(), timeout=timeout)

    def health(self) -> JsonObject:
        return self.get_json("/health")

    def status(self) -> SimulatorStatus:
        return SimulatorStatus(self.get_json("/status"))

    def wait_until_ready(self, timeout: float = 10.0, interval: float = 0.2) -> SimulatorStatus:
        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                status = self.status()
                if status.ready:
                    return status
            except Exception as error:  # noqa: BLE001 - surface the final error below.
                last_error = error
            time.sleep(interval)
        if last_error:
            raise SimulatorUnavailable(f"simulator was not ready: {last_error}") from last_error
        raise SimulatorUnavailable("simulator was not ready before timeout")

    def visual_scene(self) -> JsonObject:
        return self.get_json("/visual_scene")

    def visual_frame(self) -> JsonObject:
        return self.get_json("/visual_frame")

    def command(self, command: str, **fields: Any) -> JsonObject:
        return self.post_json("/command", {"command": command, **fields})

    def pause(self) -> JsonObject:
        return self.command("pause")

    def resume(self) -> JsonObject:
        return self.command("resume")

    def reset(self) -> JsonObject:
        return self.command("reset")

    def step(self, count: int = 1) -> list[JsonObject]:
        return [self.command("step") for _ in range(max(1, int(count)))]

    def pose(
        self,
        name: str,
        *,
        smooth: bool = False,
        duration: float | None = None,
        fps: int = 30,
    ) -> JsonObject:
        if smooth or duration is not None:
            return self.command(
                "pose",
                pose=name,
                smooth=True,
                duration=float(duration if duration is not None else 1.2),
                fps=int(fps),
            )
        return self.command("pose", pose=name)

    def hold_pose(self, name: str, teleport: bool = True) -> JsonObject:
        return self.command("hold_pose", pose=name, teleport=bool(teleport))

    def loco(self, action: str = "state", **fields: Any) -> JsonObject:
        return self.command("loco", action=action, **fields)

    def lowcmd(self, motor_cmd: list[JsonObject], **fields: Any) -> JsonObject:
        return self.command("lowcmd", motor_cmd=motor_cmd, **fields)

    def hand_sdk(self, cmds: list[JsonObject], **fields: Any) -> JsonObject:
        return self.command("hand_sdk", cmds=cmds, **fields)

    def dex3(self, hand: str, motor_cmd: list[JsonObject], **fields: Any) -> JsonObject:
        return self.command("dex3", hand=hand, motor_cmd=motor_cmd, **fields)

    def wireless_controller(self, **fields: Any) -> JsonObject:
        return self.command("wireless_controller", **fields)

    def dex3_state(self, hand: str | None = None) -> JsonObject:
        dex3 = self.status().dex3
        if hand is None:
            return dex3
        hands = dex3.get("hands") if isinstance(dex3.get("hands"), dict) else {}
        normalized = str(hand).lower()
        return hands.get(normalized, {}) if normalized in {"left", "right"} else {}

    def command_state(self) -> JsonObject:
        """Summarize the currently active simulator command/control state."""

        status = self.status().raw
        lowstate = self.lowstate()
        return _command_state_from_payload(status, lowstate)

    def lowstate(self) -> JsonObject:
        return self.get_json("/lowstate")

    def safety_check(self, **limits: Any) -> JsonObject:
        status = self.status().raw
        lowstate = self.lowstate()
        return evaluate_lowstate_safety(lowstate, status=status, **limits)

    def safety_stop(self) -> JsonObject:
        """Best-effort simulator stop: release motion, damp, neutral, pause."""

        steps: list[JsonObject] = []
        ok = True
        for label, call in (
            ("release_motion_mode", lambda: self.command("motion_switcher", action="release_mode")),
            ("damp_locomotion", lambda: self.command("loco", action="set_fsm_id", fsm_id=1, mode="damp")),
            ("neutral_pose", lambda: self.pose("neutral")),
            ("pause", self.pause),
        ):
            try:
                result = call()
                steps.append({"step": label, "ok": bool(result.get("ok", True)), "result": result})
                ok = ok and bool(result.get("ok", True))
            except Exception as error:  # noqa: BLE001 - stop paths should report every attempted step.
                steps.append({"step": label, "ok": False, "error": str(error)})
                ok = False
        return {"ok": ok, "mode": "simulator", "steps": steps}

    def joint_state(self) -> JsonObject:
        return self.get_json("/joint_state")

    def apply_joint_targets(
        self,
        targets: dict[str, float],
        *,
        kp: float = 38.0,
        kd: float = 1.4,
        tau: float = 0.0,
        dq: float = 0.0,
    ) -> JsonObject:
        return self.command(
            "joint_targets",
            targets={str(name): float(value) for name, value in targets.items()},
            kp=float(kp),
            kd=float(kd),
            tau=float(tau),
            dq=float(dq),
        )

    def camera(self, action: str = "state", **fields: Any) -> CameraState:
        if action == "state" and not fields:
            return CameraState(self.get_json("/camera"))
        return CameraState(self.post_json("/camera", {"action": action, **fields}))

    def reset_camera(self) -> CameraState:
        return self.camera("reset")

    def orbit(self, dx: float, dy: float = 0.0) -> CameraState:
        return self.camera("orbit", dx=float(dx), dy=float(dy))

    def pan(self, dx: float, dy: float = 0.0) -> CameraState:
        return self.camera("pan", dx=float(dx), dy=float(dy))

    def zoom(self, delta: float) -> CameraState:
        return self.camera("zoom", delta=float(delta))

    def snapshot_bytes(self, image_format: str = "jpeg") -> bytes:
        suffix = "png" if image_format == "png" else "jpg"
        return self.get_bytes(f"/camera_frame_0.{suffix}")

    def snapshot(self, path: str | Path, image_format: str | None = None) -> Path:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        suffix = output.suffix.lower()
        fmt = image_format or ("png" if suffix == ".png" else "jpeg")
        output.write_bytes(self.snapshot_bytes(fmt))
        return output

    def get_json(self, path: str) -> JsonObject:
        data = self.get_bytes(path)
        try:
            value = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ProtocolError(f"GET {path} did not return JSON") from error
        if not isinstance(value, dict):
            raise ProtocolError(f"GET {path} returned {type(value).__name__}, expected object")
        return value

    def post_json(self, path: str, payload: JsonObject) -> JsonObject:
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        request = Request(
            self._url(path),
            data=data,
            headers={"content-type": "application/json"},
            method="POST",
        )
        response = self._open(request).read()
        try:
            value = json.loads(response.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ProtocolError(f"POST {path} did not return JSON") from error
        if not isinstance(value, dict):
            raise ProtocolError(f"POST {path} returned {type(value).__name__}, expected object")
        if value.get("ok") is False:
            raise ProtocolError(f"POST {path} failed: {value}")
        return value

    def get_bytes(self, path: str) -> bytes:
        return self._open(self._url(path)).read()

    def _url(self, path: str) -> str:
        normalized = path if path.startswith("/") else f"/{path}"
        return f"{self.endpoints.game_control_url}{normalized}"

    def _open(self, request_or_url: Request | str):
        try:
            return urlopen(request_or_url, timeout=self.timeout)
        except HTTPError as error:
            body = error.read().decode("utf-8", errors="replace")
            raise ProtocolError(f"HTTP {error.code}: {body}") from error
        except URLError as error:
            raise SimulatorUnavailable(str(error)) from error


def _optional_float(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _command_state_from_payload(status: JsonObject, lowstate: JsonObject) -> JsonObject:
    simulation = status.get("simulation") if isinstance(status.get("simulation"), dict) else {}
    lowcmd = simulation.get("lowcmd") if isinstance(simulation.get("lowcmd"), dict) else {}
    loco = simulation.get("loco") if isinstance(simulation.get("loco"), dict) else {}
    motion_switcher = simulation.get("motion_switcher") if isinstance(simulation.get("motion_switcher"), dict) else {}
    hand_sdk = simulation.get("hand_sdk") if isinstance(simulation.get("hand_sdk"), dict) else {}
    wireless_controller = simulation.get("wireless_controller") if isinstance(simulation.get("wireless_controller"), dict) else {}
    dex3 = simulation.get("dex3") if isinstance(simulation.get("dex3"), dict) else {}
    lowstate_lowcmd = lowstate.get("lowcmd") if isinstance(lowstate.get("lowcmd"), dict) else {}
    effective_lowcmd = {**lowstate_lowcmd, **lowcmd}
    dex3_hands = dex3.get("hands") if isinstance(dex3.get("hands"), dict) else {}
    velocity = loco.get("velocity") if isinstance(loco.get("velocity"), list) else []

    lowcmd_active = bool(effective_lowcmd.get("active"))
    lowcmd_stale = bool(effective_lowcmd.get("stale"))
    moving = any(abs(_optional_float(value) or 0.0) > 1e-6 for value in velocity)
    hand_intent = str(hand_sdk.get("intent") or "idle")
    dex3_active_hands = [
        hand
        for hand, state in dex3_hands.items()
        if isinstance(state, dict) and str(state.get("intent") or "idle") not in {"", "idle", "hold"}
    ]
    control_mode = str(simulation.get("control_mode") or "")
    pose = simulation.get("pose")

    if lowcmd_active and not lowcmd_stale:
        inferred_controller = "lowcmd"
    elif lowcmd_stale:
        inferred_controller = "lowcmd_stale"
    elif moving:
        inferred_controller = "locomotion"
    elif hand_intent not in {"", "idle", "hold"}:
        inferred_controller = "hand_sdk"
    elif (
        any(abs(_optional_float(wireless_controller.get(axis)) or 0.0) > 1e-6 for axis in ("lx", "ly", "rx", "ry"))
        or int(wireless_controller.get("keys") or 0)
    ):
        inferred_controller = "wireless_controller"
    elif dex3_active_hands:
        inferred_controller = "dex3"
    elif pose:
        inferred_controller = "pose"
    else:
        inferred_controller = "idle"

    return {
        "ok": bool(status.get("ready", False)),
        "captured_at_unix": time.time(),
        "inferred_controller": inferred_controller,
        "inference_source": "simulator_status_lowstate",
        "ready": bool(status.get("ready", False)),
        "paused": bool(simulation.get("paused")),
        "fallen": bool(simulation.get("fallen")),
        "pose": pose,
        "control_mode": control_mode or None,
        "loco": {
            "fsm_id": loco.get("fsm_id"),
            "fsm_mode": loco.get("fsm_mode"),
            "velocity": velocity,
            "velocity_until": loco.get("velocity_until"),
            "balance_mode": loco.get("balance_mode"),
            "stand_height": loco.get("stand_height"),
            "swing_height": loco.get("swing_height"),
            "phase": loco.get("phase"),
            "continuous_move": bool(loco.get("continuous_move")),
            "speed_mode": loco.get("speed_mode"),
            "control_owner": loco.get("control_owner"),
            "arm_task_id": loco.get("arm_task_id"),
        },
        "motion_switcher": motion_switcher,
        "lowcmd": {
            "topic": effective_lowcmd.get("topic"),
            "source": effective_lowcmd.get("source"),
            "active": lowcmd_active,
            "stale": lowcmd_stale,
            "age_seconds": effective_lowcmd.get("age_seconds"),
            "watchdog_seconds": effective_lowcmd.get("watchdog_seconds"),
            "received_at": effective_lowcmd.get("received_at"),
            "expires_at": effective_lowcmd.get("expires_at"),
            "motor_cmd_count": effective_lowcmd.get("motor_cmd_count"),
            "applied_position_targets": effective_lowcmd.get("applied_position_targets"),
            "accepted_indices": effective_lowcmd.get("accepted_indices"),
            "ignored_indices": effective_lowcmd.get("ignored_indices"),
            "clamped_indices": effective_lowcmd.get("clamped_indices"),
            "mode_pr": lowstate.get("mode_pr", effective_lowcmd.get("mode_pr")),
            "mode_machine": lowstate.get("mode_machine", effective_lowcmd.get("mode_machine")),
            "crc": lowstate.get("crc", effective_lowcmd.get("crc")),
        },
        "hand_sdk": {
            "topic": hand_sdk.get("topic"),
            "intent": hand_intent,
            "motor_count": hand_sdk.get("motor_count"),
            "weight": hand_sdk.get("weight"),
            "tau": hand_sdk.get("tau"),
        },
        "wireless_controller": {
            "topic": wireless_controller.get("topic"),
            "lx": wireless_controller.get("lx"),
            "ly": wireless_controller.get("ly"),
            "rx": wireless_controller.get("rx"),
            "ry": wireless_controller.get("ry"),
            "keys": wireless_controller.get("keys"),
            "received_at": wireless_controller.get("received_at"),
        },
        "dex3": {
            "active_hands": dex3_active_hands,
            "hands": {
                hand: {
                    "intent": state.get("intent"),
                    "topic": state.get("topic"),
                    "motor_count": state.get("motor_count"),
                }
                for hand, state in dex3_hands.items()
                if isinstance(state, dict)
            },
        },
        "lowstate": {
            "mode_machine": lowstate.get("mode_machine"),
            "mode_pr": lowstate.get("mode_pr"),
            "crc": lowstate.get("crc"),
            "motor_count": len(lowstate.get("motor_state") or []) if isinstance(lowstate.get("motor_state"), list) else None,
        },
    }
