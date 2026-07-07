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

    def lowstate(self) -> JsonObject:
        return self.get_json("/lowstate")

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
