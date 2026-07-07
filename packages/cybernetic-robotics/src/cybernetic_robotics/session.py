from __future__ import annotations

from dataclasses import dataclass
import os
from typing import Any

from .config import RobotEndpoints
from .simulator import SimulatorClient


LOCAL_HTTP = "local_http"
DDS = "dds"
SIM = "sim"
REAL = "real"


@dataclass(frozen=True)
class UnitreeTransportConfig:
    """Session-level transport choice for Unitree-shaped user code.

    Today `local_http` is the implemented simulator bridge. The explicit `dds`
    mode is still useful: it lets tools, docs, and examples describe the future
    official SDK2 path without pretending that the compatibility shim already
    owns CycloneDDS.
    """

    transport: str = LOCAL_HTTP
    mode: str = SIM
    dds_domain_id: int = 1
    network_interface: str | None = "lo"
    safety_profile: str = "simulator"
    real_unlocked: bool = False
    endpoints: RobotEndpoints = RobotEndpoints()

    @classmethod
    def from_env(cls, endpoints: RobotEndpoints | None = None) -> "UnitreeTransportConfig":
        mode = _choice(os.environ.get("CYBER_UNITREE_MODE"), {SIM, REAL}, SIM)
        transport = _choice(os.environ.get("CYBER_UNITREE_TRANSPORT"), {LOCAL_HTTP, DDS}, LOCAL_HTTP)
        default_domain = 1 if mode == SIM else 0
        dds_domain_id = _int_env("CYBER_UNITREE_DDS_DOMAIN", default_domain)
        network_interface = os.environ.get("CYBER_UNITREE_NETWORK_INTERFACE")
        if network_interface is None and mode == SIM:
            network_interface = "lo"
        return cls(
            transport=transport,
            mode=mode,
            dds_domain_id=dds_domain_id,
            network_interface=network_interface or None,
            safety_profile=os.environ.get("CYBER_UNITREE_SAFETY_PROFILE", "simulator" if mode == SIM else "real"),
            real_unlocked=os.environ.get("CYBER_UNITREE_REAL_UNLOCK") == "I_UNDERSTAND_THIS_CONTROLS_REAL_HARDWARE",
            endpoints=endpoints or RobotEndpoints.from_env(),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "transport": self.transport,
            "mode": self.mode,
            "dds_domain_id": self.dds_domain_id,
            "network_interface": self.network_interface,
            "safety_profile": self.safety_profile,
            "real_unlocked": self.real_unlocked,
            "endpoints": {
                "game_control_url": self.endpoints.game_control_url,
                "physics_url": self.endpoints.physics_url,
            },
        }


class UnitreeSession:
    """Diagnostics facade for the current Unitree G1 control session."""

    def __init__(
        self,
        config: UnitreeTransportConfig | None = None,
        simulator: SimulatorClient | None = None,
    ):
        self.config = config or UnitreeTransportConfig.from_env()
        self.simulator = simulator or SimulatorClient(self.config.endpoints)

    @classmethod
    def from_env(cls) -> "UnitreeSession":
        return cls(UnitreeTransportConfig.from_env())

    def diagnostics(self) -> dict[str, Any]:
        channel_factory = _channel_factory()
        diagnostics: dict[str, Any] = {
            "ok": True,
            "implemented": self.config.transport == LOCAL_HTTP,
            "config": self.config.to_dict(),
            "channel_factory": channel_factory,
            "warnings": [],
            "simulator": None,
            "topics": {},
        }
        warnings = diagnostics["warnings"]

        if self.config.transport == DDS:
            warnings.append(
                "dds transport is selected but Cybernetic's Python package still uses the local_http simulator shim"
            )
        if self.config.mode == REAL:
            if not self.config.network_interface:
                warnings.append("real mode requires CYBER_UNITREE_NETWORK_INTERFACE")
            if not self.config.real_unlocked:
                warnings.append("real mode is locked; set CYBER_UNITREE_REAL_UNLOCK only when intentionally controlling hardware")

        try:
            status = self.simulator.status()
            diagnostics["simulator"] = {
                "reachable": True,
                "ready": status.ready,
                "pose": status.pose,
                "paused": status.paused,
                "fallen": status.fallen,
                "model_path": status.model_path,
            }
            lowcmd = status.lowcmd
            diagnostics["topics"]["rt/lowcmd"] = {
                "source": self.config.transport,
                "active": status.lowcmd_active,
                "stale": status.lowcmd_stale,
                "age_seconds": status.lowcmd_age_seconds,
                "watchdog_seconds": status.lowcmd_watchdog_seconds,
                "last_received_at": lowcmd.get("received_at"),
                "message_count": lowcmd.get("motor_cmd_count"),
            }
        except Exception as error:  # noqa: BLE001 - diagnostics should report, not hide, runtime state.
            diagnostics["ok"] = False
            diagnostics["simulator"] = {"reachable": False, "error": str(error)}

        try:
            lowstate = self.simulator.lowstate()
            diagnostics["topics"]["rt/lowstate"] = {
                "source": self.config.transport,
                "available": True,
                "motor_count": len(lowstate.get("motor_state", [])),
                "mode_machine": lowstate.get("mode_machine"),
                "mode_pr": lowstate.get("mode_pr"),
                "crc": lowstate.get("crc"),
                "lowcmd_active": (lowstate.get("lowcmd") or {}).get("active"),
                "lowcmd_stale": (lowstate.get("lowcmd") or {}).get("stale"),
            }
        except Exception as error:  # noqa: BLE001
            diagnostics["topics"]["rt/lowstate"] = {"available": False, "error": str(error)}

        diagnostics["ok"] = bool(diagnostics["ok"] and not any("requires" in item for item in warnings))
        return diagnostics


def _choice(value: str | None, allowed: set[str], default: str) -> str:
    if value is None:
        return default
    normalized = value.strip().lower().replace("-", "_")
    return normalized if normalized in allowed else default


def _int_env(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _channel_factory() -> dict[str, Any]:
    try:
        from unitree_sdk2py.core.channel import current_channel_factory_config

        config = current_channel_factory_config()
        return {
            "initialized": bool(getattr(config, "initialized", False)),
            "domain_id": config.domain_id,
            "network_interface": config.network_interface,
        }
    except Exception as error:  # noqa: BLE001
        return {"initialized": False, "error": str(error)}
