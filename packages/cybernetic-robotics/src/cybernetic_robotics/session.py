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
        official: Any | None = None,
    ):
        self.config = config or UnitreeTransportConfig.from_env()
        self.simulator = simulator or SimulatorClient(self.config.endpoints)
        self.official = official

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
            "official_sidecar": None,
            "topics": {},
        }
        warnings = diagnostics["warnings"]

        official_status: dict[str, Any] | None = None
        if self.config.transport == DDS and self.config.mode == SIM:
            try:
                official_status = self._official_status()
                diagnostics["official_sidecar"] = _summarize_official_status(official_status)
                if diagnostics["official_sidecar"]["ok"]:
                    diagnostics["implemented"] = True
                else:
                    warnings.append("official SDK2 sidecar status did not pass")
            except Exception as error:  # noqa: BLE001 - diagnostics should expose setup problems.
                warnings.append(f"official SDK2 sidecar unavailable: {error}")
                diagnostics["official_sidecar"] = {"ok": False, "error": str(error)}
        elif self.config.transport == DDS:
            warnings.append(
                "dds transport is selected outside simulator mode; real hardware control still requires a long-lived official provider"
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
            if self.config.transport != DDS:
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

        if official_status:
            sdk2_probe = official_status.get("sdk2_probe") or {}
            channels = sdk2_probe.get("channels") if isinstance(sdk2_probe, dict) else {}
            if isinstance(channels, dict):
                lowcmd = channels.get("rt/lowcmd")
                if isinstance(lowcmd, dict):
                    diagnostics["topics"]["rt/lowcmd"] = {
                        **diagnostics["topics"].get("rt/lowcmd", {}),
                        "source": "official_sdk2_sidecar",
                        "created": lowcmd.get("created"),
                        "role": lowcmd.get("role"),
                        "sample_motor_count": lowcmd.get("sample_motor_count"),
                    }
                lowstate = channels.get("rt/lowstate")
                if isinstance(lowstate, dict):
                    diagnostics["topics"]["rt/lowstate"] = {
                        **diagnostics["topics"].get("rt/lowstate", {}),
                        "source": "official_sdk2_sidecar",
                        "created": lowstate.get("created"),
                        "role": lowstate.get("role"),
                    }

        diagnostics["ok"] = bool(diagnostics["ok"] and not any("requires" in item for item in warnings))
        return diagnostics

    def provider_status(self) -> dict[str, Any]:
        """Summarize which Unitree provider is active and what it can do."""

        diagnostics = self.diagnostics()
        config = diagnostics["config"]
        transport = config["transport"]
        mode = config["mode"]
        official_ok = bool((diagnostics.get("official_sidecar") or {}).get("ok"))
        simulator_reachable = bool((diagnostics.get("simulator") or {}).get("reachable"))

        if transport == LOCAL_HTTP and mode == SIM:
            provider = "local_http_simulator"
            implemented = simulator_reachable
            command_path = "Cybernetic GameControl HTTP commands plus the Booster-style physics WebSocket."
            telemetry_path = "Simulator HTTP /status, /lowstate, /joint_state, and rendered camera frames."
            motion = {
                "arm_actions": "simulator_named_poses",
                "locomotion": "kinematic_base_velocity",
                "lowcmd": "simulator_joint_targets",
            }
            limitations = [
                "No CycloneDDS transport is used.",
                "Locomotion is a local approximation, not Unitree's whole-body balance controller.",
            ]
            next_step = "Use CYBER_UNITREE_TRANSPORT=dds in simulator mode when testing the official SDK2 sidecar path."
        elif transport == DDS and mode == SIM:
            provider = "official_mujoco_dds_simulator" if official_ok else "official_mujoco_dds_simulator_unready"
            implemented = official_ok
            command_path = "Official SDK2/CycloneDDS sidecar for supported arm poses; local HTTP remains the fallback for viewer and local loco tools."
            telemetry_path = "Official sidecar rt/lowcmd/rt/lowstate probes plus local simulator diagnostics when available."
            motion = {
                "arm_actions": "managed_official_mujoco_session_for_supported_poses" if official_ok else "unavailable_until_sidecar_ready",
                "locomotion": "local_http_compatibility_until_dds_loco_provider_lands",
                "lowcmd": "official_probe_or_local_http_depending_on_tool",
            }
            limitations = [
                "Only bounded arm-pose commands are routed through the managed official DDS session today.",
                "LocoClient locomotion and generic lowcmd streaming still need the long-lived DDS provider.",
            ]
            next_step = "Start or inspect the managed official MuJoCo session, then promote loco/lowcmd paths to that provider."
        else:
            provider = "real_unitree_dds"
            implemented = False
            command_path = "Not enabled: real hardware requires an explicit provider, interface, unlock, and safety model."
            telemetry_path = "Not enabled until real-mode DDS safety gates are implemented."
            motion = {"arm_actions": "disabled", "locomotion": "disabled", "lowcmd": "disabled"}
            limitations = [
                "Real hardware control is intentionally locked.",
                "Set CYBER_UNITREE_NETWORK_INTERFACE and the real unlock only after the real provider is implemented and reviewed.",
            ]
            next_step = "Finish the simulator DDS provider and safety gates before enabling physical robot control."

        return {
            "ok": bool(diagnostics["ok"] and implemented),
            "provider": provider,
            "implemented": implemented,
            "command_path": command_path,
            "telemetry_path": telemetry_path,
            "motion": motion,
            "limitations": limitations,
            "next_step": next_step,
            "config": config,
            "warnings": diagnostics["warnings"],
            "diagnostics_summary": {
                "simulator_reachable": simulator_reachable,
                "official_sidecar_ok": official_ok,
                "topics": diagnostics["topics"],
            },
        }

    def execute_arm_action(self, action_id: int, pose: str, *, timeout: float | None = None) -> dict[str, Any]:
        """Execute a Unitree-shaped arm action through the active provider.

        This is the transport boundary used by the local `unitree_sdk2py` shim.
        In the default simulator mode it preserves the existing local HTTP pose
        behavior. In simulator DDS mode it routes the supported bounded poses to
        the managed official Unitree MuJoCo + SDK2/CycloneDDS session.
        """

        if self.config.transport == LOCAL_HTTP and self.config.mode == SIM:
            response = self.simulator.pose(pose)
            return {
                **response,
                "transport": LOCAL_HTTP,
                "provider": "local_http_simulator",
                "unitree_action_id": int(action_id),
                "pose": pose,
            }

        if self.config.transport == DDS and self.config.mode == SIM:
            if pose not in {"raise_right_hand", "raise_left_hand"}:
                return {
                    "ok": False,
                    "transport": DDS,
                    "provider": "official_mujoco_dds_simulator",
                    "unitree_action_id": int(action_id),
                    "pose": pose,
                    "error": f"official DDS simulator session does not yet support arm pose: {pose}",
                    "supported_poses": ["raise_right_hand", "raise_left_hand"],
                    "next_step": "Add this pose to the managed official arm-pose lowcmd table before routing it through DDS.",
                }
            official = self._official(timeout=timeout)
            if pose == "raise_left_hand":
                response = official.raise_left_hand_session()
            else:
                response = official.raise_right_hand_session()
            return {
                **response,
                "transport": DDS,
                "provider": "official_mujoco_dds_simulator",
                "unitree_action_id": int(action_id),
                "pose": pose,
            }

        return {
            "ok": False,
            "transport": self.config.transport,
            "provider": "real_unitree_dds",
            "unitree_action_id": int(action_id),
            "pose": pose,
            "error": "real Unitree DDS arm actions are locked until the real-hardware provider and safety gates are implemented",
            "next_step": "Use simulator mode for SDK2/DDS development, then add reviewed real-mode safety gates.",
        }

    def read_lowstate(self) -> dict[str, Any]:
        """Read lowstate telemetry through the active provider when possible."""

        if self.config.transport == DDS and self.config.mode == SIM:
            response = self._official().lowstate_session()
            return {
                **response,
                "transport": DDS,
                "provider": "official_mujoco_dds_simulator",
            }
        return {
            "ok": True,
            "transport": LOCAL_HTTP,
            "provider": "local_http_simulator",
            "lowstate": self.simulator.lowstate(),
        }

    def _official_status(self) -> dict[str, Any]:
        return self._official().status()

    def _official(self, *, timeout: float | None = None) -> Any:
        if self.official is not None:
            return self.official
        from .official import OfficialG1Sim

        if timeout is None:
            return OfficialG1Sim.discover()
        return OfficialG1Sim.discover(timeout=max(30, int(round(timeout))))


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


def _summarize_official_status(status: dict[str, Any]) -> dict[str, Any]:
    sdk2_probe = status.get("sdk2_probe") or {}
    peer = status.get("official_mujoco_peer") or {}
    return {
        "ok": bool(status.get("ok")),
        "source": status.get("source"),
        "domain_initialized": bool(sdk2_probe.get("domain_initialized")) if isinstance(sdk2_probe, dict) else False,
        "dds_domain_id": sdk2_probe.get("domain") if isinstance(sdk2_probe, dict) else None,
        "network_interface": sdk2_probe.get("network_interface") if isinstance(sdk2_probe, dict) else None,
        "lowcmd_channel_created": bool((sdk2_probe.get("channels") or {}).get("rt/lowcmd", {}).get("created"))
        if isinstance(sdk2_probe, dict)
        else False,
        "lowstate_channel_created": bool((sdk2_probe.get("channels") or {}).get("rt/lowstate", {}).get("created"))
        if isinstance(sdk2_probe, dict)
        else False,
        "official_mujoco_binary_exists": bool(peer.get("binary_exists")) if isinstance(peer, dict) else False,
        "official_mujoco_scene_exists": bool(peer.get("scene_exists")) if isinstance(peer, dict) else False,
        "expected_topics": status.get("expected_topics", []),
        "next_step": status.get("next_step"),
    }
