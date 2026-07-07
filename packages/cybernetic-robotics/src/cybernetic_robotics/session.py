from __future__ import annotations

from dataclasses import dataclass
import json
import os
from typing import Any

from .config import RobotEndpoints
from .simulator import SimulatorClient


LOCAL_HTTP = "local_http"
DDS = "dds"
RPC_BRIDGE = "rpc_bridge"
SIM = "sim"
REAL = "real"


@dataclass(frozen=True)
class UnitreeTransportConfig:
    """Session-level transport choice for Unitree-shaped user code.

    `local_http` is the lightweight simulator bridge. `dds` targets the
    managed official Unitree MuJoCo sidecar for supported lowcmd/lowstate
    probes. `rpc_bridge` is the simulator-only Unitree RPC service adapter for
    high-level sport/agv calls.
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
        transport = _choice(os.environ.get("CYBER_UNITREE_TRANSPORT"), {LOCAL_HTTP, DDS, RPC_BRIDGE}, LOCAL_HTTP)
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
        if self.config.transport in {DDS, RPC_BRIDGE} and self.config.mode == SIM:
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
        elif self.config.transport in {DDS, RPC_BRIDGE}:
            warnings.append(
                f"{self.config.transport} transport is selected outside simulator mode; real hardware control still requires a long-lived official provider"
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
            next_step = "Use CYBER_UNITREE_TRANSPORT=rpc_bridge for high-level sport/agv tests, or dds for official lowcmd/lowstate sidecar probes."
        elif transport == RPC_BRIDGE and mode == SIM:
            provider = "unitree_rpc_bridge_simulator" if official_ok else "unitree_rpc_bridge_simulator_unready"
            implemented = official_ok
            command_path = "Official SDK2-shaped sport/agv RPC bridge backed by the local simulator provider."
            telemetry_path = "Bridge command evidence plus simulator HTTP /status, /lowstate, /joint_state, and rendered camera frames."
            motion = {
                "arm_actions": "sport_set_arm_task_bridge_for_wave_shake_plus_local_arm_facade",
                "locomotion": "managed_unitree_rpc_bridge_sport_agv",
                "lowcmd": "local_http_simulator_until_generic_dds_streaming_lands",
            }
            limitations = [
                "This is a simulator-side service bridge, not physical robot DDS control.",
                "Only the mapped sport/agv RPC subset is available; generic lowcmd streaming remains separate.",
            ]
            next_step = "Use CYBER_UNITREE_TRANSPORT=rpc_bridge for high-level LocoClient/AgvClient tests, then promote lowcmd streaming separately."
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

    def execute_loco_command(self, action: str, **fields: Any) -> dict[str, Any]:
        """Execute a Unitree-shaped locomotion command through the active provider.

        The official G1 `LocoClient` is RPC-over-DDS. Cybernetic has not yet
        promoted every call into the official MuJoCo DDS peer. The explicit
        `rpc_bridge` transport routes supported high-level sport/agv methods
        through the managed Unitree RPC bridge, while `dds` remains the
        lowcmd/lowstate official-session path.
        """

        if self.config.mode == REAL:
            return {
                "ok": False,
                "transport": self.config.transport,
                "provider": "real_unitree_dds",
                "action": action,
                "error": "real Unitree locomotion is locked until the real-hardware provider and safety gates are implemented",
                "next_step": "Implement and review the real SDK2 LocoClient provider before enabling physical locomotion.",
            }

        if self.config.transport == RPC_BRIDGE and self.config.mode == SIM:
            request = _loco_rpc_bridge_request(action, fields)
            if request is None:
                return {
                    "ok": False,
                    "transport": RPC_BRIDGE,
                    "provider": "unitree_rpc_bridge_simulator",
                    "action": action,
                    "fields": fields,
                    "error": f"Unitree RPC bridge does not yet support loco action: {action}",
                    "supported_actions": sorted(_LOCO_RPC_BRIDGE_METHODS),
                    "next_step": "Use local_http for this compatibility-only call or add a bridge handler for the matching Unitree sport API id.",
                }
            official = self._official(timeout=fields.get("timeout"))
            result = official.rpc_bridge_command(
                service="sport",
                method=request["method"],
                params=request["params"],
                timeout=float(fields.get("timeout", 1.0) or 1.0),
                start_if_needed=True,
            )
            return _normalize_rpc_bridge_command_response(RPC_BRIDGE, "unitree_rpc_bridge_simulator", action, result)

        response = self.simulator.loco(action=action, **fields)
        if self.config.transport == DDS:
            return {
                **response,
                "transport": DDS,
                "provider": "local_http_simulator_compatibility",
                "official_dds_supported": False,
                "compatibility_fallback": True,
                "next_step": "Promote Unitree LocoClient RPCs into the managed official SDK2/CycloneDDS sidecar.",
            }
        return {
            **response,
            "transport": LOCAL_HTTP,
            "provider": "local_http_simulator",
            "compatibility_fallback": False,
        }

    def execute_agv_command(self, action: str, **fields: Any) -> dict[str, Any]:
        """Execute a Unitree-shaped AGV command through the active provider."""

        if self.config.mode == REAL:
            return {
                "ok": False,
                "transport": self.config.transport,
                "provider": "real_unitree_dds",
                "action": action,
                "error": "real Unitree AGV control is locked until the real-hardware provider and safety gates are implemented",
                "next_step": "Implement and review the real SDK2 AGV provider before enabling physical movement.",
            }

        if self.config.transport == RPC_BRIDGE and self.config.mode == SIM:
            request = _agv_rpc_bridge_request(action, fields)
            if request is None:
                return {
                    "ok": False,
                    "transport": RPC_BRIDGE,
                    "provider": "unitree_rpc_bridge_simulator",
                    "action": action,
                    "fields": fields,
                    "error": f"Unitree RPC bridge does not yet support AGV action: {action}",
                    "supported_actions": sorted(_AGV_RPC_BRIDGE_METHODS),
                }
            official = self._official(timeout=fields.get("timeout"))
            result = official.rpc_bridge_command(
                service="agv",
                method=request["method"],
                params=request["params"],
                timeout=float(fields.get("timeout", 1.0) or 1.0),
                start_if_needed=True,
            )
            return _normalize_rpc_bridge_command_response(RPC_BRIDGE, "unitree_rpc_bridge_simulator", action, result)

        if action == "move":
            response = self.simulator.loco(
                action="set_velocity",
                velocity=fields["velocity"],
                duration=fields.get("duration", 1.0),
            )
        else:
            response = self.simulator.command(
                "agv",
                action=action,
                service="agv",
                simulated=True,
                **fields,
            )
        if response.get("ok"):
            return {
                **response,
                "transport": LOCAL_HTTP if self.config.transport != DDS else DDS,
                "provider": "local_http_simulator" if self.config.transport != DDS else "local_http_simulator_compatibility",
                "compatibility_fallback": self.config.transport == DDS,
                "agv": {"action": action, "service": "agv", "simulated": True, **fields},
            }
        if action == "height_adjust":
            return {
                "ok": True,
                "transport": LOCAL_HTTP if self.config.transport != DDS else DDS,
                "provider": "local_http_simulator" if self.config.transport != DDS else "local_http_simulator_compatibility",
                "compatibility_fallback": self.config.transport == DDS,
                "action": action,
                "service": "agv",
                "simulated": True,
                **fields,
                "agv": {"action": action, "service": "agv", "simulated": True, **fields},
            }
        return response

    def publish_lowcmd(
        self,
        topic: str,
        motor_cmd: list[dict[str, Any]],
        *,
        mode_pr: int = 0,
        mode_machine: int = 0,
        crc: int = 0,
        timeout: float | None = None,
    ) -> dict[str, Any]:
        """Publish low-level motor commands through the active provider boundary."""

        if topic not in {"rt/lowcmd", "rt/arm_sdk"}:
            raise NotImplementedError(f"Cybernetic Unitree session publisher does not support {topic}")
        if self.config.mode == REAL:
            return {
                "ok": False,
                "transport": self.config.transport,
                "provider": "real_unitree_dds",
                "topic": topic,
                "error": "real Unitree lowcmd publishing is locked until the real-hardware provider and safety gates are implemented",
                "next_step": "Implement reviewed real-mode lowcmd safety gates before enabling physical command publishing.",
            }

        previous_timeout = self.simulator.timeout
        if timeout is not None:
            self.simulator.timeout = float(timeout)
        try:
            response = self.simulator.lowcmd(
                motor_cmd,
                topic=topic,
                mode_pr=int(mode_pr),
                mode_machine=int(mode_machine),
                crc=int(crc),
            )
        finally:
            self.simulator.timeout = previous_timeout
        if self.config.transport == DDS:
            return {
                **response,
                "transport": DDS,
                "provider": "local_http_simulator_compatibility",
                "official_dds_supported": False,
                "compatibility_fallback": True,
                "next_step": "Promote generic lowcmd streaming into the managed official SDK2/CycloneDDS sidecar.",
            }
        return {
            **response,
            "transport": LOCAL_HTTP,
            "provider": "local_http_simulator",
            "compatibility_fallback": False,
        }

    def read_lowstate(self) -> dict[str, Any]:
        """Read lowstate telemetry through the active provider when possible."""

        if self.config.mode == REAL:
            return {
                "ok": False,
                "transport": self.config.transport,
                "provider": "real_unitree_dds",
                "error": "real Unitree lowstate reading is locked until the real-hardware provider and safety gates are implemented",
                "next_step": "Implement the reviewed real-mode DDS telemetry provider before reading physical robot state.",
            }
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


_LOCO_RPC_BRIDGE_METHODS = {
    "get_fsm_id",
    "get_fsm_mode",
    "get_balance_mode",
    "get_swing_height",
    "get_stand_height",
    "set_fsm_id",
    "set_balance_mode",
    "set_swing_height",
    "set_stand_height",
    "set_velocity",
    "set_arm_task",
    "wave_hand",
    "shake_hand",
}

_AGV_RPC_BRIDGE_METHODS = {"move", "height_adjust"}


def _loco_rpc_bridge_request(action: str, fields: dict[str, Any]) -> dict[str, Any] | None:
    if action in {
        "get_fsm_id",
        "get_fsm_mode",
        "get_balance_mode",
        "get_swing_height",
        "get_stand_height",
        "wave_hand",
        "shake_hand",
    }:
        return {"method": action, "params": dict(fields)}
    if action == "set_velocity":
        velocity = fields.get("velocity") or [fields.get("vx", 0.0), fields.get("vy", 0.0), fields.get("omega", 0.0)]
        return {"method": "move", "params": {"velocity": velocity, "duration": fields.get("duration", 0.0)}}
    if action in {"set_fsm_id", "set_balance_mode", "set_swing_height", "set_stand_height", "set_arm_task"}:
        return {"method": action, "params": dict(fields)}
    return None


def _agv_rpc_bridge_request(action: str, fields: dict[str, Any]) -> dict[str, Any] | None:
    if action == "move":
        velocity = fields.get("requested_velocity") or fields.get("velocity") or [0.0, 0.0, 0.0]
        return {
            "method": "move",
            "params": {
                "vx": float(velocity[0]) if len(velocity) > 0 else 0.0,
                "vy": float(velocity[1]) if len(velocity) > 1 else 0.0,
                "vyaw": float(velocity[2]) if len(velocity) > 2 else 0.0,
            },
        }
    if action == "height_adjust":
        return {"method": "height_adjust", "params": {"vz": fields.get("height_velocity", fields.get("vz", 0.0))}}
    return None


def _normalize_rpc_bridge_command_response(
    transport: str,
    provider: str,
    action: str,
    result: dict[str, Any],
) -> dict[str, Any]:
    body = _rpc_bridge_body(result)
    normalized = {
        "ok": bool(result.get("ok")),
        "transport": transport,
        "provider": provider,
        "action": action,
        "bridge_result": result,
        "summary": result.get("summary", {}),
        "compatibility_fallback": False,
    }
    if isinstance(body, dict):
        normalized.update(body)
        if "data" in body:
            key = {
                "get_fsm_id": "fsm_id",
                "get_fsm_mode": "fsm_mode",
                "get_balance_mode": "balance_mode",
                "get_swing_height": "swing_height",
                "get_stand_height": "stand_height",
                "height_adjust": "height_velocity",
            }.get(action)
            if key:
                normalized[key] = body["data"]
        if "velocity" in body:
            normalized["velocity"] = body["velocity"]
        if action == "height_adjust" and "data" in body:
            normalized["height_velocity"] = body["data"]
    if not normalized["ok"] and result.get("error"):
        normalized["error"] = result["error"]
    return normalized


def _rpc_bridge_body(result: dict[str, Any]) -> dict[str, Any]:
    calls = result.get("calls")
    if not isinstance(calls, list) or not calls:
        calls = ((result.get("command_report") or {}).get("calls") if isinstance(result.get("command_report"), dict) else [])
    if not isinstance(calls, list) or not calls:
        return {}
    value = calls[0].get("return") if isinstance(calls[0], dict) else None
    if isinstance(value, (list, tuple)) and len(value) >= 2 and isinstance(value[1], str):
        try:
            parsed = json.loads(value[1])
        except json.JSONDecodeError:
            return {}
        return parsed if isinstance(parsed, dict) else {}
    return {}


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
