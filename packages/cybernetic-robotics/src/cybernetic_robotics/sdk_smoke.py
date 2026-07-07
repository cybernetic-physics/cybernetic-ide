from __future__ import annotations

from contextlib import contextmanager
import json
import os
from pathlib import Path
from typing import Any

from .config import RobotEndpoints
from .g1 import G1Robot


def run_official_g1_sdk_smoke(
    kind: str = "all",
    endpoints: RobotEndpoints | None = None,
    output_path: str | Path | None = None,
    transport: str | None = None,
) -> dict[str, Any]:
    """Run safe behavior-level smoke checks through Unitree SDK-shaped imports."""

    requested = _normalize_kind(kind)
    normalized_transport = _normalize_transport(transport)
    cleanup = None
    with _temporary_transport(normalized_transport):
        bridge_was_running = _rpc_bridge_running(normalized_transport)
        results = {}
        if requested in {"all", "arm"}:
            results["arm"] = _smoke_arm()
        if requested in {"all", "loco"}:
            results["loco"] = _smoke_loco()
        if requested in {"all", "lowcmd"}:
            results["lowcmd"] = _smoke_lowcmd()
        if requested in {"all", "hand"}:
            results["hand"] = _smoke_hand()

        robot = G1Robot.connect(endpoints=endpoints)
        status = robot.status()
        safety = robot.safety_check()
        if normalized_transport == "rpc_bridge" and not bridge_was_running:
            cleanup = _stop_rpc_bridge()
        report = {
            "ok": all(item.get("ok") for item in results.values()) and safety.get("safe_to_command", False),
            "kind": requested,
            "transport": normalized_transport or os.environ.get("CYBER_UNITREE_TRANSPORT", "local_http"),
            "results": results,
            "status": {
                "ready": status.ready,
                "pose": status.pose,
                "paused": status.paused,
                "speed": status.speed,
                "lowcmd_active": status.lowcmd_active,
                "lowcmd_stale": status.lowcmd_stale,
                "lowcmd_age_seconds": status.lowcmd_age_seconds,
            },
            "safety": safety,
            "cleanup": cleanup,
            "agent_hints": [
                "This is behavior-level smoke coverage for safe official-style calls, not full physical parity.",
                "Use sdk-audit first for import/method coverage, then sdk-smoke for simulator behavior evidence.",
                "Use transport=rpc_bridge to prove normal LocoClient/AgvClient/G1ArmActionClient calls cross the managed Unitree RPC bridge.",
                "Run safety_stop after manual experiments that leave locomotion or lowcmd active.",
            ],
        }
    if output_path is not None:
        path = Path(output_path)
        report["output_path"] = str(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2, default=str) + "\n", encoding="utf-8")
    return report


def _normalize_kind(kind: str) -> str:
    value = (kind or "all").strip().lower()
    if value not in {"all", "arm", "loco", "lowcmd", "hand"}:
        raise ValueError(f"unknown SDK smoke kind: {kind}")
    return value


def _normalize_transport(transport: str | None) -> str | None:
    if transport is None:
        return None
    value = transport.strip().lower().replace("-", "_")
    if value not in {"local_http", "rpc_bridge", "dds"}:
        raise ValueError(f"unknown SDK smoke transport: {transport}")
    return value


@contextmanager
def _temporary_transport(transport: str | None):
    previous = os.environ.get("CYBER_UNITREE_TRANSPORT")
    if transport is not None:
        os.environ["CYBER_UNITREE_TRANSPORT"] = transport
    try:
        yield
    finally:
        if transport is not None:
            if previous is None:
                os.environ.pop("CYBER_UNITREE_TRANSPORT", None)
            else:
                os.environ["CYBER_UNITREE_TRANSPORT"] = previous


def _rpc_bridge_running(transport: str | None) -> bool:
    if transport != "rpc_bridge":
        return False
    try:
        from .official import OfficialG1Sim

        status = OfficialG1Sim.discover().rpc_bridge_status()
        return bool(status.get("running"))
    except Exception:
        return False


def _stop_rpc_bridge() -> dict[str, Any]:
    try:
        from .official import OfficialG1Sim

        return OfficialG1Sim.discover().stop_rpc_bridge()
    except Exception as error:  # noqa: BLE001 - cleanup should not hide smoke results.
        return {"ok": False, "error": str(error)}


def _smoke_arm() -> dict[str, Any]:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map

    ChannelFactoryInitialize(0, "cyber-sim")
    arm = G1ArmActionClient()
    arm.SetTimeout(10.0)
    arm.Init()
    code = arm.ExecuteAction(action_map["right hand up"])
    return {
        "ok": code == 0,
        "client": "G1ArmActionClient",
        "action": "right hand up",
        "code": code,
        "last_response": arm.last_response,
    }


def _smoke_loco() -> dict[str, Any]:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    ChannelFactoryInitialize(0, "cyber-sim")
    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()
    steps = [
        ("Damp", loco.Damp()),
        ("Squat2StandUp", loco.Squat2StandUp()),
        ("StandUp2Squat", loco.StandUp2Squat()),
        ("LowStand", loco.LowStand()),
        ("HighStand", loco.HighStand()),
        ("SwitchMoveMode", loco.SwitchMoveMode(True)),
        ("SetSpeedMode", loco.SetSpeedMode(2)),
        ("SwitchToUserCtrl", loco.SwitchToUserCtrl()),
        ("SwitchToInternalCtrl", loco.SwitchToInternalCtrl(2)),
        ("Move", loco.Move(0.08, 0.0, 0.0)),
        ("StopMove", loco.StopMove()),
        ("WaveHand", loco.WaveHand(False)),
        ("ShakeHand", loco.ShakeHand()),
    ]
    return {
        "ok": all(code == 0 for _name, code in steps),
        "client": "LocoClient",
        "steps": [{"method": name, "code": code} for name, code in steps],
        "last_response": loco.last_response,
    }


def _smoke_lowcmd() -> dict[str, Any]:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

    ChannelFactoryInitialize(0, "cyber-sim")
    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init()
    lowstate = subscriber.Read()

    lowcmd = unitree_hg_msg_dds__LowCmd_()
    lowcmd.mode_pr = 0
    lowcmd.mode_machine = lowstate.mode_machine
    for index, motor in enumerate(lowstate.motor_state):
        lowcmd.motor_cmd[index].mode = 1
        lowcmd.motor_cmd[index].q = motor.q
        lowcmd.motor_cmd[index].dq = 0.0
        lowcmd.motor_cmd[index].tau = 0.0
        lowcmd.motor_cmd[index].kp = 8.0
        lowcmd.motor_cmd[index].kd = 0.5
    lowcmd.motor_cmd[22].q = lowstate.motor_state[22].q - 0.05
    lowcmd.motor_cmd[25].q = lowstate.motor_state[25].q + 0.05
    lowcmd.crc = CRC().Crc(lowcmd)

    publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
    publisher.Init()
    wrote = publisher.Write(lowcmd)
    final_state = subscriber.Read()
    subscriber.Close()
    publisher.Close()
    return {
        "ok": bool(wrote),
        "client": "ChannelPublisher/ChannelSubscriber",
        "topic_write": "rt/lowcmd",
        "topic_read": "rt/lowstate",
        "crc": lowcmd.crc,
        "mode_machine": final_state.mode_machine,
        "lowcmd_active": bool(getattr(final_state, "lowcmd_active", False)),
        "right_shoulder_pitch": final_state.motor_state[22].q,
        "right_elbow": final_state.motor_state[25].q,
    }


def _smoke_hand() -> dict[str, Any]:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmds_, unitree_hg_msg_dds__HandCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_

    ChannelFactoryInitialize(0, "cyber-sim")

    simple = unitree_go_msg_dds__MotorCmds_()
    simple.cmds[0].mode = 100
    for cmd in simple.cmds:
        cmd.tau = 0.25

    hand_sdk_publisher = ChannelPublisher("rt/hand_sdk", MotorCmds_)
    hand_sdk_publisher.Init()
    hand_sdk_ok = hand_sdk_publisher.Write(simple)

    dex3 = unitree_hg_msg_dds__HandCmd_()
    for index, cmd in enumerate(dex3.motor_cmd):
        cmd.mode = 0x10 | index
        cmd.q = 0.25
        cmd.kp = 1.5
        cmd.kd = 0.1

    dex3_publisher = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
    dex3_publisher.Init()
    dex3_ok = dex3_publisher.Write(dex3)

    dex3_subscriber = ChannelSubscriber("rt/lf/dex3/right/state", HandState_)
    dex3_subscriber.Init()
    dex3_state = dex3_subscriber.Read()
    dex3_subscriber.Close()
    dex3_publisher.Close()
    hand_sdk_publisher.Close()

    return {
        "ok": bool(hand_sdk_ok and dex3_ok),
        "client": "ChannelPublisher/ChannelSubscriber",
        "topics": {
            "hand_sdk_write": "rt/hand_sdk",
            "dex3_write": "rt/dex3/right/cmd",
            "dex3_read": "rt/lf/dex3/right/state",
        },
        "hand_sdk": hand_sdk_publisher.last_response.get("hand_sdk"),
        "dex3": {
            "provider": getattr(dex3_state, "provider", None),
            "hand": getattr(dex3_state, "hand", None),
            "intent": getattr(dex3_state, "intent", None),
            "first_motor_q": dex3_state.motor_state[0].q if dex3_state.motor_state else None,
            "first_pressure": dex3_state.press_sensor_state[0].pressure if dex3_state.press_sensor_state else None,
        },
    }
