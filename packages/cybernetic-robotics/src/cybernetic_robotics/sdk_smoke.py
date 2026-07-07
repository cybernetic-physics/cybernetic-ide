from __future__ import annotations

from typing import Any

from .config import RobotEndpoints
from .g1 import G1Robot


def run_official_g1_sdk_smoke(kind: str = "all", endpoints: RobotEndpoints | None = None) -> dict[str, Any]:
    """Run safe behavior-level smoke checks through Unitree SDK-shaped imports."""

    requested = _normalize_kind(kind)
    results = {}
    if requested in {"all", "arm"}:
        results["arm"] = _smoke_arm()
    if requested in {"all", "loco"}:
        results["loco"] = _smoke_loco()
    if requested in {"all", "lowcmd"}:
        results["lowcmd"] = _smoke_lowcmd()

    robot = G1Robot.connect(endpoints=endpoints)
    status = robot.status()
    safety = robot.safety_check()
    return {
        "ok": all(item.get("ok") for item in results.values()) and safety.get("safe_to_command", False),
        "kind": requested,
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
        "agent_hints": [
            "This is behavior-level smoke coverage for safe official-style calls, not full physical parity.",
            "Use sdk-audit first for import/method coverage, then sdk-smoke for simulator behavior evidence.",
            "Run safety_stop after manual experiments that leave locomotion or lowcmd active.",
        ],
    }


def _normalize_kind(kind: str) -> str:
    value = (kind or "all").strip().lower()
    if value not in {"all", "arm", "loco", "lowcmd"}:
        raise ValueError(f"unknown SDK smoke kind: {kind}")
    return value


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
