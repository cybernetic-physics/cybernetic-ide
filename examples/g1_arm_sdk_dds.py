#!/usr/bin/env python3
"""
Drive the G1 arm through Unitree's official rt/arm_sdk-shaped topic.

This mirrors the shape of Unitree's G1 arm SDK examples while staying small:

- initialize the SDK2 channel factory
- subscribe to rt/lowstate
- publish unitree_hg LowCmd_ frames on rt/arm_sdk
- set motor slot 29 to 1.0 to enable the arm SDK path
- compute CRC before each write

With the default Cybernetic simulator transport this routes to the local MuJoCo
harness. With `CYBER_UNITREE_TRANSPORT=dds`, the same script publishes bounded,
sanitized frames through the managed official Unitree MuJoCo + SDK2 sidecar.
"""

from __future__ import annotations

import math
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


RIGHT_SHOULDER_PITCH = 22
RIGHT_SHOULDER_ROLL = 23
RIGHT_SHOULDER_YAW = 24
RIGHT_ELBOW = 25
ARM_SDK_ENABLE_SLOT = 29


def arm_sdk_command(low_state: LowState_, phase: float):
    low_cmd = unitree_hg_msg_dds__LowCmd_()
    low_cmd.mode_pr = low_state.mode_pr
    low_cmd.mode_machine = low_state.mode_machine

    for index, motor in enumerate(low_state.motor_state):
        low_cmd.motor_cmd[index].mode = 1
        low_cmd.motor_cmd[index].q = motor.q
        low_cmd.motor_cmd[index].dq = 0.0
        low_cmd.motor_cmd[index].tau = 0.0
        low_cmd.motor_cmd[index].kp = 0.0
        low_cmd.motor_cmd[index].kd = 0.0

    lift = 0.5 - 0.5 * math.cos(math.pi * phase)
    targets = {
        RIGHT_SHOULDER_PITCH: -0.55 * lift,
        RIGHT_SHOULDER_ROLL: -0.30 * lift,
        RIGHT_SHOULDER_YAW: 0.16 * lift,
        RIGHT_ELBOW: -0.45 * lift,
    }
    for index, target in targets.items():
        low_cmd.motor_cmd[index].q = target
        low_cmd.motor_cmd[index].kp = 40.0
        low_cmd.motor_cmd[index].kd = 1.0

    low_cmd.motor_cmd[ARM_SDK_ENABLE_SLOT].q = 1.0
    low_cmd.crc = CRC().Crc(low_cmd)
    return low_cmd


def main():
    ChannelFactoryInitialize(0, "cyber-sim")

    lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
    lowstate_sub.Init()

    arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    arm_pub.Init()

    for step in range(20):
        low_state = lowstate_sub.Read()
        phase = min(1.0, (step + 1) / 20.0)
        ok = arm_pub.Write(arm_sdk_command(low_state, phase), timeout=6.0)
        if not ok:
            raise RuntimeError(f"rt/arm_sdk write failed at step {step}")
        time.sleep(0.04)

    final_state = lowstate_sub.Read()
    print(
        {
            "mode_machine": final_state.mode_machine,
            "right_shoulder_pitch": round(final_state.motor_state[RIGHT_SHOULDER_PITCH].q, 3),
            "right_elbow": round(final_state.motor_state[RIGHT_ELBOW].q, 3),
            "topic": "rt/arm_sdk",
        }
    )


if __name__ == "__main__":
    main()
