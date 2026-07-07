#!/usr/bin/env python3
"""
Publish a conservative Unitree G1 user-control LowCmd frame stream.

Unitree's C++ `g1_userctrl_dds_example.cpp` publishes HG `LowCmd_` messages to
`rt/user_lowcmd` after switching the locomotion controller into user-control
mode. This example keeps the same topic and SDK2-shaped Python imports while
staying bounded for the local Cybernetic simulator: it raises one left arm
joint a little, reads `rt/lowstate`, and prints the provider evidence.
"""

from __future__ import annotations

import math
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


LEFT_SHOULDER_PITCH = 15
LEFT_ELBOW = 18


def user_lowcmd(low_state: LowState_, phase: float):
    low_cmd = unitree_hg_msg_dds__LowCmd_()
    low_cmd.mode_pr = low_state.mode_pr
    low_cmd.mode_machine = low_state.mode_machine

    for index, motor in enumerate(low_state.motor_state):
        low_cmd.motor_cmd[index].mode = 1
        low_cmd.motor_cmd[index].q = motor.q
        low_cmd.motor_cmd[index].dq = 0.0
        low_cmd.motor_cmd[index].tau = 0.0
        low_cmd.motor_cmd[index].kp = 8.0
        low_cmd.motor_cmd[index].kd = 0.5

    lift = 0.5 - 0.5 * math.cos(math.pi * phase)
    low_cmd.motor_cmd[LEFT_SHOULDER_PITCH].q = -0.45 * lift
    low_cmd.motor_cmd[LEFT_SHOULDER_PITCH].kp = 30.0
    low_cmd.motor_cmd[LEFT_SHOULDER_PITCH].kd = 1.0
    low_cmd.motor_cmd[LEFT_ELBOW].q = 0.35 * lift
    low_cmd.motor_cmd[LEFT_ELBOW].kp = 30.0
    low_cmd.motor_cmd[LEFT_ELBOW].kd = 1.0
    low_cmd.crc = CRC().Crc(low_cmd)
    return low_cmd


def main():
    ChannelFactoryInitialize(0, "cyber-sim")

    loco = LocoClient()
    loco.SetTimeout(3.0)
    loco.Init()
    loco.SwitchToUserCtrl()

    lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
    lowstate_sub.Init()

    user_pub = ChannelPublisher("rt/user_lowcmd", LowCmd_)
    user_pub.Init()

    for step in range(16):
        low_state = lowstate_sub.Read()
        phase = min(1.0, (step + 1) / 16.0)
        if not user_pub.Write(user_lowcmd(low_state, phase), timeout=5.0):
            raise RuntimeError(f"rt/user_lowcmd write failed at step {step}")
        time.sleep(0.04)

    final_state = lowstate_sub.Read()
    print(
        {
            "topic": "rt/user_lowcmd",
            "provider": getattr(final_state, "provider", None),
            "mode_machine": final_state.mode_machine,
            "left_shoulder_pitch": round(final_state.motor_state[LEFT_SHOULDER_PITCH].q, 3),
            "left_elbow": round(final_state.motor_state[LEFT_ELBOW].q, 3),
        }
    )


if __name__ == "__main__":
    main()
