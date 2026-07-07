#!/usr/bin/env python3
"""
Send a Unitree SDK2-shaped low-level command to the local G1 MuJoCo simulator.

This mirrors the important shape of Unitree's official low-level examples:

- initialize the SDK channel factory
- subscribe to rt/lowstate
- publish LowCmd_ messages on rt/lowcmd
- compute a CRC before writing

The simulator bridge applies commanded joint targets as held MuJoCo frames and
records torque estimates for telemetry. Keep this example small and
conservative: low-level joint control is powerful, and the current harness is a
developer playground rather than a full whole-body controller.
"""

from __future__ import annotations

import math
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


RIGHT_SHOULDER_PITCH = 22
RIGHT_SHOULDER_ROLL = 23
RIGHT_SHOULDER_YAW = 24
RIGHT_ELBOW = 25
RIGHT_WRIST_ROLL = 26
RIGHT_WRIST_PITCH = 27
RIGHT_WRIST_YAW = 28


def arm_target_command(low_state: LowState_, phase: float):
    low_cmd = unitree_hg_msg_dds__LowCmd_()
    low_cmd.mode_pr = 0
    low_cmd.mode_machine = low_state.mode_machine

    for index, motor in enumerate(low_state.motor_state):
        low_cmd.motor_cmd[index].mode = 1
        low_cmd.motor_cmd[index].q = motor.q
        low_cmd.motor_cmd[index].dq = 0.0
        low_cmd.motor_cmd[index].tau = 0.0
        low_cmd.motor_cmd[index].kp = 12.0
        low_cmd.motor_cmd[index].kd = 0.8

    lift = 0.5 - 0.5 * math.cos(math.pi * phase)
    targets = {
        RIGHT_SHOULDER_PITCH: -2.2 * lift,
        RIGHT_SHOULDER_ROLL: -0.22 * lift,
        RIGHT_SHOULDER_YAW: 0.1 * lift,
        RIGHT_ELBOW: 1.05 * lift,
        RIGHT_WRIST_ROLL: 0.0,
        RIGHT_WRIST_PITCH: 0.0,
        RIGHT_WRIST_YAW: 0.0,
    }
    for index, target in targets.items():
        low_cmd.motor_cmd[index].q = target
        low_cmd.motor_cmd[index].kp = 38.0
        low_cmd.motor_cmd[index].kd = 1.4
    low_cmd.crc = CRC().Crc(low_cmd)
    return low_cmd


def main():
    ChannelFactoryInitialize(0, "cyber-sim")

    motion = MotionSwitcherClient()
    motion.SetTimeout(5.0)
    motion.Init()
    motion.ReleaseMode()

    lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
    lowstate_sub.Init()

    lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    lowcmd_pub.Init()

    for step in range(40):
        low_state = lowstate_sub.Read()
        phase = min(1.0, (step + 1) / 40.0)
        lowcmd_pub.Write(arm_target_command(low_state, phase))
        time.sleep(0.04)

    final_state = lowstate_sub.Read()
    print(
        {
            "mode_machine": final_state.mode_machine,
            "right_shoulder_pitch": round(final_state.motor_state[RIGHT_SHOULDER_PITCH].q, 3),
            "right_elbow": round(final_state.motor_state[RIGHT_ELBOW].q, 3),
        }
    )


if __name__ == "__main__":
    main()
