#!/usr/bin/env python3
"""
Publish Unitree G1 Dex3 hand commands through the Cybernetic simulator shim.

This mirrors the official C++ `g1_dex3_example.cpp` topic/message shape:

- publish `unitree_hg.msg.dds_.HandCmd_` to `rt/dex3/{left,right}/cmd`
- read `unitree_hg.msg.dds_.HandState_` from `rt/lf/dex3/{left,right}/state`
- command seven Dex3 motors per hand

The current simulator records Dex3 hand intent and synthesizes readable hand
state. It does not yet model full finger physics in MuJoCo.
"""

from __future__ import annotations

import argparse

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_


def dex3_command(q: float, kp: float = 1.5, kd: float = 0.1):
    msg = unitree_hg_msg_dds__HandCmd_()
    for index, cmd in enumerate(msg.motor_cmd):
        cmd.mode = 0x10 | index
        cmd.q = float(q)
        cmd.dq = 0.0
        cmd.tau = 0.0
        cmd.kp = float(kp)
        cmd.kd = float(kd)
    return msg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand", choices=["left", "right"], default="right")
    parser.add_argument("--q", type=float, default=0.25, help="Dex3 motor target position.")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, "cyber-sim")

    publisher = ChannelPublisher(f"rt/dex3/{args.hand}/cmd", HandCmd_)
    publisher.Init()
    if not publisher.Write(dex3_command(args.q), timeout=5.0):
        raise RuntimeError(f"Dex3 {args.hand} command failed: {publisher.last_response}")

    subscriber = ChannelSubscriber(f"rt/lf/dex3/{args.hand}/state", HandState_)
    subscriber.Init()
    state = subscriber.Read(timeout=5.0)

    print(
        {
            "topic": f"rt/dex3/{args.hand}/cmd",
            "state_topic": f"rt/lf/dex3/{args.hand}/state",
            "intent": getattr(state, "intent", None),
            "first_motor_q": state.motor_state[0].q,
            "first_pressure": state.press_sensor_state[0].pressure,
            "provider": getattr(state, "provider", None),
        }
    )


if __name__ == "__main__":
    main()
