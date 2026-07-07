#!/usr/bin/env python3
"""
Publish Unitree's simple G1 hand SDK open/close intent to the simulator.

This mirrors the official C++ `g1_hand_sdk_example.cpp` shape:

- publish `unitree_go.msg.dds_.MotorCmds_` to `rt/hand_sdk`
- use four motor commands
- encode blend weight as `cmds[0].mode = weight * 100`
- use positive tau to close the hand and negative tau to open it

The current Cybernetic simulator records this as hand intent telemetry. It does
not yet drive full finger physics in MuJoCo.
"""

from __future__ import annotations

import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmds_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_


def hand_command(tau: float, weight: float = 1.0):
    msg = unitree_go_msg_dds__MotorCmds_()
    msg.cmds[0].mode = int(max(0.0, min(1.0, weight)) * 100)
    for cmd in msg.cmds:
        cmd.tau = float(tau)
    return msg


def main():
    ChannelFactoryInitialize(0, "cyber-sim")

    publisher = ChannelPublisher("rt/hand_sdk", MotorCmds_)
    publisher.Init()

    for label, tau in [("close", 0.3), ("open", -0.3)]:
        ok = publisher.Write(hand_command(tau), timeout=5.0)
        if not ok:
            raise RuntimeError(f"rt/hand_sdk {label} write failed")
        print({"topic": "rt/hand_sdk", "intent": label, "response": publisher.last_response.get("hand_sdk")})
        time.sleep(0.5)


if __name__ == "__main__":
    main()
