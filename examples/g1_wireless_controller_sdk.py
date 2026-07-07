#!/usr/bin/env python3
"""Publish and read Unitree-style WirelessController_ joystick intent.

This is a simulator convenience surface. It mirrors Unitree's
`rt/wirelesscontroller` message shape so examples and agents can test code that
expects joystick axes/buttons, while real hardware publishing remains locked
behind the future reviewed DDS provider.
"""

from __future__ import annotations

import argparse
import json

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lx", type=float, default=0.35)
    parser.add_argument("--ly", type=float, default=0.0)
    parser.add_argument("--rx", type=float, default=0.0)
    parser.add_argument("--ry", type=float, default=-0.2)
    parser.add_argument("--keys", type=lambda value: int(value, 0), default=0x1234)
    args = parser.parse_args()

    ChannelFactoryInitialize(0, "cyber-sim")
    command = WirelessController_(lx=args.lx, ly=args.ly, rx=args.rx, ry=args.ry, keys=args.keys)
    publisher = ChannelPublisher("rt/wirelesscontroller", WirelessController_)
    publisher.Init()
    if not publisher.Write(command, timeout=5.0):
        raise RuntimeError(f"wireless controller publish failed: {publisher.last_response}")

    subscriber = ChannelSubscriber("rt/wirelesscontroller", WirelessController_)
    subscriber.Init()
    state = subscriber.Read(timeout=5.0)
    print(
        json.dumps(
            {
                "ok": True,
                "published": publisher.last_response,
                "readback": {
                    "lx": state.lx,
                    "ly": state.ly,
                    "rx": state.rx,
                    "ry": state.ry,
                    "keys": state.keys,
                    "metadata": getattr(state, "metadata", {}),
                },
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
