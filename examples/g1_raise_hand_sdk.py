#!/usr/bin/env python3
"""Raise the G1's right hand through a Unitree SDK2-shaped Python API."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
from urllib.request import urlopen


REPO_ROOT = Path(__file__).resolve().parents[1]
SDK_SHIM = REPO_ROOT / "overlays" / "unitree-g1-sdk-shim"
sys.path.insert(0, str(SDK_SHIM))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import (  # noqa: E402
    G1ArmActionClient,
    action_map,
)


HTTP_URL = os.environ.get("CYBER_G1_GAME_CONTROL_URL", "http://127.0.0.1:38383")


def save_camera_frame(path: Path):
    with urlopen(f"{HTTP_URL}/camera_frame_0.jpg", timeout=5.0) as response:
        path.write_bytes(response.read())


def main():
    out_dir = REPO_ROOT / ".runtime" / "g1-control-demo"
    out_dir.mkdir(parents=True, exist_ok=True)

    ChannelFactoryInitialize(0, "cyber-sim")

    arm = G1ArmActionClient()
    arm.SetTimeout(10.0)
    arm.Init()

    code = arm.ExecuteAction(action_map["right hand up"])
    print(f"ExecuteAction(action_map['right hand up']) -> {code}")
    print(json.dumps(arm.last_response, indent=2))

    save_camera_frame(out_dir / "sdk_right_hand_up.jpg")
    print(f"Saved frame: {out_dir / 'sdk_right_hand_up.jpg'}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
