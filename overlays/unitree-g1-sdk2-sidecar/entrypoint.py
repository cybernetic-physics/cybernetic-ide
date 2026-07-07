#!/usr/bin/env python3
from __future__ import annotations

import json
import os
from pathlib import Path


def exists(path: str) -> bool:
    return Path(path).exists()


def main() -> int:
    sdk2_python_root = os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python")
    sdk2_root = os.environ.get("UNITREE_SDK2_ROOT", "/opt/unitree_sdk2")
    mujoco_root = os.environ.get("UNITREE_MUJOCO_ROOT", "/opt/unitree_mujoco")
    mode = os.environ.get("CYBER_UNITREE_MODE", "sim")
    transport = os.environ.get("CYBER_UNITREE_TRANSPORT", "dds")
    domain = int(os.environ.get("CYBER_UNITREE_DDS_DOMAIN", "1" if mode == "sim" else "0"))
    interface = os.environ.get("CYBER_UNITREE_NETWORK_INTERFACE", "lo" if mode == "sim" else "")

    report = {
        "status": "ready",
        "implemented": False,
        "purpose": "SDK2/CycloneDDS sidecar scaffold for the future Unitree G1 provider",
        "mode": mode,
        "transport": transport,
        "dds_domain_id": domain,
        "network_interface": interface or None,
        "sources": {
            "unitree_sdk2_python": {
                "path": sdk2_python_root,
                "exists": exists(sdk2_python_root),
                "revision": os.environ.get("UNITREE_SDK2_PYTHON_REVISION"),
            },
            "unitree_sdk2": {
                "path": sdk2_root,
                "exists": exists(sdk2_root),
                "revision": os.environ.get("UNITREE_SDK2_REVISION"),
            },
            "unitree_mujoco": {
                "path": mujoco_root,
                "exists": exists(mujoco_root),
                "revision": os.environ.get("UNITREE_MUJOCO_REVISION"),
            },
        },
        "expected_topics": [
            "rt/lowcmd",
            "rt/lowstate",
            "rt/sportmodestate",
            "rt/arm_sdk",
            "rt/hand_sdk",
            "rt/api/sport/request",
            "rt/api/sport/response",
            "rt/api/arm/request",
            "rt/api/arm/response",
        ],
        "next_step": "Install/build CycloneDDS and unitree_sdk2_python here, then run SDK2 probes against official unitree_mujoco on domain 1 interface lo.",
    }
    print(json.dumps(report, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
