#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys
import traceback
from pathlib import Path


def exists(path: str) -> bool:
    return Path(path).exists()


def official_mujoco_plan(mujoco_root: str, domain: int, interface: str | None) -> dict:
    root = Path(mujoco_root)
    robot = os.environ.get("CYBER_UNITREE_ROBOT", "g1")
    scene = os.environ.get("CYBER_UNITREE_SCENE", "scene_29dof.xml" if robot == "g1" else "scene.xml")
    simulate_root = root / "simulate"
    executable = simulate_root / "build" / "unitree_mujoco"
    scene_path = root / "unitree_robots" / robot / scene
    command = [
        str(executable),
        "-r",
        robot,
        "-s",
        scene,
        "-i",
        str(domain),
        "-n",
        interface or "",
    ]
    return {
        "robot": robot,
        "scene": scene,
        "scene_path": str(scene_path),
        "scene_exists": scene_path.exists(),
        "recommended_runtime": "official C++ simulator",
        "why_cpp": "Unitree documents simulate/ as the recommended simulator and it starts the SDK2 bridge thread with G1Bridge when the model has more than 20 actuators.",
        "viewer_required": True,
        "headless_supported_by_upstream": False,
        "simulate_root": str(simulate_root),
        "config_path": str(simulate_root / "config.yaml"),
        "config_exists": (simulate_root / "config.yaml").exists(),
        "cmake_path": str(simulate_root / "CMakeLists.txt"),
        "cmake_exists": (simulate_root / "CMakeLists.txt").exists(),
        "binary_path": str(executable),
        "binary_exists": executable.exists(),
        "unitree_sdk2_install_prefix": "/opt/unitree_robotics",
        "mujoco_symlink_path": str(simulate_root / "mujoco"),
        "mujoco_symlink_exists": (simulate_root / "mujoco").exists(),
        "native_dependencies": [
            "libyaml-cpp-dev",
            "libspdlog-dev",
            "libboost-all-dev",
            "libglfw3-dev",
            "unitree_sdk2 installed to /opt/unitree_robotics",
            "MuJoCo release symlinked to simulate/mujoco",
        ],
        "build_commands": [
            "cd /opt/unitree_sdk2 && cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics && cmake --build build --target install",
            "cd /opt/unitree_mujoco/simulate && ln -s /path/to/mujoco-3.3.6 mujoco",
            "cd /opt/unitree_mujoco/simulate && cmake -S . -B build && cmake --build build -j4",
        ],
        "launch_command": " ".join(part for part in command if part),
        "dds_topics_to_probe_after_launch": [
            "rt/lowstate",
            "rt/lowcmd",
            "rt/sportmodestate",
            "rt/secondary_imu",
        ],
    }


def probe_official_sdk2(sdk2_python_root: str, domain: int, interface: str | None) -> dict:
    report = {
        "python_path_inserted": False,
        "imports": {},
        "domain_initialized": False,
        "channels": {},
        "errors": [],
    }
    if not Path(sdk2_python_root).exists():
        report["errors"].append(f"missing SDK2 Python root: {sdk2_python_root}")
        return report

    sys.path.insert(0, sdk2_python_root)
    report["python_path_inserted"] = True

    try:
        import cyclonedds  # type: ignore

        report["imports"]["cyclonedds"] = getattr(cyclonedds, "__version__", "available")
    except Exception as exc:  # pragma: no cover - diagnostic path
        report["imports"]["cyclonedds"] = False
        report["errors"].append(f"import cyclonedds failed: {exc}")
        report["traceback"] = traceback.format_exc()
        return report

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    except Exception as exc:  # pragma: no cover - diagnostic path
        report["imports"]["unitree_sdk2py"] = False
        report["errors"].append(f"import unitree_sdk2py failed: {exc}")
        report["traceback"] = traceback.format_exc()
        return report

    report["imports"]["unitree_sdk2py"] = True
    report["imports"]["unitree_hg_lowcmd_type"] = getattr(LowCmd_, "__idl_typename__", "unitree_hg.msg.dds_.LowCmd_")
    report["imports"]["unitree_hg_lowstate_type"] = getattr(LowState_, "__idl_typename__", "unitree_hg.msg.dds_.LowState_")

    try:
        ChannelFactoryInitialize(domain, interface)
        report["domain_initialized"] = True
    except Exception as exc:  # pragma: no cover - diagnostic path
        report["errors"].append(f"ChannelFactoryInitialize failed: {exc}")
        report["traceback"] = traceback.format_exc()
        return report

    try:
        lowcmd = unitree_hg_msg_dds__LowCmd_()
        publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        publisher.Init()
        publisher.Close()
        report["channels"]["rt/lowcmd"] = {
            "role": "publisher",
            "created": True,
            "sample_motor_count": len(lowcmd.motor_cmd),
        }
    except Exception as exc:  # pragma: no cover - diagnostic path
        report["channels"]["rt/lowcmd"] = {"role": "publisher", "created": False, "error": str(exc)}
        report["errors"].append(f"rt/lowcmd publisher failed: {exc}")

    try:
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(None, 0)
        subscriber.Close()
        report["channels"]["rt/lowstate"] = {"role": "subscriber", "created": True}
    except Exception as exc:  # pragma: no cover - diagnostic path
        report["channels"]["rt/lowstate"] = {"role": "subscriber", "created": False, "error": str(exc)}
        report["errors"].append(f"rt/lowstate subscriber failed: {exc}")

    return report


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
        "implemented": "diagnostic_only",
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
        "next_step": f"Install/build CycloneDDS and unitree_sdk2_python here, then run SDK2 probes against official unitree_mujoco on domain {domain} interface {interface or 'auto'}.",
    }
    report["sdk2_probe"] = probe_official_sdk2(sdk2_python_root, domain, interface or None)
    report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    if report["sdk2_probe"]["domain_initialized"]:
        report["next_step"] = f"Launch official unitree_mujoco with -r g1 on DDS domain {domain} interface {interface or 'auto'}, then run lowstate/lowcmd pub-sub probes."
    print(json.dumps(report, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
