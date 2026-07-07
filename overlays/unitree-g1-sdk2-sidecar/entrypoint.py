#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import subprocess
import sys
import traceback
from pathlib import Path


def exists(path: str) -> bool:
    return Path(path).exists()


def run_command(args: list[str], cwd: Path | None = None, timeout: int = 900, env: dict | None = None) -> dict:
    result = subprocess.run(
        args,
        cwd=str(cwd) if cwd else None,
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
        env=env,
    )
    return {
        "command": " ".join(args),
        "cwd": str(cwd) if cwd else None,
        "returncode": result.returncode,
        "stdout_tail": result.stdout[-12000:],
        "stderr_tail": result.stderr[-12000:],
        "ok": result.returncode == 0,
    }


def ensure_symlink(link_path: Path, target_path: Path) -> None:
    if link_path.is_symlink() and link_path.resolve() == target_path.resolve():
        return
    if link_path.exists() or link_path.is_symlink():
        if link_path.is_dir() and not link_path.is_symlink():
            raise RuntimeError(f"Refusing to replace non-symlink directory: {link_path}")
        link_path.unlink()
    link_path.symlink_to(target_path, target_is_directory=True)


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
        "runtime_library_path": runtime_library_path(),
        "native_dependencies": [
            "libyaml-cpp-dev",
            "libspdlog-dev",
            "libboost-all-dev",
            "libeigen3-dev",
            "libglfw3-dev",
            "xvfb",
            "unitree_sdk2 installed to /opt/unitree_robotics",
            "MuJoCo release symlinked to simulate/mujoco",
        ],
        "build_commands": [
            "cd /opt/unitree_sdk2 && cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics && cmake --build build --target install",
            "cd /opt/unitree_mujoco/simulate && ln -s /path/to/mujoco-3.3.6 mujoco",
            "cd /opt/unitree_mujoco/simulate && cmake -S . -B build && cmake --build build -j4",
        ],
        "launch_command": " ".join(part for part in command if part),
        "launch_probe_command": f"cd {simulate_root} && timeout 8 xvfb-run -a {' '.join(part for part in command if part)}",
        "dds_topics_to_probe_after_launch": [
            "rt/lowstate",
            "rt/lowcmd",
            "rt/sportmodestate",
            "rt/secondary_imu",
        ],
    }


def runtime_library_path() -> str:
    paths = [
        "/opt/unitree_sdk2/thirdparty/lib/aarch64",
        "/opt/unitree_sdk2/lib/aarch64",
        "/opt/unitree_robotics/lib",
        "/opt/mujoco/lib",
    ]
    existing = [path for path in paths if Path(path).exists()]
    return ":".join(existing)


def runtime_env() -> dict:
    env = os.environ.copy()
    current = env.get("LD_LIBRARY_PATH")
    library_path = runtime_library_path()
    if current:
        library_path = f"{library_path}:{current}" if library_path else current
    if library_path:
        env["LD_LIBRARY_PATH"] = library_path
    env.setdefault("MUJOCO_GL", "glfw")
    return env


def build_official_mujoco_peer(sdk2_root: str, mujoco_root: str) -> dict:
    sdk2_path = Path(sdk2_root)
    mujoco_path = Path(mujoco_root)
    release_path = Path(os.environ.get("MUJOCO_ROOT", "/opt/mujoco"))
    simulate_root = mujoco_path / "simulate"
    executable = simulate_root / "build" / "unitree_mujoco"
    report = {
        "action": "build_official_mujoco",
        "sdk2_root": str(sdk2_path),
        "unitree_mujoco_root": str(mujoco_path),
        "mujoco_release_root": str(release_path),
        "steps": [],
        "binary_path": str(executable),
        "binary_exists_before": executable.exists(),
        "binary_exists_after": False,
    }

    required_paths = [
        sdk2_path / "CMakeLists.txt",
        mujoco_path / "simulate" / "CMakeLists.txt",
        release_path / "include" / "mujoco" / "mujoco.h",
        release_path / "lib" / "libmujoco.so",
    ]
    missing = [str(path) for path in required_paths if not path.exists()]
    if missing:
        report["missing"] = missing
        report["ok"] = False
        return report

    try:
        ensure_symlink(simulate_root / "mujoco", release_path)
    except Exception as exc:
        report["ok"] = False
        report["error"] = str(exc)
        return report

    commands = [
        ["cmake", "-S", str(sdk2_path), "-B", str(sdk2_path / "build"), "-DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics", "-DBUILD_EXAMPLES=OFF"],
        ["cmake", "--build", str(sdk2_path / "build"), "--target", "install", "-j4"],
        ["cmake", "-S", str(simulate_root), "-B", str(simulate_root / "build")],
        ["cmake", "--build", str(simulate_root / "build"), "-j4"],
    ]
    for command in commands:
        step = run_command(command, timeout=1200)
        report["steps"].append(step)
        if not step["ok"]:
            report["ok"] = False
            report["binary_exists_after"] = executable.exists()
            return report

    report["binary_exists_after"] = executable.exists()
    report["ok"] = executable.exists()
    return report


def launch_probe_official_mujoco_peer(mujoco_root: str, domain: int, interface: str | None) -> dict:
    plan = official_mujoco_plan(mujoco_root, domain, interface)
    simulate_root = Path(plan["simulate_root"])
    executable = Path(plan["binary_path"])
    report = {
        "action": "launch_probe_official_mujoco",
        "plan": plan,
        "binary_exists": executable.exists(),
        "library_path": runtime_library_path(),
        "used_xvfb": True,
        "timeout_seconds": 8,
    }
    if not executable.exists():
        report["ok"] = False
        report["error"] = "missing official unitree_mujoco binary; run build_official_mujoco first"
        return report
    command = [
        "timeout",
        str(report["timeout_seconds"]),
        "xvfb-run",
        "-a",
        str(executable),
        "-r",
        plan["robot"],
        "-s",
        plan["scene"],
        "-i",
        str(domain),
    ]
    if interface:
        command.extend(["-n", interface])
    step = run_command(command, cwd=simulate_root, timeout=report["timeout_seconds"] + 4, env=runtime_env())
    output = f"{step['stdout_tail']}\n{step['stderr_tail']}"
    report["step"] = step
    report["startup_reached_mujoco"] = "MuJoCo version" in output
    report["loader_error"] = "error while loading shared libraries" in output
    report["glfw_error"] = "could not initialize GLFW" in output
    report["timed_out_after_start"] = step["returncode"] == 124
    report["ok"] = report["startup_reached_mujoco"] and report["timed_out_after_start"] and not report["loader_error"] and not report["glfw_error"]
    if report["ok"]:
        report["next_step"] = "Run the official peer as a managed background service and probe rt/lowstate/rt/lowcmd DDS sample exchange."
    return report


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
    action = os.environ.get("CYBER_UNITREE_ACTION", "status")

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
    if action == "build_official_mujoco":
        report["build"] = build_official_mujoco_peer(sdk2_root, mujoco_root)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action == "launch_probe_official_mujoco":
        report["launch_probe"] = launch_probe_official_mujoco_peer(mujoco_root, domain, interface or None)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action != "status":
        report["status"] = "error"
        report["error"] = f"Unsupported CYBER_UNITREE_ACTION: {action}"
    if report["sdk2_probe"]["domain_initialized"]:
        report["next_step"] = f"Launch official unitree_mujoco with -r g1 on DDS domain {domain} interface {interface or 'auto'}, then run lowstate/lowcmd pub-sub probes."
    print(json.dumps(report, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
