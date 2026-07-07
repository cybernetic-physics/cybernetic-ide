#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
import traceback
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


DEFAULT_SIMULATOR_GAME_CONTROL_URL = "http://127.0.0.1:38383"

G1_ARM_ACTIONS = {
    "release arm": 99,
    "two-hand kiss": 11,
    "left kiss": 12,
    "right kiss": 13,
    "hands up": 15,
    "clap": 17,
    "high five": 18,
    "hug": 19,
    "heart": 20,
    "right heart": 21,
    "reject": 22,
    "right hand up": 23,
    "x-ray": 24,
    "face wave": 25,
    "high wave": 26,
    "shake hand": 27,
}

G1_ARM_POSES_BY_ACTION_ID = {
    99: "neutral",
    11: "two_hand_kiss",
    12: "left_kiss",
    13: "right_kiss",
    15: "hands_up",
    17: "clap",
    18: "high_five",
    19: "hug",
    20: "heart",
    21: "right_heart",
    22: "reject",
    23: "raise_right_hand",
    24: "x_ray",
    25: "face_wave",
    26: "high_wave",
    27: "shake_hand",
}

ROBOT_API_ID_LOCO_GET_PHASE = 7006
ROBOT_API_ID_LOCO_SET_SPEED_MODE = 7107
ROBOT_API_ID_LOCO_SWITCH_MOVE_MODE = 7108
ROBOT_API_ID_LOCO_SWITCH_TO_USER_CTRL = 7110
ROBOT_API_ID_LOCO_SWITCH_TO_INTERNAL_CTRL = 7111


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


def serve_official_mujoco_peer(mujoco_root: str, domain: int, interface: str | None) -> int:
    plan = official_mujoco_plan(mujoco_root, domain, interface)
    simulate_root = Path(plan["simulate_root"])
    executable = Path(plan["binary_path"])
    report = {
        "action": "serve_official_mujoco",
        "plan": plan,
        "binary_exists": executable.exists(),
        "library_path": runtime_library_path(),
        "used_xvfb": True,
        "read_topics": ["rt/lowstate"],
        "write_topics": ["rt/lowcmd"],
        "ok": False,
    }
    if not executable.exists():
        report["error"] = "missing official unitree_mujoco binary; run build_official_mujoco first"
        print(json.dumps(report, indent=2), flush=True)
        return 1

    command = [
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
    process = subprocess.Popen(
        command,
        cwd=str(simulate_root),
        env=runtime_env(),
        start_new_session=True,
    )
    report.update(
        {
            "ok": True,
            "peer_started": True,
            "peer_pid": process.pid,
            "command": " ".join(command),
            "next_step": "Keep this container running as the official Unitree MuJoCo DDS peer, then connect SDK2 lowstate/lowcmd clients to the same domain.",
        }
    )
    print(json.dumps(report, indent=2), flush=True)

    stopping = False

    def stop_peer(_signum, _frame):
        nonlocal stopping
        stopping = True
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except Exception:
                process.terminate()

    signal.signal(signal.SIGTERM, stop_peer)
    signal.signal(signal.SIGINT, stop_peer)

    while process.poll() is None:
        time.sleep(0.5)
    returncode = process.returncode
    print(
        json.dumps(
            {
                "action": "serve_official_mujoco_exit",
                "peer_pid": process.pid,
                "returncode": returncode,
                "stopping": stopping,
            },
            indent=2,
        ),
        flush=True,
    )
    return int(returncode or 0)


def summarize_lowstate(sample) -> dict:
    motors = list(getattr(sample, "motor_state", []) or [])
    imu = getattr(sample, "imu_state", None)
    summary = {
        "sample_type": type(sample).__name__,
        "idl_typename": getattr(type(sample), "__idl_typename__", None),
        "mode_machine": getattr(sample, "mode_machine", None),
        "motor_count": len(motors),
        "first_motors": [],
    }
    for index, motor in enumerate(motors[:6]):
        summary["first_motors"].append(
            {
                "index": index,
                "q": float(getattr(motor, "q", 0.0)),
                "dq": float(getattr(motor, "dq", 0.0)),
                "tau_est": float(getattr(motor, "tau_est", 0.0)),
                "temperature": list(getattr(motor, "temperature", []) or []),
            }
        )
    if imu is not None:
        summary["imu"] = {
            "rpy": [float(value) for value in list(getattr(imu, "rpy", []) or [])],
            "quaternion": [float(value) for value in list(getattr(imu, "quaternion", []) or [])],
            "gyroscope": [float(value) for value in list(getattr(imu, "gyroscope", []) or [])],
            "accelerometer": [float(value) for value in list(getattr(imu, "accelerometer", []) or [])],
        }
    return summary


def read_official_mujoco_lowstate(domain: int, interface: str | None) -> dict:
    report = {
        "action": "read_official_mujoco_lowstate",
        "topic": "rt/lowstate",
        "domain": domain,
        "interface": interface or None,
        "lowstate_sample_received": False,
        "read_attempts": 0,
        "timeout_seconds": float(os.environ.get("CYBER_UNITREE_LOWSTATE_READ_TIMEOUT", "6")),
    }
    try:
        sys.path.insert(0, os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python"))
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(None, 0)
        deadline = time.monotonic() + report["timeout_seconds"]
        while time.monotonic() < deadline:
            report["read_attempts"] += 1
            sample = subscriber.Read(0.5)
            if sample is not None:
                report["lowstate_sample_received"] = True
                report["lowstate_summary"] = summarize_lowstate(sample)
                break
        subscriber.Close()
    except Exception as exc:
        report["probe_error"] = str(exc)
        report["traceback"] = traceback.format_exc()

    report["ok"] = report["lowstate_sample_received"]
    if report["ok"]:
        report["next_step"] = "Use this managed-session lowstate read as the telemetry side of the long-lived DDS provider."
    else:
        report["next_step"] = "Start unitree-g1-sdk2-session and confirm DDS domain/interface discovery before reading official lowstate."
    return report


def make_hold_lowcmd(low_state, lowcmd_factory, crc_factory, kp: float = 0.0, kd: float = 0.0):
    low_cmd = lowcmd_factory()
    low_cmd.mode_pr = getattr(low_state, "mode_pr", 0)
    low_cmd.mode_machine = getattr(low_state, "mode_machine", 0)
    state_motors = list(getattr(low_state, "motor_state", []) or [])
    command_motors = list(getattr(low_cmd, "motor_cmd", []) or [])
    for index, motor_cmd in enumerate(command_motors):
        motor_state = state_motors[index] if index < len(state_motors) else None
        motor_cmd.mode = 1
        motor_cmd.tau = 0.0
        motor_cmd.q = float(getattr(motor_state, "q", 0.0)) if motor_state is not None else 0.0
        motor_cmd.dq = 0.0
        motor_cmd.kp = float(kp)
        motor_cmd.kd = float(kd)
    low_cmd.crc = crc_factory.Crc(low_cmd)
    return low_cmd


G1_JOINT_INDEX = {
    "left_shoulder_pitch": 15,
    "left_shoulder_roll": 16,
    "left_shoulder_yaw": 17,
    "left_elbow": 18,
    "left_wrist_roll": 19,
    "left_wrist_pitch": 20,
    "left_wrist_yaw": 21,
    "right_shoulder_pitch": 22,
    "right_shoulder_roll": 23,
    "right_shoulder_yaw": 24,
    "right_elbow": 25,
    "right_wrist_roll": 26,
    "right_wrist_pitch": 27,
    "right_wrist_yaw": 28,
}

G1_ARM_POSE_PRESETS = {
    "raise_right_hand": {
        "right_shoulder_pitch": -0.45,
        "right_shoulder_roll": -0.35,
        "right_shoulder_yaw": 0.20,
        "right_elbow": -0.40,
        "right_wrist_pitch": 0.15,
    },
    "raise_left_hand": {
        "left_shoulder_pitch": -0.45,
        "left_shoulder_roll": 0.35,
        "left_shoulder_yaw": -0.20,
        "left_elbow": -0.40,
        "left_wrist_pitch": 0.15,
    },
}


def motor_position(low_state, joint_index: int) -> float | None:
    motors = list(getattr(low_state, "motor_state", []) or [])
    if joint_index < 0 or joint_index >= len(motors):
        return None
    return float(getattr(motors[joint_index], "q", 0.0))


def make_single_joint_lowcmd(
    low_state,
    lowcmd_factory,
    crc_factory,
    joint_index: int,
    target_position: float,
    kp: float,
    kd: float,
    hold_kp: float,
    hold_kd: float,
):
    low_cmd = make_hold_lowcmd(low_state, lowcmd_factory, crc_factory, kp=hold_kp, kd=hold_kd)
    low_cmd.motor_cmd[joint_index].mode = 1
    low_cmd.motor_cmd[joint_index].tau = 0.0
    low_cmd.motor_cmd[joint_index].q = float(target_position)
    low_cmd.motor_cmd[joint_index].dq = 0.0
    low_cmd.motor_cmd[joint_index].kp = float(kp)
    low_cmd.motor_cmd[joint_index].kd = float(kd)
    low_cmd.crc = crc_factory.Crc(low_cmd)
    return low_cmd


def make_multi_joint_lowcmd(
    low_state,
    lowcmd_factory,
    crc_factory,
    targets: dict[int, float],
    kp: float,
    kd: float,
    hold_kp: float,
    hold_kd: float,
):
    low_cmd = make_hold_lowcmd(low_state, lowcmd_factory, crc_factory, kp=hold_kp, kd=hold_kd)
    for joint_index, target_position in targets.items():
        low_cmd.motor_cmd[joint_index].mode = 1
        low_cmd.motor_cmd[joint_index].tau = 0.0
        low_cmd.motor_cmd[joint_index].q = float(target_position)
        low_cmd.motor_cmd[joint_index].dq = 0.0
        low_cmd.motor_cmd[joint_index].kp = float(kp)
        low_cmd.motor_cmd[joint_index].kd = float(kd)
    low_cmd.crc = crc_factory.Crc(low_cmd)
    return low_cmd


def summarize_lowcmd(command) -> dict:
    motors = list(getattr(command, "motor_cmd", []) or [])
    return {
        "sample_type": type(command).__name__,
        "idl_typename": getattr(type(command), "__idl_typename__", None),
        "mode_pr": getattr(command, "mode_pr", None),
        "mode_machine": getattr(command, "mode_machine", None),
        "motor_count": len(motors),
        "crc": getattr(command, "crc", None),
        "first_motors": [
            {
                "index": index,
                "mode": getattr(motor, "mode", None),
                "q": float(getattr(motor, "q", 0.0)),
                "dq": float(getattr(motor, "dq", 0.0)),
                "tau": float(getattr(motor, "tau", 0.0)),
                "kp": float(getattr(motor, "kp", 0.0)),
                "kd": float(getattr(motor, "kd", 0.0)),
            }
            for index, motor in enumerate(motors[:6])
        ],
    }


def terminate_process(process: subprocess.Popen, timeout: float = 4.0) -> tuple[str, str, int | None]:
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except Exception:
            process.terminate()
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except Exception:
            process.kill()
        try:
            stdout, stderr = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            stdout, stderr = "", ""
    return stdout[-12000:], stderr[-12000:], process.returncode


def probe_official_mujoco_dds_exchange(mujoco_root: str, domain: int, interface: str | None) -> dict:
    plan = official_mujoco_plan(mujoco_root, domain, interface)
    simulate_root = Path(plan["simulate_root"])
    executable = Path(plan["binary_path"])
    report = {
        "action": "probe_official_mujoco_dds",
        "plan": plan,
        "binary_exists": executable.exists(),
        "library_path": runtime_library_path(),
        "used_xvfb": True,
        "topic": "rt/lowstate",
        "domain": domain,
        "interface": interface or None,
        "peer_started": False,
        "lowstate_sample_received": False,
        "read_attempts": 0,
        "timeout_seconds": float(os.environ.get("CYBER_UNITREE_DDS_PROBE_TIMEOUT", "10")),
    }
    if not executable.exists():
        report["ok"] = False
        report["error"] = "missing official unitree_mujoco binary; run build_official_mujoco first"
        return report

    command = [
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

    process = subprocess.Popen(
        command,
        cwd=str(simulate_root),
        env=runtime_env(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    report["peer_started"] = True
    report["peer_pid"] = process.pid
    report["command"] = " ".join(command)

    try:
        sys.path.insert(0, os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python"))
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(None, 0)
        deadline = time.monotonic() + report["timeout_seconds"]
        while time.monotonic() < deadline:
            if process.poll() is not None:
                report["peer_exited_before_sample"] = True
                break
            report["read_attempts"] += 1
            sample = subscriber.Read(0.5)
            if sample is not None:
                report["lowstate_sample_received"] = True
                report["lowstate_summary"] = summarize_lowstate(sample)
                break
        subscriber.Close()
    except Exception as exc:
        report["probe_error"] = str(exc)
        report["traceback"] = traceback.format_exc()
    finally:
        stdout_tail, stderr_tail, returncode = terminate_process(process)

    output = f"{stdout_tail}\n{stderr_tail}"
    report["stdout_tail"] = stdout_tail
    report["stderr_tail"] = stderr_tail
    report["returncode"] = returncode
    report["startup_reached_mujoco"] = "MuJoCo version" in output
    report["bridge_started"] = "Mujoco data is prepared" in output or "Try to start sdk2 thread" in output
    report["loader_error"] = "error while loading shared libraries" in output
    report["glfw_error"] = "could not initialize GLFW" in output
    report["cyclonedds_warning"] = "selected interface" in output or "ddsi_udp_conn_write" in output
    report["ok"] = (
        report["peer_started"]
        and report["startup_reached_mujoco"]
        and report["lowstate_sample_received"]
        and not report["loader_error"]
        and not report["glfw_error"]
    )
    if report["ok"]:
        report["next_step"] = "Publish rt/lowcmd with official SDK2 types while this peer is running, then map the Cybernetic SDK facade onto that transport."
    elif report["cyclonedds_warning"]:
        report["next_step"] = "Retry with a multicast-capable CYBER_UNITREE_NETWORK_INTERFACE or a CycloneDDS unicast config; the peer launched but DDS discovery/sample exchange did not prove out."
    else:
        report["next_step"] = "Inspect stdout/stderr tails and retry after confirming the official peer reaches the SDK2 bridge before the read deadline."
    return report


def probe_official_mujoco_lowcmd_exchange(mujoco_root: str, domain: int, interface: str | None) -> dict:
    plan = official_mujoco_plan(mujoco_root, domain, interface)
    simulate_root = Path(plan["simulate_root"])
    executable = Path(plan["binary_path"])
    report = {
        "action": "probe_official_mujoco_lowcmd",
        "plan": plan,
        "binary_exists": executable.exists(),
        "library_path": runtime_library_path(),
        "used_xvfb": True,
        "read_topic": "rt/lowstate",
        "write_topic": "rt/lowcmd",
        "domain": domain,
        "interface": interface or None,
        "peer_started": False,
        "lowstate_sample_received": False,
        "lowcmd_write_attempts": 0,
        "lowcmd_write_successes": 0,
        "timeout_seconds": float(os.environ.get("CYBER_UNITREE_LOWCMD_PROBE_TIMEOUT", "10")),
        "hold_frames": int(os.environ.get("CYBER_UNITREE_LOWCMD_PROBE_FRAMES", "8")),
    }
    if not executable.exists():
        report["ok"] = False
        report["error"] = "missing official unitree_mujoco binary; run build_official_mujoco first"
        return report

    command = [
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

    process = subprocess.Popen(
        command,
        cwd=str(simulate_root),
        env=runtime_env(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    report["peer_started"] = True
    report["peer_pid"] = process.pid
    report["command"] = " ".join(command)

    try:
        sys.path.insert(0, os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python"))
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(None, 0)
        sample = None
        deadline = time.monotonic() + report["timeout_seconds"]
        while time.monotonic() < deadline:
            if process.poll() is not None:
                report["peer_exited_before_sample"] = True
                break
            sample = subscriber.Read(0.5)
            if sample is not None:
                report["lowstate_sample_received"] = True
                report["lowstate_summary_before_write"] = summarize_lowstate(sample)
                break

        if sample is not None:
            publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
            publisher.Init()
            crc = CRC()
            low_cmd = make_hold_lowcmd(sample, unitree_hg_msg_dds__LowCmd_, crc)
            report["lowcmd_summary"] = summarize_lowcmd(low_cmd)
            for _ in range(max(1, report["hold_frames"])):
                report["lowcmd_write_attempts"] += 1
                if publisher.Write(low_cmd, 1.0):
                    report["lowcmd_write_successes"] += 1
                time.sleep(0.002)
            publisher.Close()
        subscriber.Close()
    except Exception as exc:
        report["probe_error"] = str(exc)
        report["traceback"] = traceback.format_exc()
    finally:
        stdout_tail, stderr_tail, returncode = terminate_process(process)

    output = f"{stdout_tail}\n{stderr_tail}"
    report["stdout_tail"] = stdout_tail
    report["stderr_tail"] = stderr_tail
    report["returncode"] = returncode
    report["startup_reached_mujoco"] = "MuJoCo version" in output
    report["bridge_started"] = "Mujoco data is prepared" in output or "Try to start sdk2 thread" in output
    report["loader_error"] = "error while loading shared libraries" in output
    report["glfw_error"] = "could not initialize GLFW" in output
    report["cyclonedds_warning"] = "selected interface" in output or "ddsi_udp_conn_write" in output
    report["ok"] = (
        report["peer_started"]
        and report["startup_reached_mujoco"]
        and report["lowstate_sample_received"]
        and report["lowcmd_write_successes"] > 0
        and not report["loader_error"]
        and not report["glfw_error"]
    )
    if report["ok"]:
        report["next_step"] = "Run probe_official_mujoco_arm_motion for bounded movement, then promote the proven writer into a long-lived DDS transport behind cybernetic_robotics."
    elif report["lowstate_sample_received"]:
        report["next_step"] = "Lowstate is readable, but rt/lowcmd writes did not match a subscriber; inspect DDS discovery/interface settings before moving control to the official transport."
    else:
        report["next_step"] = "Lowstate was not received, so re-run unitree_probe_official_mujoco_dds before attempting lowcmd control."
    return report


def probe_official_mujoco_arm_motion(mujoco_root: str, domain: int, interface: str | None) -> dict:
    plan = official_mujoco_plan(mujoco_root, domain, interface)
    simulate_root = Path(plan["simulate_root"])
    executable = Path(plan["binary_path"])
    joint_name = os.environ.get("CYBER_UNITREE_ARM_MOTION_JOINT", "right_shoulder_roll")
    joint_index = G1_JOINT_INDEX.get(joint_name)
    delta = float(os.environ.get("CYBER_UNITREE_ARM_MOTION_DELTA", "-0.25"))
    kp = float(os.environ.get("CYBER_UNITREE_ARM_MOTION_KP", "35.0"))
    kd = float(os.environ.get("CYBER_UNITREE_ARM_MOTION_KD", "1.2"))
    hold_kp = float(os.environ.get("CYBER_UNITREE_ARM_MOTION_HOLD_KP", "18.0"))
    hold_kd = float(os.environ.get("CYBER_UNITREE_ARM_MOTION_HOLD_KD", "0.8"))
    threshold = float(os.environ.get("CYBER_UNITREE_ARM_MOTION_THRESHOLD", "0.025"))
    report = {
        "action": "probe_official_mujoco_arm_motion",
        "plan": plan,
        "binary_exists": executable.exists(),
        "library_path": runtime_library_path(),
        "used_xvfb": True,
        "read_topic": "rt/lowstate",
        "write_topic": "rt/lowcmd",
        "domain": domain,
        "interface": interface or None,
        "peer_started": False,
        "lowstate_sample_received": False,
        "lowcmd_write_attempts": 0,
        "lowcmd_write_successes": 0,
        "timeout_seconds": float(os.environ.get("CYBER_UNITREE_ARM_MOTION_TIMEOUT", "12")),
        "motion_frames": int(os.environ.get("CYBER_UNITREE_ARM_MOTION_FRAMES", "220")),
        "joint_name": joint_name,
        "joint_index": joint_index,
        "target_delta": delta,
        "kp": kp,
        "kd": kd,
        "hold_kp": hold_kp,
        "hold_kd": hold_kd,
        "movement_threshold": threshold,
    }
    if joint_index is None:
        report["ok"] = False
        report["error"] = f"unknown G1 arm joint: {joint_name}"
        report["known_joints"] = sorted(G1_JOINT_INDEX)
        return report
    if not executable.exists():
        report["ok"] = False
        report["error"] = "missing official unitree_mujoco binary; run build_official_mujoco first"
        return report

    command = [
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

    process = subprocess.Popen(
        command,
        cwd=str(simulate_root),
        env=runtime_env(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    report["peer_started"] = True
    report["peer_pid"] = process.pid
    report["command"] = " ".join(command)

    try:
        sys.path.insert(0, os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python"))
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(None, 0)
        sample = None
        deadline = time.monotonic() + report["timeout_seconds"]
        while time.monotonic() < deadline:
            if process.poll() is not None:
                report["peer_exited_before_sample"] = True
                break
            sample = subscriber.Read(0.5)
            if sample is not None:
                report["lowstate_sample_received"] = True
                report["lowstate_summary_before_motion"] = summarize_lowstate(sample)
                break

        if sample is not None:
            initial_q = motor_position(sample, joint_index)
            if initial_q is None:
                raise RuntimeError(f"lowstate does not contain joint index {joint_index}")
            target_q = initial_q + delta
            report["initial_q"] = initial_q
            report["target_q"] = target_q

            publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
            publisher.Init()
            crc = CRC()
            low_cmd = make_single_joint_lowcmd(
                sample,
                unitree_hg_msg_dds__LowCmd_,
                crc,
                joint_index,
                target_q,
                kp,
                kd,
                hold_kp,
                hold_kd,
            )
            report["lowcmd_summary"] = summarize_lowcmd(low_cmd)
            for _ in range(max(1, report["motion_frames"])):
                report["lowcmd_write_attempts"] += 1
                if publisher.Write(low_cmd, 1.0):
                    report["lowcmd_write_successes"] += 1
                time.sleep(0.003)
            publisher.Close()

            final_sample = sample
            read_until = time.monotonic() + 1.0
            while time.monotonic() < read_until:
                next_sample = subscriber.Read(0.1)
                if next_sample is not None:
                    final_sample = next_sample
            final_q = motor_position(final_sample, joint_index)
            report["lowstate_summary_after_motion"] = summarize_lowstate(final_sample)
            report["final_q"] = final_q
            if final_q is not None:
                report["actual_delta"] = final_q - initial_q
                report["target_error"] = target_q - final_q
                report["motion_detected"] = abs(report["actual_delta"]) >= threshold
        subscriber.Close()
    except Exception as exc:
        report["probe_error"] = str(exc)
        report["traceback"] = traceback.format_exc()
    finally:
        stdout_tail, stderr_tail, returncode = terminate_process(process)

    output = f"{stdout_tail}\n{stderr_tail}"
    report["stdout_tail"] = stdout_tail
    report["stderr_tail"] = stderr_tail
    report["returncode"] = returncode
    report["startup_reached_mujoco"] = "MuJoCo version" in output
    report["bridge_started"] = "Mujoco data is prepared" in output or "Try to start sdk2 thread" in output
    report["loader_error"] = "error while loading shared libraries" in output
    report["glfw_error"] = "could not initialize GLFW" in output
    report["cyclonedds_warning"] = "selected interface" in output or "ddsi_udp_conn_write" in output
    report["ok"] = (
        report["peer_started"]
        and report["startup_reached_mujoco"]
        and report["lowstate_sample_received"]
        and report["lowcmd_write_successes"] > 0
        and report.get("motion_detected") is True
        and not report["loader_error"]
        and not report["glfw_error"]
    )
    if report["ok"]:
        report["next_step"] = "Move this bounded arm-motion path into a long-lived DDS session provider and expose it through cybernetic_robotics examples."
    elif report.get("lowcmd_write_successes", 0) > 0:
        report["next_step"] = "Lowcmd writes matched, but lowstate did not show enough movement; increase duration/gain carefully or inspect the official bridge control mode."
    else:
        report["next_step"] = "Lowcmd writes did not prove out; re-run the hold-command lowcmd probe before attempting motion."
    return report


def probe_official_mujoco_arm_pose(mujoco_root: str, domain: int, interface: str | None) -> dict:
    plan = official_mujoco_plan(mujoco_root, domain, interface)
    simulate_root = Path(plan["simulate_root"])
    executable = Path(plan["binary_path"])
    preset = os.environ.get("CYBER_UNITREE_ARM_POSE_PRESET", "raise_right_hand")
    raw_deltas = os.environ.get("CYBER_UNITREE_ARM_POSE_DELTAS", "")
    kp = float(os.environ.get("CYBER_UNITREE_ARM_POSE_KP", "30.0"))
    kd = float(os.environ.get("CYBER_UNITREE_ARM_POSE_KD", "1.0"))
    hold_kp = float(os.environ.get("CYBER_UNITREE_ARM_POSE_HOLD_KP", "18.0"))
    hold_kd = float(os.environ.get("CYBER_UNITREE_ARM_POSE_HOLD_KD", "0.8"))
    threshold = float(os.environ.get("CYBER_UNITREE_ARM_POSE_THRESHOLD", "0.025"))
    min_moved_joints = int(os.environ.get("CYBER_UNITREE_ARM_POSE_MIN_MOVED_JOINTS", "2"))
    report = {
        "action": "probe_official_mujoco_arm_pose",
        "plan": plan,
        "binary_exists": executable.exists(),
        "library_path": runtime_library_path(),
        "used_xvfb": True,
        "read_topic": "rt/lowstate",
        "write_topic": "rt/lowcmd",
        "domain": domain,
        "interface": interface or None,
        "peer_started": False,
        "lowstate_sample_received": False,
        "lowcmd_write_attempts": 0,
        "lowcmd_write_successes": 0,
        "timeout_seconds": float(os.environ.get("CYBER_UNITREE_ARM_POSE_TIMEOUT", "12")),
        "motion_frames": int(os.environ.get("CYBER_UNITREE_ARM_POSE_FRAMES", "180")),
        "preset": preset,
        "kp": kp,
        "kd": kd,
        "hold_kp": hold_kp,
        "hold_kd": hold_kd,
        "movement_threshold": threshold,
        "min_moved_joints": min_moved_joints,
    }
    if not executable.exists():
        report["ok"] = False
        report["error"] = "missing official unitree_mujoco binary; run build_official_mujoco first"
        return report

    try:
        requested_deltas = json.loads(raw_deltas) if raw_deltas else G1_ARM_POSE_PRESETS[preset]
    except KeyError:
        report["ok"] = False
        report["error"] = f"unknown G1 arm pose preset: {preset}"
        report["known_presets"] = sorted(G1_ARM_POSE_PRESETS)
        return report
    except Exception as exc:
        report["ok"] = False
        report["error"] = f"invalid CYBER_UNITREE_ARM_POSE_DELTAS JSON: {exc}"
        return report

    target_deltas: dict[str, float] = {}
    unknown_joints: list[str] = []
    for joint_name, raw_delta in dict(requested_deltas).items():
        if joint_name not in G1_JOINT_INDEX:
            unknown_joints.append(joint_name)
            continue
        try:
            target_deltas[joint_name] = max(-0.5, min(0.5, float(raw_delta)))
        except (TypeError, ValueError):
            report["ok"] = False
            report["error"] = f"invalid delta for {joint_name}: {raw_delta!r}"
            return report
    if unknown_joints:
        report["ok"] = False
        report["error"] = "unknown G1 arm joints in pose target"
        report["unknown_joints"] = unknown_joints
        report["known_joints"] = sorted(G1_JOINT_INDEX)
        return report
    if not target_deltas:
        report["ok"] = False
        report["error"] = "arm pose target is empty"
        return report
    report["target_deltas"] = target_deltas

    command = [
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

    process = subprocess.Popen(
        command,
        cwd=str(simulate_root),
        env=runtime_env(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    report["peer_started"] = True
    report["peer_pid"] = process.pid
    report["command"] = " ".join(command)

    try:
        sys.path.insert(0, os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python"))
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(None, 0)
        sample = None
        deadline = time.monotonic() + report["timeout_seconds"]
        while time.monotonic() < deadline:
            if process.poll() is not None:
                report["peer_exited_before_sample"] = True
                break
            sample = subscriber.Read(0.5)
            if sample is not None:
                report["lowstate_sample_received"] = True
                report["lowstate_summary_before_motion"] = summarize_lowstate(sample)
                break

        if sample is not None:
            joint_reports: dict[str, dict] = {}
            target_positions: dict[int, float] = {}
            for joint_name, delta in target_deltas.items():
                joint_index = G1_JOINT_INDEX[joint_name]
                initial_q = motor_position(sample, joint_index)
                if initial_q is None:
                    raise RuntimeError(f"lowstate does not contain joint index {joint_index}")
                target_q = initial_q + delta
                target_positions[joint_index] = target_q
                joint_reports[joint_name] = {
                    "joint_index": joint_index,
                    "initial_q": initial_q,
                    "target_delta": delta,
                    "target_q": target_q,
                }

            publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
            publisher.Init()
            crc = CRC()
            low_cmd = make_multi_joint_lowcmd(
                sample,
                unitree_hg_msg_dds__LowCmd_,
                crc,
                target_positions,
                kp,
                kd,
                hold_kp,
                hold_kd,
            )
            report["lowcmd_summary"] = summarize_lowcmd(low_cmd)
            for _ in range(max(1, report["motion_frames"])):
                report["lowcmd_write_attempts"] += 1
                if publisher.Write(low_cmd, 1.0):
                    report["lowcmd_write_successes"] += 1
                time.sleep(0.003)
            publisher.Close()

            final_sample = sample
            read_until = time.monotonic() + 1.0
            while time.monotonic() < read_until:
                next_sample = subscriber.Read(0.1)
                if next_sample is not None:
                    final_sample = next_sample
            report["lowstate_summary_after_motion"] = summarize_lowstate(final_sample)
            moved_joints = []
            for joint_name, joint_report in joint_reports.items():
                final_q = motor_position(final_sample, joint_report["joint_index"])
                joint_report["final_q"] = final_q
                if final_q is not None:
                    joint_report["actual_delta"] = final_q - joint_report["initial_q"]
                    joint_report["target_error"] = joint_report["target_q"] - final_q
                    joint_report["motion_detected"] = abs(joint_report["actual_delta"]) >= threshold
                    if joint_report["motion_detected"]:
                        moved_joints.append(joint_name)
            report["joints"] = joint_reports
            report["moved_joints"] = moved_joints
            report["moved_joint_count"] = len(moved_joints)
        subscriber.Close()
    except Exception as exc:
        report["probe_error"] = str(exc)
        report["traceback"] = traceback.format_exc()
    finally:
        stdout_tail, stderr_tail, returncode = terminate_process(process)

    output = f"{stdout_tail}\n{stderr_tail}"
    report["stdout_tail"] = stdout_tail
    report["stderr_tail"] = stderr_tail
    report["returncode"] = returncode
    report["startup_reached_mujoco"] = "MuJoCo version" in output
    report["bridge_started"] = "Mujoco data is prepared" in output or "Try to start sdk2 thread" in output
    report["loader_error"] = "error while loading shared libraries" in output
    report["glfw_error"] = "could not initialize GLFW" in output
    report["cyclonedds_warning"] = "selected interface" in output or "ddsi_udp_conn_write" in output
    report["ok"] = (
        report["peer_started"]
        and report["startup_reached_mujoco"]
        and report["lowstate_sample_received"]
        and report["lowcmd_write_successes"] > 0
        and report.get("moved_joint_count", 0) >= min_moved_joints
        and not report["loader_error"]
        and not report["glfw_error"]
    )
    if report["ok"]:
        report["next_step"] = "Promote this multi-joint official lowcmd pose into a long-lived DDS session provider and SDK-shaped arm action demo."
    elif report.get("lowcmd_write_successes", 0) > 0:
        report["next_step"] = "Lowcmd writes matched, but too few joints moved; reduce the pose, increase duration carefully, or inspect the official bridge control mode."
    else:
        report["next_step"] = "Lowcmd writes did not prove out; re-run the single-joint arm-motion probe before attempting pose control."
    return report


def command_official_mujoco_arm_pose(domain: int, interface: str | None) -> dict:
    preset = os.environ.get("CYBER_UNITREE_ARM_POSE_PRESET", "raise_right_hand")
    raw_deltas = os.environ.get("CYBER_UNITREE_ARM_POSE_DELTAS", "")
    kp = float(os.environ.get("CYBER_UNITREE_ARM_POSE_KP", "30.0"))
    kd = float(os.environ.get("CYBER_UNITREE_ARM_POSE_KD", "1.0"))
    hold_kp = float(os.environ.get("CYBER_UNITREE_ARM_POSE_HOLD_KP", "18.0"))
    hold_kd = float(os.environ.get("CYBER_UNITREE_ARM_POSE_HOLD_KD", "0.8"))
    threshold = float(os.environ.get("CYBER_UNITREE_ARM_POSE_THRESHOLD", "0.025"))
    min_moved_joints = int(os.environ.get("CYBER_UNITREE_ARM_POSE_MIN_MOVED_JOINTS", "2"))
    report = {
        "action": "command_official_mujoco_arm_pose",
        "read_topic": "rt/lowstate",
        "write_topic": "rt/lowcmd",
        "domain": domain,
        "interface": interface or None,
        "lowstate_sample_received": False,
        "lowcmd_write_attempts": 0,
        "lowcmd_write_successes": 0,
        "timeout_seconds": float(os.environ.get("CYBER_UNITREE_ARM_POSE_TIMEOUT", "12")),
        "motion_frames": int(os.environ.get("CYBER_UNITREE_ARM_POSE_FRAMES", "180")),
        "preset": preset,
        "kp": kp,
        "kd": kd,
        "hold_kp": hold_kp,
        "hold_kd": hold_kd,
        "movement_threshold": threshold,
        "min_moved_joints": min_moved_joints,
    }
    try:
        requested_deltas = json.loads(raw_deltas) if raw_deltas else G1_ARM_POSE_PRESETS[preset]
    except KeyError:
        report["ok"] = False
        report["error"] = f"unknown G1 arm pose preset: {preset}"
        report["known_presets"] = sorted(G1_ARM_POSE_PRESETS)
        return report
    except Exception as exc:
        report["ok"] = False
        report["error"] = f"invalid CYBER_UNITREE_ARM_POSE_DELTAS JSON: {exc}"
        return report

    target_deltas: dict[str, float] = {}
    unknown_joints: list[str] = []
    for joint_name, raw_delta in dict(requested_deltas).items():
        if joint_name not in G1_JOINT_INDEX:
            unknown_joints.append(joint_name)
            continue
        try:
            target_deltas[joint_name] = max(-0.5, min(0.5, float(raw_delta)))
        except (TypeError, ValueError):
            report["ok"] = False
            report["error"] = f"invalid delta for {joint_name}: {raw_delta!r}"
            return report
    if unknown_joints:
        report["ok"] = False
        report["error"] = "unknown G1 arm joints in pose target"
        report["unknown_joints"] = unknown_joints
        report["known_joints"] = sorted(G1_JOINT_INDEX)
        return report
    if not target_deltas:
        report["ok"] = False
        report["error"] = "arm pose target is empty"
        return report
    report["target_deltas"] = target_deltas

    try:
        sys.path.insert(0, os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python"))
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(None, 0)
        sample = None
        deadline = time.monotonic() + report["timeout_seconds"]
        while time.monotonic() < deadline:
            sample = subscriber.Read(0.5)
            if sample is not None:
                report["lowstate_sample_received"] = True
                report["lowstate_summary_before_motion"] = summarize_lowstate(sample)
                break

        if sample is not None:
            joint_reports: dict[str, dict] = {}
            target_positions: dict[int, float] = {}
            for joint_name, delta in target_deltas.items():
                joint_index = G1_JOINT_INDEX[joint_name]
                initial_q = motor_position(sample, joint_index)
                if initial_q is None:
                    raise RuntimeError(f"lowstate does not contain joint index {joint_index}")
                target_q = initial_q + delta
                target_positions[joint_index] = target_q
                joint_reports[joint_name] = {
                    "joint_index": joint_index,
                    "initial_q": initial_q,
                    "target_delta": delta,
                    "target_q": target_q,
                }

            publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
            publisher.Init()
            crc = CRC()
            low_cmd = make_multi_joint_lowcmd(
                sample,
                unitree_hg_msg_dds__LowCmd_,
                crc,
                target_positions,
                kp,
                kd,
                hold_kp,
                hold_kd,
            )
            report["lowcmd_summary"] = summarize_lowcmd(low_cmd)
            for _ in range(max(1, report["motion_frames"])):
                report["lowcmd_write_attempts"] += 1
                if publisher.Write(low_cmd, 1.0):
                    report["lowcmd_write_successes"] += 1
                time.sleep(0.003)
            publisher.Close()

            final_sample = sample
            read_until = time.monotonic() + 1.0
            while time.monotonic() < read_until:
                next_sample = subscriber.Read(0.1)
                if next_sample is not None:
                    final_sample = next_sample
            report["lowstate_summary_after_motion"] = summarize_lowstate(final_sample)
            moved_joints = []
            for joint_name, joint_report in joint_reports.items():
                final_q = motor_position(final_sample, joint_report["joint_index"])
                joint_report["final_q"] = final_q
                if final_q is not None:
                    joint_report["actual_delta"] = final_q - joint_report["initial_q"]
                    joint_report["target_error"] = joint_report["target_q"] - final_q
                    joint_report["motion_detected"] = abs(joint_report["actual_delta"]) >= threshold
                    if joint_report["motion_detected"]:
                        moved_joints.append(joint_name)
            report["joints"] = joint_reports
            report["moved_joints"] = moved_joints
            report["moved_joint_count"] = len(moved_joints)
        subscriber.Close()
    except Exception as exc:
        report["probe_error"] = str(exc)
        report["traceback"] = traceback.format_exc()

    report["ok"] = (
        report["lowstate_sample_received"]
        and report["lowcmd_write_successes"] > 0
        and report.get("moved_joint_count", 0) >= min_moved_joints
    )
    if report["ok"]:
        report["next_step"] = "Route the Unitree-shaped Python arm facade to this managed-session command path when CYBER_UNITREE_TRANSPORT=dds."
    elif report.get("lowcmd_write_successes", 0) > 0:
        report["next_step"] = "Lowcmd writes matched, but too few joints moved; check whether the managed official peer is in the expected control mode."
    else:
        report["next_step"] = "No lowcmd writes succeeded; start unitree-g1-sdk2-session before commanding the managed peer."
    return report


def probe_official_mujoco_loco_rpc(domain: int, interface: str | None) -> dict:
    """Probe the official G1 sport/LocoClient RPC service on a managed session."""

    timeout = float(os.environ.get("CYBER_UNITREE_LOCO_RPC_TIMEOUT", "2.0"))
    include_stop = os.environ.get("CYBER_UNITREE_LOCO_RPC_STOP_MOVE", "0") == "1"
    report = {
        "action": "probe_official_mujoco_loco_rpc",
        "request_topic": "rt/api/sport/request",
        "response_topic": "rt/api/sport/response",
        "domain": domain,
        "interface": interface or None,
        "timeout_seconds": timeout,
        "client_initialized": False,
        "calls": [],
        "safe_motion_call_enabled": include_stop,
    }
    try:
        sys.path.insert(0, os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python"))
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

        client = LocoClient()
        client.SetTimeout(timeout)
        client.Init()
        report["client_initialized"] = True

        def record_call(name: str, fn) -> None:
            started = time.monotonic()
            entry = {"name": name}
            try:
                value = fn()
                entry["ok"] = _loco_call_ok(value)
                entry["return"] = _jsonable_return(value)
                code = _loco_return_code(value)
                if code is not None:
                    entry["rpc_code"] = code
                    entry["rpc_status"] = _unitree_rpc_status(code)
            except Exception as exc:
                entry["ok"] = False
                entry["error"] = str(exc)
            entry["elapsed_seconds"] = round(time.monotonic() - started, 4)
            report["calls"].append(entry)

        record_call("GetFsmId", client.GetFsmId)
        if include_stop:
            record_call("StopMove", client.StopMove)
    except Exception as exc:
        report["probe_error"] = str(exc)
        report["traceback"] = traceback.format_exc()

    report["ok"] = bool(report["client_initialized"] and any(call.get("ok") for call in report["calls"]))
    if report["ok"]:
        report["next_step"] = "Route CYBER_UNITREE_TRANSPORT=dds LocoClient calls to this official sport RPC path with safety gating."
    elif report["client_initialized"]:
        report["diagnosis"] = _diagnose_loco_rpc_calls(report["calls"])
        report["next_step"] = report["diagnosis"]["next_step"]
    else:
        report["next_step"] = "Fix official Unitree SDK2 Python LocoClient initialization before promoting locomotion to DDS."
    return report


def probe_official_mujoco_rpc_discovery(domain: int, interface: str | None) -> dict:
    """Inspect which Unitree RPC service request topics have matched readers."""

    wait_seconds = float(os.environ.get("CYBER_UNITREE_RPC_DISCOVERY_WAIT", "1.0"))
    services = [
        {
            "name": "sport",
            "source": "unitree_sdk2_python.g1.loco",
            "request_topic": "rt/api/sport/request",
            "response_topic": "rt/api/sport/response",
            "client": "LocoClient",
        },
        {
            "name": "agv",
            "source": "unitree_sdk2 C++ g1/agv headers",
            "request_topic": "rt/api/agv/request",
            "response_topic": "rt/api/agv/response",
            "client": "AgvClient",
        },
        {
            "name": "arm",
            "source": "unitree_sdk2_python.g1.arm",
            "request_topic": "rt/api/arm/request",
            "response_topic": "rt/api/arm/response",
            "client": "G1ArmActionClient",
        },
        {
            "name": "voice",
            "source": "unitree_sdk2_python.g1.audio",
            "request_topic": "rt/api/voice/request",
            "response_topic": "rt/api/voice/response",
            "client": "AudioClient",
        },
    ]
    report = {
        "action": "probe_official_mujoco_rpc_discovery",
        "domain": domain,
        "interface": interface or None,
        "wait_seconds": wait_seconds,
        "services": [],
    }
    try:
        sys.path.insert(0, os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python"))
        from unitree_sdk2py.core.channel import ChannelFactory, ChannelFactoryInitialize
        from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_ as Request
        from unitree_sdk2py.idl.unitree_api.msg.dds_ import Response_ as Response

        ChannelFactoryInitialize(domain, interface)
        factory = ChannelFactory()
        channels = []
        request_channels = []
        for service in services:
            entry = dict(service)
            try:
                request_channel = factory.CreateSendChannel(service["request_topic"], Request)
                response_channel = factory.CreateRecvChannel(service["response_topic"], Response)
                channels.extend([request_channel, response_channel])
                request_channels.append((entry, request_channel))
                entry["request_writer_created"] = True
                entry["response_reader_created"] = True
            except Exception as exc:
                entry["request_writer_created"] = False
                entry["response_reader_created"] = False
                entry["error"] = str(exc)
            report["services"].append(entry)

        time.sleep(max(0.0, wait_seconds))

        for entry, request_channel in request_channels:
            if not entry.get("request_writer_created"):
                continue
            entry["request_matched_readers"] = _channel_publication_matched_count(request_channel)
            entry["request_service_reader_present"] = entry["request_matched_readers"] > 0

        for channel in channels:
            try:
                channel.CloseWriter()
            except Exception:
                pass
            try:
                channel.CloseReader()
            except Exception:
                pass
    except Exception as exc:
        report["error"] = str(exc)
        report["traceback"] = traceback.format_exc()

    matched = [service["name"] for service in report["services"] if service.get("request_service_reader_present")]
    missing = [
        service["name"]
        for service in report["services"]
        if service.get("request_writer_created") and not service.get("request_service_reader_present")
    ]
    report["matched_services"] = matched
    report["missing_request_readers"] = missing
    report["ok"] = bool(matched)
    if "sport" in missing:
        report["sport_rpc_diagnosis"] = {
            "likely_cause": "no_service_reader_for_rt_api_sport_request",
            "evidence": "A Unitree Request_ writer on rt/api/sport/request did not match any reader during DDS discovery.",
            "next_step": "Keep LocoClient on the local compatibility route or add a sport service bridge for the managed official MuJoCo peer.",
        }
    if "agv" in missing:
        report["agv_rpc_diagnosis"] = {
            "likely_cause": "no_service_reader_for_rt_api_agv_request",
            "evidence": "The C++ G1 AGV service topic rt/api/agv/request did not match any reader during DDS discovery.",
            "next_step": "Do not promote authored AgvClient calls to official DDS until an AGV service reader is present.",
        }
    return report


def probe_unitree_rpc_bridge_smoke(domain: int, interface: str | None) -> dict:
    """Start temporary Unitree-shaped RPC services and call them with SDK clients."""

    timeout = float(os.environ.get("CYBER_UNITREE_RPC_BRIDGE_TIMEOUT", "1.0"))
    report = {
        "action": "probe_unitree_rpc_bridge_smoke",
        "domain": domain,
        "interface": interface or None,
        "timeout_seconds": timeout,
        "services_started": [],
        "calls": [],
        "bridge_state": {
            "sport": {
                "fsm_id": 500,
                "fsm_mode": "idle",
                "balance_mode": 0,
                "swing_height": 0.08,
                "stand_height": 0.72,
                "velocity": [0.0, 0.0, 0.0],
            },
            "agv": {
                "velocity": [0.0, 0.0, 0.0],
                "height_velocity": 0.0,
            },
            "arm": {
                "last_action_id": None,
                "last_pose": None,
                "action_count": 0,
            },
        },
    }
    try:
        sys.path.insert(0, os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python"))
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.arm.g1_arm_action_api import (
            ARM_ACTION_API_VERSION,
            ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION,
            ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST,
        )
        from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
        from unitree_sdk2py.g1.loco.g1_loco_api import (
            LOCO_API_VERSION,
            ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
            ROBOT_API_ID_LOCO_GET_FSM_ID,
            ROBOT_API_ID_LOCO_GET_FSM_MODE,
            ROBOT_API_ID_LOCO_GET_STAND_HEIGHT,
            ROBOT_API_ID_LOCO_GET_SWING_HEIGHT,
            ROBOT_API_ID_LOCO_SET_FSM_ID,
            ROBOT_API_ID_LOCO_SET_STAND_HEIGHT,
            ROBOT_API_ID_LOCO_SET_VELOCITY,
        )
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.rpc.client import Client
        from unitree_sdk2py.rpc.server import Server

        ChannelFactoryInitialize(domain, interface)

        sport = Server("sport")
        sport._SetApiVersion(LOCO_API_VERSION)
        sport_state = report["bridge_state"]["sport"]

        def sport_get_fsm_id(_parameter: str):
            return 0, json.dumps({"data": sport_state["fsm_id"]})

        def sport_get_fsm_mode(_parameter: str):
            return 0, json.dumps({"data": sport_state["fsm_mode"]})

        def sport_get_balance_mode(_parameter: str):
            return 0, json.dumps({"data": sport_state["balance_mode"]})

        def sport_get_swing_height(_parameter: str):
            return 0, json.dumps({"data": sport_state["swing_height"]})

        def sport_get_stand_height(_parameter: str):
            return 0, json.dumps({"data": sport_state["stand_height"]})

        def sport_set_fsm_id(parameter: str):
            payload = _safe_json_loads(parameter)
            sport_state["fsm_id"] = int(payload.get("data", sport_state["fsm_id"]))
            return 0, json.dumps({"data": sport_state["fsm_id"]})

        def sport_set_stand_height(parameter: str):
            payload = _safe_json_loads(parameter)
            sport_state["stand_height"] = float(payload.get("data", sport_state["stand_height"]))
            return 0, json.dumps({"data": sport_state["stand_height"]})

        def sport_set_velocity(parameter: str):
            payload = _safe_json_loads(parameter)
            velocity = payload.get("velocity", sport_state["velocity"])
            sport_state["velocity"] = [float(value) for value in velocity[:3]]
            sport_state["duration"] = float(payload.get("duration", 0.0))
            return 0, json.dumps({"velocity": sport_state["velocity"], "duration": sport_state["duration"]})

        for api_id, handler in [
            (ROBOT_API_ID_LOCO_GET_FSM_ID, sport_get_fsm_id),
            (ROBOT_API_ID_LOCO_GET_FSM_MODE, sport_get_fsm_mode),
            (ROBOT_API_ID_LOCO_GET_BALANCE_MODE, sport_get_balance_mode),
            (ROBOT_API_ID_LOCO_GET_SWING_HEIGHT, sport_get_swing_height),
            (ROBOT_API_ID_LOCO_GET_STAND_HEIGHT, sport_get_stand_height),
            (ROBOT_API_ID_LOCO_SET_FSM_ID, sport_set_fsm_id),
            (ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, sport_set_stand_height),
            (ROBOT_API_ID_LOCO_SET_VELOCITY, sport_set_velocity),
        ]:
            sport._RegistHandler(api_id, handler, False)
        sport.Start(False)
        report["services_started"].append("sport")

        agv = Server("agv")
        agv._SetApiVersion("1.0.0.1")
        agv_state = report["bridge_state"]["agv"]

        def agv_move(parameter: str):
            payload = _safe_json_loads(parameter)
            agv_state["velocity"] = [
                float(payload.get("vx", 0.0)),
                float(payload.get("vy", 0.0)),
                float(payload.get("vyaw", 0.0)),
            ]
            return 0, json.dumps({"velocity": agv_state["velocity"]})

        def agv_height_adjust(parameter: str):
            payload = _safe_json_loads(parameter)
            agv_state["height_velocity"] = float(payload.get("data", 0.0))
            return 0, json.dumps({"data": agv_state["height_velocity"]})

        agv._RegistHandler(1001, agv_move, False)
        agv._RegistHandler(1002, agv_height_adjust, False)
        agv.Start(False)
        report["services_started"].append("agv")

        arm = Server("arm")
        arm._SetApiVersion(ARM_ACTION_API_VERSION)
        arm_state = report["bridge_state"]["arm"]

        def arm_execute_action(parameter: str):
            payload = _safe_json_loads(parameter)
            action_id = int(payload.get("data", 0))
            pose = G1_ARM_POSES_BY_ACTION_ID.get(action_id)
            arm_state["last_action_id"] = action_id
            arm_state["last_pose"] = pose
            arm_state["action_count"] = int(arm_state.get("action_count", 0)) + 1
            if pose:
                simulator_forward = _forward_simulator_command({"command": "pose", "pose": pose, "unitree_action_id": action_id})
            else:
                simulator_forward = _bridge_state_only_simulator_result(
                    {"command": "arm_action", "action_id": action_id},
                    f"Unknown G1 arm action id {action_id}; bridge recorded intent only.",
                )
            arm_state["last_simulator_forward"] = simulator_forward
            return 0, json.dumps({"data": action_id, "pose": pose, "simulator_forward": simulator_forward})

        def arm_get_action_list(_parameter: str):
            actions = [
                {"name": name, "id": action_id, "simulated": action_id in G1_ARM_POSES_BY_ACTION_ID}
                for name, action_id in sorted(G1_ARM_ACTIONS.items(), key=lambda item: item[1])
            ]
            return 0, json.dumps(actions)

        arm._RegistHandler(ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION, arm_execute_action, False)
        arm._RegistHandler(ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST, arm_get_action_list, False)
        arm.Start(False)
        report["services_started"].append("arm")

        time.sleep(0.3)

        loco = LocoClient()
        loco.SetTimeout(timeout)
        loco.Init()
        _record_rpc_call(report["calls"], "sport.GetFsmId", loco.GetFsmId)
        _record_rpc_call(report["calls"], "sport.SetStandHeight", lambda: loco.SetStandHeight(0.73))
        _record_rpc_call(report["calls"], "sport.SetVelocity", lambda: loco.SetVelocity(0.0, 0.0, 0.0, 0.1))

        agv_client = Client("agv", False)
        agv_client.SetTimeout(timeout)
        agv_client._SetApiVerson("1.0.0.1")
        agv_client._RegistApi(1001, 0)
        agv_client._RegistApi(1002, 0)
        _record_rpc_call(report["calls"], "agv.Move", lambda: agv_client._Call(1001, json.dumps({"vx": 0.0, "vy": 0.0, "vyaw": 0.0})))
        _record_rpc_call(report["calls"], "agv.HeightAdjust", lambda: agv_client._Call(1002, json.dumps({"data": 0.0})))

        arm_client = G1ArmActionClient()
        arm_client.SetTimeout(timeout)
        arm_client.Init()
        _record_rpc_call(report["calls"], "arm.GetActionList", arm_client.GetActionList)
        _record_rpc_call(report["calls"], "arm.ExecuteAction", lambda: arm_client.ExecuteAction(action_map["right hand up"]))
    except Exception as exc:
        report["error"] = str(exc)
        report["traceback"] = traceback.format_exc()

    report["ok"] = bool(report.get("services_started") and report["calls"] and all(call.get("ok") for call in report["calls"]))
    if report["ok"]:
        report["next_step"] = "Promote this temporary SDK2 RPC server pattern into a managed bridge that maps sport/agv/arm requests onto the simulator provider boundary."
    else:
        report["next_step"] = "Fix the temporary SDK2 RPC bridge before using it as the high-level Unitree service adapter."
    return report


def serve_unitree_rpc_bridge(domain: int, interface: str | None) -> int:
    """Serve Unitree-shaped sport/agv RPC endpoints until the container stops."""

    simulator_url = _simulator_game_control_url()
    report = {
        "action": "serve_unitree_rpc_bridge",
        "domain": domain,
        "interface": interface or None,
        "simulator_game_control_url": simulator_url,
        "services_started": [],
        "bridge_state": _initial_rpc_bridge_state(),
        "ok": False,
    }
    try:
        services = _start_unitree_rpc_bridge_services(domain, interface, report["bridge_state"])
        report["services_started"] = services
        report["ok"] = True
        report["next_step"] = "Keep this container running as the Unitree sport/agv/arm RPC bridge; supported setter/action RPCs now forward to Cybernetic's simulator HTTP provider when it is reachable."
    except Exception as exc:
        report["error"] = str(exc)
        report["traceback"] = traceback.format_exc()
        print(json.dumps(report, indent=2), flush=True)
        return 1

    print(json.dumps(report, indent=2), flush=True)

    stopping = False

    def stop_bridge(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop_bridge)
    signal.signal(signal.SIGINT, stop_bridge)

    while not stopping:
        time.sleep(0.5)

    print(
        json.dumps(
            {
                "action": "serve_unitree_rpc_bridge_exit",
                "services_started": services,
                "stopping": stopping,
                "bridge_state": report["bridge_state"],
            },
            indent=2,
        ),
        flush=True,
    )
    return 0


def probe_unitree_rpc_bridge_client(domain: int, interface: str | None) -> dict:
    """Call an already-running Unitree-shaped RPC bridge with SDK clients."""

    timeout = float(os.environ.get("CYBER_UNITREE_RPC_BRIDGE_TIMEOUT", "1.0"))
    report = {
        "action": "probe_unitree_rpc_bridge_client",
        "domain": domain,
        "interface": interface or None,
        "timeout_seconds": timeout,
        "calls": [],
    }
    try:
        sys.path.insert(0, os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python"))
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.rpc.client import Client

        ChannelFactoryInitialize(domain, interface)
        _call_unitree_rpc_bridge_clients(report["calls"], timeout)
    except Exception as exc:
        report["error"] = str(exc)
        report["traceback"] = traceback.format_exc()

    report["ok"] = bool(report["calls"] and all(call.get("ok") for call in report["calls"]))
    if report["ok"]:
        report["next_step"] = "Route simulator-mode Unitree LocoClient/AgvClient calls through the managed RPC bridge once handlers are mapped onto simulator providers."
    else:
        report["next_step"] = "Start the managed Unitree RPC bridge and inspect call rpc_status fields."
    return report


def command_unitree_rpc_bridge_client(domain: int, interface: str | None) -> dict:
    """Call one Unitree-shaped RPC on an already-running managed bridge."""

    service = os.environ.get("CYBER_UNITREE_RPC_BRIDGE_SERVICE", "sport").strip().lower()
    method = os.environ.get("CYBER_UNITREE_RPC_BRIDGE_METHOD", "get_fsm_id").strip().lower()
    params = _safe_json_loads(os.environ.get("CYBER_UNITREE_RPC_BRIDGE_PARAMS", "{}"))
    timeout = float(os.environ.get("CYBER_UNITREE_RPC_BRIDGE_TIMEOUT", "1.0"))
    if not isinstance(params, dict):
        params = {"data": params}
    report = {
        "action": "command_unitree_rpc_bridge_client",
        "domain": domain,
        "interface": interface or None,
        "service": service,
        "method": method,
        "params": params,
        "timeout_seconds": timeout,
        "calls": [],
    }
    try:
        sys.path.insert(0, os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python"))
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize(domain, interface)
        report["normalized_request"] = _call_unitree_rpc_bridge_command(
            report["calls"],
            service,
            method,
            params,
            timeout,
        )
    except Exception as exc:
        report["error"] = str(exc)
        report["traceback"] = traceback.format_exc()

    report["ok"] = bool(report["calls"] and all(call.get("ok") for call in report["calls"]))
    if report["ok"]:
        report["next_step"] = "Use this SDK-shaped bridge command from the Python facade or Agent-panel MCP tool for single robot actions."
    else:
        report["next_step"] = "Check service/method names, start the managed Unitree RPC bridge, and inspect rpc_status/error fields."
    return report


def _initial_rpc_bridge_state() -> dict:
    return {
        "sport": {
            "fsm_id": 500,
            "fsm_mode": "start",
            "balance_mode": 0,
            "swing_height": 0.08,
            "stand_height": 0.72,
            "velocity": [0.0, 0.0, 0.0],
            "arm_task_id": None,
            "phase": [0.0, 0.0],
            "continuous_move": False,
            "speed_mode": 0,
            "control_owner": "internal",
            "internal_mode": 0,
        },
        "agv": {
            "velocity": [0.0, 0.0, 0.0],
            "height_velocity": 0.0,
        },
        "arm": {
            "last_action_id": None,
            "last_pose": None,
            "action_count": 0,
        },
    }


def _start_unitree_rpc_bridge_services(domain: int, interface: str | None, bridge_state: dict) -> list[str]:
    sys.path.insert(0, os.environ.get("UNITREE_SDK2_PYTHON_ROOT", "/opt/unitree_sdk2_python"))
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.arm.g1_arm_action_api import (
        ARM_ACTION_API_VERSION,
        ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION,
        ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST,
    )
    from unitree_sdk2py.g1.loco.g1_loco_api import (
        LOCO_API_VERSION,
        ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
        ROBOT_API_ID_LOCO_GET_FSM_ID,
        ROBOT_API_ID_LOCO_GET_FSM_MODE,
        ROBOT_API_ID_LOCO_GET_STAND_HEIGHT,
        ROBOT_API_ID_LOCO_GET_SWING_HEIGHT,
        ROBOT_API_ID_LOCO_SET_FSM_ID,
        ROBOT_API_ID_LOCO_SET_ARM_TASK,
        ROBOT_API_ID_LOCO_SET_BALANCE_MODE,
        ROBOT_API_ID_LOCO_SET_STAND_HEIGHT,
        ROBOT_API_ID_LOCO_SET_SWING_HEIGHT,
        ROBOT_API_ID_LOCO_SET_VELOCITY,
    )
    from unitree_sdk2py.rpc.server import Server

    ChannelFactoryInitialize(domain, interface)
    started = []

    sport = Server("sport")
    sport._SetApiVersion(LOCO_API_VERSION)
    sport_state = bridge_state["sport"]

    def sport_get_fsm_id(_parameter: str):
        value, simulator_readback = _read_simulator_loco_value("get_fsm_id", "fsm_id", sport_state["fsm_id"])
        sport_state["fsm_id"] = int(value)
        sport_state["last_simulator_readback"] = simulator_readback
        return 0, json.dumps({"data": sport_state["fsm_id"], "simulator_readback": simulator_readback})

    def sport_get_fsm_mode(_parameter: str):
        value, simulator_readback = _read_simulator_loco_value("get_fsm_mode", "fsm_mode", sport_state["fsm_mode"])
        sport_state["fsm_mode"] = str(value)
        sport_state["last_simulator_readback"] = simulator_readback
        return 0, json.dumps({"data": sport_state["fsm_mode"], "simulator_readback": simulator_readback})

    def sport_get_balance_mode(_parameter: str):
        value, simulator_readback = _read_simulator_loco_value(
            "get_balance_mode", "balance_mode", sport_state["balance_mode"]
        )
        sport_state["balance_mode"] = int(value)
        sport_state["last_simulator_readback"] = simulator_readback
        return 0, json.dumps({"data": sport_state["balance_mode"], "simulator_readback": simulator_readback})

    def sport_get_swing_height(_parameter: str):
        value, simulator_readback = _read_simulator_loco_value(
            "get_swing_height", "swing_height", sport_state["swing_height"]
        )
        sport_state["swing_height"] = float(value)
        sport_state["last_simulator_readback"] = simulator_readback
        return 0, json.dumps({"data": sport_state["swing_height"], "simulator_readback": simulator_readback})

    def sport_get_stand_height(_parameter: str):
        value, simulator_readback = _read_simulator_loco_value(
            "get_stand_height", "stand_height", sport_state["stand_height"]
        )
        sport_state["stand_height"] = float(value)
        sport_state["last_simulator_readback"] = simulator_readback
        return 0, json.dumps({"data": sport_state["stand_height"], "simulator_readback": simulator_readback})

    def sport_get_phase(_parameter: str):
        value, simulator_readback = _read_simulator_loco_value("get_phase", "phase", sport_state["phase"])
        sport_state["phase"] = list(value) if isinstance(value, list) else sport_state["phase"]
        sport_state["last_simulator_readback"] = simulator_readback
        return 0, json.dumps({"data": sport_state["phase"], "simulator_readback": simulator_readback})

    def sport_set_fsm_id(parameter: str):
        payload = _safe_json_loads(parameter)
        sport_state["fsm_id"] = int(payload.get("data", sport_state["fsm_id"]))
        sport_state["fsm_mode"] = _sport_fsm_mode(sport_state["fsm_id"])
        simulator_forward = _forward_simulator_command(
            {
                "command": "loco",
                "action": "set_fsm_id",
                "fsm_id": sport_state["fsm_id"],
                "mode": sport_state["fsm_mode"],
            }
        )
        sport_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps({"data": sport_state["fsm_id"], "simulator_forward": simulator_forward})

    def sport_set_balance_mode(parameter: str):
        payload = _safe_json_loads(parameter)
        sport_state["balance_mode"] = int(payload.get("data", sport_state["balance_mode"]))
        simulator_forward = _forward_simulator_command(
            {"command": "loco", "action": "set_balance_mode", "balance_mode": sport_state["balance_mode"]}
        )
        sport_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps({"data": sport_state["balance_mode"], "simulator_forward": simulator_forward})

    def sport_set_swing_height(parameter: str):
        payload = _safe_json_loads(parameter)
        sport_state["swing_height"] = float(payload.get("data", sport_state["swing_height"]))
        simulator_forward = _forward_simulator_command(
            {"command": "loco", "action": "set_swing_height", "swing_height": sport_state["swing_height"]}
        )
        sport_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps({"data": sport_state["swing_height"], "simulator_forward": simulator_forward})

    def sport_set_stand_height(parameter: str):
        payload = _safe_json_loads(parameter)
        sport_state["stand_height"] = float(payload.get("data", sport_state["stand_height"]))
        if sport_state["stand_height"] >= 4294967295:
            simulator_payload = {"command": "loco", "action": "high_stand", "stand_height": sport_state["stand_height"]}
        elif sport_state["stand_height"] <= 0:
            simulator_payload = {"command": "loco", "action": "low_stand", "stand_height": sport_state["stand_height"]}
        else:
            simulator_payload = {"command": "loco", "action": "set_stand_height", "stand_height": sport_state["stand_height"]}
        simulator_forward = _forward_simulator_command(simulator_payload)
        sport_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps({"data": sport_state["stand_height"], "simulator_forward": simulator_forward})

    def sport_set_velocity(parameter: str):
        payload = _safe_json_loads(parameter)
        velocity = payload.get("velocity", sport_state["velocity"])
        sport_state["velocity"] = [float(value) for value in velocity[:3]]
        sport_state["duration"] = float(payload.get("duration", 0.0))
        simulator_forward = _forward_simulator_command(
            {
                "command": "loco",
                "action": "set_velocity",
                "velocity": sport_state["velocity"],
                "duration": sport_state["duration"],
            }
        )
        sport_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps(
            {
                "velocity": sport_state["velocity"],
                "duration": sport_state["duration"],
                "simulator_forward": simulator_forward,
            }
        )

    def sport_set_arm_task(parameter: str):
        payload = _safe_json_loads(parameter)
        task_id = int(payload.get("data", 0))
        sport_state["arm_task_id"] = task_id
        action = _sport_arm_task_action(task_id)
        simulator_payload = {"command": "loco", "action": action, "task_id": task_id}
        pose = _sport_arm_task_pose(task_id)
        if pose:
            simulator_payload["pose"] = pose
        simulator_forward = _forward_simulator_command(simulator_payload)
        sport_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps({"data": task_id, "action": action, "pose": pose, "simulator_forward": simulator_forward})

    def sport_set_speed_mode(parameter: str):
        payload = _safe_json_loads(parameter)
        sport_state["speed_mode"] = int(payload.get("data", sport_state["speed_mode"]))
        simulator_forward = _forward_simulator_command(
            {"command": "loco", "action": "set_speed_mode", "speed_mode": sport_state["speed_mode"]}
        )
        sport_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps({"data": sport_state["speed_mode"], "loco": sport_state, "simulator_forward": simulator_forward})

    def sport_switch_move_mode(parameter: str):
        payload = _safe_json_loads(parameter)
        sport_state["continuous_move"] = bool(payload.get("continuous_move", payload.get("data", False)))
        simulator_forward = _forward_simulator_command(
            {"command": "loco", "action": "switch_move_mode", "continuous_move": sport_state["continuous_move"]}
        )
        sport_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps({"data": sport_state["continuous_move"], "loco": sport_state, "simulator_forward": simulator_forward})

    def sport_switch_to_user_ctrl(_parameter: str):
        sport_state["control_owner"] = "user"
        simulator_forward = _forward_simulator_command({"command": "loco", "action": "switch_to_user_ctrl"})
        sport_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps({"data": sport_state["control_owner"], "loco": sport_state, "simulator_forward": simulator_forward})

    def sport_switch_to_internal_ctrl(parameter: str):
        payload = _safe_json_loads(parameter)
        sport_state["control_owner"] = "internal"
        sport_state["internal_mode"] = int(payload.get("internal_mode", payload.get("data", 0)))
        simulator_forward = _forward_simulator_command(
            {"command": "loco", "action": "switch_to_internal_ctrl", "internal_mode": sport_state["internal_mode"]}
        )
        sport_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps({"data": sport_state["internal_mode"], "loco": sport_state, "simulator_forward": simulator_forward})

    for api_id, handler in [
        (ROBOT_API_ID_LOCO_GET_FSM_ID, sport_get_fsm_id),
        (ROBOT_API_ID_LOCO_GET_FSM_MODE, sport_get_fsm_mode),
        (ROBOT_API_ID_LOCO_GET_BALANCE_MODE, sport_get_balance_mode),
        (ROBOT_API_ID_LOCO_GET_SWING_HEIGHT, sport_get_swing_height),
        (ROBOT_API_ID_LOCO_GET_STAND_HEIGHT, sport_get_stand_height),
        (ROBOT_API_ID_LOCO_GET_PHASE, sport_get_phase),
        (ROBOT_API_ID_LOCO_SET_FSM_ID, sport_set_fsm_id),
        (ROBOT_API_ID_LOCO_SET_BALANCE_MODE, sport_set_balance_mode),
        (ROBOT_API_ID_LOCO_SET_SWING_HEIGHT, sport_set_swing_height),
        (ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, sport_set_stand_height),
        (ROBOT_API_ID_LOCO_SET_VELOCITY, sport_set_velocity),
        (ROBOT_API_ID_LOCO_SET_ARM_TASK, sport_set_arm_task),
        (ROBOT_API_ID_LOCO_SET_SPEED_MODE, sport_set_speed_mode),
        (ROBOT_API_ID_LOCO_SWITCH_MOVE_MODE, sport_switch_move_mode),
        (ROBOT_API_ID_LOCO_SWITCH_TO_USER_CTRL, sport_switch_to_user_ctrl),
        (ROBOT_API_ID_LOCO_SWITCH_TO_INTERNAL_CTRL, sport_switch_to_internal_ctrl),
    ]:
        sport._RegistHandler(api_id, handler, False)
    sport.Start(False)
    started.append("sport")

    agv = Server("agv")
    agv._SetApiVersion("1.0.0.1")
    agv_state = bridge_state["agv"]

    def agv_move(parameter: str):
        payload = _safe_json_loads(parameter)
        agv_state["velocity"] = [
            float(payload.get("vx", 0.0)),
            float(payload.get("vy", 0.0)),
            float(payload.get("vyaw", 0.0)),
        ]
        simulator_forward = _forward_simulator_command(
            {
                "command": "loco",
                "action": "set_velocity",
                "velocity": [agv_state["velocity"][0], 0.0, agv_state["velocity"][2]],
                "duration": float(payload.get("duration", 1.0)),
                "agv": {
                    "service": "agv",
                    "simulated": True,
                    "requested_velocity": agv_state["velocity"],
                    "ignored_lateral_velocity": agv_state["velocity"][1],
                },
            }
        )
        agv_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps({"velocity": agv_state["velocity"], "simulator_forward": simulator_forward})

    def agv_height_adjust(parameter: str):
        payload = _safe_json_loads(parameter)
        agv_state["height_velocity"] = float(payload.get("data", 0.0))
        simulator_forward = _bridge_state_only_simulator_result(
            {
                "command": "agv",
                "action": "height_adjust",
                "service": "agv",
                "simulated": True,
                "height_velocity": agv_state["height_velocity"],
            },
            "The local G1 MuJoCo harness records AGV height intent in bridge state; it has no height actuator endpoint yet.",
        )
        agv_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps({"data": agv_state["height_velocity"], "simulator_forward": simulator_forward})

    agv._RegistHandler(1001, agv_move, False)
    agv._RegistHandler(1002, agv_height_adjust, False)
    agv.Start(False)
    started.append("agv")

    arm = Server("arm")
    arm._SetApiVersion(ARM_ACTION_API_VERSION)
    arm_state = bridge_state["arm"]

    def arm_execute_action(parameter: str):
        payload = _safe_json_loads(parameter)
        action_id = int(payload.get("data", 0))
        pose = G1_ARM_POSES_BY_ACTION_ID.get(action_id)
        arm_state["last_action_id"] = action_id
        arm_state["last_pose"] = pose
        arm_state["action_count"] = int(arm_state.get("action_count", 0)) + 1
        if pose:
            simulator_forward = _forward_simulator_command({"command": "pose", "pose": pose, "unitree_action_id": action_id})
        else:
            simulator_forward = _bridge_state_only_simulator_result(
                {"command": "arm_action", "action_id": action_id},
                f"Unknown G1 arm action id {action_id}; bridge recorded intent only.",
            )
        arm_state["last_simulator_forward"] = simulator_forward
        return 0, json.dumps({"data": action_id, "pose": pose, "simulator_forward": simulator_forward})

    def arm_get_action_list(_parameter: str):
        actions = [
            {"name": name, "id": action_id, "simulated": action_id in G1_ARM_POSES_BY_ACTION_ID}
            for name, action_id in sorted(G1_ARM_ACTIONS.items(), key=lambda item: item[1])
        ]
        return 0, json.dumps(actions)

    arm._RegistHandler(ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION, arm_execute_action, False)
    arm._RegistHandler(ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST, arm_get_action_list, False)
    arm.Start(False)
    started.append("arm")

    return started


def _call_unitree_rpc_bridge_clients(calls: list[dict], timeout: float) -> None:
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
    from unitree_sdk2py.g1.loco.g1_loco_api import (
        LOCO_API_VERSION,
        ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
        ROBOT_API_ID_LOCO_GET_FSM_ID,
        ROBOT_API_ID_LOCO_GET_FSM_MODE,
        ROBOT_API_ID_LOCO_GET_STAND_HEIGHT,
        ROBOT_API_ID_LOCO_GET_SWING_HEIGHT,
        ROBOT_API_ID_LOCO_SET_ARM_TASK,
        ROBOT_API_ID_LOCO_SET_BALANCE_MODE,
        ROBOT_API_ID_LOCO_SET_FSM_ID,
        ROBOT_API_ID_LOCO_SET_SWING_HEIGHT,
        ROBOT_API_ID_LOCO_SET_VELOCITY,
    )
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.rpc.client import Client

    loco = LocoClient()
    loco.SetTimeout(timeout)
    loco.Init()
    _record_rpc_call(calls, "sport.GetFsmId", loco.GetFsmId)
    _record_rpc_call(calls, "sport.SetStandHeight", lambda: loco.SetStandHeight(0.73))
    _record_rpc_call(calls, "sport.SetVelocity", lambda: loco.SetVelocity(0.0, 0.0, 0.0, 0.1))
    _record_rpc_call(calls, "sport.StopMove", loco.StopMove)
    _record_rpc_call(calls, "sport.Damp", loco.Damp)
    _record_rpc_call(calls, "sport.WaveHand", loco.WaveHand)
    _record_rpc_call(calls, "sport.ShakeHand", loco.ShakeHand)

    sport_client = Client("sport", False)
    sport_client.SetTimeout(timeout)
    sport_client._SetApiVerson(LOCO_API_VERSION)
    for api_id in [
        ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
        ROBOT_API_ID_LOCO_GET_FSM_ID,
        ROBOT_API_ID_LOCO_GET_FSM_MODE,
        ROBOT_API_ID_LOCO_GET_PHASE,
        ROBOT_API_ID_LOCO_GET_STAND_HEIGHT,
        ROBOT_API_ID_LOCO_GET_SWING_HEIGHT,
        ROBOT_API_ID_LOCO_SET_ARM_TASK,
        ROBOT_API_ID_LOCO_SET_BALANCE_MODE,
        ROBOT_API_ID_LOCO_SET_FSM_ID,
        ROBOT_API_ID_LOCO_SET_SPEED_MODE,
        ROBOT_API_ID_LOCO_SET_SWING_HEIGHT,
        ROBOT_API_ID_LOCO_SET_VELOCITY,
        ROBOT_API_ID_LOCO_SWITCH_MOVE_MODE,
        ROBOT_API_ID_LOCO_SWITCH_TO_USER_CTRL,
        ROBOT_API_ID_LOCO_SWITCH_TO_INTERNAL_CTRL,
    ]:
        sport_client._RegistApi(api_id, 0)
    _record_rpc_call(
        calls,
        "sport.RawGetFsmIdDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, "{}"),
    )
    _record_rpc_call(
        calls,
        "sport.RawGetFsmModeDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_GET_FSM_MODE, "{}"),
    )
    _record_rpc_call(
        calls,
        "sport.RawGetBalanceModeDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_GET_BALANCE_MODE, "{}"),
    )
    _record_rpc_call(
        calls,
        "sport.RawGetSwingHeightDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_GET_SWING_HEIGHT, "{}"),
    )
    _record_rpc_call(
        calls,
        "sport.RawGetStandHeightDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_GET_STAND_HEIGHT, "{}"),
    )
    _record_rpc_call(
        calls,
        "sport.RawGetPhaseDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_GET_PHASE, "{}"),
    )
    _record_rpc_call(
        calls,
        "sport.RawSetFsmIdDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_SET_FSM_ID, json.dumps({"data": 1})),
    )
    _record_rpc_call(
        calls,
        "sport.RawSetBalanceModeDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_SET_BALANCE_MODE, json.dumps({"data": 0})),
    )
    _record_rpc_call(
        calls,
        "sport.RawSetSwingHeightDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_SET_SWING_HEIGHT, json.dumps({"data": 0.08})),
    )
    _record_rpc_call(
        calls,
        "sport.RawSetVelocityDebug",
        lambda: sport_client._Call(
            ROBOT_API_ID_LOCO_SET_VELOCITY,
            json.dumps({"velocity": [0.0, 0.0, 0.0], "duration": 0.1}),
        ),
    )
    _record_rpc_call(
        calls,
        "sport.RawSetArmTaskDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_SET_ARM_TASK, json.dumps({"data": 0})),
    )
    _record_rpc_call(
        calls,
        "sport.RawSwitchMoveModeDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_SWITCH_MOVE_MODE, json.dumps({"continuous_move": True})),
    )
    _record_rpc_call(
        calls,
        "sport.RawSetSpeedModeDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_SET_SPEED_MODE, json.dumps({"data": 2})),
    )
    _record_rpc_call(
        calls,
        "sport.RawSwitchToUserCtrlDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_SWITCH_TO_USER_CTRL, "{}"),
    )
    _record_rpc_call(
        calls,
        "sport.RawSwitchToInternalCtrlDebug",
        lambda: sport_client._Call(ROBOT_API_ID_LOCO_SWITCH_TO_INTERNAL_CTRL, json.dumps({"internal_mode": 2})),
    )

    agv_client = Client("agv", False)
    agv_client.SetTimeout(timeout)
    agv_client._SetApiVerson("1.0.0.1")
    agv_client._RegistApi(1001, 0)
    agv_client._RegistApi(1002, 0)
    _record_rpc_call(calls, "agv.Move", lambda: agv_client._Call(1001, json.dumps({"vx": 0.0, "vy": 0.0, "vyaw": 0.0})))
    _record_rpc_call(calls, "agv.HeightAdjust", lambda: agv_client._Call(1002, json.dumps({"data": 0.0})))

    arm_client = G1ArmActionClient()
    arm_client.SetTimeout(timeout)
    arm_client.Init()
    _record_rpc_call(calls, "arm.GetActionList", arm_client.GetActionList)
    _record_rpc_call(calls, "arm.ExecuteAction", lambda: arm_client.ExecuteAction(action_map["right hand up"]))


def _safe_json_loads(text: str) -> dict:
    try:
        value = json.loads(text or "{}")
        return value if isinstance(value, dict) else {}
    except Exception:
        return {}


def _sport_fsm_mode(fsm_id: int) -> str:
    return {
        0: "zero_torque",
        1: "damp",
        2: "squat",
        3: "sit",
        4: "stand_up",
        500: "start",
        702: "lie_to_stand_up",
        706: "squat_transition",
    }.get(int(fsm_id), f"fsm_{int(fsm_id)}")


def _sport_arm_task_action(task_id: int) -> str:
    return {
        0: "wave_hand",
        1: "wave_hand",
        2: "shake_hand",
        3: "shake_hand",
    }.get(int(task_id), "set_arm_task")


def _sport_arm_task_pose(task_id: int) -> str | None:
    return G1_ARM_POSES_BY_ACTION_ID.get(int(task_id)) or {
        0: "raise_right_hand",
        1: "raise_right_hand",
        2: "shake_hand",
        3: "shake_hand",
    }.get(int(task_id))


def _simulator_game_control_url() -> str:
    return (
        os.environ.get("CYBER_SIMULATOR_GAME_CONTROL_URL")
        or os.environ.get("CYBER_G1_GAME_CONTROL_URL")
        or DEFAULT_SIMULATOR_GAME_CONTROL_URL
    ).rstrip("/")


def _forward_simulator_command(payload: dict) -> dict:
    """Best-effort adapter from official SDK RPC handlers to GameControl HTTP."""

    simulator_url = _simulator_game_control_url()
    timeout = float(os.environ.get("CYBER_UNITREE_RPC_BRIDGE_SIM_TIMEOUT", "0.35"))
    request = Request(
        f"{simulator_url}/command",
        data=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
        headers={"content-type": "application/json"},
        method="POST",
    )
    try:
        with urlopen(request, timeout=timeout) as response:
            body = response.read()
        try:
            value = json.loads(body.decode("utf-8"))
        except Exception:
            value = {"raw": body.decode("utf-8", errors="replace")[-2000:]}
        ok = not (isinstance(value, dict) and value.get("ok") is False)
        return {
            "ok": ok,
            "provider": "cybernetic_game_control_http",
            "url": simulator_url,
            "payload": payload,
            "response": value,
            "note": None if ok else "Simulator HTTP endpoint responded, but the simulator reported ok=false.",
        }
    except (HTTPError, URLError, TimeoutError, OSError) as exc:
        return _bridge_state_only_simulator_result(payload, str(exc), simulator_url=simulator_url)


def _bridge_state_only_simulator_result(payload: dict, reason: str, *, simulator_url: str | None = None) -> dict:
    return {
        "ok": False,
        "provider": "bridge_state_only",
        "url": simulator_url or _simulator_game_control_url(),
        "payload": payload,
        "error": reason,
        "note": "SDK RPC returned RPC_OK, but the local simulator HTTP bridge was not updated.",
    }


def _read_simulator_loco_value(action: str, response_key: str, fallback) -> tuple[object, dict]:
    simulator_readback = _forward_simulator_command({"command": "loco", "action": action})
    if simulator_readback.get("ok"):
        response = simulator_readback.get("response")
        if isinstance(response, dict):
            if response_key in response and response[response_key] is not None:
                return response[response_key], simulator_readback
            loco = response.get("loco")
            if isinstance(loco, dict) and response_key in loco and loco[response_key] is not None:
                return loco[response_key], simulator_readback
    return fallback, simulator_readback


def _record_rpc_call(calls: list[dict], name: str, fn) -> None:
    started = time.monotonic()
    entry = {"name": name}
    try:
        value = fn()
        entry["return"] = _jsonable_return(value)
        code = _loco_return_code(value)
        if code is not None:
            entry["rpc_code"] = code
            entry["rpc_status"] = _unitree_rpc_status(code)
        entry["ok"] = _loco_call_ok(value)
    except Exception as exc:
        entry["ok"] = False
        entry["error"] = str(exc)
    entry["elapsed_seconds"] = round(time.monotonic() - started, 4)
    calls.append(entry)


def _call_unitree_rpc_bridge_command(
    calls: list[dict],
    service: str,
    method: str,
    params: dict,
    timeout: float,
) -> dict:
    from unitree_sdk2py.g1.arm.g1_arm_action_api import (
        ARM_ACTION_API_VERSION,
        ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION,
        ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST,
    )
    from unitree_sdk2py.g1.loco.g1_loco_api import (
        LOCO_API_VERSION,
        ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
        ROBOT_API_ID_LOCO_GET_FSM_ID,
        ROBOT_API_ID_LOCO_GET_FSM_MODE,
        ROBOT_API_ID_LOCO_GET_STAND_HEIGHT,
        ROBOT_API_ID_LOCO_GET_SWING_HEIGHT,
        ROBOT_API_ID_LOCO_SET_ARM_TASK,
        ROBOT_API_ID_LOCO_SET_BALANCE_MODE,
        ROBOT_API_ID_LOCO_SET_FSM_ID,
        ROBOT_API_ID_LOCO_SET_STAND_HEIGHT,
        ROBOT_API_ID_LOCO_SET_SWING_HEIGHT,
        ROBOT_API_ID_LOCO_SET_VELOCITY,
    )
    from unitree_sdk2py.rpc.client import Client

    normalized_service = (service or "sport").strip().lower()
    normalized_method = (method or "").strip().lower().replace("-", "_")

    sport_methods = {
        "get_fsm_id": ("sport.GetFsmId", ROBOT_API_ID_LOCO_GET_FSM_ID, lambda: {}),
        "get_fsm_mode": ("sport.GetFsmMode", ROBOT_API_ID_LOCO_GET_FSM_MODE, lambda: {}),
        "get_balance_mode": ("sport.GetBalanceMode", ROBOT_API_ID_LOCO_GET_BALANCE_MODE, lambda: {}),
        "get_swing_height": ("sport.GetSwingHeight", ROBOT_API_ID_LOCO_GET_SWING_HEIGHT, lambda: {}),
        "get_stand_height": ("sport.GetStandHeight", ROBOT_API_ID_LOCO_GET_STAND_HEIGHT, lambda: {}),
        "get_phase": ("sport.GetPhase", ROBOT_API_ID_LOCO_GET_PHASE, lambda: {}),
        "set_fsm_id": (
            "sport.SetFsmId",
            ROBOT_API_ID_LOCO_SET_FSM_ID,
            lambda: {"data": int(_first_param(params, "fsm_id", "data", default=500))},
        ),
        "damp": ("sport.Damp", ROBOT_API_ID_LOCO_SET_FSM_ID, lambda: {"data": 1}),
        "start": ("sport.Start", ROBOT_API_ID_LOCO_SET_FSM_ID, lambda: {"data": 500}),
        "zero_torque": ("sport.ZeroTorque", ROBOT_API_ID_LOCO_SET_FSM_ID, lambda: {"data": 0}),
        "sit": ("sport.Sit", ROBOT_API_ID_LOCO_SET_FSM_ID, lambda: {"data": 3}),
        "stand_up": ("sport.StandUp", ROBOT_API_ID_LOCO_SET_FSM_ID, lambda: {"data": 4}),
        "set_balance_mode": (
            "sport.SetBalanceMode",
            ROBOT_API_ID_LOCO_SET_BALANCE_MODE,
            lambda: {"data": int(_first_param(params, "balance_mode", "data", default=0))},
        ),
        "set_swing_height": (
            "sport.SetSwingHeight",
            ROBOT_API_ID_LOCO_SET_SWING_HEIGHT,
            lambda: {"data": float(_first_param(params, "swing_height", "data", default=0.08))},
        ),
        "set_stand_height": (
            "sport.SetStandHeight",
            ROBOT_API_ID_LOCO_SET_STAND_HEIGHT,
            lambda: {"data": float(_first_param(params, "stand_height", "data", default=0.72))},
        ),
        "set_velocity": (
            "sport.SetVelocity",
            ROBOT_API_ID_LOCO_SET_VELOCITY,
            lambda: _velocity_payload(params),
        ),
        "move": (
            "sport.SetVelocity",
            ROBOT_API_ID_LOCO_SET_VELOCITY,
            lambda: _velocity_payload(params),
        ),
        "stop_move": (
            "sport.StopMove",
            ROBOT_API_ID_LOCO_SET_VELOCITY,
            lambda: {"velocity": [0.0, 0.0, 0.0], "duration": 0.0},
        ),
        "set_arm_task": (
            "sport.SetArmTask",
            ROBOT_API_ID_LOCO_SET_ARM_TASK,
            lambda: {"data": int(_first_param(params, "task_id", "data", default=0))},
        ),
        "set_speed_mode": (
            "sport.SetSpeedMode",
            ROBOT_API_ID_LOCO_SET_SPEED_MODE,
            lambda: {"data": int(_first_param(params, "speed_mode", "data", default=0))},
        ),
        "switch_move_mode": (
            "sport.SwitchMoveMode",
            ROBOT_API_ID_LOCO_SWITCH_MOVE_MODE,
            lambda: {"continuous_move": bool(_first_param(params, "continuous_move", "flag", "data", default=False))},
        ),
        "switch_to_user_ctrl": ("sport.SwitchToUserCtrl", ROBOT_API_ID_LOCO_SWITCH_TO_USER_CTRL, lambda: {}),
        "switch_to_internal_ctrl": (
            "sport.SwitchToInternalCtrl",
            ROBOT_API_ID_LOCO_SWITCH_TO_INTERNAL_CTRL,
            lambda: {"internal_mode": int(_first_param(params, "internal_mode", "mode", "data", default=0))},
        ),
        "high_stand": ("sport.HighStand", ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, lambda: {"data": 4294967295}),
        "low_stand": ("sport.LowStand", ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, lambda: {"data": 0}),
        "wave_hand": (
            "sport.WaveHand",
            ROBOT_API_ID_LOCO_SET_ARM_TASK,
            lambda: {"data": 1 if bool(_first_param(params, "turn", "turn_flag", default=False)) else 0},
        ),
        "shake_hand": (
            "sport.ShakeHand",
            ROBOT_API_ID_LOCO_SET_ARM_TASK,
            lambda: {"data": 2 if int(_first_param(params, "stage", default=1)) == 0 else 3},
        ),
    }
    agv_methods = {
        "move": (
            "agv.Move",
            1001,
            lambda: {
                "vx": float(_first_param(params, "vx", "x", default=0.0)),
                "vy": float(_first_param(params, "vy", "y", default=0.0)),
                "vyaw": float(_first_param(params, "vyaw", "omega", "yaw", default=0.0)),
            },
        ),
        "height_adjust": (
            "agv.HeightAdjust",
            1002,
            lambda: {"data": float(_first_param(params, "vz", "z", "data", default=0.0))},
        ),
    }
    arm_methods = {
        "execute_action": (
            "arm.ExecuteAction",
            ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION,
            lambda: {"data": int(_first_param(params, "action_id", "data", default=23))},
        ),
        "right_hand_up": ("arm.ExecuteAction", ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION, lambda: {"data": 23}),
        "raise_right_hand": ("arm.ExecuteAction", ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION, lambda: {"data": 23}),
        "release_arm": ("arm.ExecuteAction", ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION, lambda: {"data": 99}),
        "get_action_list": ("arm.GetActionList", ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST, lambda: {}),
    }

    if normalized_service == "sport":
        methods = sport_methods
        api_version = LOCO_API_VERSION
    elif normalized_service == "agv":
        methods = agv_methods
        api_version = "1.0.0.1"
    elif normalized_service == "arm":
        methods = arm_methods
        api_version = ARM_ACTION_API_VERSION
    else:
        raise ValueError("unsupported Unitree RPC bridge service; expected sport, agv, or arm")
    if normalized_method not in methods:
        raise ValueError(
            f"unsupported {normalized_service} bridge method {method!r}; "
            f"known methods: {', '.join(sorted(methods))}"
        )

    name, api_id, make_payload = methods[normalized_method]
    payload = make_payload()
    client = Client(normalized_service, False)
    client.SetTimeout(timeout)
    client._SetApiVerson(api_version)
    client._RegistApi(api_id, 0)
    _record_rpc_call(calls, name, lambda: client._Call(api_id, json.dumps(payload)))
    return {
        "service": normalized_service,
        "method": normalized_method,
        "name": name,
        "api_id": api_id,
        "api_version": api_version,
        "payload": payload,
    }


def _first_param(params: dict, *names: str, default=None):
    for name in names:
        if name in params and params[name] is not None:
            return params[name]
    return default


def _velocity_payload(params: dict) -> dict:
    if isinstance(params.get("velocity"), list):
        velocity = [float(value) for value in params["velocity"][:3]]
    else:
        velocity = [
            float(_first_param(params, "vx", "x", default=0.0)),
            float(_first_param(params, "vy", "y", default=0.0)),
            float(_first_param(params, "omega", "vyaw", "yaw", default=0.0)),
        ]
    while len(velocity) < 3:
        velocity.append(0.0)
    return {
        "velocity": velocity[:3],
        "duration": float(_first_param(params, "duration", "timeout", default=0.0)),
    }


def _channel_publication_matched_count(channel) -> int:
    try:
        writer = getattr(channel, "_Channel__writer")
        return int(getattr(writer, "_Channel__Writer__publication_matched_count", 0))
    except Exception:
        return 0


def _loco_call_ok(value) -> bool:
    if isinstance(value, tuple) and value:
        return value[0] == 0
    if isinstance(value, int):
        return value == 0
    return value is None


def _loco_return_code(value) -> int | None:
    if isinstance(value, tuple) and value and isinstance(value[0], int):
        return value[0]
    if isinstance(value, int):
        return value
    return None


def _unitree_rpc_status(code: int) -> dict:
    statuses = {
        0: {
            "name": "RPC_OK",
            "meaning": "Unitree RPC call succeeded.",
        },
        3001: {
            "name": "RPC_ERR_UNKNOWN",
            "meaning": "Unitree RPC client reported an unknown failure.",
        },
        3102: {
            "name": "RPC_ERR_CLIENT_SEND",
            "meaning": "The Unitree RPC request could not be written to DDS. In unitree_sdk2_python this happens when the request writer has no matched reader before the timeout or the DDS write fails.",
        },
        3103: {
            "name": "RPC_ERR_CLIENT_API_NOT_REG",
            "meaning": "The API ID was not registered on the client.",
        },
        3104: {
            "name": "RPC_ERR_CLIENT_API_TIMEOUT",
            "meaning": "The request was written, but no matching response arrived before the timeout.",
        },
        3105: {
            "name": "RPC_ERR_CLIENT_API_NOT_MATCH",
            "meaning": "The response API ID did not match the request API ID.",
        },
        3106: {
            "name": "RPC_ERR_CLIENT_API_DATA",
            "meaning": "The client could not decode or validate returned API data.",
        },
        3107: {
            "name": "RPC_ERR_CLIENT_LEASE_INVALID",
            "meaning": "The client lease is invalid.",
        },
        3201: {
            "name": "RPC_ERR_SERVER_SEND",
            "meaning": "The server could not publish the response.",
        },
        3202: {
            "name": "RPC_ERR_SERVER_INTERNAL",
            "meaning": "The server reported an internal failure.",
        },
        3203: {
            "name": "RPC_ERR_SERVER_API_NOT_IMPL",
            "meaning": "The server does not implement the requested API.",
        },
        3204: {
            "name": "RPC_ERR_SERVER_API_PARAMETER",
            "meaning": "The server rejected the request parameters.",
        },
        3205: {
            "name": "RPC_ERR_SERVER_LEASE_DENIED",
            "meaning": "The server denied the requested lease.",
        },
    }
    return statuses.get(
        code,
        {
            "name": "RPC_ERR_UNMAPPED",
            "meaning": "Unitree RPC returned a code that Cybernetic IDE has not mapped yet.",
        },
    )


def _diagnose_loco_rpc_calls(calls: list[dict]) -> dict:
    codes = [call.get("rpc_code") for call in calls if isinstance(call.get("rpc_code"), int)]
    if 3102 in codes:
        return {
            "likely_cause": "no_matched_sport_request_reader_or_dds_write_failure",
            "evidence": "Unitree LocoClient returned RPC_ERR_CLIENT_SEND before any response wait; unitree_sdk2_python only returns this when Channel.Write() fails.",
            "next_step": "Treat the official sport RPC service as absent on this managed unitree_mujoco peer; inspect DDS discovery/network interface or add a real sport service bridge before routing LocoClient to DDS.",
        }
    if 3104 in codes:
        return {
            "likely_cause": "sport_request_written_but_no_response",
            "evidence": "Unitree LocoClient returned RPC_ERR_CLIENT_API_TIMEOUT, which means the request was sent but no matching response arrived.",
            "next_step": "Inspect rt/api/sport/response discovery and whether the peer implements the requested G1 Loco API.",
        }
    return {
        "likely_cause": "unknown_sport_rpc_failure",
        "evidence": "The official LocoClient initialized but no sport RPC call returned success.",
        "next_step": "Inspect each call rpc_status and compare the managed peer against Unitree SDK2's sport service expectations.",
    }


def _jsonable_return(value):
    if isinstance(value, tuple):
        return [_jsonable_return(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, (list, dict)):
        return value
    return repr(value)


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
            "rt/api/agv/request",
            "rt/api/agv/response",
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
    elif action == "probe_official_mujoco_dds":
        report["dds_probe"] = probe_official_mujoco_dds_exchange(mujoco_root, domain, interface or None)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action == "probe_official_mujoco_lowcmd":
        report["lowcmd_probe"] = probe_official_mujoco_lowcmd_exchange(mujoco_root, domain, interface or None)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action == "probe_official_mujoco_arm_motion":
        report["arm_motion_probe"] = probe_official_mujoco_arm_motion(mujoco_root, domain, interface or None)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action == "probe_official_mujoco_arm_pose":
        report["arm_pose_probe"] = probe_official_mujoco_arm_pose(mujoco_root, domain, interface or None)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action == "command_official_mujoco_arm_pose":
        report["arm_pose_command"] = command_official_mujoco_arm_pose(domain, interface or None)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action == "probe_official_mujoco_loco_rpc":
        report["loco_rpc_probe"] = probe_official_mujoco_loco_rpc(domain, interface or None)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action == "probe_official_mujoco_rpc_discovery":
        report["rpc_discovery_probe"] = probe_official_mujoco_rpc_discovery(domain, interface or None)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action == "probe_unitree_rpc_bridge_smoke":
        report["rpc_bridge_smoke"] = probe_unitree_rpc_bridge_smoke(domain, interface or None)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action == "probe_unitree_rpc_bridge_client":
        report["rpc_bridge_client"] = probe_unitree_rpc_bridge_client(domain, interface or None)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action == "command_unitree_rpc_bridge":
        report["rpc_bridge_command"] = command_unitree_rpc_bridge_client(domain, interface or None)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action == "read_official_mujoco_lowstate":
        report["lowstate_read"] = read_official_mujoco_lowstate(domain, interface or None)
        report["official_mujoco_peer"] = official_mujoco_plan(mujoco_root, domain, interface or None)
    elif action == "serve_official_mujoco":
        return serve_official_mujoco_peer(mujoco_root, domain, interface or None)
    elif action == "serve_unitree_rpc_bridge":
        return serve_unitree_rpc_bridge(domain, interface or None)
    elif action != "status":
        report["status"] = "error"
        report["error"] = f"Unsupported CYBER_UNITREE_ACTION: {action}"
    if report["sdk2_probe"]["domain_initialized"]:
        report["next_step"] = f"Launch official unitree_mujoco with -r g1 on DDS domain {domain} interface {interface or 'auto'}, then run lowstate/lowcmd pub-sub probes."
    print(json.dumps(report, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
