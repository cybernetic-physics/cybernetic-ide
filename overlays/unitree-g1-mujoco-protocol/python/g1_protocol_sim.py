#!/usr/bin/env python3
import asyncio
import io
import json
import math
import os
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

os.environ.setdefault("MUJOCO_GL", "osmesa")

import msgpack
import mujoco
import numpy as np
import websockets
from PIL import Image


MESSAGE_TYPES = {
    "camera_frame_0": 3,
    "simulation_state": 6,
    "visual_scene": 10,
    "visual_frame": 11,
    "visual_scene_error": 12,
}

NAMED_POSES = {
    "raise_right_hand": {
        "right_shoulder_pitch_joint": -2.2,
        "right_shoulder_roll_joint": -0.2,
        "right_shoulder_yaw_joint": 0.0,
        "right_elbow_joint": 1.0,
        "right_wrist_roll_joint": 0.0,
        "right_wrist_pitch_joint": 0.0,
        "right_wrist_yaw_joint": 0.0,
    },
    "neutral": {},
    # --- Yoga flow poses ---
    # Static held frames: apply_named_pose() sets qpos and leaves the sim paused,
    # so these are visual asanas, not balanced dynamic control. Joint values are
    # clamped to model ranges by apply_named_pose(), so mild over-reach is safe.
    "mountain": {
        "left_elbow_joint": 0.12,
        "right_elbow_joint": 0.12,
    },
    "upward_salute": {
        "left_shoulder_pitch_joint": -2.7,
        "left_shoulder_roll_joint": 0.12,
        "left_elbow_joint": 0.0,
        "right_shoulder_pitch_joint": -2.7,
        "right_shoulder_roll_joint": -0.12,
        "right_elbow_joint": 0.0,
    },
    # forward_fold and chair fold the torso forward over planted feet. The
    # upright-pelvis + joint-targets representation cannot express that (hip
    # flexion swings the legs forward instead), so these poses pitch the
    # floating base or waist pitch and keep the legs vertical by matching hip
    # flexion. Values tuned against
    # packages/g1-yoga-rl/g1_yoga_rl/analyze_pose_stability.py.
    # Sign conventions (verified numerically against both MJCFs): forward is
    # +x; hip_pitch/knee/ankle_pitch all rotate about +y and add, with
    # positive values swinging the segment backward. Hip flexion (thigh
    # forward) is therefore NEGATIVE hip_pitch, and a flat foot requires
    # base_pitch + hip + knee + ankle ~= 0. The original values used positive
    # hips for forward bends, which swung the legs backward instead — one of
    # the reasons these poses toppled under PD hold.
    "forward_fold": {
        # torso pitch 0.87 rad (~50 deg) stays under the fall detector's
        # torso-up-axis 0.5 threshold; the feet sit ~14 cm ahead of the hips
        # (legs world tilt -0.2) so the hips counterweight the folded torso
        "base_pitch": 0.35,
        "waist_pitch_joint": 0.52,
        "left_hip_pitch_joint": -0.55,
        "right_hip_pitch_joint": -0.55,
        "left_knee_joint": 0.12,
        "right_knee_joint": 0.12,
        "left_ankle_pitch_joint": 0.08,
        "right_ankle_pitch_joint": 0.08,
        "left_shoulder_pitch_joint": -1.2,
        "right_shoulder_pitch_joint": -1.2,
        "left_elbow_joint": 0.1,
        "right_elbow_joint": 0.1,
    },
    "chair": {
        "waist_pitch_joint": 0.5,
        "left_hip_pitch_joint": -0.75,
        "right_hip_pitch_joint": -0.75,
        "left_knee_joint": 0.85,
        "right_knee_joint": 0.85,
        "left_ankle_pitch_joint": -0.1,
        "right_ankle_pitch_joint": -0.1,
        "left_shoulder_pitch_joint": -1.7,
        "right_shoulder_pitch_joint": -1.7,
        "left_shoulder_roll_joint": 0.15,
        "right_shoulder_roll_joint": -0.15,
        "left_elbow_joint": 0.15,
        "right_elbow_joint": 0.15,
    },
    # Warrior stances are tuned so both feet reach the floor: front and rear
    # leg vertical reach match within ~1 cm (the old values left the bent
    # leg's foot floating mid-air).
    "warrior_one": {
        "left_hip_pitch_joint": -0.6,
        "left_knee_joint": 0.6,
        "left_ankle_pitch_joint": 0.0,
        "right_hip_pitch_joint": 0.35,
        "right_knee_joint": 0.2,
        "right_ankle_pitch_joint": -0.55,
        "waist_pitch_joint": 0.1,
        "left_shoulder_pitch_joint": -2.4,
        "right_shoulder_pitch_joint": -2.4,
        "left_shoulder_roll_joint": 0.12,
        "right_shoulder_roll_joint": -0.12,
    },
    "warrior_two": {
        "left_hip_roll_joint": 0.55,
        "left_hip_pitch_joint": -0.35,
        "left_knee_joint": 0.35,
        "left_ankle_pitch_joint": 0.0,
        "left_ankle_roll_joint": -0.26,
        "right_hip_roll_joint": -0.55,
        "right_hip_pitch_joint": 0.05,
        "right_knee_joint": 0.1,
        "right_ankle_pitch_joint": -0.15,
        "right_ankle_roll_joint": 0.26,
        "left_shoulder_pitch_joint": 0.0,
        "left_shoulder_roll_joint": 1.4,
        "right_shoulder_pitch_joint": 0.0,
        "right_shoulder_roll_joint": -1.4,
    },
    "goddess": {
        "waist_pitch_joint": 0.2,
        "left_hip_roll_joint": 0.45,
        "right_hip_roll_joint": -0.45,
        "left_hip_pitch_joint": -0.5,
        "right_hip_pitch_joint": -0.5,
        "left_knee_joint": 0.5,
        "right_knee_joint": 0.5,
        "left_ankle_pitch_joint": 0.0,
        "right_ankle_pitch_joint": 0.0,
        "left_ankle_roll_joint": -0.26,
        "right_ankle_roll_joint": 0.26,
        "left_shoulder_pitch_joint": -0.3,
        "left_shoulder_roll_joint": 1.2,
        "left_elbow_joint": 1.5,
        "right_shoulder_pitch_joint": -0.3,
        "right_shoulder_roll_joint": -1.2,
        "right_elbow_joint": 1.5,
    },
    "tree": {
        "left_hip_roll_joint": 0.7,
        "left_hip_pitch_joint": -0.2,
        "left_knee_joint": 2.2,
        "left_shoulder_pitch_joint": -2.7,
        "left_shoulder_roll_joint": 0.15,
        "right_shoulder_pitch_joint": -2.7,
        "right_shoulder_roll_joint": -0.15,
    },
    "namaste": {
        "left_shoulder_pitch_joint": -0.5,
        "left_shoulder_roll_joint": 0.3,
        "left_elbow_joint": 1.6,
        "right_shoulder_pitch_joint": -0.5,
        "right_shoulder_roll_joint": -0.3,
        "right_elbow_joint": 1.6,
    },
}


def env_float(name, default):
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def env_int(name, default):
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError:
        return default


def named(model, obj_type, index, fallback):
    name = mujoco.mj_id2name(model, obj_type, index)
    return name if name else fallback


def as_list(array):
    return np.asarray(array, dtype=float).tolist()


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def query_float(query, name, default=0.0):
    values = query.get(name)
    if not values:
        return default
    try:
        return float(values[0])
    except ValueError:
        return default


class G1MujocoState:
    def __init__(self):
        self.model_path = Path(
            os.environ.get(
                "UNITREE_G1_MODEL_PATH",
                "/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml",
            )
        )
        self.model_revision = os.environ.get(
            "UNITREE_G1_MODEL_REVISION",
            "unitreerobotics/unitree_mujoco@unknown",
        )
        self.robot_name = os.environ.get("UNITREE_G1_ROBOT_NAME", "g1")
        self.frame_hz = env_float("UNITREE_G1_FRAME_HZ", 20.0)
        self.render_hz = env_float("UNITREE_G1_RENDER_HZ", 8.0)
        self.render_width = env_int("UNITREE_G1_RENDER_WIDTH", 640)
        self.render_height = env_int("UNITREE_G1_RENDER_HEIGHT", 480)
        self.lowcmd_watchdog_seconds = env_float("UNITREE_G1_LOWCMD_WATCHDOG_SECONDS", 2.0)
        self.paused = os.environ.get("UNITREE_G1_AUTORUN", "0") != "1"
        self.frame_id = 0
        self.last_step_wall_time = time.monotonic()
        self.last_render_error = None
        self.active_pose = None
        self.loco_state = {
            "fsm_id": 1,
            "fsm_mode": "damp",
            "balance_mode": 0,
            "swing_height": None,
            "stand_height": None,
            "velocity": [0.0, 0.0, 0.0],
            "velocity_until": None,
            "arm_task_id": None,
        }
        self.lowcmd_state = {
            "topic": None,
            "received_at": None,
            "age_seconds": None,
            "expires_at": None,
            "watchdog_seconds": self.lowcmd_watchdog_seconds,
            "active": False,
            "stale": False,
            "motor_cmd_count": 0,
            "mode_pr": 0,
            "mode_machine": None,
            "crc": None,
            "accepted": 0,
            "applied_position_targets": 0,
            "clamped": [],
            "ignored": [],
        }
        self.latest_jpeg = None
        self.latest_jpeg_frame_id = None
        self.latest_jpeg_rendered_at = None
        self.render_seq = 0
        self.lock = threading.RLock()
        self.cache_lock = threading.RLock()
        self.camera_lock = threading.RLock()

        if not self.model_path.exists():
            raise FileNotFoundError(f"Missing Unitree G1 MJCF model: {self.model_path}")

        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = env_float("UNITREE_G1_TIMESTEP", 0.003)
        mujoco.mj_forward(self.model, self.data)

        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.default_camera = {
            "lookat": [0.0, 0.0, 0.72],
            "distance": env_float("UNITREE_G1_CAMERA_DISTANCE", 3.2),
            "azimuth": env_float("UNITREE_G1_CAMERA_AZIMUTH", -130.0),
            "elevation": env_float("UNITREE_G1_CAMERA_ELEVATION", -18.0),
        }
        self.desired_camera = dict(self.default_camera)
        self.reset_camera()
        self.apply_desired_camera_locked()

        self.renderer = mujoco.Renderer(
            self.model,
            height=self.render_height,
            width=self.render_width,
        )

        self._setup_hold_controller()

    def _setup_hold_controller(self):
        """Precompute per-actuator PD gains and joint address maps.

        The G1 uses torque (motor) actuators, so holding a pose against gravity
        means computing joint torques each physics step:
        tau = kp*(target - q) - kd*qd, clamped to each motor's ctrlrange.
        """

        count = self.model.nu
        self.actuator_joint_id = np.zeros(count, dtype=int)
        self.actuator_qpos_adr = np.zeros(count, dtype=int)
        self.actuator_dof_adr = np.zeros(count, dtype=int)
        self.kp = np.zeros(count)
        self.kd = np.zeros(count)
        for index in range(count):
            joint_id = int(self.model.actuator_trnid[index, 0])
            self.actuator_joint_id[index] = joint_id
            self.actuator_qpos_adr[index] = int(self.model.jnt_qposadr[joint_id])
            self.actuator_dof_adr[index] = int(self.model.jnt_dofadr[joint_id])
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index) or ""
            if "ankle" in name:
                kp, kd = 120.0, 5.0
            elif "knee" in name or "hip" in name or "waist" in name:
                kp, kd = 400.0, 15.0
            elif "shoulder" in name or "elbow" in name or "wrist" in name:
                kp, kd = 90.0, 3.0
            else:
                kp, kd = 120.0, 5.0
            self.kp[index] = kp
            self.kd[index] = kd

        self.ctrl_lo = self.model.actuator_ctrlrange[:, 0].astype(float).copy()
        self.ctrl_hi = self.model.actuator_ctrlrange[:, 1].astype(float).copy()
        unlimited = self.ctrl_hi <= self.ctrl_lo
        self.ctrl_lo[unlimited] = -np.inf
        self.ctrl_hi[unlimited] = np.inf

        self.hold_target_qpos = np.zeros(count)
        self.control_mode = None
        self.base_yaw = 0.0

    def reset(self):
        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            self.frame_id = 0
            self.active_pose = None
            self.control_mode = None
            self.base_yaw = 0.0
            self.loco_state.update(
                {
                    "fsm_id": 1,
                    "fsm_mode": "damp",
                    "velocity": [0.0, 0.0, 0.0],
                    "velocity_until": None,
                    "arm_task_id": None,
                }
            )
            self.lowcmd_state = {
                "topic": None,
                "received_at": None,
                "age_seconds": None,
                "expires_at": None,
                "watchdog_seconds": self.lowcmd_watchdog_seconds,
                "active": False,
                "stale": False,
                "motor_cmd_count": 0,
                "mode_pr": 0,
                "mode_machine": None,
                "crc": None,
                "accepted": 0,
                "applied_position_targets": 0,
                "clamped": [],
                "ignored": [],
            }
            self.data.ctrl[:] = 0.0
            self.paused = True
            self.last_step_wall_time = time.monotonic()

    def set_paused(self, value):
        with self.lock:
            self.paused = value
            if not value:
                self.active_pose = None
            self.last_step_wall_time = time.monotonic()

    def step(self, count=1):
        with self.lock:
            for _ in range(max(1, count)):
                self.refresh_lowcmd_watchdog_locked()
                self.apply_hold_control_locked()
                self.apply_loco_velocity_locked(self.model.opt.timestep)
                mujoco.mj_step(self.model, self.data)
                self.frame_id += 1

    def refresh_lowcmd_watchdog_locked(self):
        received_at = self.lowcmd_state.get("received_at")
        if received_at is None:
            self.lowcmd_state["age_seconds"] = None
            self.lowcmd_state["expires_at"] = None
            self.lowcmd_state["watchdog_seconds"] = self.lowcmd_watchdog_seconds
            self.lowcmd_state["active"] = False
            self.lowcmd_state["stale"] = False
            return

        now = time.time()
        age = max(0.0, now - float(received_at))
        self.lowcmd_state["age_seconds"] = age
        self.lowcmd_state["watchdog_seconds"] = self.lowcmd_watchdog_seconds
        if self.lowcmd_watchdog_seconds <= 0:
            self.lowcmd_state["expires_at"] = None
            self.lowcmd_state["active"] = True
            self.lowcmd_state["stale"] = False
            return

        expires_at = float(received_at) + self.lowcmd_watchdog_seconds
        stale = now > expires_at
        if stale and not bool(self.lowcmd_state.get("stale")):
            self.data.ctrl[:] = 0.0
        self.lowcmd_state["expires_at"] = expires_at
        self.lowcmd_state["active"] = not stale
        self.lowcmd_state["stale"] = stale

    def joint_qpos_addr(self, joint_name):
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise KeyError(f"unknown joint: {joint_name}")
        return joint_id, int(self.model.jnt_qposadr[joint_id])

    def actuator_joint_map_locked(self):
        joints = []
        for index in range(self.model.nu):
            joint_id = int(self.actuator_joint_id[index])
            qpos_addr = int(self.actuator_qpos_adr[index])
            dof_addr = int(self.actuator_dof_adr[index])
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}"
            actuator_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index) or f"actuator_{index}"
            limited = bool(self.model.jnt_limited[joint_id])
            minimum, maximum = self.model.jnt_range[joint_id]
            joints.append(
                {
                    "motor_index": index,
                    "joint_id": joint_id,
                    "joint_name": joint_name,
                    "actuator_name": actuator_name,
                    "q": float(self.data.qpos[qpos_addr]),
                    "dq": float(self.data.qvel[dof_addr]),
                    "tau_est": float(self.data.ctrl[index]),
                    "limited": limited,
                    "range": [float(minimum), float(maximum)] if limited else None,
                }
            )
        return joints

    def _target_qpos_locked(self, targets):
        """Full qpos vector for a joint-target dict (reset baseline + targets).

        Supports a "base_pitch" pseudo-key that pitches the floating base
        forward (radians about +y). Poses like a standing forward fold are not
        expressible with an upright pelvis: hip flexion alone swings the legs
        forward instead of folding the torso over them.

        Leaves self.data.qpos as it found it; the caller decides what to do with
        the returned vector. Must run with self.lock held.
        """

        saved = self.data.qpos.copy()
        mujoco.mj_resetData(self.model, self.data)
        targets = dict(targets)
        base_pitch = float(targets.pop("base_pitch", 0.0))
        if base_pitch and self.model.njnt > 0 and self.model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE:
            quat_addr = int(self.model.jnt_qposadr[0]) + 3
            half = 0.5 * base_pitch
            self.data.qpos[quat_addr:quat_addr + 4] = [math.cos(half), 0.0, math.sin(half), 0.0]
        for joint_name, value in targets.items():
            joint_id, qpos_addr = self.joint_qpos_addr(joint_name)
            minimum, maximum = self.model.jnt_range[joint_id]
            self.data.qpos[qpos_addr] = clamp(float(value), float(minimum), float(maximum))
        target = self.data.qpos.copy()
        self.data.qpos[:] = saved
        return target

    def _settle_locked(self, qpos, pose_name):
        """Seat a qpos vector as a held, grounded, paused frame."""

        self.data.qpos[:] = qpos
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.paused = True
        self.active_pose = pose_name
        self.frame_id += 1
        self.last_step_wall_time = time.monotonic()
        mujoco.mj_forward(self.model, self.data)
        self.drop_to_floor_locked()

    def apply_named_pose(self, pose_name):
        targets = NAMED_POSES.get(pose_name)
        if targets is None:
            return {
                "ok": False,
                "error": f"unsupported pose: {pose_name}",
                "available_poses": sorted(NAMED_POSES),
            }

        with self.lock:
            target = self._target_qpos_locked(targets)
            self._settle_locked(target, pose_name)
            self.refresh_jpeg_cache()
            return {
                "ok": True,
                "pose": pose_name,
                "paused": self.paused,
                "frame_id": self.frame_id,
                "joints": targets,
            }

    def animate_to_pose(self, pose_name, duration=1.2, fps=30.0):
        """Interpolate from the currently held pose to a named pose.

        Each intermediate frame is re-grounded and rendered, so the transition
        is visible as continuous motion in the viewer and to snapshot readers.
        The base stays upright throughout, so lerping qpos (identity base quat in
        both endpoints) is sufficient — no slerp needed.
        """

        targets = NAMED_POSES.get(pose_name)
        if targets is None:
            return {
                "ok": False,
                "error": f"unsupported pose: {pose_name}",
                "available_poses": sorted(NAMED_POSES),
            }

        with self.lock:
            target = self._target_qpos_locked(targets)
            start = self.data.qpos.copy()

        frames = max(1, int(float(duration) * float(fps)))
        delay = 1.0 / max(float(fps), 1.0)
        # The render loop (render_hz) and websocket writer (frame_hz) render live
        # qpos on their own cadence, so this loop only advances the state and
        # paces itself; rendering a JPEG per frame here would make each glide run
        # many times slower than its requested duration under osmesa.
        for index in range(1, frames + 1):
            fraction = index / frames
            eased = fraction * fraction * (3.0 - 2.0 * fraction)
            blended = start + (target - start) * eased
            with self.lock:
                self._settle_locked(blended, pose_name)
            if index < frames:
                time.sleep(delay)
        self.refresh_jpeg_cache()

        return {
            "ok": True,
            "pose": pose_name,
            "paused": True,
            "frame_id": self.frame_id,
            "animated": True,
            "frames": frames,
            "joints": targets,
        }

    def apply_hold_control_locked(self):
        """Set actuator torques to PD-hold the current target pose.

        Gravity/bias forces are fed forward (qfrc_bias) so the motors supply the
        torque needed just to stand in the pose; the PD terms then only correct
        deviation. Without this the joints buckle under a loaded pose (a squat)
        because position error alone can't produce enough holding torque.
        """

        if self.control_mode != "hold":
            return
        q = self.data.qpos[self.actuator_qpos_adr]
        qd = self.data.qvel[self.actuator_dof_adr]
        gravity_comp = self.data.qfrc_bias[self.actuator_dof_adr]
        tau = gravity_comp + self.kp * (self.hold_target_qpos - q) - self.kd * qd
        np.clip(tau, self.ctrl_lo, self.ctrl_hi, out=tau)
        self.data.ctrl[:] = tau

    def apply_loco_velocity_locked(self, dt):
        if self.control_mode != "loco_velocity":
            return
        now = time.monotonic()
        velocity_until = self.loco_state.get("velocity_until")
        if velocity_until is not None and now > velocity_until:
            self.loco_state["velocity"] = [0.0, 0.0, 0.0]
            self.loco_state["velocity_until"] = None
            self.control_mode = None
            self.active_pose = "stand"
            self.paused = True
            return
        if self.model.njnt == 0 or self.model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
            return
        vx, vy, omega = [float(value) for value in self.loco_state.get("velocity", [0.0, 0.0, 0.0])]
        base_addr = int(self.model.jnt_qposadr[0])
        yaw = self.base_yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        self.data.qpos[base_addr + 0] += (vx * cos_yaw - vy * sin_yaw) * dt
        self.data.qpos[base_addr + 1] += (vx * sin_yaw + vy * cos_yaw) * dt
        self.base_yaw += omega * dt
        half_yaw = self.base_yaw * 0.5
        self.data.qpos[base_addr + 3 : base_addr + 7] = [
            math.cos(half_yaw),
            0.0,
            0.0,
            math.sin(half_yaw),
        ]

    def low_state_payload(self):
        with self.lock:
            self.refresh_lowcmd_watchdog_locked()
            self.apply_loco_velocity_locked(0.0)
            motor_state = []
            for index in range(35):
                if index < self.model.nu:
                    qpos_addr = int(self.actuator_qpos_adr[index])
                    dof_addr = int(self.actuator_dof_adr[index])
                    motor_state.append(
                        {
                            "mode": 1,
                            "q": float(self.data.qpos[qpos_addr]),
                            "dq": float(self.data.qvel[dof_addr]),
                            "tau_est": float(self.data.ctrl[index]),
                        }
                    )
                else:
                    motor_state.append({"mode": 0, "q": 0.0, "dq": 0.0, "tau_est": 0.0})
            return {
                "mode_pr": int(self.lowcmd_state.get("mode_pr") or 0),
                "mode_machine": int(self.loco_state.get("fsm_id") or 0),
                "crc": int(self.lowcmd_state.get("crc") or 0),
                "imu_state": {
                    "quaternion": as_list(self.data.qpos[3:7]) if self.model.nq >= 7 else [1.0, 0.0, 0.0, 0.0],
                    "gyroscope": [0.0, 0.0, 0.0],
                    "accelerometer": [0.0, 0.0, 0.0],
                },
                "motor_state": motor_state,
                "lowcmd": dict(self.lowcmd_state),
            }

    def joint_state_payload(self):
        with self.lock:
            joints = self.actuator_joint_map_locked()
            return {
                "robot": self.robot_name,
                "model_path": str(self.model_path),
                "actuator_count": int(self.model.nu),
                "message_motor_slots": 35,
                "joints": joints,
                "by_name": {joint["joint_name"]: joint for joint in joints},
                "lowcmd": dict(self.lowcmd_state),
            }

    def handle_lowcmd_command(self, payload):
        motor_cmd = payload.get("motor_cmd", [])
        if not isinstance(motor_cmd, list):
            return {"ok": False, "error": "motor_cmd must be a list"}
        if len(motor_cmd) > 35:
            return {"ok": False, "error": "motor_cmd supports at most 35 entries"}
        with self.lock:
            q = self.data.qpos[self.actuator_qpos_adr]
            qd = self.data.qvel[self.actuator_dof_adr]
            applied_targets = 0
            accepted = 0
            clamped = []
            ignored = []
            for index, command in enumerate(motor_cmd[: self.model.nu]):
                if not isinstance(command, dict):
                    return {"ok": False, "error": f"motor_cmd[{index}] must be an object"}
                try:
                    mode = int(command.get("mode", 0))
                    tau = float(command.get("tau", 0.0))
                    kp = float(command.get("kp", 0.0))
                    kd = float(command.get("kd", 0.0))
                    target_q = float(command.get("q", q[index]))
                    target_dq = float(command.get("dq", 0.0))
                except (TypeError, ValueError) as error:
                    return {"ok": False, "error": f"motor_cmd[{index}] has non-numeric fields: {error}"}
                accepted += 1
                if mode and (kp > 0.0 or kd > 0.0):
                    joint_id = int(self.actuator_joint_id[index])
                    qpos_addr = int(self.actuator_qpos_adr[index])
                    requested_q = target_q
                    if self.model.jnt_limited[joint_id]:
                        minimum, maximum = self.model.jnt_range[joint_id]
                        target_q = clamp(target_q, float(minimum), float(maximum))
                        if target_q != requested_q:
                            clamped.append(
                                {
                                    "index": index,
                                    "requested_q": requested_q,
                                    "applied_q": target_q,
                                    "minimum": float(minimum),
                                    "maximum": float(maximum),
                                }
                            )
                    self.data.qpos[qpos_addr] = target_q
                    self.data.qvel[int(self.actuator_dof_adr[index])] = target_dq
                    applied_targets += 1
                elif not mode:
                    ignored.append({"index": index, "reason": "mode disabled"})
                self.data.ctrl[index] = clamp(
                    tau + kp * (target_q - q[index]) + kd * (target_dq - qd[index]),
                    self.ctrl_lo[index],
                    self.ctrl_hi[index],
                )
            for index in range(min(len(motor_cmd), self.model.nu), len(motor_cmd)):
                ignored.append({"index": index, "reason": "no actuator in this G1 model"})
            mujoco.mj_forward(self.model, self.data)
            self.drop_to_floor_locked()
            received_at = time.time()
            expires_at = (
                received_at + self.lowcmd_watchdog_seconds
                if self.lowcmd_watchdog_seconds > 0
                else None
            )
            self.lowcmd_state = {
                "topic": payload.get("topic", "rt/lowcmd"),
                "received_at": received_at,
                "age_seconds": 0.0,
                "expires_at": expires_at,
                "watchdog_seconds": self.lowcmd_watchdog_seconds,
                "active": True,
                "stale": False,
                "motor_cmd_count": len(motor_cmd),
                "mode_pr": int(payload.get("mode_pr", 0) or 0),
                "mode_machine": int(payload.get("mode_machine", self.loco_state.get("fsm_id") or 0) or 0),
                "crc": int(payload.get("crc", 0) or 0),
                "accepted": accepted,
                "applied_position_targets": applied_targets,
                "clamped": clamped,
                "ignored": ignored,
            }
            self.control_mode = "lowcmd"
            self.active_pose = "lowcmd"
            self.paused = True
            self.last_step_wall_time = time.monotonic()
            self.refresh_jpeg_cache()
            return {
                "ok": True,
                "control_mode": self.control_mode,
                "paused": self.paused,
                "lowcmd": dict(self.lowcmd_state),
            }

    def handle_joint_targets_command(self, payload):
        targets = payload.get("targets", {})
        if not isinstance(targets, dict) or not targets:
            return {"ok": False, "error": "joint_targets requires a non-empty targets object"}

        try:
            kp = float(payload.get("kp", 38.0))
            kd = float(payload.get("kd", 1.4))
            tau = float(payload.get("tau", 0.0))
            dq = float(payload.get("dq", 0.0))
        except (TypeError, ValueError) as error:
            return {"ok": False, "error": f"joint target gains must be numeric: {error}"}

        with self.lock:
            joints = self.actuator_joint_map_locked()
            by_name = {joint["joint_name"]: joint for joint in joints}
            motor_cmd = [{"mode": 0, "q": 0.0, "dq": 0.0, "tau": 0.0, "kp": 0.0, "kd": 0.0} for _ in range(35)]
            unknown = []
            for joint_name, target in targets.items():
                joint = by_name.get(str(joint_name))
                if joint is None:
                    unknown.append(str(joint_name))
                    continue
                motor_cmd[int(joint["motor_index"])] = {
                    "mode": 1,
                    "q": target,
                    "dq": dq,
                    "tau": tau,
                    "kp": kp,
                    "kd": kd,
                }
            if unknown:
                return {
                    "ok": False,
                    "error": f"unknown joints: {', '.join(unknown)}",
                    "available_joints": sorted(by_name),
                }

        response = self.handle_lowcmd_command(
            {
                "command": "lowcmd",
                "topic": payload.get("topic", "rt/lowcmd/joint_targets"),
                "mode_pr": payload.get("mode_pr", 0),
                "mode_machine": payload.get("mode_machine", self.loco_state.get("fsm_id") or 0),
                "crc": payload.get("crc", 0),
                "motor_cmd": motor_cmd,
            }
        )
        if response.get("ok"):
            with self.lock:
                self.lowcmd_state["source"] = "joint_targets"
                self.lowcmd_state["joint_targets"] = {str(key): float(value) for key, value in targets.items()}
                response["lowcmd"] = dict(self.lowcmd_state)
        return response

    def handle_loco_command(self, payload):
        action = payload.get("action", "state")
        with self.lock:
            if action == "state":
                return {"ok": True, "loco": dict(self.loco_state), "control_mode": self.control_mode}
            if action == "get_fsm_id":
                return {"ok": True, "fsm_id": self.loco_state["fsm_id"], "loco": dict(self.loco_state)}
            if action == "get_fsm_mode":
                return {"ok": True, "fsm_mode": self.loco_state["fsm_mode"], "loco": dict(self.loco_state)}
            if action == "get_balance_mode":
                return {"ok": True, "balance_mode": self.loco_state["balance_mode"], "loco": dict(self.loco_state)}
            if action == "get_swing_height":
                return {"ok": True, "swing_height": self.loco_state["swing_height"], "loco": dict(self.loco_state)}
            if action == "get_stand_height":
                return {"ok": True, "stand_height": self.loco_state["stand_height"], "loco": dict(self.loco_state)}
            if action == "set_fsm_id":
                fsm_id = int(payload.get("fsm_id", 1))
                self.loco_state["fsm_id"] = fsm_id
                self.loco_state["fsm_mode"] = payload.get("mode") or str(fsm_id)
                if fsm_id in (0, 1):
                    self.control_mode = None
                    self.lowcmd_state["received_at"] = None
                    self.refresh_lowcmd_watchdog_locked()
                    self.loco_state["velocity"] = [0.0, 0.0, 0.0]
                    self.loco_state["velocity_until"] = None
                    self.paused = True
                    self.active_pose = "zero_torque" if fsm_id == 0 else "damp"
                elif fsm_id == 500:
                    self.active_pose = "stand"
                return {"ok": True, "loco": dict(self.loco_state), "paused": self.paused}
            if action == "set_balance_mode":
                self.loco_state["balance_mode"] = int(payload.get("balance_mode", 0))
                return {"ok": True, "loco": dict(self.loco_state)}
            if action == "set_swing_height":
                self.loco_state["swing_height"] = float(payload.get("swing_height", 0.0))
                return {"ok": True, "loco": dict(self.loco_state)}
            if action == "set_stand_height":
                stand_height = float(payload.get("stand_height", 0.0))
                self.loco_state["stand_height"] = stand_height
                return {"ok": True, "loco": dict(self.loco_state)}
            if action == "set_velocity":
                velocity = payload.get("velocity", [0.0, 0.0, 0.0])
                if not isinstance(velocity, list) or len(velocity) != 3:
                    return {"ok": False, "error": "velocity must be [vx, vy, omega]"}
                duration = float(payload.get("duration", 1.0))
                self.loco_state["velocity"] = [float(velocity[0]), float(velocity[1]), float(velocity[2])]
                if all(abs(value) < 1e-9 for value in self.loco_state["velocity"]) or duration <= 0:
                    self.loco_state["velocity_until"] = None
                    self.control_mode = None
                    self.active_pose = "stand"
                    self.paused = True
                else:
                    self.loco_state["velocity_until"] = None if duration >= 864000.0 else time.monotonic() + max(0.0, duration)
                    self.active_pose = "loco_move"
                    self.control_mode = "loco_velocity"
                    self.paused = False
                self.last_step_wall_time = time.monotonic()
                return {"ok": True, "loco": dict(self.loco_state), "paused": self.paused}
            if action == "set_arm_task":
                task_id = int(payload.get("task_id", 0))
                self.loco_state["arm_task_id"] = task_id
                pose_name = "raise_right_hand" if task_id in (0, 1, 2, 3) else "neutral"
                response = self.apply_named_pose(pose_name)
                response["loco"] = dict(self.loco_state)
                return response
            if action == "high_stand":
                self.loco_state["stand_height"] = "high"
                response = self.apply_named_pose("mountain")
                response["loco"] = dict(self.loco_state)
                return response
            if action == "low_stand":
                self.loco_state["stand_height"] = "low"
                response = self.apply_named_pose("chair")
                response["loco"] = dict(self.loco_state)
                return response
            if action in ("wave_hand", "shake_hand"):
                self.loco_state["arm_task_id"] = 1 if action == "wave_hand" else 2
                response = self.apply_named_pose("raise_right_hand")
                response["loco"] = dict(self.loco_state)
                return response
            return {"ok": False, "error": f"unsupported loco action: {action}"}

    def set_hold_pose(self, pose_name, teleport=True):
        """Drive the motors to hold a pose while physics runs.

        With teleport=True the robot is first seated in the pose (zero velocity)
        and the PD controller then fights gravity to keep it there. Statically
        stable, well-grounded poses hold; poses whose center of mass falls
        outside the support (single-leg balances) will topple — by design, since
        there is no whole-body balance controller.
        """

        targets = NAMED_POSES.get(pose_name)
        if targets is None:
            return {
                "ok": False,
                "error": f"unsupported pose: {pose_name}",
                "available_poses": sorted(NAMED_POSES),
            }

        with self.lock:
            target_qpos = np.zeros(self.model.nu)
            for index in range(self.model.nu):
                joint_id = int(self.model.actuator_trnid[index, 0])
                joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or ""
                if joint_name in targets:
                    minimum, maximum = self.model.jnt_range[joint_id]
                    target_qpos[index] = clamp(float(targets[joint_name]), float(minimum), float(maximum))
            self.hold_target_qpos = target_qpos

            if teleport:
                full = self._target_qpos_locked(targets)
                self.data.qpos[:] = full
                self.data.qvel[:] = 0.0
                mujoco.mj_forward(self.model, self.data)
                self.drop_to_floor_locked()

            self.data.ctrl[:] = 0.0
            self.control_mode = "hold"
            self.paused = False
            self.active_pose = pose_name
            self.frame_id += 1
            self.last_step_wall_time = time.monotonic()

        self.refresh_jpeg_cache()
        return {
            "ok": True,
            "pose": pose_name,
            "held": True,
            "paused": False,
            "frame_id": self.frame_id,
        }

    def is_fallen_locked(self):
        pelvis_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        if pelvis_id < 0:
            return False, None
        height = float(self.data.xpos[pelvis_id, 2])
        upright = float(self.data.xmat[pelvis_id].reshape(3, 3)[2, 2])
        fallen = bool(height < 0.45 or upright < 0.5)
        return fallen, height

    def drop_to_floor_locked(self, clearance=0.02):
        """Lower the floating base so the lowest robot geom rests on the floor.

        Static poses set only joint angles, so bending the knees leaves the
        feet floating at the default standing height. This re-seats the pose on
        the ground by translating the free base in z. No-op when the model has
        no floating base as its first joint.
        """

        if self.model.njnt == 0 or self.model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
            return
        base_z_addr = int(self.model.jnt_qposadr[0]) + 2
        robot_geoms = self.model.geom_bodyid != 0
        if not bool(np.any(robot_geoms)):
            return
        lowest = float(np.min(self.data.geom_xpos[robot_geoms, 2]))
        self.data.qpos[base_z_addr] += clearance - lowest
        mujoco.mj_forward(self.model, self.data)

    def reset_camera(self):
        with self.camera_lock:
            self.desired_camera = dict(self.default_camera)
            return self._camera_payload_locked()

    def camera_payload(self):
        with self.camera_lock:
            return self._camera_payload_locked()

    def _camera_payload_locked(self):
        return {
            "cameraId": 0,
            "type": "free",
            "lookat": list(self.desired_camera["lookat"]),
            "distance": float(self.desired_camera["distance"]),
            "azimuth": float(self.desired_camera["azimuth"]),
            "elevation": float(self.desired_camera["elevation"]),
        }

    def apply_desired_camera_locked(self):
        with self.camera_lock:
            desired = self.desired_camera
            self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.camera.lookat[:] = desired["lookat"]
            self.camera.distance = desired["distance"]
            self.camera.azimuth = desired["azimuth"]
            self.camera.elevation = desired["elevation"]

    def desired_camera_snapshot(self):
        with self.camera_lock:
            return dict(self.desired_camera)

    def desired_camera_position(self):
        desired = self.desired_camera_snapshot()
        return [
            float(desired["lookat"][0]),
            float(desired["lookat"][1] - desired["distance"]),
            float(desired["lookat"][2] + desired["distance"] * 0.35),
        ]

    def desired_camera_matrix(self):
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    def orbit_camera(self, dx=0.0, dy=0.0):
        with self.camera_lock:
            self.desired_camera["azimuth"] = (
                float(self.desired_camera["azimuth"]) + float(dx) * 0.28
            )
            self.desired_camera["elevation"] = clamp(
                float(self.desired_camera["elevation"]) - float(dy) * 0.22,
                -85.0,
                85.0,
            )
            return self._camera_payload_locked()

    def pan_camera(self, dx=0.0, dy=0.0):
        with self.camera_lock:
            azimuth = math.radians(float(self.desired_camera["azimuth"]))
            right = np.array([math.cos(azimuth), math.sin(azimuth), 0.0])
            up = np.array([0.0, 0.0, 1.0])
            scale = max(float(self.desired_camera["distance"]), 0.5) * 0.0018
            lookat = np.array(self.desired_camera["lookat"], dtype=float) + (
                -float(dx) * scale * right + float(dy) * scale * up
            )
            lookat[2] = clamp(float(lookat[2]), -0.5, 2.5)
            self.desired_camera["lookat"] = lookat.tolist()
            return self._camera_payload_locked()

    def zoom_camera(self, delta=0.0):
        with self.camera_lock:
            factor = math.exp(-float(delta) * 0.0018)
            self.desired_camera["distance"] = clamp(
                float(self.desired_camera["distance"]) * factor,
                0.45,
                12.0,
            )
            return self._camera_payload_locked()

    def advance_for_wall_time(self):
        with self.lock:
            now = time.monotonic()
            if self.paused:
                self.last_step_wall_time = now
                self.refresh_lowcmd_watchdog_locked()
                mujoco.mj_forward(self.model, self.data)
                return
            elapsed = min(now - self.last_step_wall_time, 0.08)
            steps = max(1, int(elapsed / self.model.opt.timestep))
            for _ in range(steps):
                self.refresh_lowcmd_watchdog_locked()
                self.apply_hold_control_locked()
                self.apply_loco_velocity_locked(self.model.opt.timestep)
                mujoco.mj_step(self.model, self.data)
                self.frame_id += 1
            self.last_step_wall_time = now

    def simulation_state_payload(self):
        with self.lock:
            self.refresh_lowcmd_watchdog_locked()
            render_cache = self.render_cache_payload()
            fallen, pelvis_height = self.is_fallen_locked()
            return {
                "actual_speed_factor": 0.0 if self.paused else 1.0,
                "paused": self.paused,
                "control_mode": self.control_mode,
                "fallen": fallen,
                "pelvis_height": pelvis_height,
                "robot_statuses": {self.robot_name: True},
                "is_multi_robot": False,
                "robot_modes": {
                    self.robot_name: self.active_pose
                    if self.active_pose is not None
                    else ("stand" if self.paused else "free_sim")
                },
                "pose": self.active_pose,
                "loco": dict(self.loco_state),
                "model_path": str(self.model_path),
                "model_revision": self.model_revision,
                "all_robot_names": [self.robot_name],
                "robot_pose_specs": {
                    self.robot_name: {
                        "qpos_offset": 0,
                        "qpos_size": int(self.model.nq),
                    }
                },
                "mujoco": {
                    "version": mujoco.mj_versionString(),
                    "nq": int(self.model.nq),
                    "nv": int(self.model.nv),
                    "nu": int(self.model.nu),
                    "nbody": int(self.model.nbody),
                    "ngeom": int(self.model.ngeom),
                },
                "render": {
                    "width": self.render_width,
                    "height": self.render_height,
                    "camera_frame_topic": "camera_frame_0",
                    "render_hz": self.render_hz,
                    **render_cache,
                    "last_error": self.last_render_error,
                    "camera": self.camera_payload(),
                },
                "lowcmd": dict(self.lowcmd_state),
            }

    def render_cache_payload(self):
        with self.cache_lock:
            return {
                "render_seq": self.render_seq,
                "latest_jpeg_frame_id": self.latest_jpeg_frame_id,
                "latest_jpeg_rendered_at": self.latest_jpeg_rendered_at,
            }

    def visual_scene_payload(self, request_id=None):
        with self.lock:
            return {
                "request_id": request_id,
                "revision": self.model_revision,
                "model_path": str(self.model_path),
                "robot": self.robot_name,
                "bodies": [
                    {
                        "id": index,
                        "name": named(
                            self.model,
                            mujoco.mjtObj.mjOBJ_BODY,
                            index,
                            f"body_{index}",
                        ),
                    }
                    for index in range(self.model.nbody)
                ],
                "geoms": [
                    {
                        "id": index,
                        "name": named(
                            self.model,
                            mujoco.mjtObj.mjOBJ_GEOM,
                            index,
                            f"geom_{index}",
                        ),
                        "rgba": as_list(self.model.geom_rgba[index]),
                    }
                    for index in range(self.model.ngeom)
                ],
                "cameras": [
                    {
                        "cameraId": 0,
                        "name": "free_camera",
                        "fovy": 45.0,
                        "znear": 0.01,
                        "zfar": 50.0,
                    }
                ],
            }

    def visual_frame_payload(self):
        with self.lock:
            pelvis_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                "pelvis",
            )
            if pelvis_id < 0:
                pelvis_id = 1 if self.model.nbody > 1 else 0

            robot_position = as_list(self.data.xpos[pelvis_id])
            robot_rotation = as_list(self.data.xmat[pelvis_id])

            return {
                "revision": self.model_revision,
                "time": float(self.data.time),
                "frame_id": int(self.frame_id),
                "geoms": [
                    {
                        "id": index,
                        "name": named(
                            self.model,
                            mujoco.mjtObj.mjOBJ_GEOM,
                            index,
                            f"geom_{index}",
                        ),
                        "position": as_list(self.data.geom_xpos[index]),
                        "matrix": as_list(self.data.geom_xmat[index]),
                        "rgba": as_list(self.model.geom_rgba[index]),
                    }
                    for index in range(self.model.ngeom)
                ],
                "bodies": [
                    {
                        "id": index,
                        "name": named(
                            self.model,
                            mujoco.mjtObj.mjOBJ_BODY,
                            index,
                            f"body_{index}",
                        ),
                        "position": as_list(self.data.xpos[index]),
                        "matrix": as_list(self.data.xmat[index]),
                        "quaternion": as_list(self.data.xquat[index]),
                    }
                    for index in range(self.model.nbody)
                ],
                "cameras": [
                    {
                        "cameraId": 0,
                        "name": "free_camera",
                        "fovy": 45.0,
                        "znear": 0.01,
                        "zfar": 50.0,
                        "position": self.desired_camera_position(),
                        "matrix": self.desired_camera_matrix(),
                    }
                ],
                "robotLabels": [
                    {
                        "bodyId": int(pelvis_id),
                        "name": self.robot_name,
                        "position": robot_position,
                        "rotation": robot_rotation,
                    }
                ],
            }

    def render_image(self, image_format="PNG"):
        with self.lock:
            try:
                self.apply_desired_camera_locked()
                self.renderer.update_scene(self.data, camera=self.camera)
                rgb = self.renderer.render()
                image = Image.fromarray(rgb)
                output = io.BytesIO()
                if image_format == "JPEG":
                    image.save(output, format="JPEG", quality=82, optimize=False)
                else:
                    image.save(output, format="PNG", compress_level=1)
                self.last_render_error = None
                return output.getvalue()
            except Exception as error:
                self.last_render_error = str(error)
                raise

    def render_png(self):
        return self.render_image("PNG")

    def render_jpeg(self):
        with self.cache_lock:
            if self.latest_jpeg is not None:
                return self.latest_jpeg

        self.refresh_jpeg_cache()
        with self.cache_lock:
            if self.latest_jpeg is None:
                raise RuntimeError("JPEG render cache is empty")
            return self.latest_jpeg

    def refresh_jpeg_cache(self):
        with self.lock:
            try:
                self.apply_desired_camera_locked()
                self.renderer.update_scene(self.data, camera=self.camera)
                rgb = self.renderer.render()
                image = Image.fromarray(rgb)
                output = io.BytesIO()
                image.save(output, format="JPEG", quality=82, optimize=False)
                jpeg = output.getvalue()
                frame_id = int(self.frame_id)
                rendered_at = time.time()
                with self.cache_lock:
                    self.latest_jpeg = jpeg
                    self.latest_jpeg_frame_id = frame_id
                    self.latest_jpeg_rendered_at = rendered_at
                    self.render_seq += 1
                self.last_render_error = None
            except Exception as error:
                self.last_render_error = str(error)
                raise

    def camera_frame_payload(self):
        png = self.render_png()
        with self.lock:
            return {
                "frame_id": int(self.frame_id),
                "time": float(self.data.time),
                "cameraId": 0,
                "encoding": "png",
                "width": self.render_width,
                "height": self.render_height,
                "data": png,
            }

    def envelope(self, topic, payload):
        encoded = msgpack.packb(payload, use_bin_type=True)
        message_type = MESSAGE_TYPES.get(topic, MESSAGE_TYPES["visual_scene_error"])
        return struct.pack(">II", message_type, len(encoded)) + encoded

    def frame_for_topic(self, topic):
        if topic == "simulation_state":
            return self.envelope(topic, self.simulation_state_payload())
        if topic == "visual_scene":
            return self.envelope(topic, self.visual_scene_payload())
        if topic == "visual_frame":
            return self.envelope(topic, self.visual_frame_payload())
        if topic == "camera_frame_0":
            return self.envelope(topic, self.camera_frame_payload())
        return self.envelope(
            "visual_scene_error",
            {"topic": topic, "error": f"unsupported topic: {topic}"},
        )

    def handle_command(self, command, payload=None):
        payload = payload or {}
        if command == "pause":
            self.set_paused(True)
            return {"ok": True, "paused": True}
        if command in ("resume", "play"):
            self.set_paused(False)
            return {"ok": True, "paused": False}
        if command == "reset":
            self.reset()
            return {"ok": True, "paused": self.paused}
        if command == "step":
            self.step(1)
            return {"ok": True, "paused": self.paused, "frame_id": self.frame_id}
        if command == "pose":
            pose_name = payload.get("pose", "raise_right_hand")
            if payload.get("smooth") or payload.get("duration") is not None:
                return self.animate_to_pose(
                    pose_name,
                    duration=payload.get("duration", 1.2),
                    fps=payload.get("fps", 30.0),
                )
            return self.apply_named_pose(pose_name)
        if command == "hold_pose":
            return self.set_hold_pose(
                payload.get("pose", "raise_right_hand"),
                teleport=payload.get("teleport", True),
            )
        if command == "loco":
            return self.handle_loco_command(payload)
        if command == "lowcmd":
            return self.handle_lowcmd_command(payload)
        if command == "joint_targets":
            return self.handle_joint_targets_command(payload)
        if command in NAMED_POSES:
            return self.apply_named_pose(command)
        return {"ok": False, "error": f"unsupported command: {command}"}

    def handle_camera_command(self, command):
        action = command.get("action", command.get("command", "state"))
        if action == "state":
            return {"ok": True, "camera": self.camera_payload()}
        if action == "reset":
            self.reset_camera()
            return {"ok": True, "camera": self.camera_payload()}
        if action == "orbit":
            camera = self.orbit_camera(command.get("dx", 0.0), command.get("dy", 0.0))
            return {"ok": True, "camera": camera}
        if action == "pan":
            camera = self.pan_camera(command.get("dx", 0.0), command.get("dy", 0.0))
            return {"ok": True, "camera": camera}
        if action == "zoom":
            camera = self.zoom_camera(command.get("delta", 0.0))
            return {"ok": True, "camera": camera}
        return {"ok": False, "error": f"unsupported camera action: {action}"}


def start_http_server(state):
    class Handler(BaseHTTPRequestHandler):
        server_version = "UnitreeG1MujocoProtocol/0.1"

        def log_message(self, fmt, *args):
            print(f"[http] {self.address_string()} {fmt % args}", flush=True)

        def write_json(self, status, body):
            data = json.dumps(body, indent=2).encode("utf-8")
            self.send_response(status)
            self.send_header("content-type", "application/json")
            self.send_header("content-length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def write_image(self, data, content_type):
            self.send_response(200)
            self.send_header("content-type", content_type)
            self.send_header("cache-control", "no-store")
            self.send_header("content-length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def read_json_body(self):
            length = int(self.headers.get("content-length", "0"))
            if length <= 0:
                return {}
            data = self.rfile.read(length)
            return json.loads(data.decode("utf-8"))

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path in ("/health", "/status"):
                payload = state.simulation_state_payload()
                self.write_json(
                    200,
                    {
                        "status": "ok",
                        "ready": True,
                        "checks": {
                            "mujoco": True,
                            "model": str(state.model_path),
                            "renderer": state.last_render_error is None,
                        },
                        "simulation": payload,
                    },
                )
                return

            if parsed.path == "/camera":
                self.write_json(200, {"ok": True, "camera": state.camera_payload()})
                return

            if parsed.path == "/lowstate":
                self.write_json(200, state.low_state_payload())
                return

            if parsed.path == "/joint_state":
                self.write_json(200, state.joint_state_payload())
                return

            if parsed.path == "/visual_frame":
                self.write_json(200, state.visual_frame_payload())
                return

            if parsed.path == "/visual_scene":
                self.write_json(200, state.visual_scene_payload())
                return

            if parsed.path == "/camera_control":
                query = parse_qs(parsed.query)
                command = {
                    "action": query.get("action", ["state"])[0],
                    "dx": query_float(query, "dx"),
                    "dy": query_float(query, "dy"),
                    "delta": query_float(query, "delta"),
                }
                self.write_json(200, state.handle_camera_command(command))
                return

            if parsed.path in ("/camera_frame_0.png", "/frame.png"):
                try:
                    data = state.render_png()
                except Exception as error:
                    self.write_json(500, {"status": "error", "error": str(error)})
                    return
                self.write_image(data, "image/png")
                return

            if parsed.path in ("/camera_frame_0.jpg", "/frame.jpg"):
                try:
                    data = state.render_jpeg()
                except Exception as error:
                    self.write_json(500, {"status": "error", "error": str(error)})
                    return
                self.write_image(data, "image/jpeg")
                return

            self.write_json(404, {"status": "not_found", "path": self.path})

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path not in (
                "/command",
                "/camera",
                "/camera_frame_0.png",
                "/frame.png",
                "/camera_frame_0.jpg",
                "/frame.jpg",
            ):
                self.write_json(404, {"status": "not_found", "path": self.path})
                return

            try:
                command = self.read_json_body()
                if parsed.path == "/command":
                    self.write_json(
                        200,
                        state.handle_command(command.get("command"), command),
                    )
                    return
                result = state.handle_camera_command(command)
            except Exception as error:
                self.write_json(400, {"ok": False, "error": str(error)})
                return

            if parsed.path == "/camera":
                self.write_json(200, result)
                return

            if not result.get("ok"):
                self.write_json(400, result)
                return

            try:
                if parsed.path in ("/camera_frame_0.jpg", "/frame.jpg"):
                    data = state.render_jpeg()
                    content_type = "image/jpeg"
                else:
                    data = state.render_png()
                    content_type = "image/png"
            except Exception as error:
                self.write_json(500, {"status": "error", "error": str(error)})
                return
            self.write_image(data, content_type)

    server = ThreadingHTTPServer(("0.0.0.0", 38383), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def start_render_loop(state):
    def loop():
        interval = 1.0 / max(state.render_hz, 1.0)
        while True:
            started = time.monotonic()
            try:
                state.advance_for_wall_time()
                state.refresh_jpeg_cache()
            except Exception as error:
                print(f"[render] {error}", flush=True)

            elapsed = time.monotonic() - started
            time.sleep(max(0.001, interval - elapsed))

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    return thread


async def websocket_handler(state, websocket):
    subscriptions = set()
    closed = False

    async def writer():
        while not closed:
            if subscriptions:
                for topic in list(subscriptions):
                    await websocket.send(state.frame_for_topic(topic))
            await asyncio.sleep(1.0 / max(state.frame_hz, 1.0))

    writer_task = asyncio.create_task(writer())
    try:
        async for message in websocket:
            if isinstance(message, bytes):
                continue
            if message.startswith("subscribe:"):
                topic = message.split(":", 1)[1].strip()
                subscriptions.add(topic)
                await websocket.send(state.frame_for_topic(topic))
                continue
            if message.startswith("unsubscribe:"):
                topic = message.split(":", 1)[1].strip()
                subscriptions.discard(topic)
                continue

            try:
                command = json.loads(message)
            except json.JSONDecodeError:
                await websocket.send(json.dumps({"ok": False, "error": "invalid json"}))
                continue

            if command.get("type") == "command":
                result = state.handle_command(command.get("command"), command)
                await websocket.send(json.dumps(result))
                continue

            if command.get("type") == "camera":
                result = state.handle_camera_command(command)
                await websocket.send(json.dumps(result))
                continue

            if command.get("type") == "visual_scene_request":
                request_id = command.get("request_id")
                await websocket.send(
                    state.envelope(
                        "visual_scene",
                        state.visual_scene_payload(request_id=request_id),
                    )
                )
                continue

            await websocket.send(
                json.dumps({"ok": False, "error": f"unsupported message: {command}"})
            )
    finally:
        closed = True
        writer_task.cancel()


async def main():
    state = G1MujocoState()
    start_render_loop(state)
    start_http_server(state)
    print(
        json.dumps(
            {
                "status": "ready",
                "model_path": str(state.model_path),
                "model_revision": state.model_revision,
                "mujoco": mujoco.mj_versionString(),
                "physics_ws": "ws://0.0.0.0:8788",
                "game_control": "http://0.0.0.0:38383",
                "paused": state.paused,
            },
            indent=2,
        ),
        flush=True,
    )
    async with websockets.serve(
        lambda websocket: websocket_handler(state, websocket),
        "0.0.0.0",
        8788,
        max_size=None,
        ping_interval=20,
        ping_timeout=20,
    ):
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
