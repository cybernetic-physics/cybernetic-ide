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
        self.paused = os.environ.get("UNITREE_G1_AUTORUN", "0") != "1"
        self.frame_id = 0
        self.last_step_wall_time = time.monotonic()
        self.last_render_error = None
        self.active_pose = None
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

    def reset(self):
        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            self.frame_id = 0
            self.active_pose = None
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
                mujoco.mj_step(self.model, self.data)
                self.frame_id += 1

    def joint_qpos_addr(self, joint_name):
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise KeyError(f"unknown joint: {joint_name}")
        return joint_id, int(self.model.jnt_qposadr[joint_id])

    def apply_named_pose(self, pose_name):
        targets = NAMED_POSES.get(pose_name)
        if targets is None:
            return {
                "ok": False,
                "error": f"unsupported pose: {pose_name}",
                "available_poses": sorted(NAMED_POSES),
            }

        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            for joint_name, value in targets.items():
                joint_id, qpos_addr = self.joint_qpos_addr(joint_name)
                minimum, maximum = self.model.jnt_range[joint_id]
                self.data.qpos[qpos_addr] = clamp(float(value), float(minimum), float(maximum))
            self.data.qvel[:] = 0.0
            self.data.ctrl[:] = 0.0
            self.paused = True
            self.active_pose = pose_name
            self.frame_id += 1
            self.last_step_wall_time = time.monotonic()
            mujoco.mj_forward(self.model, self.data)
            self.refresh_jpeg_cache()
            return {
                "ok": True,
                "pose": pose_name,
                "paused": self.paused,
                "frame_id": self.frame_id,
                "joints": targets,
            }

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
                mujoco.mj_forward(self.model, self.data)
                return
            elapsed = min(now - self.last_step_wall_time, 0.08)
            steps = max(1, int(elapsed / self.model.opt.timestep))
            for _ in range(steps):
                mujoco.mj_step(self.model, self.data)
                self.frame_id += 1
            self.last_step_wall_time = now

    def simulation_state_payload(self):
        with self.lock:
            render_cache = self.render_cache_payload()
            return {
                "actual_speed_factor": 0.0 if self.paused else 1.0,
                "paused": self.paused,
                "robot_statuses": {self.robot_name: True},
                "is_multi_robot": False,
                "robot_modes": {
                    self.robot_name: self.active_pose
                    if self.active_pose is not None
                    else ("stand" if self.paused else "free_sim")
                },
                "pose": self.active_pose,
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
            return self.apply_named_pose(payload.get("pose", "raise_right_hand"))
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
