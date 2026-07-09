#!/usr/bin/env python3
"""Native, GPU-accelerated renderer for the G1 MuJoCo Robot Viewer.

Runs outside Docker (unlike g1_protocol_sim.py, which stays containerized and
does the physics stepping). This process owns a single MuJoCo Renderer on its
main thread -- MUJOCO_GL is deliberately left unset so mujoco auto-selects the
native GPU-backed backend (CGL on macOS) instead of the container's software
osmesa renderer. It receives qpos telemetry from the containerized simulator
over the existing WebSocket protocol (topic "physics_qpos") and serves camera
commands + rendered frames over a small local HTTP server, mirroring the
GameControl HTTP surface g1_protocol_sim.py already exposes so the Rust panel
can talk to either one with the same request shapes.
"""

import io
import json
import os
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import msgpack
import mujoco
import numpy as np
from PIL import Image
from websockets.sync.client import connect as ws_connect

import g1_camera

PHYSICS_QPOS_MESSAGE_TYPE = 13
PHYSICS_QPOS_TOPIC = "physics_qpos"


def env_float(name, default):
    value = os.environ.get(name)
    return float(value) if value else default


def env_int(name, default):
    value = os.environ.get(name)
    return int(value) if value else default


RENDER_WIDTH = env_int("UNITREE_G1_NATIVE_RENDER_WIDTH", 1280)
RENDER_HEIGHT = env_int("UNITREE_G1_NATIVE_RENDER_HEIGHT", 960)
RENDER_HZ = env_float("UNITREE_G1_NATIVE_RENDER_HZ", 60.0)
HTTP_PORT = env_int("UNITREE_G1_NATIVE_RENDER_HTTP_PORT", 38384)
JPEG_QUALITY = env_int("UNITREE_G1_NATIVE_RENDER_JPEG_QUALITY", 90)
PHYSICS_WS_URL = os.environ.get("UNITREE_G1_PHYSICS_WS", "ws://127.0.0.1:8788")


def resolve_model_path():
    env_path = os.environ.get("UNITREE_G1_NATIVE_MODEL_PATH")
    if env_path:
        return Path(env_path)
    repo_root = Path(__file__).resolve().parents[3]
    return (
        repo_root
        / ".runtime/unitree-g1-mujoco/unitree_mujoco/unitree_robots/g1/scene_29dof.xml"
    )


class PhysicsState:
    def __init__(self):
        self.lock = threading.Lock()
        self.qpos = None
        self.frame_id = 0
        self.time = 0.0


class CameraState:
    def __init__(self, default):
        self.lock = threading.Lock()
        self.default = default
        self.camera = dict(default)


class FrameCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.frame_id = 0
        self.rendered_at = 0.0


def apply_camera_command(camera_state, command):
    action = command.get("action", command.get("command", "state"))
    with camera_state.lock:
        if action == "state":
            pass
        elif action == "reset":
            camera_state.camera = dict(camera_state.default)
        elif action == "orbit":
            g1_camera.orbit(
                camera_state.camera, command.get("dx", 0.0), command.get("dy", 0.0)
            )
        elif action == "pan":
            g1_camera.pan(
                camera_state.camera, command.get("dx", 0.0), command.get("dy", 0.0)
            )
        elif action == "zoom":
            g1_camera.zoom(camera_state.camera, command.get("delta", 0.0))
        elif action == "set":
            g1_camera.set_absolute(camera_state.camera, command)
        else:
            raise ValueError(f"unsupported camera action: {action}")
        return g1_camera.camera_payload(camera_state.camera)


def run_physics_ws_client(physics_state, ws_url):
    backoff = 0.5
    while True:
        try:
            with ws_connect(ws_url, open_timeout=5) as ws:
                ws.send(f"subscribe:{PHYSICS_QPOS_TOPIC}")
                backoff = 0.5
                for message in ws:
                    if not isinstance(message, (bytes, bytearray)) or len(message) < 8:
                        continue
                    message_type, length = struct.unpack(">II", message[:8])
                    if message_type != PHYSICS_QPOS_MESSAGE_TYPE:
                        continue
                    payload = msgpack.unpackb(message[8 : 8 + length], raw=False)
                    qpos = np.asarray(payload["qpos"], dtype=float)
                    with physics_state.lock:
                        physics_state.qpos = qpos
                        physics_state.frame_id = payload.get(
                            "frame_id", physics_state.frame_id
                        )
                        physics_state.time = payload.get("time", physics_state.time)
        except Exception as error:
            print(
                f"[g1_native_renderer] physics WS error: {error!r}; retrying in {backoff:.1f}s",
                file=sys.stderr,
                flush=True,
            )
        time.sleep(backoff)
        backoff = min(backoff * 2.0, 5.0)


def make_http_handler(camera_state, frame_cache):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, format, *args):  # noqa: A002 - stdlib signature
            pass

        def _write_json(self, status, payload):
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _write_empty(self, status):
            self.send_response(status)
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/health":
                self._write_json(200, {"ok": True})
                return
            if path in ("/camera_frame_0.jpg", "/frame.jpg"):
                with frame_cache.lock:
                    jpeg = frame_cache.jpeg
                if jpeg is None:
                    self._write_empty(503)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
                return
            if path == "/camera":
                with camera_state.lock:
                    payload = g1_camera.camera_payload(camera_state.camera)
                self._write_json(200, payload)
                return
            self._write_empty(404)

        def do_POST(self):
            if urlparse(self.path).path != "/camera":
                self._write_empty(404)
                return
            length = int(self.headers.get("Content-Length", "0") or "0")
            raw = self.rfile.read(length) if length else b"{}"
            try:
                command = json.loads(raw or b"{}")
                payload = apply_camera_command(camera_state, command)
                self._write_json(200, payload)
            except Exception as error:
                self._write_json(400, {"ok": False, "error": str(error)})

    return Handler


def run_http_server(camera_state, frame_cache, port):
    handler = make_http_handler(camera_state, frame_cache)
    server = ThreadingHTTPServer(("0.0.0.0", port), handler)
    server.daemon_threads = True
    server.serve_forever()


def run_render_loop(model, data, physics_state, camera_state, frame_cache):
    renderer = mujoco.Renderer(model, height=RENDER_HEIGHT, width=RENDER_WIDTH)
    try:
        gl_type = type(renderer._gl_context).__module__ + "." + type(
            renderer._gl_context
        ).__name__
    except Exception:
        gl_type = "unknown"
    print(
        f"[g1_native_renderer] renderer ready: {RENDER_WIDTH}x{RENDER_HEIGHT} "
        f"@ {RENDER_HZ}Hz gl_context={gl_type} http_port={HTTP_PORT}",
        flush=True,
    )

    mjv_camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(mjv_camera)
    interval = 1.0 / max(RENDER_HZ, 1.0)
    frame_id = 0

    while True:
        loop_start = time.monotonic()

        with physics_state.lock:
            qpos = physics_state.qpos
        if qpos is not None and qpos.shape[0] == model.nq:
            data.qpos[:] = qpos
            mujoco.mj_kinematics(model, data)
            mujoco.mj_camlight(model, data)

        with camera_state.lock:
            camera_snapshot = dict(camera_state.camera)
        g1_camera.apply_to_mjv_camera(camera_snapshot, mjv_camera, mujoco)

        renderer.update_scene(data, camera=mjv_camera)
        frame = renderer.render()

        buffer = io.BytesIO()
        Image.fromarray(frame).save(
            buffer, format="JPEG", quality=JPEG_QUALITY, optimize=False
        )
        frame_id += 1
        with frame_cache.lock:
            frame_cache.jpeg = buffer.getvalue()
            frame_cache.frame_id = frame_id
            frame_cache.rendered_at = time.time()

        remaining = interval - (time.monotonic() - loop_start)
        if remaining > 0:
            time.sleep(remaining)


def main():
    model_path = resolve_model_path()
    if not model_path.exists():
        raise FileNotFoundError(f"MuJoCo model not found: {model_path}")

    spec = mujoco.MjSpec.from_file(str(model_path))
    spec.visual.global_.offwidth = RENDER_WIDTH
    spec.visual.global_.offheight = RENDER_HEIGHT
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    physics_state = PhysicsState()
    camera_state = CameraState(
        g1_camera.default_camera_state(
            lookat=(0.0, 0.0, 0.72),
            distance=env_float("UNITREE_G1_CAMERA_DISTANCE", 3.2),
            azimuth=env_float("UNITREE_G1_CAMERA_AZIMUTH", -130.0),
            elevation=env_float("UNITREE_G1_CAMERA_ELEVATION", -18.0),
        )
    )
    frame_cache = FrameCache()

    threading.Thread(
        target=run_physics_ws_client,
        args=(physics_state, PHYSICS_WS_URL),
        daemon=True,
    ).start()
    threading.Thread(
        target=run_http_server,
        args=(camera_state, frame_cache, HTTP_PORT),
        daemon=True,
    ).start()

    run_render_loop(model, data, physics_state, camera_state, frame_cache)


if __name__ == "__main__":
    main()
