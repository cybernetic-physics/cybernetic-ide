from __future__ import annotations

import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import unittest

from cybernetic_robotics import G1Robot, RobotEndpoints, SimulatorClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, current_channel_factory_config
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map


class FakeG1Handler(BaseHTTPRequestHandler):
    pose = "neutral"
    paused = True
    camera = {
        "cameraId": 0,
        "type": "free",
        "lookat": [0, 0, 0.72],
        "distance": 2.7,
        "azimuth": -90.0,
        "elevation": -8.0,
    }

    def do_GET(self):  # noqa: N802 - stdlib handler method.
        if self.path == "/health":
            return self._json({"ok": True})
        if self.path == "/status":
            return self._json(
                {
                    "status": "ok",
                    "ready": True,
                    "simulation": {
                        "pose": type(self).pose,
                        "paused": type(self).paused,
                        "actual_speed_factor": 0.0,
                        "model_path": "/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml",
                    },
                }
            )
        if self.path == "/camera":
            return self._json(type(self).camera)
        if self.path == "/visual_frame":
            return self._json({"frame_id": 7, "time": 0.25})
        if self.path == "/visual_scene":
            return self._json({"robots": [{"name": "g1"}]})
        if self.path == "/camera_frame_0.jpg":
            return self._bytes(b"fake-jpeg", "image/jpeg")
        self.send_error(404)

    def do_POST(self):  # noqa: N802 - stdlib handler method.
        body = self.rfile.read(int(self.headers.get("content-length", "0") or 0))
        payload = json.loads(body.decode("utf-8")) if body else {}
        if self.path == "/command":
            command = payload.get("command")
            if command == "pose":
                type(self).pose = payload["pose"]
            elif command == "pause":
                type(self).paused = True
            elif command == "resume":
                type(self).paused = False
            elif command == "reset":
                type(self).pose = "neutral"
                type(self).paused = True
            return self._json({"ok": True, "command": command, "pose": type(self).pose})
        if self.path == "/camera":
            type(self).camera = {**type(self).camera, **payload}
            return self._json(type(self).camera)
        self.send_error(404)

    def log_message(self, _format, *_args):
        return

    def _json(self, value):
        data = json.dumps(value).encode("utf-8")
        self.send_response(200)
        self.send_header("content-type", "application/json")
        self.send_header("content-length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _bytes(self, value: bytes, content_type: str):
        self.send_response(200)
        self.send_header("content-type", content_type)
        self.send_header("content-length", str(len(value)))
        self.end_headers()
        self.wfile.write(value)


class FakeServer:
    def __enter__(self):
        FakeG1Handler.pose = "neutral"
        FakeG1Handler.paused = True
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), FakeG1Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        host, port = self.server.server_address
        self.url = f"http://{host}:{port}"
        return self

    def __exit__(self, *_exc):
        self.server.shutdown()
        self.thread.join(timeout=2)
        self.server.server_close()


class RobotApiTests(unittest.TestCase):
    def test_beginner_robot_api_raises_hand_and_saves_snapshot(self):
        with FakeServer() as fake:
            endpoints = RobotEndpoints(game_control_url=fake.url)
            robot = G1Robot.connect(endpoints=endpoints)

            result = robot.raise_right_hand()
            status = robot.status()

            self.assertTrue(result["ok"])
            self.assertEqual(status.pose, "raise_right_hand")
            self.assertTrue(robot.sim.snapshot_bytes().startswith(b"fake-jpeg"))

    def test_power_user_client_can_drive_camera_and_commands(self):
        with FakeServer() as fake:
            client = SimulatorClient(RobotEndpoints(game_control_url=fake.url))

            client.resume()
            camera = client.orbit(dx=20, dy=-5)
            status = client.status()

            self.assertFalse(status.paused)
            self.assertEqual(camera.raw["action"], "orbit")
            self.assertEqual(camera.raw["dx"], 20.0)

    def test_unitree_style_arm_action_uses_same_simulator(self):
        with FakeServer() as fake:
            previous = os.environ.get("CYBER_G1_GAME_CONTROL_URL")
            os.environ["CYBER_G1_GAME_CONTROL_URL"] = fake.url
            try:
                ChannelFactoryInitialize(3, "cyber-sim")
                arm = G1ArmActionClient()
                arm.SetTimeout(2.0)
                arm.Init()

                code = arm.ExecuteAction(action_map["right hand up"])

                self.assertEqual(code, 0)
                self.assertEqual(arm.last_response["pose"], "raise_right_hand")
                self.assertEqual(current_channel_factory_config().domain_id, 3)
            finally:
                if previous is None:
                    os.environ.pop("CYBER_G1_GAME_CONTROL_URL", None)
                else:
                    os.environ["CYBER_G1_GAME_CONTROL_URL"] = previous


if __name__ == "__main__":
    unittest.main()
