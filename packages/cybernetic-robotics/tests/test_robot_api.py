from __future__ import annotations

import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import unittest

from cybernetic_robotics import G1Robot, RobotEndpoints, SimulatorClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, current_channel_factory_config
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber


class FakeG1Handler(BaseHTTPRequestHandler):
    pose = "neutral"
    paused = True
    loco = {
        "fsm_id": 1,
        "fsm_mode": "damp",
        "velocity": [0.0, 0.0, 0.0],
        "balance_mode": 0,
        "stand_height": None,
    }
    camera = {
        "cameraId": 0,
        "type": "free",
        "lookat": [0, 0, 0.72],
        "distance": 2.7,
        "azimuth": -90.0,
        "elevation": -8.0,
    }
    lowcmd_count = 0

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
        if self.path == "/lowstate":
            return self._json(
                {
                    "mode_machine": 1,
                    "imu_state": {
                        "quaternion": [1.0, 0.0, 0.0, 0.0],
                        "gyroscope": [0.0, 0.0, 0.0],
                        "accelerometer": [0.0, 0.0, 0.0],
                    },
                    "motor_state": [
                        {"mode": 1, "q": 0.1 * index, "dq": 0.0, "tau_est": 0.0}
                        for index in range(35)
                    ],
                    "lowcmd": {"motor_cmd_count": type(self).lowcmd_count},
                }
            )
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
            elif command == "loco":
                action = payload.get("action")
                if action == "set_velocity":
                    type(self).loco["velocity"] = payload["velocity"]
                    type(self).paused = False
                elif action == "set_fsm_id":
                    type(self).loco["fsm_id"] = payload["fsm_id"]
                    type(self).loco["fsm_mode"] = payload.get("mode")
                elif action == "get_fsm_id":
                    return self._json({"ok": True, "fsm_id": type(self).loco["fsm_id"], "loco": type(self).loco})
                elif action in {"wave_hand", "shake_hand", "set_arm_task"}:
                    type(self).pose = "raise_right_hand"
                return self._json({"ok": True, "command": command, "action": action, "loco": type(self).loco})
            elif command == "lowcmd":
                type(self).lowcmd_count = len(payload.get("motor_cmd", []))
                type(self).paused = False
                return self._json(
                    {
                        "ok": True,
                        "control_mode": "lowcmd",
                        "lowcmd": {"motor_cmd_count": type(self).lowcmd_count},
                    }
                )
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
        FakeG1Handler.loco = {
            "fsm_id": 1,
            "fsm_mode": "damp",
            "velocity": [0.0, 0.0, 0.0],
            "balance_mode": 0,
            "stand_height": None,
        }
        FakeG1Handler.lowcmd_count = 0
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

    def test_unitree_style_loco_client_uses_same_simulator(self):
        with FakeServer() as fake:
            previous = os.environ.get("CYBER_G1_GAME_CONTROL_URL")
            os.environ["CYBER_G1_GAME_CONTROL_URL"] = fake.url
            try:
                ChannelFactoryInitialize(3, "cyber-sim")
                loco = LocoClient()
                loco.SetTimeout(2.0)
                loco.Init()

                move_code = loco.Move(0.25, 0.0, 0.1)
                fsm_code = loco.Start()
                get_code, fsm_id = loco.GetFsmId()

                self.assertEqual(move_code, 0)
                self.assertEqual(fsm_code, 0)
                self.assertEqual(get_code, 0)
                self.assertEqual(fsm_id, 500)
                self.assertEqual(loco.last_response["loco"]["fsm_id"], 500)
            finally:
                if previous is None:
                    os.environ.pop("CYBER_G1_GAME_CONTROL_URL", None)
                else:
                    os.environ["CYBER_G1_GAME_CONTROL_URL"] = previous

    def test_unitree_style_lowcmd_and_lowstate_channels_use_same_simulator(self):
        with FakeServer() as fake:
            previous = os.environ.get("CYBER_G1_GAME_CONTROL_URL")
            os.environ["CYBER_G1_GAME_CONTROL_URL"] = fake.url
            try:
                lowcmd = unitree_hg_msg_dds__LowCmd_()
                lowcmd.motor_cmd[22].mode = 1
                lowcmd.motor_cmd[22].q = -1.0
                lowcmd.motor_cmd[22].kp = 40.0
                lowcmd.motor_cmd[22].kd = 1.0

                publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
                publisher.Init()
                self.assertTrue(publisher.Write(lowcmd))

                subscriber = ChannelSubscriber("rt/lowstate", LowState_)
                subscriber.Init()
                lowstate = subscriber.Read()

                self.assertEqual(FakeG1Handler.lowcmd_count, 35)
                self.assertEqual(lowstate.mode_machine, 1)
                self.assertAlmostEqual(lowstate.motor_state[1].q, 0.1)
            finally:
                if previous is None:
                    os.environ.pop("CYBER_G1_GAME_CONTROL_URL", None)
                else:
                    os.environ["CYBER_G1_GAME_CONTROL_URL"] = previous


if __name__ == "__main__":
    unittest.main()
