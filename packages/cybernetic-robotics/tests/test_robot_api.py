from __future__ import annotations

import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import unittest

from cybernetic_robotics import G1Robot, ProtocolError, RobotEndpoints, SimulatorClient, UnitreeSession, UnitreeTransportConfig
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, current_channel_factory_config
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_, WirelessController_
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
    audio = {
        "volume": 50,
        "led": {"R": 0, "G": 0, "B": 0},
        "tts": [],
        "streams": {},
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
    last_lowcmd = {}
    lowcmd_meta = {
        "active": False,
        "stale": False,
        "age_seconds": None,
        "watchdog_seconds": 2.0,
    }
    joint_targets = {}

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
                        "pelvis_height": 0.81,
                        "model_path": "/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml",
                        "loco": type(self).loco,
                        "render": {"camera": type(self).camera},
                        "lowcmd": {
                            "motor_cmd_count": type(self).lowcmd_count,
                            **type(self).lowcmd_meta,
                        },
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
                    "mode_pr": int(type(self).last_lowcmd.get("mode_pr", 0)),
                    "crc": int(type(self).last_lowcmd.get("crc", 0)),
                    "imu_state": {
                        "quaternion": [1.0, 0.0, 0.0, 0.0],
                        "gyroscope": [0.0, 0.0, 0.0],
                        "accelerometer": [0.0, 0.0, 0.0],
                    },
                    "motor_state": [
                        {"mode": 1, "q": 0.1 * index, "dq": 0.0, "tau_est": 0.0}
                        for index in range(35)
                    ],
                    "wireless_remote": [127, 0, 255, 64, 0, 0, 0, 0, 0x34, 0x12],
                    "lowcmd": {
                        "motor_cmd_count": type(self).lowcmd_count,
                        **type(self).lowcmd_meta,
                    },
                }
            )
        if self.path == "/joint_state":
            joints = [
                {
                    "motor_index": 22,
                    "joint_name": "right_shoulder_pitch_joint",
                    "q": type(self).joint_targets.get("right_shoulder_pitch_joint", 0.0),
                    "dq": 0.0,
                    "tau_est": 0.0,
                    "limited": True,
                    "range": [-3.1, 2.5],
                },
                {
                    "motor_index": 25,
                    "joint_name": "right_elbow_joint",
                    "q": type(self).joint_targets.get("right_elbow_joint", 0.0),
                    "dq": 0.0,
                    "tau_est": 0.0,
                    "limited": True,
                    "range": [-1.0, 2.0],
                },
            ]
            return self._json(
                {
                    "robot": "g1",
                    "actuator_count": 29,
                    "message_motor_slots": 35,
                    "joints": joints,
                    "by_name": {joint["joint_name"]: joint for joint in joints},
                    "lowcmd": {},
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
                if not isinstance(payload.get("motor_cmd"), list):
                    return self._json({"ok": False, "error": "motor_cmd must be a list"})
                if len(payload.get("motor_cmd", [])) > 35:
                    return self._json({"ok": False, "error": "motor_cmd supports at most 35 entries"})
                if any(not isinstance(item, dict) for item in payload.get("motor_cmd", [])):
                    return self._json({"ok": False, "error": "motor_cmd entries must be objects"})
                type(self).lowcmd_count = len(payload.get("motor_cmd", []))
                type(self).last_lowcmd = payload
                type(self).lowcmd_meta = {
                    "active": True,
                    "stale": False,
                    "age_seconds": 0.0,
                    "watchdog_seconds": 2.0,
                }
                type(self).paused = False
                return self._json(
                    {
                        "ok": True,
                        "control_mode": "lowcmd",
                        "lowcmd": {
                            "motor_cmd_count": type(self).lowcmd_count,
                            "mode_pr": payload.get("mode_pr"),
                            "mode_machine": payload.get("mode_machine"),
                            "crc": payload.get("crc"),
                            **type(self).lowcmd_meta,
                        },
                    }
                )
            elif command == "joint_targets":
                type(self).joint_targets = dict(payload.get("targets") or {})
                type(self).paused = True
                return self._json(
                    {
                        "ok": True,
                        "control_mode": "lowcmd",
                        "paused": True,
                        "lowcmd": {
                            "source": "joint_targets",
                            "joint_targets": type(self).joint_targets,
                        },
                    }
                )
            elif command == "audio":
                action = payload.get("action")
                if action == "set_volume":
                    type(self).audio["volume"] = int(payload["volume"])
                elif action == "get_volume":
                    pass
                elif action == "set_rgb_led":
                    type(self).audio["led"] = {"R": payload["R"], "G": payload["G"], "B": payload["B"]}
                elif action == "tts":
                    type(self).audio["tts"].append(payload)
                elif action == "start_play":
                    type(self).audio["streams"][payload["stream_id"]] = payload
                elif action == "stop_play":
                    type(self).audio["streams"].clear()
                return self._json({"ok": True, "command": command, "audio": type(self).audio, **payload})
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
        FakeG1Handler.last_lowcmd = {}
        FakeG1Handler.lowcmd_meta = {
            "active": False,
            "stale": False,
            "age_seconds": None,
            "watchdog_seconds": 2.0,
        }
        FakeG1Handler.joint_targets = {}
        FakeG1Handler.audio = {
            "volume": 50,
            "led": {"R": 0, "G": 0, "B": 0},
            "tts": [],
            "streams": {},
        }
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
                lowcmd.mode_pr = 1
                lowcmd.mode_machine = 500
                lowcmd.crc = 12345
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
                self.assertEqual(FakeG1Handler.last_lowcmd["mode_pr"], 1)
                self.assertEqual(FakeG1Handler.last_lowcmd["mode_machine"], 500)
                self.assertEqual(FakeG1Handler.last_lowcmd["crc"], 12345)
                self.assertEqual(lowstate.mode_pr, 1)
                self.assertEqual(lowstate.mode_machine, 1)
                self.assertEqual(lowstate.crc, 12345)
                self.assertTrue(lowstate.lowcmd_active)
                self.assertFalse(lowstate.lowcmd_stale)
                self.assertAlmostEqual(lowstate.lowcmd_age_seconds, 0.0)
                self.assertAlmostEqual(lowstate.lowcmd_watchdog_seconds, 2.0)
                self.assertAlmostEqual(lowstate.motor_state[1].q, 0.1)

                status = G1Robot.connect(endpoints=RobotEndpoints(game_control_url=fake.url)).status()
                self.assertTrue(status.lowcmd_active)
                self.assertFalse(status.lowcmd_stale)
                self.assertAlmostEqual(status.lowcmd_age_seconds or 0.0, 0.0)
            finally:
                if previous is None:
                    os.environ.pop("CYBER_G1_GAME_CONTROL_URL", None)
                else:
                    os.environ["CYBER_G1_GAME_CONTROL_URL"] = previous

    def test_unitree_session_diagnostics_reports_transport_and_topics(self):
        with FakeServer() as fake:
            endpoints = RobotEndpoints(game_control_url=fake.url)
            config = UnitreeTransportConfig.from_env(endpoints)
            diagnostics = UnitreeSession(config, SimulatorClient(endpoints)).diagnostics()

            self.assertTrue(diagnostics["ok"])
            self.assertTrue(diagnostics["implemented"])
            self.assertEqual(diagnostics["config"]["transport"], "local_http")
            self.assertEqual(diagnostics["config"]["mode"], "sim")
            self.assertEqual(diagnostics["config"]["dds_domain_id"], 1)
            self.assertEqual(diagnostics["config"]["network_interface"], "lo")
            self.assertTrue(diagnostics["simulator"]["reachable"])
            self.assertTrue(diagnostics["topics"]["rt/lowstate"]["available"])
            self.assertIn("rt/lowcmd", diagnostics["topics"])

    def test_unitree_session_diagnostics_surfaces_real_mode_safety_gate(self):
        previous = {
            key: os.environ.get(key)
            for key in (
                "CYBER_UNITREE_MODE",
                "CYBER_UNITREE_TRANSPORT",
                "CYBER_UNITREE_DDS_DOMAIN",
                "CYBER_UNITREE_NETWORK_INTERFACE",
                "CYBER_UNITREE_REAL_UNLOCK",
            )
        }
        os.environ["CYBER_UNITREE_MODE"] = "real"
        os.environ["CYBER_UNITREE_TRANSPORT"] = "dds"
        os.environ.pop("CYBER_UNITREE_NETWORK_INTERFACE", None)
        os.environ.pop("CYBER_UNITREE_REAL_UNLOCK", None)
        try:
            with FakeServer() as fake:
                endpoints = RobotEndpoints(game_control_url=fake.url)
                config = UnitreeTransportConfig.from_env(endpoints)
                diagnostics = UnitreeSession(config, SimulatorClient(endpoints)).diagnostics()

            self.assertFalse(diagnostics["ok"])
            self.assertFalse(diagnostics["implemented"])
            self.assertEqual(diagnostics["config"]["mode"], "real")
            self.assertEqual(diagnostics["config"]["transport"], "dds")
            self.assertEqual(diagnostics["config"]["dds_domain_id"], 0)
            self.assertIsNone(diagnostics["config"]["network_interface"])
            self.assertTrue(any("requires CYBER_UNITREE_NETWORK_INTERFACE" in item for item in diagnostics["warnings"]))
            self.assertTrue(any("real mode is locked" in item for item in diagnostics["warnings"]))
        finally:
            for key, value in previous.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value

    def test_lowcmd_rejects_malformed_command_lists(self):
        with FakeServer() as fake:
            client = SimulatorClient(RobotEndpoints(game_control_url=fake.url))

            with self.assertRaises(ProtocolError):
                client.command("lowcmd", motor_cmd="not-a-list")
            with self.assertRaises(ProtocolError):
                client.command("lowcmd", motor_cmd=[{} for _ in range(36)])
            with self.assertRaises(ProtocolError):
                client.command("lowcmd", motor_cmd=[{"mode": 0}, "bad-entry"])

    def test_unitree_go_telemetry_channels_use_same_simulator(self):
        with FakeServer() as fake:
            previous = os.environ.get("CYBER_G1_GAME_CONTROL_URL")
            os.environ["CYBER_G1_GAME_CONTROL_URL"] = fake.url
            try:
                loco = LocoClient()
                loco.Init()
                self.assertEqual(loco.Start(), 0)
                self.assertEqual(loco.Move(0.2, -0.1, 0.3), 0)

                sport_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
                sport_sub.Init()
                sport = sport_sub.Read()

                wireless_sub = ChannelSubscriber("rt/wirelesscontroller", WirelessController_)
                wireless_sub.Init()
                wireless = wireless_sub.Read()

                self.assertEqual(sport.mode, 500 & 0xFF)
                self.assertAlmostEqual(sport.body_height, 0.81)
                self.assertEqual(sport.velocity, [0.2, -0.1, 0.3])
                self.assertAlmostEqual(sport.yaw_speed, 0.3)
                self.assertAlmostEqual(wireless.lx, 1.0)
                self.assertAlmostEqual(wireless.rx, -1 / 127)
                self.assertEqual(wireless.keys, 0x1234)
            finally:
                if previous is None:
                    os.environ.pop("CYBER_G1_GAME_CONTROL_URL", None)
                else:
                    os.environ["CYBER_G1_GAME_CONTROL_URL"] = previous

    def test_named_joint_targets_use_simulator_protocol(self):
        with FakeServer() as fake:
            endpoints = RobotEndpoints(game_control_url=fake.url)
            robot = G1Robot.connect(endpoints=endpoints)

            before = robot.joint_state()
            result = robot.apply_joint_targets(
                {
                    "right_shoulder_pitch_joint": -1.2,
                    "right_elbow_joint": 0.8,
                }
            )
            after = robot.joint_state()

            self.assertEqual(before["by_name"]["right_elbow_joint"]["q"], 0.0)
            self.assertTrue(result["ok"])
            self.assertEqual(result["lowcmd"]["source"], "joint_targets")
            self.assertEqual(after["by_name"]["right_shoulder_pitch_joint"]["q"], -1.2)
            self.assertEqual(after["by_name"]["right_elbow_joint"]["q"], 0.8)

    def test_unitree_style_audio_client_records_simulator_intent(self):
        with FakeServer() as fake:
            previous = os.environ.get("CYBER_G1_GAME_CONTROL_URL")
            os.environ["CYBER_G1_GAME_CONTROL_URL"] = fake.url
            try:
                audio = AudioClient()
                audio.SetTimeout(2.0)
                audio.Init()

                self.assertEqual(audio.TtsMaker("hello robot", 0), 0)
                self.assertEqual(audio.SetVolume(33), 0)
                self.assertEqual(audio.GetVolume(), (0, {"volume": 33}))
                self.assertEqual(audio.LedControl(1, 2, 3), 0)
                stream_code, stream_response = audio.PlayStream("cyber", "stream-1", b"1234")
                self.assertEqual(stream_code, 0)
                self.assertEqual(stream_response["pcm_bytes"], 4)
                self.assertEqual(audio.PlayStop("cyber"), 0)

                self.assertEqual(FakeG1Handler.audio["tts"][0]["text"], "hello robot")
                self.assertEqual(FakeG1Handler.audio["led"], {"R": 1, "G": 2, "B": 3})
            finally:
                if previous is None:
                    os.environ.pop("CYBER_G1_GAME_CONTROL_URL", None)
                else:
                    os.environ["CYBER_G1_GAME_CONTROL_URL"] = previous


if __name__ == "__main__":
    unittest.main()
