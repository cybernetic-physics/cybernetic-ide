from __future__ import annotations

import json
import os
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import patch

from cybernetic_robotics import (
    G1Robot,
    OfficialG1Sim,
    ProtocolError,
    RobotEndpoints,
    SceneWorkspace,
    SimulatorClient,
    UnitreeSession,
    UnitreeTransportConfig,
    evaluate_lowstate_safety,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, current_channel_factory_config
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
from unitree_sdk2py.g1.agv.g1_agv_client import AgvClient
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
        "swing_height": None,
        "phase": [0.0, 0.0],
        "continuous_move": False,
        "speed_mode": 0,
        "control_owner": "internal",
        "internal_mode": 0,
    }
    motion_switcher = {
        "name": "",
        "silent": False,
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
                        {"mode": 1, "q": 0.1 * index, "dq": 0.0, "tau_est": 0.0, "temperature": [35, 40]}
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
                elif action == "get_fsm_mode":
                    return self._json({"ok": True, "fsm_mode": type(self).loco["fsm_mode"], "loco": type(self).loco})
                elif action == "get_balance_mode":
                    return self._json({"ok": True, "balance_mode": type(self).loco["balance_mode"], "loco": type(self).loco})
                elif action == "get_swing_height":
                    return self._json({"ok": True, "swing_height": type(self).loco["swing_height"], "loco": type(self).loco})
                elif action == "get_stand_height":
                    return self._json({"ok": True, "stand_height": type(self).loco["stand_height"], "loco": type(self).loco})
                elif action == "get_phase":
                    return self._json({"ok": True, "phase": type(self).loco["phase"], "loco": type(self).loco})
                elif action == "set_balance_mode":
                    type(self).loco["balance_mode"] = payload["balance_mode"]
                elif action == "set_swing_height":
                    type(self).loco["swing_height"] = payload["swing_height"]
                elif action == "set_stand_height":
                    type(self).loco["stand_height"] = payload["stand_height"]
                elif action == "set_speed_mode":
                    type(self).loco["speed_mode"] = payload["speed_mode"]
                elif action == "switch_move_mode":
                    type(self).loco["continuous_move"] = payload["continuous_move"]
                elif action == "switch_to_user_ctrl":
                    type(self).loco["control_owner"] = "user"
                elif action == "switch_to_internal_ctrl":
                    type(self).loco["control_owner"] = "internal"
                    type(self).loco["internal_mode"] = payload["internal_mode"]
                elif action in {"wave_hand", "shake_hand", "set_arm_task"}:
                    type(self).pose = "raise_right_hand"
                return self._json({"ok": True, "command": command, "action": action, "loco": type(self).loco})
            elif command == "motion_switcher":
                action = payload.get("action", "check_mode")
                if action == "select_mode":
                    type(self).motion_switcher["name"] = payload["name"]
                elif action == "release_mode":
                    type(self).motion_switcher["name"] = ""
                    type(self).loco["fsm_id"] = 1
                    type(self).loco["fsm_mode"] = "damp"
                elif action == "set_silent":
                    type(self).motion_switcher["silent"] = bool(payload.get("silent"))
                elif action == "get_silent":
                    return self._json({"ok": True, "silent": type(self).motion_switcher["silent"]})
                return self._json({"ok": True, "command": command, "action": action, "mode": type(self).motion_switcher})
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
            "swing_height": None,
            "stand_height": None,
            "phase": [0.0, 0.0],
            "continuous_move": False,
            "speed_mode": 0,
            "control_owner": "internal",
            "internal_mode": 0,
        }
        FakeG1Handler.motion_switcher = {
            "name": "",
            "silent": False,
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

    def test_beginner_robot_api_safety_stop_reports_steps(self):
        with FakeServer() as fake:
            endpoints = RobotEndpoints(game_control_url=fake.url)
            robot = G1Robot.connect(endpoints=endpoints)

            robot.raise_right_hand()
            result = robot.safety_stop()
            status = robot.status()

            self.assertTrue(result["ok"])
            self.assertEqual([step["step"] for step in result["steps"]], [
                "release_motion_mode",
                "damp_locomotion",
                "neutral_pose",
                "pause",
            ])
            self.assertEqual(status.pose, "neutral")
            self.assertTrue(status.paused)
            self.assertEqual(FakeG1Handler.loco["fsm_mode"], "damp")
            self.assertEqual(FakeG1Handler.motion_switcher["name"], "")

    def test_beginner_robot_api_safety_check_reports_safe_lowstate(self):
        with FakeServer() as fake:
            endpoints = RobotEndpoints(game_control_url=fake.url)
            robot = G1Robot.connect(endpoints=endpoints)

            result = robot.safety_check()

            self.assertTrue(result["ok"])
            self.assertTrue(result["safe_to_command"])
            self.assertEqual(result["recommendation"], "continue")
            self.assertFalse(result["failed_checks"])
            self.assertIn("bad_orientation", {check["name"] for check in result["checks"]})

    def test_safety_check_flags_unitree_termination_conditions(self):
        lowstate = {
            "imu_state": {
                "quaternion": [0.0, 1.0, 0.0, 0.0],
                "gyroscope": [0.0, 7.5, 0.0],
            },
            "motor_state": [
                {"dq": 12.0, "temperature": [90, 130]},
                {"dq": 0.0, "temperature": [35, 40]},
            ],
            "lowcmd": {"stale": True},
        }
        status = {"simulation": {"fallen": True}}

        result = evaluate_lowstate_safety(lowstate, status=status)

        self.assertFalse(result["ok"])
        self.assertFalse(result["safe_to_command"])
        failed = {check["name"] for check in result["failed_checks"]}
        self.assertIn("bad_orientation", failed)
        self.assertIn("joint_vel_out_of_limit", failed)
        self.assertIn("ang_vel_out_of_limit", failed)
        self.assertIn("motor_casing_overheat", failed)
        self.assertIn("motor_winding_overheat", failed)
        self.assertIn("lowcmd_stale", failed)
        self.assertIn("fallen", failed)
        self.assertEqual(result["recommendation"], "call safety_stop before issuing more motion")

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
                high_five_code = arm.ExecuteAction(action_map["high five"])
                list_code, actions = arm.GetActionList()

                self.assertEqual(code, 0)
                self.assertEqual(high_five_code, 0)
                self.assertEqual(list_code, 0)
                self.assertTrue(all(action["simulated"] for action in actions))
                self.assertEqual(arm.last_response["pose"], "high_five")
                self.assertEqual(current_channel_factory_config().domain_id, 3)
            finally:
                if previous is None:
                    os.environ.pop("CYBER_G1_GAME_CONTROL_URL", None)
                else:
                    os.environ["CYBER_G1_GAME_CONTROL_URL"] = previous

    def test_unitree_style_arm_action_routes_to_official_dds_session(self):
        previous = {
            "CYBER_UNITREE_TRANSPORT": os.environ.get("CYBER_UNITREE_TRANSPORT"),
            "CYBER_UNITREE_MODE": os.environ.get("CYBER_UNITREE_MODE"),
        }
        os.environ["CYBER_UNITREE_TRANSPORT"] = "dds"
        os.environ["CYBER_UNITREE_MODE"] = "sim"

        class FakeOfficial:
            @classmethod
            def discover(cls, *, timeout: int = 300):
                return cls(timeout)

            def __init__(self, timeout: int):
                self.timeout = timeout

            def raise_right_hand_session(self):
                return {
                    "ok": True,
                    "source": "official_unitree_mujoco_managed_session",
                    "moved_joints": ["right_shoulder_pitch", "right_elbow"],
                }

        try:
            with patch("cybernetic_robotics.official.OfficialG1Sim", FakeOfficial):
                arm = G1ArmActionClient()
                arm.SetTimeout(2.0)
                arm.Init()

                code = arm.ExecuteAction(action_map["right hand up"])

            self.assertEqual(code, 0)
            self.assertTrue(arm.last_response["ok"])
            self.assertEqual(arm.last_response["transport"], "dds")
            self.assertEqual(arm.last_response["unitree_action_id"], action_map["right hand up"])
            self.assertEqual(arm.last_response["moved_joints"], ["right_shoulder_pitch", "right_elbow"])
        finally:
            for key, value in previous.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value

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
                mode_code, fsm_mode = loco.GetFsmMode()
                self.assertEqual(loco.SetBalanceMode(2), 0)
                balance_code, balance_mode = loco.GetBalanceMode()
                self.assertEqual(loco.SetSwingHeight(0.09), 0)
                swing_code, swing_height = loco.GetSwingHeight()
                self.assertEqual(loco.SetStandHeight(0.18), 0)
                stand_code, stand_height = loco.GetStandHeight()
                phase_code, phase = loco.GetPhase()
                self.assertEqual(loco.ContinuousGait(True), 0)
                self.assertEqual(loco.SwitchMoveMode(True), 0)
                continuous_move_code = loco.Move(0.1, 0.0, 0.0)
                self.assertEqual(loco.SetSpeedMode(2), 0)
                self.assertEqual(loco.SwitchToUserCtrl(), 0)
                self.assertEqual(loco.SwitchToInternalCtrl(2), 0)

                self.assertEqual(move_code, 0)
                self.assertEqual(fsm_code, 0)
                self.assertEqual(get_code, 0)
                self.assertEqual(mode_code, 0)
                self.assertEqual(balance_code, 0)
                self.assertEqual(swing_code, 0)
                self.assertEqual(stand_code, 0)
                self.assertEqual(phase_code, 0)
                self.assertEqual(fsm_id, 500)
                self.assertEqual(fsm_mode, "start")
                self.assertEqual(balance_mode, 2)
                self.assertAlmostEqual(swing_height, 0.09)
                self.assertAlmostEqual(stand_height, 0.18)
                self.assertEqual(phase, [0.0, 0.0])
                self.assertEqual(continuous_move_code, 0)
                self.assertEqual(FakeG1Handler.loco["velocity"], [0.1, 0.0, 0.0])
                self.assertTrue(FakeG1Handler.loco["continuous_move"])
                self.assertEqual(FakeG1Handler.loco["speed_mode"], 2)
                self.assertEqual(FakeG1Handler.loco["control_owner"], "internal")
                self.assertEqual(FakeG1Handler.loco["internal_mode"], 2)
                self.assertEqual(loco.last_response["loco"]["fsm_id"], 500)
            finally:
                if previous is None:
                    os.environ.pop("CYBER_G1_GAME_CONTROL_URL", None)
                else:
                    os.environ["CYBER_G1_GAME_CONTROL_URL"] = previous

    def test_unitree_style_g1_agv_client_uses_simulator_motion(self):
        with FakeServer() as fake:
            previous = os.environ.get("CYBER_G1_GAME_CONTROL_URL")
            os.environ["CYBER_G1_GAME_CONTROL_URL"] = fake.url
            try:
                agv = AgvClient()
                agv.SetTimeout(2.0)
                agv.Init()

                move_code = agv.Move(2.0, 0.4, 1.4)
                height_code = agv.HeightAdjust(2.5)

                self.assertEqual(move_code, 0)
                self.assertEqual(height_code, 0)
                self.assertEqual(agv.api_version, "1.0.0.0")
                self.assertEqual(FakeG1Handler.loco["velocity"], [1.5, 0.0, 0.6])
                self.assertEqual(agv.last_response["height_velocity"], 1.0)
            finally:
                if previous is None:
                    os.environ.pop("CYBER_G1_GAME_CONTROL_URL", None)
                else:
                    os.environ["CYBER_G1_GAME_CONTROL_URL"] = previous

    def test_motion_switcher_client_uses_simulator_state(self):
        with FakeServer() as fake:
            previous = os.environ.get("CYBER_G1_GAME_CONTROL_URL")
            os.environ["CYBER_G1_GAME_CONTROL_URL"] = fake.url
            try:
                switcher = MotionSwitcherClient()
                switcher.SetTimeout(2.0)
                switcher.Init()

                select_code, _ = switcher.SelectMode("ai")
                check_code, mode = switcher.CheckMode()
                silent_code, _ = switcher.SetSilent(True)
                get_silent_code, silent = switcher.GetSilent()
                release_code, _ = switcher.ReleaseMode()
                final_code, final_mode = switcher.CheckMode()

                self.assertEqual(select_code, 0)
                self.assertEqual(check_code, 0)
                self.assertEqual(mode, {"name": "ai"})
                self.assertEqual(silent_code, 0)
                self.assertEqual(get_silent_code, 0)
                self.assertTrue(silent)
                self.assertEqual(release_code, 0)
                self.assertEqual(final_code, 0)
                self.assertEqual(final_mode, {"name": ""})
                self.assertEqual(switcher.last_response["mode"]["name"], "")
                self.assertEqual(FakeG1Handler.loco["fsm_mode"], "damp")
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

    def test_unitree_session_provider_status_reports_local_http_provider(self):
        with FakeServer() as fake:
            endpoints = RobotEndpoints(game_control_url=fake.url)
            config = UnitreeTransportConfig(transport="local_http", mode="sim", endpoints=endpoints)
            provider = UnitreeSession(config, SimulatorClient(endpoints)).provider_status()

            self.assertTrue(provider["ok"])
            self.assertTrue(provider["implemented"])
            self.assertEqual(provider["provider"], "local_http_simulator")
            self.assertEqual(provider["motion"]["locomotion"], "kinematic_base_velocity")
            self.assertTrue(provider["diagnostics_summary"]["simulator_reachable"])

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

    def test_unitree_session_diagnostics_uses_official_sidecar_for_sim_dds(self):
        class FakeOfficial:
            def status(self):
                return {
                    "ok": True,
                    "source": "official_unitree_mujoco_sdk2_sidecar",
                    "sdk2_probe": {
                        "domain_initialized": True,
                        "domain": 1,
                        "network_interface": "lo",
                        "channels": {
                            "rt/lowcmd": {"role": "publisher", "created": True, "sample_motor_count": 35},
                            "rt/lowstate": {"role": "subscriber", "created": True},
                        },
                    },
                    "official_mujoco_peer": {"binary_exists": True, "scene_exists": True},
                    "expected_topics": ["rt/lowcmd", "rt/lowstate"],
                    "next_step": "launch official unitree_mujoco",
                }

        with FakeServer() as fake:
            endpoints = RobotEndpoints(game_control_url=fake.url)
            config = UnitreeTransportConfig(transport="dds", mode="sim", endpoints=endpoints)
            diagnostics = UnitreeSession(config, SimulatorClient(endpoints), official=FakeOfficial()).diagnostics()

        self.assertTrue(diagnostics["ok"])
        self.assertTrue(diagnostics["implemented"])
        self.assertTrue(diagnostics["official_sidecar"]["ok"])
        self.assertTrue(diagnostics["official_sidecar"]["domain_initialized"])
        self.assertEqual(diagnostics["topics"]["rt/lowcmd"]["source"], "official_sdk2_sidecar")
        self.assertEqual(diagnostics["topics"]["rt/lowcmd"]["sample_motor_count"], 35)
        self.assertTrue(diagnostics["topics"]["rt/lowstate"]["created"])

    def test_unitree_session_provider_status_reports_dds_sim_provider(self):
        class FakeOfficial:
            def status(self):
                return {
                    "ok": True,
                    "source": "official_unitree_mujoco_sdk2_sidecar",
                    "sdk2_probe": {
                        "domain_initialized": True,
                        "domain": 1,
                        "network_interface": "lo",
                        "channels": {
                            "rt/lowcmd": {"role": "publisher", "created": True, "sample_motor_count": 35},
                            "rt/lowstate": {"role": "subscriber", "created": True},
                        },
                    },
                    "official_mujoco_peer": {"binary_exists": True, "scene_exists": True},
                    "expected_topics": ["rt/lowcmd", "rt/lowstate"],
                    "next_step": "launch official unitree_mujoco",
                }

        with FakeServer() as fake:
            endpoints = RobotEndpoints(game_control_url=fake.url)
            config = UnitreeTransportConfig(transport="dds", mode="sim", endpoints=endpoints)
            provider = UnitreeSession(config, SimulatorClient(endpoints), official=FakeOfficial()).provider_status()

        self.assertTrue(provider["ok"])
        self.assertTrue(provider["implemented"])
        self.assertEqual(provider["provider"], "official_mujoco_dds_simulator")
        self.assertEqual(provider["motion"]["arm_actions"], "managed_official_mujoco_session_for_supported_poses")
        self.assertTrue(provider["diagnostics_summary"]["official_sidecar_ok"])

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

    def test_scene_workspace_adds_box_to_generated_scene_copy(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            asset_root = root / ".runtime/unitree-g1-mujoco/unitree_mujoco"
            model_path = asset_root / "unitree_robots/g1/scene_29dof.xml"
            model_path.parent.mkdir(parents=True)
            model_path.write_text(
                "<mujoco>\n"
                "  <worldbody>\n"
                "    <body name=\"g1\"/>\n"
                "  </worldbody>\n"
                "</mujoco>\n",
                encoding="utf-8",
            )
            compose_env = root / ".runtime/unitree-g1-mujoco/compose.env"
            compose_env.parent.mkdir(parents=True, exist_ok=True)
            compose_env.write_text(
                "\n".join(
                    [
                        "UNITREE_G1_MUJOCO_IMAGE=cyber/unitree-g1-mujoco-protocol:0.1.0",
                        f"UNITREE_G1_MUJOCO_ASSET_ROOT={asset_root}",
                        "UNITREE_G1_MODEL_PATH=/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml",
                        "UNITREE_G1_ROBOT_NAME=g1",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            output, container_path = SceneWorkspace(root).add_box(
                "agent_obstacle",
                position=(0.8, 0.0, 0.08),
                size=(0.12, 0.12, 0.08),
                rgba=(0.9, 0.2, 0.1, 1.0),
                activate=True,
            )

            self.assertEqual(
                container_path,
                "/opt/unitree_mujoco/cybernetic_scenes/g1_agent_obstacle.xml",
            )
            self.assertTrue(output.exists())
            generated = output.read_text(encoding="utf-8")
            self.assertIn('body name="agent_obstacle"', generated)
            self.assertIn('geom name="agent_obstacle_geom" type="box"', generated)
            self.assertIn(
                "UNITREE_G1_MODEL_PATH=/opt/unitree_mujoco/cybernetic_scenes/g1_agent_obstacle.xml",
                compose_env.read_text(encoding="utf-8"),
            )

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

    def test_official_g1_sim_raise_hand_builds_sidecar_probe_command(self):
        captured: dict[str, object] = {}

        def fake_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
            captured["args"] = args
            captured["cwd"] = cwd
            captured["timeout"] = timeout
            return subprocess.CompletedProcess(
                args,
                0,
                stdout=json.dumps(
                    {
                        "arm_pose_probe": {
                            "ok": True,
                            "preset": "raise_right_hand",
                            "moved_joints": ["right_shoulder_pitch", "right_elbow"],
                            "lowcmd_write_successes": 180,
                        }
                    }
                ),
                stderr="",
            )

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / ".runtime/unitree-g1-sdk2").mkdir(parents=True)
            (root / ".runtime/unitree-g1-sdk2/compose.env").write_text("UNITREE=test\n", encoding="utf-8")
            (root / "overlays/unitree-g1-sdk2-sidecar").mkdir(parents=True)
            (root / "overlays/unitree-g1-sdk2-sidecar/compose.yaml").write_text("services: {}\n", encoding="utf-8")
            official = OfficialG1Sim(root, timeout=123, _runner=fake_runner)

            result = official.raise_right_hand(frames=999, joint_deltas={"right_elbow": 0.75})

        self.assertTrue(result["ok"])
        self.assertEqual(result["moved_joints"], ["right_shoulder_pitch", "right_elbow"])
        self.assertEqual(result["lowcmd_write_successes"], 180)
        self.assertEqual(result["parameters"]["frames"], 600)
        self.assertEqual(result["parameters"]["joint_deltas"], {"right_elbow": 0.5})
        self.assertEqual(captured["timeout"], 123)
        command = " ".join(captured["args"])
        self.assertIn("CYBER_UNITREE_ACTION=probe_official_mujoco_arm_pose", command)
        self.assertIn("CYBER_UNITREE_ARM_POSE_PRESET=raise_right_hand", command)
        self.assertIn("CYBER_UNITREE_ARM_POSE_FRAMES=600", command)
        self.assertIn("unitree-g1-sdk2-sidecar", command)

    def test_official_g1_sim_status_reports_sdk2_sidecar_readiness(self):
        def fake_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
            return subprocess.CompletedProcess(
                args,
                0,
                stdout=json.dumps(
                    {
                        "expected_topics": ["rt/lowcmd", "rt/lowstate"],
                        "sdk2_probe": {
                            "domain_initialized": True,
                            "channels": {
                                "rt/lowcmd": {"created": True, "sample_motor_count": 35},
                                "rt/lowstate": {"created": True},
                            },
                        },
                        "official_mujoco_peer": {"binary_exists": True, "scene_exists": True},
                    }
                ),
                stderr="",
            )

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / ".runtime/unitree-g1-sdk2").mkdir(parents=True)
            (root / ".runtime/unitree-g1-sdk2/compose.env").write_text("UNITREE=test\n", encoding="utf-8")
            (root / "overlays/unitree-g1-sdk2-sidecar").mkdir(parents=True)
            (root / "overlays/unitree-g1-sdk2-sidecar/compose.yaml").write_text("services: {}\n", encoding="utf-8")
            status = OfficialG1Sim(root, _runner=fake_runner).status()

        self.assertTrue(status["ok"])
        self.assertEqual(status["source"], "official_unitree_mujoco_sdk2_sidecar")
        self.assertEqual(status["expected_topics"], ["rt/lowcmd", "rt/lowstate"])
        self.assertTrue(status["official_mujoco_peer"]["binary_exists"])

    def test_official_g1_sim_manages_named_session_lifecycle(self):
        checked_calls = []
        unchecked_calls = []

        def fake_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
            checked_calls.append(args)
            return subprocess.CompletedProcess(args, 0, stdout="container-id\n", stderr="")

        def fake_unchecked_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
            unchecked_calls.append(args)
            command = " ".join(args)
            if "inspect unitree-g1-sdk2-session" in command:
                return subprocess.CompletedProcess(
                    args,
                    0,
                    stdout=json.dumps(
                        {
                            "Running": True,
                            "Status": "running",
                            "ExitCode": 0,
                            "StartedAt": "2026-07-07T00:00:00Z",
                            "FinishedAt": "0001-01-01T00:00:00Z",
                        }
                    ),
                    stderr="",
                )
            if "logs --tail" in command:
                return subprocess.CompletedProcess(
                    args,
                    0,
                    stdout=json.dumps(
                        {
                            "action": "serve_official_mujoco",
                            "ok": True,
                            "peer_started": True,
                            "read_topics": ["rt/lowstate"],
                            "write_topics": ["rt/lowcmd"],
                        }
                    ),
                    stderr="",
                )
            return subprocess.CompletedProcess(args, 0, stdout="removed\n", stderr="")

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / ".runtime/unitree-g1-sdk2").mkdir(parents=True)
            (root / ".runtime/unitree-g1-sdk2/compose.env").write_text("UNITREE=test\n", encoding="utf-8")
            (root / "overlays/unitree-g1-sdk2-sidecar").mkdir(parents=True)
            (root / "overlays/unitree-g1-sdk2-sidecar/compose.yaml").write_text("services: {}\n", encoding="utf-8")
            official = OfficialG1Sim(root, timeout=123, _runner=fake_runner, _unchecked_runner=fake_unchecked_runner)

            started = official.start_session(wait=False)
            status = official.session_status()

        self.assertTrue(started["ok"])
        self.assertEqual(started["container"], "unitree-g1-sdk2-session")
        self.assertTrue(status["ready"])
        self.assertEqual(status["ready_report"]["read_topics"], ["rt/lowstate"])
        start_command = " ".join(checked_calls[0])
        self.assertIn("run -d --name unitree-g1-sdk2-session", start_command)
        self.assertIn("CYBER_UNITREE_ACTION=serve_official_mujoco", start_command)
        self.assertTrue(any("rm -f unitree-g1-sdk2-session" in " ".join(call) for call in unchecked_calls))

    def test_official_g1_sim_stop_session_removes_named_container(self):
        unchecked_calls = []

        def fake_unchecked_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
            unchecked_calls.append(args)
            if args[:2] == ["docker", "rm"]:
                return subprocess.CompletedProcess(args, 0, stdout="unitree-g1-sdk2-session\n", stderr="")
            return subprocess.CompletedProcess(args, 1, stdout="", stderr="No such container")

        with tempfile.TemporaryDirectory() as tmp:
            official = OfficialG1Sim(Path(tmp), _unchecked_runner=fake_unchecked_runner)

            stopped = official.stop_session()

        self.assertTrue(stopped["ok"])
        self.assertTrue(stopped["removed"])
        self.assertFalse(stopped["status"]["exists"])
        self.assertIn("unitree-g1-sdk2-session", stopped["stdout"])
        self.assertEqual(unchecked_calls[0], ["docker", "rm", "-f", "unitree-g1-sdk2-session"])

    def test_official_g1_managed_session_context_starts_and_stops_peer(self):
        checked_calls = []
        unchecked_calls = []

        def fake_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
            checked_calls.append(args)
            return subprocess.CompletedProcess(args, 0, stdout="container-id\n", stderr="")

        def fake_unchecked_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
            unchecked_calls.append(args)
            command = " ".join(args)
            if "inspect unitree-g1-sdk2-session" in command:
                return subprocess.CompletedProcess(
                    args,
                    0,
                    stdout=json.dumps({"Running": True, "Status": "running", "ExitCode": 0}),
                    stderr="",
                )
            if "logs --tail" in command:
                return subprocess.CompletedProcess(
                    args,
                    0,
                    stdout=json.dumps({"action": "serve_official_mujoco", "ok": True, "peer_started": True}),
                    stderr="",
                )
            return subprocess.CompletedProcess(args, 0, stdout="unitree-g1-sdk2-session\n", stderr="")

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / ".runtime/unitree-g1-sdk2").mkdir(parents=True)
            (root / ".runtime/unitree-g1-sdk2/compose.env").write_text("UNITREE=test\n", encoding="utf-8")
            (root / "overlays/unitree-g1-sdk2-sidecar").mkdir(parents=True)
            (root / "overlays/unitree-g1-sdk2-sidecar/compose.yaml").write_text("services: {}\n", encoding="utf-8")
            official = OfficialG1Sim(root, _runner=fake_runner, _unchecked_runner=fake_unchecked_runner)

            with official.session() as session:
                self.assertTrue(session.started["ok"])
                self.assertTrue(session.status()["ready"])

            self.assertTrue(session.stopped["removed"])

        self.assertTrue(any("run -d --name unitree-g1-sdk2-session" in " ".join(call) for call in checked_calls))
        self.assertGreaterEqual(
            sum(1 for call in unchecked_calls if "rm -f unitree-g1-sdk2-session" in " ".join(call)),
            2,
        )

    def test_official_g1_managed_session_writes_arm_pose_evidence(self):
        def fake_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
            command = " ".join(args)
            if "CYBER_UNITREE_ACTION=read_official_mujoco_lowstate" in command:
                stdout = {
                    "lowstate_read": {
                        "ok": True,
                        "lowstate_summary": {
                            "mode_machine": 5,
                            "motor_count": 35,
                            "first_motors": [{"index": 0, "q": 0.0, "dq": 0.0}],
                        },
                    }
                }
            elif "CYBER_UNITREE_ACTION=command_official_mujoco_arm_pose" in command:
                stdout = {
                    "arm_pose_command": {
                        "ok": True,
                        "moved_joints": ["right_shoulder_pitch", "right_elbow"],
                        "lowcmd_write_successes": 180,
                    }
                }
            else:
                stdout = "container-id\n"
            return subprocess.CompletedProcess(
                args,
                0,
                stdout=json.dumps(stdout) if isinstance(stdout, dict) else stdout,
                stderr="",
            )

        def fake_unchecked_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
            command = " ".join(args)
            if "inspect unitree-g1-sdk2-session" in command:
                return subprocess.CompletedProcess(args, 0, stdout=json.dumps({"Running": True, "Status": "running"}), stderr="")
            if "logs --tail" in command:
                return subprocess.CompletedProcess(args, 0, stdout=json.dumps({"ok": True, "peer_started": True}), stderr="")
            return subprocess.CompletedProcess(args, 0, stdout="unitree-g1-sdk2-session\n", stderr="")

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / ".runtime/unitree-g1-sdk2").mkdir(parents=True)
            (root / ".runtime/unitree-g1-sdk2/compose.env").write_text("UNITREE=test\n", encoding="utf-8")
            (root / "overlays/unitree-g1-sdk2-sidecar").mkdir(parents=True)
            (root / "overlays/unitree-g1-sdk2-sidecar/compose.yaml").write_text("services: {}\n", encoding="utf-8")
            output_path = root / ".runtime/official-mujoco-evidence/test.json"
            official = OfficialG1Sim(root, _runner=fake_runner, _unchecked_runner=fake_unchecked_runner)

            with official.session() as session:
                bundle = session.arm_pose_evidence(output_path=output_path)

            written = json.loads(output_path.read_text(encoding="utf-8"))

        self.assertTrue(bundle["ok"])
        self.assertEqual(bundle["command"]["moved_joints"], ["right_shoulder_pitch", "right_elbow"])
        self.assertEqual(bundle["before"]["motor_count"], 35)
        self.assertEqual(written["source"], "official_unitree_mujoco_managed_session")
        self.assertEqual(written["command"]["lowcmd_write_successes"], 180)

    def test_official_g1_sim_can_command_managed_session(self):
        captured = {}

        def fake_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
            captured["args"] = args
            captured["cwd"] = cwd
            captured["timeout"] = timeout
            return subprocess.CompletedProcess(
                args,
                0,
                stdout=json.dumps(
                    {
                        "arm_pose_command": {
                            "ok": True,
                            "preset": "raise_left_hand",
                            "moved_joints": ["left_shoulder_pitch", "left_elbow"],
                            "lowcmd_write_successes": 120,
                        }
                    }
                ),
                stderr="",
            )

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / ".runtime/unitree-g1-sdk2").mkdir(parents=True)
            (root / ".runtime/unitree-g1-sdk2/compose.env").write_text("UNITREE=test\n", encoding="utf-8")
            (root / "overlays/unitree-g1-sdk2-sidecar").mkdir(parents=True)
            (root / "overlays/unitree-g1-sdk2-sidecar/compose.yaml").write_text("services: {}\n", encoding="utf-8")
            official = OfficialG1Sim(root, timeout=77, _runner=fake_runner)

            result = official.raise_left_hand_session(frames=120, min_moved_joints=1)

        self.assertTrue(result["ok"])
        self.assertEqual(result["source"], "official_unitree_mujoco_managed_session")
        self.assertEqual(result["moved_joints"], ["left_shoulder_pitch", "left_elbow"])
        self.assertEqual(result["lowcmd_write_successes"], 120)
        self.assertEqual(captured["timeout"], 77)
        command = " ".join(captured["args"])
        self.assertIn("CYBER_UNITREE_ACTION=command_official_mujoco_arm_pose", command)
        self.assertIn("CYBER_UNITREE_ARM_POSE_PRESET=raise_left_hand", command)
        self.assertIn("CYBER_UNITREE_ARM_POSE_FRAMES=120", command)

    def test_official_g1_sim_can_read_managed_session_lowstate(self):
        captured = {}

        def fake_runner(args: list[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
            captured["args"] = args
            captured["cwd"] = cwd
            captured["timeout"] = timeout
            return subprocess.CompletedProcess(
                args,
                0,
                stdout=json.dumps(
                    {
                        "lowstate_read": {
                            "ok": True,
                            "lowstate_sample_received": True,
                            "lowstate_summary": {
                                "mode_machine": 5,
                                "motor_count": 35,
                                "first_motors": [{"index": 0, "q": 0.01, "dq": 0.0}],
                            },
                        }
                    }
                ),
                stderr="",
            )

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / ".runtime/unitree-g1-sdk2").mkdir(parents=True)
            (root / ".runtime/unitree-g1-sdk2/compose.env").write_text("UNITREE=test\n", encoding="utf-8")
            (root / "overlays/unitree-g1-sdk2-sidecar").mkdir(parents=True)
            (root / "overlays/unitree-g1-sdk2-sidecar/compose.yaml").write_text("services: {}\n", encoding="utf-8")
            official = OfficialG1Sim(root, timeout=42, _runner=fake_runner)

            result = official.lowstate_session()

        self.assertTrue(result["ok"])
        self.assertEqual(result["source"], "official_unitree_mujoco_managed_session")
        self.assertEqual(result["lowstate_summary"]["mode_machine"], 5)
        self.assertEqual(result["lowstate_summary"]["motor_count"], 35)
        self.assertEqual(captured["timeout"], 42)
        command = " ".join(captured["args"])
        self.assertIn("CYBER_UNITREE_ACTION=read_official_mujoco_lowstate", command)


if __name__ == "__main__":
    unittest.main()
