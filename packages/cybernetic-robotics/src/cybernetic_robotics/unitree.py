from __future__ import annotations

import os
from typing import Any

from .simulator import SimulatorClient


LOCO_SERVICE_NAME = "sport"
LOCO_API_VERSION = "1.0.0.0"
AGV_SERVICE_NAME = "agv"
AGV_API_VERSION = "1.0.0.0"
AUDIO_SERVICE_NAME = "voice"
AUDIO_API_VERSION = "1.0.0.0"
ROBOT_API_ID_AGV_MOVE = 8001
ROBOT_API_ID_AGV_HEIGHT_ADJUST = 8002
ROBOT_API_ID_LOCO_GET_FSM_ID = 7001
ROBOT_API_ID_LOCO_GET_FSM_MODE = 7002
ROBOT_API_ID_LOCO_GET_BALANCE_MODE = 7003
ROBOT_API_ID_LOCO_GET_SWING_HEIGHT = 7004
ROBOT_API_ID_LOCO_GET_STAND_HEIGHT = 7005
ROBOT_API_ID_LOCO_GET_PHASE = 7006
ROBOT_API_ID_LOCO_SET_FSM_ID = 7101
ROBOT_API_ID_LOCO_SET_BALANCE_MODE = 7102
ROBOT_API_ID_LOCO_SET_SWING_HEIGHT = 7103
ROBOT_API_ID_LOCO_SET_STAND_HEIGHT = 7104
ROBOT_API_ID_LOCO_SET_VELOCITY = 7105
ROBOT_API_ID_LOCO_SET_ARM_TASK = 7106
ROBOT_API_ID_LOCO_SET_SPEED_MODE = 7107
ROBOT_API_ID_LOCO_SWITCH_TO_USER_CTRL = 7110
ROBOT_API_ID_LOCO_SWITCH_TO_INTERNAL_CTRL = 7111
ROBOT_API_ID_AUDIO_TTS = 1001
ROBOT_API_ID_AUDIO_ASR = 1002
ROBOT_API_ID_AUDIO_START_PLAY = 1003
ROBOT_API_ID_AUDIO_STOP_PLAY = 1004
ROBOT_API_ID_AUDIO_GET_VOLUME = 1005
ROBOT_API_ID_AUDIO_SET_VOLUME = 1006
ROBOT_API_ID_AUDIO_SET_RGB_LED = 1010

_FSM_NAMES = {
    0: "zero_torque",
    1: "damp",
    3: "sit",
    500: "start",
    702: "lie_to_stand_up",
    706: "squat_stand_transition",
}


action_map = {
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

_ACTION_POSES = {
    action_map["release arm"]: "neutral",
    action_map["two-hand kiss"]: "two_hand_kiss",
    action_map["left kiss"]: "left_kiss",
    action_map["right kiss"]: "right_kiss",
    action_map["hands up"]: "hands_up",
    action_map["clap"]: "clap",
    action_map["high five"]: "high_five",
    action_map["hug"]: "hug",
    action_map["heart"]: "heart",
    action_map["right heart"]: "right_heart",
    action_map["reject"]: "reject",
    action_map["right hand up"]: "raise_right_hand",
    action_map["x-ray"]: "x_ray",
    action_map["face wave"]: "face_wave",
    action_map["high wave"]: "high_wave",
    action_map["shake hand"]: "shake_hand",
}


class G1ArmActionClient:
    """Simulator-backed subset of Unitree's `G1ArmActionClient`."""

    def __init__(self):
        self.service_name = "arm"
        self.api_version: str | None = None
        self.timeout = 1.0
        self.last_response: dict[str, Any] | None = None
        self._simulator = SimulatorClient.from_env(timeout=self.timeout)

    def SetTimeout(self, timeout: float):
        self.timeout = float(timeout)
        self._simulator.timeout = self.timeout

    def Init(self):
        self.api_version = "1.0.0.14"
        self._registered_apis = {7106, 7107}

    def ExecuteAction(self, action_id: int):
        pose = _ACTION_POSES.get(action_id)
        if pose is None:
            self.last_response = {
                "ok": False,
                "error": f"unsupported simulated G1 arm action id: {action_id}",
                "supported_actions": self.GetActionList()[1],
            }
            return -1

        if _use_official_dds_session():
            if pose not in {"raise_right_hand", "raise_left_hand"}:
                self.last_response = {
                    "ok": False,
                    "error": f"official DDS simulator session does not yet support arm pose: {pose}",
                    "supported_poses": ["raise_right_hand", "raise_left_hand"],
                    "transport": "dds",
                }
                return -1
            try:
                from .official import OfficialG1Sim

                official = OfficialG1Sim.discover(timeout=max(30, int(round(self.timeout))))
                if pose == "raise_left_hand":
                    self.last_response = official.raise_left_hand_session()
                else:
                    self.last_response = official.raise_right_hand_session()
                self.last_response["transport"] = "dds"
                self.last_response["unitree_action_id"] = action_id
            except Exception as error:  # noqa: BLE001 - mirror SDK integer error style.
                self.last_response = {"ok": False, "error": str(error), "transport": "dds"}
                return -1
            return 0 if self.last_response.get("ok") else -1

        try:
            self.last_response = self._simulator.pose(pose)
        except Exception as error:  # noqa: BLE001 - mirror SDK integer error style.
            self.last_response = {"ok": False, "error": str(error)}
            return -1
        return 0

    def GetActionList(self):
        actions = [
            {"name": name, "id": action_id, "simulated": action_id in _ACTION_POSES}
            for name, action_id in sorted(action_map.items(), key=lambda item: item[1])
        ]
        return 0, actions


def _use_official_dds_session() -> bool:
    transport = os.environ.get("CYBER_UNITREE_TRANSPORT", "").strip().lower().replace("-", "_")
    mode = os.environ.get("CYBER_UNITREE_MODE", "sim").strip().lower()
    return transport == "dds" and mode != "real"


class LocoClient:
    """Simulator-backed subset of Unitree's official G1 `LocoClient`.

    The official SDK sends RPC requests to the `sport` service over DDS. The
    Cybernetic simulator bridge keeps the same Python method names and maps
    them onto local GameControl commands, so user examples can move between
    simulator mode and future real-SDK mode with minimal edits.
    """

    def __init__(self):
        self.service_name = LOCO_SERVICE_NAME
        self.api_version: str | None = None
        self.timeout = 1.0
        self.last_response: dict[str, Any] | None = None
        self.first_shake_hand_stage_ = False
        self._simulator = SimulatorClient.from_env(timeout=self.timeout)

    def SetTimeout(self, timeout: float):
        self.timeout = float(timeout)
        self._simulator.timeout = self.timeout

    def Init(self):
        self.api_version = LOCO_API_VERSION
        self._registered_apis = {
            ROBOT_API_ID_LOCO_GET_FSM_ID,
            ROBOT_API_ID_LOCO_GET_FSM_MODE,
            ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
            ROBOT_API_ID_LOCO_GET_SWING_HEIGHT,
            ROBOT_API_ID_LOCO_GET_STAND_HEIGHT,
            ROBOT_API_ID_LOCO_GET_PHASE,
            ROBOT_API_ID_LOCO_SET_FSM_ID,
            ROBOT_API_ID_LOCO_SET_BALANCE_MODE,
            ROBOT_API_ID_LOCO_SET_SWING_HEIGHT,
            ROBOT_API_ID_LOCO_SET_STAND_HEIGHT,
            ROBOT_API_ID_LOCO_SET_VELOCITY,
            ROBOT_API_ID_LOCO_SET_ARM_TASK,
            ROBOT_API_ID_LOCO_SET_SPEED_MODE,
            ROBOT_API_ID_LOCO_SWITCH_TO_USER_CTRL,
            ROBOT_API_ID_LOCO_SWITCH_TO_INTERNAL_CTRL,
        }

    def GetFsmId(self):
        response = self._call_loco("get_fsm_id")
        return _code_and_data(response, response.get("fsm_id"))

    def GetFsmMode(self):
        response = self._call_loco("get_fsm_mode")
        return _code_and_data(response, response.get("fsm_mode"))

    def GetBalanceMode(self):
        response = self._call_loco("get_balance_mode")
        return _code_and_data(response, response.get("balance_mode"))

    def GetSwingHeight(self):
        response = self._call_loco("get_swing_height")
        return _code_and_data(response, response.get("swing_height"))

    def GetStandHeight(self):
        response = self._call_loco("get_stand_height")
        return _code_and_data(response, response.get("stand_height"))

    def GetPhase(self):
        response = self._call_loco("get_phase")
        return _code_and_data(response, response.get("phase"))

    def SetFsmId(self, fsm_id: int):
        return self._code(self._call_loco("set_fsm_id", fsm_id=int(fsm_id), mode=_FSM_NAMES.get(int(fsm_id))))

    def SetBalanceMode(self, balance_mode: int):
        return self._code(self._call_loco("set_balance_mode", balance_mode=int(balance_mode)))

    def SetSwingHeight(self, swing_height: float):
        return self._code(self._call_loco("set_swing_height", swing_height=float(swing_height)))

    def SetStandHeight(self, stand_height: float):
        return self._code(self._call_loco("set_stand_height", stand_height=float(stand_height)))

    def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
        return self._code(
            self._call_loco(
                "set_velocity",
                velocity=[float(vx), float(vy), float(omega)],
                duration=float(duration),
            )
        )

    def SetTaskId(self, task_id: float):
        return self._code(self._call_loco("set_arm_task", task_id=int(task_id)))

    def SetSpeedMode(self, speed_mode: int):
        return self._code(self._call_loco("set_speed_mode", speed_mode=int(speed_mode)))

    def SwitchToUserCtrl(self):
        return self._code(self._call_loco("switch_to_user_ctrl"))

    def SwitchToInternalCtrl(self, mode: int = 0):
        return self._code(self._call_loco("switch_to_internal_ctrl", internal_mode=int(mode)))

    def Damp(self):
        return self.SetFsmId(1)

    def Start(self):
        return self.SetFsmId(500)

    def Squat(self):
        return self.SetFsmId(2)

    def Squat2StandUp(self):
        return self.SetFsmId(706)

    def Lie2StandUp(self):
        return self.SetFsmId(702)

    def Sit(self):
        return self.SetFsmId(3)

    def StandUp(self):
        return self.SetFsmId(4)

    def StandUp2Squat(self):
        return self.SetFsmId(706)

    def ZeroTorque(self):
        return self.SetFsmId(0)

    def StopMove(self):
        return self.SetVelocity(0.0, 0.0, 0.0)

    def HighStand(self):
        return self._code(self._call_loco("high_stand"))

    def LowStand(self):
        return self._code(self._call_loco("low_stand"))

    def Move(self, vx: float, vy: float, vyaw: float, continous_move: bool | None = None):
        if continous_move is None:
            state = self.last_response.get("loco") if isinstance(self.last_response, dict) else {}
            continous_move = bool(state.get("continuous_move")) if isinstance(state, dict) else False
        duration = 864000.0 if continous_move else 1.0
        return self.SetVelocity(vx, vy, vyaw, duration)

    def BalanceStand(self, balance_mode: int = 0):
        return self.SetBalanceMode(balance_mode)

    def ContinuousGait(self, flag: bool):
        return self.SetBalanceMode(1 if flag else 0)

    def SwitchMoveMode(self, flag: bool):
        return self._code(self._call_loco("switch_move_mode", continuous_move=bool(flag)))

    def WaveHand(self, turn_flag: bool = False):
        return self._code(self._call_loco("wave_hand", turn=bool(turn_flag)))

    def ShakeHand(self, stage: int = -1):
        if stage == 0:
            self.first_shake_hand_stage_ = False
            return self._code(self._call_loco("shake_hand", stage=0))
        if stage == 1:
            self.first_shake_hand_stage_ = True
            return self._code(self._call_loco("shake_hand", stage=1))
        self.first_shake_hand_stage_ = not self.first_shake_hand_stage_
        return self._code(self._call_loco("shake_hand", stage=1 if self.first_shake_hand_stage_ else 0))

    def _call_loco(self, action: str, **fields: Any) -> dict[str, Any]:
        try:
            self.last_response = self._simulator.loco(action=action, **fields)
        except Exception as error:  # noqa: BLE001 - mirror SDK integer error style.
            self.last_response = {"ok": False, "error": str(error), "action": action}
        return self.last_response

    def _code(self, response: dict[str, Any]) -> int:
        return 0 if response.get("ok") else -1


def _code_and_data(response: dict[str, Any], data: Any):
    return (0, data) if response.get("ok") else (-1, None)


class AgvClient:
    """Simulator-backed subset of Unitree's official G1 `AgvClient`.

    Upstream Unitree SDK2 exposes `unitree::robot::g1::AgvClient` for G1
    wheeled-base style velocity and height-column commands. Cybernetic keeps
    the Python call shape available at
    `unitree_sdk2py.g1.agv.g1_agv_client.AgvClient` and maps it onto the local
    simulator's locomotion state. `vy` is accepted for API compatibility, but
    the upstream AGV API documents lateral motion as unsupported.
    """

    def __init__(self):
        self.service_name = AGV_SERVICE_NAME
        self.api_version: str | None = None
        self.timeout = 1.0
        self.last_response: dict[str, Any] | None = None
        self.height_velocity = 0.0
        self._simulator = SimulatorClient.from_env(timeout=self.timeout)

    def SetTimeout(self, timeout: float):  # noqa: N802 - match Unitree SDK2 API.
        self.timeout = float(timeout)
        self._simulator.timeout = self.timeout

    def Init(self):  # noqa: N802 - match Unitree SDK2 API.
        self.api_version = AGV_API_VERSION
        self._registered_apis = {ROBOT_API_ID_AGV_MOVE, ROBOT_API_ID_AGV_HEIGHT_ADJUST}

    def Move(self, vx: float, vy: float, vyaw: float):  # noqa: N802 - match Unitree SDK2 API.
        vx_clamped = _clamp_float(float(vx), -1.5, 1.5)
        vyaw_clamped = _clamp_float(float(vyaw), -0.6, 0.6)
        response = self._call_agv(
            "move",
            velocity=[vx_clamped, 0.0, vyaw_clamped],
            requested_velocity=[float(vx), float(vy), float(vyaw)],
            ignored_lateral_velocity=float(vy),
            duration=1.0,
        )
        return self._code(response)

    def HeightAdjust(self, vz: float):  # noqa: N802 - match Unitree SDK2 API.
        self.height_velocity = _clamp_float(float(vz), -1.0, 1.0)
        response = self._call_agv("height_adjust", height_velocity=self.height_velocity)
        return self._code(response)

    def _call_agv(self, action: str, **fields: Any) -> dict[str, Any]:
        payload = {
            "action": action,
            "service": AGV_SERVICE_NAME,
            "simulated": True,
            **fields,
        }
        try:
            if action == "move":
                response = self._simulator.loco(
                    action="set_velocity",
                    velocity=payload["velocity"],
                    duration=payload["duration"],
                )
            else:
                response = self._simulator.command("agv", **payload)
            if response.get("ok"):
                self.last_response = {**response, **payload, "agv": payload}
                return self.last_response
        except Exception as error:  # noqa: BLE001 - mirror SDK integer error style.
            payload["transport_error"] = str(error)
        self.last_response = {"ok": action == "height_adjust", **payload}
        return self.last_response

    def _code(self, response: dict[str, Any]) -> int:
        return 0 if response.get("ok") else -1


def _clamp_float(value: float, minimum: float, maximum: float) -> float:
    return min(max(value, minimum), maximum)


class AudioClient:
    """Simulator-backed subset of Unitree's official G1 `AudioClient`.

    MuJoCo has no speaker, microphone, or LED hardware. This shim preserves the
    official SDK2 Python call shape and records intent through the local
    GameControl endpoint when available. If the current simulator image does not
    implement the optional `audio` command yet, calls still succeed locally and
    expose the recorded metadata through `last_response`.
    """

    def __init__(self):
        self.service_name = AUDIO_SERVICE_NAME
        self.api_version: str | None = None
        self.timeout = 1.0
        self.tts_index = 0
        self.volume = 50
        self.led = {"R": 0, "G": 0, "B": 0}
        self.last_response: dict[str, Any] | None = None
        self._simulator = SimulatorClient.from_env(timeout=self.timeout)

    def SetTimeout(self, timeout: float):  # noqa: N802 - match Unitree SDK2 API.
        self.timeout = float(timeout)
        self._simulator.timeout = self.timeout

    def Init(self):  # noqa: N802 - match Unitree SDK2 API.
        self.api_version = AUDIO_API_VERSION
        self._registered_apis = {
            ROBOT_API_ID_AUDIO_TTS,
            ROBOT_API_ID_AUDIO_ASR,
            ROBOT_API_ID_AUDIO_START_PLAY,
            ROBOT_API_ID_AUDIO_STOP_PLAY,
            ROBOT_API_ID_AUDIO_GET_VOLUME,
            ROBOT_API_ID_AUDIO_SET_VOLUME,
            ROBOT_API_ID_AUDIO_SET_RGB_LED,
        }

    def TtsMaker(self, text: str, speaker_id: int):  # noqa: N802 - match Unitree SDK2 API.
        self.tts_index += 1
        response = self._call_audio("tts", index=self.tts_index, text=str(text), speaker_id=int(speaker_id))
        return self._code(response)

    def GetVolume(self):  # noqa: N802 - match Unitree SDK2 API.
        response = self._call_audio("get_volume", volume=self.volume)
        volume = response.get("volume", self.volume)
        return _code_and_data(response, {"volume": volume})

    def SetVolume(self, volume: int):  # noqa: N802 - match Unitree SDK2 API.
        self.volume = int(volume)
        return self._code(self._call_audio("set_volume", volume=self.volume))

    def LedControl(self, R: int, G: int, B: int):  # noqa: N802,N803 - match Unitree SDK2 API.
        self.led = {"R": int(R), "G": int(G), "B": int(B)}
        return self._code(self._call_audio("set_rgb_led", **self.led))

    def PlayStream(self, app_name: str, stream_id: str, pcm_data: bytes):  # noqa: N802 - match Unitree SDK2 API.
        response = self._call_audio(
            "start_play",
            app_name=str(app_name),
            stream_id=str(stream_id),
            pcm_bytes=len(bytes(pcm_data)),
        )
        return self._code(response), response

    def PlayStop(self, app_name: str):  # noqa: N802 - match Unitree SDK2 API.
        return self._code(self._call_audio("stop_play", app_name=str(app_name)))

    def _call_audio(self, action: str, **fields: Any) -> dict[str, Any]:
        payload = {"action": action, **fields}
        try:
            response = self._simulator.command("audio", **payload)
            if response.get("ok"):
                self.last_response = response
                if action in {"get_volume", "set_volume"} and "volume" in response:
                    self.volume = int(response["volume"])
                return response
        except Exception as error:  # noqa: BLE001 - simulator audio is optional.
            payload["transport_error"] = str(error)
        self.last_response = {"ok": True, "simulated": True, "service": AUDIO_SERVICE_NAME, **payload}
        return self.last_response

    def _code(self, response: dict[str, Any]) -> int:
        return 0 if response.get("ok") else -1
