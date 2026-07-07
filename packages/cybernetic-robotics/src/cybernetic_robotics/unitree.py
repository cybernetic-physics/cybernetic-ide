from __future__ import annotations

from typing import Any

from .simulator import SimulatorClient


LOCO_SERVICE_NAME = "sport"
LOCO_API_VERSION = "1.0.0.0"
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
    action_map["right hand up"]: "raise_right_hand",
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
        }

    def GetFsmId(self):
        response = self._call_loco("get_fsm_id")
        return _code_and_data(response, response.get("fsm_id"))

    def SetFsmId(self, fsm_id: int):
        return self._code(self._call_loco("set_fsm_id", fsm_id=int(fsm_id), mode=_FSM_NAMES.get(int(fsm_id))))

    def SetBalanceMode(self, balance_mode: int):
        return self._code(self._call_loco("set_balance_mode", balance_mode=int(balance_mode)))

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

    def Damp(self):
        return self.SetFsmId(1)

    def Start(self):
        return self.SetFsmId(500)

    def Squat2StandUp(self):
        return self.SetFsmId(706)

    def Lie2StandUp(self):
        return self.SetFsmId(702)

    def Sit(self):
        return self.SetFsmId(3)

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

    def Move(self, vx: float, vy: float, vyaw: float, continous_move: bool = False):
        duration = 864000.0 if continous_move else 1.0
        return self.SetVelocity(vx, vy, vyaw, duration)

    def BalanceStand(self, balance_mode: int):
        return self.SetBalanceMode(balance_mode)

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
