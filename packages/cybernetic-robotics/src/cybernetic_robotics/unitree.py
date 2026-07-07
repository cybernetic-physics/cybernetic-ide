from __future__ import annotations

from typing import Any

from .simulator import SimulatorClient


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
