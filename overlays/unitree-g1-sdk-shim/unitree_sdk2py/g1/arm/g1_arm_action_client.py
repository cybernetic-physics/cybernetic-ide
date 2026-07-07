from __future__ import annotations

import json
import os
from urllib.error import URLError
from urllib.request import Request, urlopen

from .g1_arm_action_api import (
    ARM_ACTION_API_VERSION,
    ARM_ACTION_SERVICE_NAME,
    ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION,
    ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST,
)


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
    def __init__(self):
        self.service_name = ARM_ACTION_SERVICE_NAME
        self.api_version = None
        self.timeout = 1.0
        self.last_response = None

    def SetTimeout(self, timeout: float):
        self.timeout = float(timeout)

    def Init(self):
        self.api_version = ARM_ACTION_API_VERSION
        self._registered_apis = {
            ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION,
            ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST,
        }

    def ExecuteAction(self, action_id: int):
        pose = _ACTION_POSES.get(action_id)
        if pose is None:
            self.last_response = {
                "ok": False,
                "error": f"unsupported simulated G1 arm action id: {action_id}",
                "supported_actions": self.GetActionList()[1],
            }
            return -1

        code, response = self._post_command({"command": "pose", "pose": pose})
        self.last_response = response
        return code

    def GetActionList(self):
        actions = [
            {"name": name, "id": action_id, "simulated": action_id in _ACTION_POSES}
            for name, action_id in sorted(action_map.items(), key=lambda item: item[1])
        ]
        return 0, actions

    def _post_command(self, payload: dict):
        url = os.environ.get("CYBER_G1_GAME_CONTROL_URL", "http://127.0.0.1:38383")
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        request = Request(
            f"{url}/command",
            data=data,
            headers={"content-type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                body = json.loads(response.read().decode("utf-8"))
        except (OSError, URLError, json.JSONDecodeError) as error:
            return -1, {"ok": False, "error": str(error)}

        if body.get("ok"):
            return 0, body
        return -1, body

