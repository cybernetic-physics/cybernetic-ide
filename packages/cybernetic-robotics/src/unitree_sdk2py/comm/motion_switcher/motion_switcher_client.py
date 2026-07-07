from __future__ import annotations

from cybernetic_robotics.simulator import SimulatorClient


class MotionSwitcherClient:
    """Simulator-backed subset of Unitree's motion-switcher client."""

    def __init__(self):
        self.timeout = 5.0
        self.inited = False
        self.api_version: str | None = None
        self.selected_mode = ""
        self.last_response: dict | None = None
        self._simulator = SimulatorClient.from_env(timeout=self.timeout)

    def SetTimeout(self, timeout: float):  # noqa: N802 - match Unitree SDK2 API.
        self.timeout = float(timeout)
        self._simulator.timeout = self.timeout

    def Init(self):  # noqa: N802 - match Unitree SDK2 API.
        self.api_version = "1.0.0.1"
        self.inited = True

    def CheckMode(self):  # noqa: N802 - match Unitree SDK2 API.
        response = self._call("check_mode")
        mode = response.get("mode", {}) if response.get("ok") else {}
        self.selected_mode = str(mode.get("name", ""))
        return (0, {"name": self.selected_mode}) if response.get("ok") else (-1, None)

    def SelectMode(self, nameOrAlias):  # noqa: N802,N803 - match Unitree SDK2 API.
        self.selected_mode = str(nameOrAlias)
        response = self._call("select_mode", name=self.selected_mode)
        return (0, None) if response.get("ok") else (-1, None)

    def ReleaseMode(self):  # noqa: N802 - match Unitree SDK2 API.
        self.selected_mode = ""
        response = self._call("release_mode")
        return (0, None) if response.get("ok") else (-1, None)

    def SetSilent(self, silent: bool):  # noqa: N802 - match Unitree SDK2 API.
        response = self._call("set_silent", silent=bool(silent))
        return (0, None) if response.get("ok") else (-1, None)

    def GetSilent(self):  # noqa: N802 - match Unitree SDK2 API.
        response = self._call("get_silent")
        return (0, bool(response.get("silent"))) if response.get("ok") else (-1, None)

    def _call(self, action: str, **fields):
        try:
            self.last_response = self._simulator.command("motion_switcher", action=action, **fields)
        except Exception as error:  # noqa: BLE001 - mirror SDK integer error style.
            self.last_response = {"ok": False, "error": str(error), "action": action}
        return self.last_response
