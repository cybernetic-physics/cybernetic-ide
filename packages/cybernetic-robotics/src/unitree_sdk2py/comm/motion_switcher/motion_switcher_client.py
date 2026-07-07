from __future__ import annotations

from cybernetic_robotics.simulator import SimulatorClient


class MotionSwitcherClient:
    """Simulator-backed subset of Unitree's motion-switcher client."""

    def __init__(self):
        self.timeout = 5.0
        self.inited = False
        self.selected_mode = ""

    def SetTimeout(self, timeout: float):  # noqa: N802 - match Unitree SDK2 API.
        self.timeout = float(timeout)

    def Init(self):  # noqa: N802 - match Unitree SDK2 API.
        self.inited = True

    def CheckMode(self):  # noqa: N802 - match Unitree SDK2 API.
        return 0, {"name": self.selected_mode}

    def SelectMode(self, nameOrAlias):  # noqa: N802,N803 - match Unitree SDK2 API.
        self.selected_mode = str(nameOrAlias)
        return 0, None

    def ReleaseMode(self):  # noqa: N802 - match Unitree SDK2 API.
        self.selected_mode = ""
        try:
            SimulatorClient.from_env(timeout=self.timeout).loco("damp")
        except Exception:
            pass
        return 0, None
