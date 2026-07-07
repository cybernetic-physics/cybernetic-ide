from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from cybernetic_robotics.simulator import SimulatorClient


@dataclass
class ChannelFactoryConfig:
    domain_id: int = 0
    network_interface: str | None = None


_CONFIG = ChannelFactoryConfig()


def ChannelFactoryInitialize(id: int = 0, networkInterface: str | None = None):
    """Record SDK2 channel settings.

    Unitree's real SDK initializes CycloneDDS here. Cybernetic's simulator shim
    keeps the same call visible to user code and routes high-level actions to
    the local MuJoCo harness.
    """

    global _CONFIG
    _CONFIG = ChannelFactoryConfig(id, networkInterface)


def current_channel_factory_config() -> ChannelFactoryConfig:
    return _CONFIG


class ChannelPublisher:
    def __init__(self, name: str, message_type):
        self.name = name
        self.message_type = message_type
        self.inited = False

    def Init(self):
        self.inited = True

    def Write(self, message, timeout: float | None = None):
        if not self.inited:
            raise RuntimeError(f"ChannelPublisher {self.name} is not initialized")
        if self.name not in {"rt/lowcmd", "rt/arm_sdk"}:
            raise NotImplementedError(f"Cybernetic simulator channel publisher does not support {self.name}")
        motor_cmd = [_motor_cmd_to_json(cmd) for cmd in getattr(message, "motor_cmd", [])]
        SimulatorClient.from_env(timeout=timeout or 5.0).lowcmd(motor_cmd, topic=self.name)
        return True

    def Close(self):
        self.inited = False


class ChannelSubscriber:
    def __init__(self, name: str, message_type):
        self.name = name
        self.message_type = message_type
        self.inited = False

    def Init(self, handler=None, queueLen: int = 0):
        self.handler = handler
        self.queue_len = queueLen
        self.inited = True
        self._closed = False
        self._thread: threading.Thread | None = None
        if handler is not None and self.name == "rt/lowstate":
            self._thread = threading.Thread(target=self._poll_lowstate, name="cyber-lowstate", daemon=True)
            self._thread.start()

    def Close(self):
        self._closed = True
        if getattr(self, "_thread", None) is not None:
            self._thread.join(timeout=1.0)
        self.inited = False

    def Read(self, timeout: int | None = None):
        if not self.inited:
            raise RuntimeError(f"ChannelSubscriber {self.name} is not initialized")
        if self.name != "rt/lowstate":
            raise NotImplementedError(f"Cybernetic simulator channel subscriber does not support {self.name}")
        client = SimulatorClient.from_env(timeout=float(timeout or 5.0))
        return _lowstate_from_json(client.lowstate())

    def _poll_lowstate(self):
        while not getattr(self, "_closed", False):
            try:
                self.handler(self.Read(timeout=2.0))
            except Exception:
                pass
            time.sleep(0.02)


def _motor_cmd_to_json(cmd) -> dict:
    return {
        "mode": int(getattr(cmd, "mode", 0)),
        "q": float(getattr(cmd, "q", 0.0)),
        "dq": float(getattr(cmd, "dq", 0.0)),
        "tau": float(getattr(cmd, "tau", 0.0)),
        "kp": float(getattr(cmd, "kp", 0.0)),
        "kd": float(getattr(cmd, "kd", 0.0)),
    }


def _lowstate_from_json(value: dict):
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_

    state = unitree_hg_msg_dds__LowState_()
    state.mode_machine = int(value.get("mode_machine", 0))
    imu = value.get("imu_state") or {}
    state.imu_state.quaternion[:] = list(imu.get("quaternion", state.imu_state.quaternion))[:4]
    state.imu_state.gyroscope[:] = list(imu.get("gyroscope", state.imu_state.gyroscope))[:3]
    state.imu_state.accelerometer[:] = list(imu.get("accelerometer", state.imu_state.accelerometer))[:3]
    for index, motor in enumerate(value.get("motor_state", [])):
        if index >= len(state.motor_state):
            break
        target = state.motor_state[index]
        target.mode = int(motor.get("mode", 0))
        target.q = float(motor.get("q", 0.0))
        target.dq = float(motor.get("dq", 0.0))
        target.tau_est = float(motor.get("tau_est", 0.0))
    return state
