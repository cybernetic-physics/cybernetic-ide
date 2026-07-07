from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from cybernetic_robotics.simulator import SimulatorClient


@dataclass
class ChannelFactoryConfig:
    domain_id: int = 0
    network_interface: str | None = None
    initialized: bool = False


_CONFIG = ChannelFactoryConfig()


def ChannelFactoryInitialize(id: int = 0, networkInterface: str | None = None):
    """Record SDK2 channel settings.

    Unitree's real SDK initializes CycloneDDS here. Cybernetic's simulator shim
    keeps the same call visible to user code and routes high-level actions to
    the local MuJoCo harness.
    """

    global _CONFIG
    _CONFIG = ChannelFactoryConfig(id, networkInterface, True)


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
        SimulatorClient.from_env(timeout=timeout or 5.0).lowcmd(
            motor_cmd,
            topic=self.name,
            mode_pr=int(getattr(message, "mode_pr", 0)),
            mode_machine=int(getattr(message, "mode_machine", 0)),
            crc=int(getattr(message, "crc", 0)),
        )
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
        if handler is not None and self.name in {"rt/lowstate", "rt/sportmodestate", "rt/wirelesscontroller"}:
            self._thread = threading.Thread(target=self._poll, name=f"cyber-{self.name.replace('/', '-')}", daemon=True)
            self._thread.start()

    def Close(self):
        self._closed = True
        if getattr(self, "_thread", None) is not None:
            self._thread.join(timeout=1.0)
        self.inited = False

    def Read(self, timeout: int | None = None):
        if not self.inited:
            raise RuntimeError(f"ChannelSubscriber {self.name} is not initialized")
        client = SimulatorClient.from_env(timeout=float(timeout or 5.0))
        if self.name == "rt/lowstate":
            return _lowstate_from_json(client.lowstate())
        if self.name == "rt/sportmodestate":
            return _sportmode_from_status(client.status().raw)
        if self.name == "rt/wirelesscontroller":
            return _wireless_from_lowstate(client.lowstate())
        raise NotImplementedError(f"Cybernetic simulator channel subscriber does not support {self.name}")

    def _poll(self):
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
    state.mode_pr = int(value.get("mode_pr", 0))
    state.mode_machine = int(value.get("mode_machine", 0))
    state.crc = int(value.get("crc", 0))
    lowcmd = value.get("lowcmd") if isinstance(value.get("lowcmd"), dict) else {}
    state.lowcmd_active = bool(lowcmd.get("active"))
    state.lowcmd_stale = bool(lowcmd.get("stale"))
    state.lowcmd_age_seconds = float(lowcmd.get("age_seconds") or 0.0)
    state.lowcmd_watchdog_seconds = float(lowcmd.get("watchdog_seconds") or 0.0)
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


def _sportmode_from_status(value: dict):
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

    simulation = value.get("simulation") if isinstance(value.get("simulation"), dict) else {}
    loco = simulation.get("loco") if isinstance(simulation.get("loco"), dict) else {}
    render = simulation.get("render") if isinstance(simulation.get("render"), dict) else {}
    state = SportModeState_()
    state.mode = int(loco.get("fsm_id") or 0) & 0xFF
    state.progress = float(simulation.get("actual_speed_factor") or 0.0)
    state.gait_type = 0
    state.body_height = float(simulation.get("pelvis_height") or 0.0)
    velocity = loco.get("velocity") if isinstance(loco.get("velocity"), list) else [0.0, 0.0, 0.0]
    state.velocity[:] = [float(item) for item in (velocity + [0.0, 0.0, 0.0])[:3]]
    state.yaw_speed = state.velocity[2]
    if isinstance(render.get("camera"), dict):
        camera = render["camera"]
        lookat = camera.get("lookat") if isinstance(camera.get("lookat"), list) else []
        state.position[:] = [float(item) for item in (lookat + [0.0, 0.0, 0.0])[:3]]
    return state


def _wireless_from_lowstate(value: dict):
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_

    state = WirelessController_()
    wireless = value.get("wireless_remote") if isinstance(value.get("wireless_remote"), list) else []
    if len(wireless) >= 10:
        state.lx = _signed_byte_to_axis(wireless[0])
        state.ly = _signed_byte_to_axis(wireless[1])
        state.rx = _signed_byte_to_axis(wireless[2])
        state.ry = _signed_byte_to_axis(wireless[3])
        state.keys = int(wireless[8]) | (int(wireless[9]) << 8)
    return state


def _signed_byte_to_axis(value) -> float:
    number = int(value) & 0xFF
    if number >= 128:
        number -= 256
    return max(-1.0, min(1.0, number / 127.0))
