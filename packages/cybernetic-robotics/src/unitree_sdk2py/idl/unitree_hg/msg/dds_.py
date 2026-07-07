from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class MotorCmd_:
    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    reserve: int = 0


@dataclass
class MotorState_:
    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    ddq: float = 0.0
    tau_est: float = 0.0
    temperature: list[int] = field(default_factory=lambda: [0, 0])
    vol: float = 0.0
    sensor: list[int] = field(default_factory=lambda: [0, 0])
    motorstate: int = 0
    reserve: list[int] = field(default_factory=lambda: [0, 0])


@dataclass
class PressSensorState_:
    pressure: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    temperature: float = 0.0


@dataclass
class IMUState_:
    quaternion: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    gyroscope: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accelerometer: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    temperature: int = 0


@dataclass
class LowCmd_:
    mode_pr: int = 0
    mode_machine: int = 0
    motor_cmd: list[MotorCmd_] = field(default_factory=lambda: [MotorCmd_() for _ in range(35)])
    reserve: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    crc: int = 0
    __idl_typename__: str = "unitree_hg.msg.dds_.LowCmd_"


@dataclass
class LowState_:
    version: list[int] = field(default_factory=lambda: [0, 0])
    mode_pr: int = 0
    mode_machine: int = 0
    tick: int = 0
    imu_state: IMUState_ = field(default_factory=IMUState_)
    motor_state: list[MotorState_] = field(default_factory=lambda: [MotorState_() for _ in range(35)])
    wireless_remote: list[int] = field(default_factory=lambda: [0 for _ in range(40)])
    lowcmd_active: bool = False
    lowcmd_stale: bool = False
    lowcmd_age_seconds: float = 0.0
    lowcmd_watchdog_seconds: float = 0.0
    reserve: list[int] = field(default_factory=lambda: [0, 0, 0, 0, 0])
    crc: int = 0
    __idl_typename__: str = "unitree_hg.msg.dds_.LowState_"


@dataclass
class HandCmd_:
    motor_cmd: list[MotorCmd_] = field(default_factory=lambda: [MotorCmd_() for _ in range(7)])
    reserve: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    __idl_typename__: str = "unitree_hg.msg.dds_.HandCmd_"


@dataclass
class HandState_:
    motor_state: list[MotorState_] = field(default_factory=lambda: [MotorState_() for _ in range(7)])
    press_sensor_state: list[PressSensorState_] = field(default_factory=lambda: [PressSensorState_() for _ in range(9)])
    imu_state: IMUState_ = field(default_factory=IMUState_)
    power_v: float = 0.0
    power_a: float = 0.0
    system_v: float = 0.0
    device_v: float = 0.0
    error: list[int] = field(default_factory=lambda: [0, 0])
    reserve: list[int] = field(default_factory=lambda: [0, 0])
    __idl_typename__: str = "unitree_hg.msg.dds_.HandState_"


__all__ = [
    "HandCmd_",
    "HandState_",
    "IMUState_",
    "LowCmd_",
    "LowState_",
    "MotorCmd_",
    "MotorState_",
    "PressSensorState_",
]
