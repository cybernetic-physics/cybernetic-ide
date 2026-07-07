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
    reserve: list[int] = field(default_factory=lambda: [0, 0, 0, 0, 0])
    crc: int = 0
    __idl_typename__: str = "unitree_hg.msg.dds_.LowState_"


__all__ = ["IMUState_", "LowCmd_", "LowState_", "MotorCmd_", "MotorState_"]
