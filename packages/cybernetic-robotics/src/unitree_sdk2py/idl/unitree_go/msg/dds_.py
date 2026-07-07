from __future__ import annotations

from dataclasses import dataclass, field

@dataclass
class TimeSpec_:
    sec: int = 0
    nanosec: int = 0
    __idl_typename__: str = "unitree_go.msg.dds_.TimeSpec_"


@dataclass
class PathPoint_:
    timeFromStart: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0
    __idl_typename__: str = "unitree_go.msg.dds_.PathPoint_"


@dataclass
class IMUState_:
    quaternion: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    gyroscope: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accelerometer: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    temperature: int = 0
    __idl_typename__: str = "unitree_go.msg.dds_.IMUState_"


@dataclass
class SportModeState_:
    stamp: TimeSpec_ = field(default_factory=TimeSpec_)
    error_code: int = 0
    imu_state: IMUState_ = field(default_factory=IMUState_)
    mode: int = 0
    progress: float = 0.0
    gait_type: int = 0
    foot_raise_height: float = 0.0
    position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    body_height: float = 0.0
    velocity: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    yaw_speed: float = 0.0
    range_obstacle: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    foot_force: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    foot_position_body: list[float] = field(default_factory=lambda: [0.0 for _ in range(12)])
    foot_speed_body: list[float] = field(default_factory=lambda: [0.0 for _ in range(12)])
    path_point: list[PathPoint_] = field(default_factory=lambda: [PathPoint_() for _ in range(10)])
    __idl_typename__: str = "unitree_go.msg.dds_.SportModeState_"


@dataclass
class WirelessController_:
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    keys: int = 0
    __idl_typename__: str = "unitree_go.msg.dds_.WirelessController_"


@dataclass
class BmsCmd_:
    off: int = 0
    reserve: list[int] = field(default_factory=lambda: [0, 0, 0])
    __idl_typename__: str = "unitree_go.msg.dds_.BmsCmd_"


@dataclass
class BmsState_:
    version_high: int = 0
    version_low: int = 0
    status: int = 0
    soc: int = 0
    current: int = 0
    cycle: int = 0
    bq_ntc: list[int] = field(default_factory=lambda: [0, 0])
    mcu_ntc: list[int] = field(default_factory=lambda: [0, 0])
    cell_vol: list[int] = field(default_factory=lambda: [0 for _ in range(15)])
    __idl_typename__: str = "unitree_go.msg.dds_.BmsState_"


@dataclass
class MotorCmd_:
    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    __idl_typename__: str = "unitree_go.msg.dds_.MotorCmd_"


@dataclass
class MotorState_:
    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    ddq: float = 0.0
    tau_est: float = 0.0
    q_raw: float = 0.0
    dq_raw: float = 0.0
    ddq_raw: float = 0.0
    temperature: int = 0
    lost: int = 0
    reserve: list[int] = field(default_factory=lambda: [0, 0])
    __idl_typename__: str = "unitree_go.msg.dds_.MotorState_"


@dataclass
class LowCmd_:
    head: list[int] = field(default_factory=lambda: [0xFE, 0xEF])
    level_flag: int = 0
    frame_reserve: int = 0
    sn: list[int] = field(default_factory=lambda: [0, 0])
    version: list[int] = field(default_factory=lambda: [0, 0])
    bandwidth: int = 0
    motor_cmd: list[MotorCmd_] = field(default_factory=lambda: [MotorCmd_() for _ in range(20)])
    bms_cmd: BmsCmd_ = field(default_factory=BmsCmd_)
    wireless_remote: list[int] = field(default_factory=lambda: [0 for _ in range(40)])
    led: list[int] = field(default_factory=lambda: [0 for _ in range(12)])
    fan: list[int] = field(default_factory=lambda: [0, 0])
    gpio: int = 0
    reserve: int = 0
    crc: int = 0
    __idl_typename__: str = "unitree_go.msg.dds_.LowCmd_"


@dataclass
class LowState_:
    head: list[int] = field(default_factory=lambda: [0xFE, 0xEF])
    level_flag: int = 0
    frame_reserve: int = 0
    sn: list[int] = field(default_factory=lambda: [0, 0])
    version: list[int] = field(default_factory=lambda: [0, 0])
    bandwidth: int = 0
    imu_state: IMUState_ = field(default_factory=IMUState_)
    motor_state: list[MotorState_] = field(default_factory=lambda: [MotorState_() for _ in range(20)])
    bms_state: BmsState_ = field(default_factory=BmsState_)
    foot_force: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    foot_force_est: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    tick: int = 0
    wireless_remote: list[int] = field(default_factory=lambda: [0 for _ in range(40)])
    bit_flag: int = 0
    adc_reel: float = 0.0
    temperature_ntc1: int = 0
    temperature_ntc2: int = 0
    power_v: float = 0.0
    power_a: float = 0.0
    fan_frequency: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    reserve: int = 0
    crc: int = 0
    __idl_typename__: str = "unitree_go.msg.dds_.LowState_"


@dataclass
class MotorCmds_:
    cmds: list[MotorCmd_] = field(default_factory=lambda: [MotorCmd_() for _ in range(4)])
    __idl_typename__: str = "unitree_go.msg.dds_.MotorCmds_"


__all__ = [
    "BmsCmd_",
    "BmsState_",
    "IMUState_",
    "LowCmd_",
    "LowState_",
    "MotorCmd_",
    "MotorCmds_",
    "MotorState_",
    "PathPoint_",
    "SportModeState_",
    "TimeSpec_",
    "WirelessController_",
]
