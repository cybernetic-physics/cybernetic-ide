from __future__ import annotations

from dataclasses import dataclass, field

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_


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


__all__ = ["PathPoint_", "SportModeState_", "TimeSpec_", "WirelessController_"]
