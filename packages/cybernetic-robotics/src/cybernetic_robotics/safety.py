from __future__ import annotations

import math
import time
from typing import Any


DEFAULT_ORIENTATION_LIMIT_RAD = 1.0
DEFAULT_JOINT_VELOCITY_LIMIT = 10.0
DEFAULT_ANGULAR_VELOCITY_LIMIT = 6.0
DEFAULT_MOTOR_CASING_TEMP_LIMIT = 85.0
DEFAULT_MOTOR_WINDING_TEMP_LIMIT = 120.0


def evaluate_lowstate_safety(
    lowstate: dict[str, Any],
    *,
    status: dict[str, Any] | None = None,
    orientation_limit_rad: float = DEFAULT_ORIENTATION_LIMIT_RAD,
    joint_velocity_limit: float = DEFAULT_JOINT_VELOCITY_LIMIT,
    angular_velocity_limit: float = DEFAULT_ANGULAR_VELOCITY_LIMIT,
    motor_casing_temp_limit: float = DEFAULT_MOTOR_CASING_TEMP_LIMIT,
    motor_winding_temp_limit: float = DEFAULT_MOTOR_WINDING_TEMP_LIMIT,
) -> dict[str, Any]:
    """Evaluate Unitree G1-inspired runtime termination checks.

    This mirrors the intent of Unitree's `g1/common/terminations.hpp` using the
    simulator's JSON lowstate. It is read-only and conservative: failed or
    missing telemetry becomes a warning unless it directly proves an unsafe
    condition.
    """

    checks: list[dict[str, Any]] = []
    status_value = status if isinstance(status, dict) else {}
    simulation = status_value.get("simulation") if isinstance(status_value.get("simulation"), dict) else {}
    imu = lowstate.get("imu_state") if isinstance(lowstate.get("imu_state"), dict) else {}
    motors = lowstate.get("motor_state") if isinstance(lowstate.get("motor_state"), list) else []
    lowcmd = lowstate.get("lowcmd") if isinstance(lowstate.get("lowcmd"), dict) else {}

    orientation_angle = _orientation_angle(imu.get("quaternion"))
    _add_check(
        checks,
        "bad_orientation",
        orientation_angle is None or orientation_angle <= orientation_limit_rad,
        (
            "orientation unavailable"
            if orientation_angle is None
            else f"orientation angle {orientation_angle:.3f} rad within {orientation_limit_rad:.3f} rad"
        ),
        value=orientation_angle,
        limit=orientation_limit_rad,
        source="unitree_g1_terminations.bad_orientation",
    )

    max_joint_velocity = _max_abs_motor_field(motors, "dq")
    _add_check(
        checks,
        "joint_vel_out_of_limit",
        max_joint_velocity is None or max_joint_velocity <= joint_velocity_limit,
        (
            "joint velocities unavailable"
            if max_joint_velocity is None
            else f"max joint velocity {max_joint_velocity:.3f} within {joint_velocity_limit:.3f}"
        ),
        value=max_joint_velocity,
        limit=joint_velocity_limit,
        source="unitree_g1_terminations.joint_vel_out_of_limit",
    )

    max_angular_velocity = _max_abs_sequence(imu.get("gyroscope"))
    _add_check(
        checks,
        "ang_vel_out_of_limit",
        max_angular_velocity is None or max_angular_velocity <= angular_velocity_limit,
        (
            "angular velocity unavailable"
            if max_angular_velocity is None
            else f"max angular velocity {max_angular_velocity:.3f} within {angular_velocity_limit:.3f}"
        ),
        value=max_angular_velocity,
        limit=angular_velocity_limit,
        source="unitree_g1_terminations.ang_vel_out_of_limit",
    )

    max_casing_temp = _max_temperature(motors, 0)
    _add_check(
        checks,
        "motor_casing_overheat",
        max_casing_temp is None or max_casing_temp <= motor_casing_temp_limit,
        (
            "motor casing temperatures unavailable"
            if max_casing_temp is None
            else f"max casing temperature {max_casing_temp:.1f}C within {motor_casing_temp_limit:.1f}C"
        ),
        value=max_casing_temp,
        limit=motor_casing_temp_limit,
        source="unitree_g1_terminations.motor_casing_overheat",
    )

    max_winding_temp = _max_temperature(motors, 1)
    _add_check(
        checks,
        "motor_winding_overheat",
        max_winding_temp is None or max_winding_temp <= motor_winding_temp_limit,
        (
            "motor winding temperatures unavailable"
            if max_winding_temp is None
            else f"max winding temperature {max_winding_temp:.1f}C within {motor_winding_temp_limit:.1f}C"
        ),
        value=max_winding_temp,
        limit=motor_winding_temp_limit,
        source="unitree_g1_terminations.motor_winding_overheat",
    )

    lowcmd_stale = bool(lowcmd.get("stale"))
    _add_check(
        checks,
        "lowcmd_stale",
        not lowcmd_stale,
        "lowcmd watchdog reports stale command" if lowcmd_stale else "lowcmd watchdog is not stale",
        value=lowcmd_stale,
        source="cybernetic_simulator.lowcmd_watchdog",
    )

    fallen = bool(simulation.get("fallen"))
    _add_check(
        checks,
        "fallen",
        not fallen,
        "simulator reports robot fallen" if fallen else "simulator does not report a fall",
        value=fallen,
        source="cybernetic_simulator.status",
    )

    failed = [check for check in checks if not check["ok"]]
    return {
        "ok": len(failed) == 0,
        "safe_to_command": len(failed) == 0,
        "source": "unitree_g1_terminations_json_lowstate",
        "checked_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "limits": {
            "orientation_limit_rad": orientation_limit_rad,
            "joint_velocity_limit": joint_velocity_limit,
            "angular_velocity_limit": angular_velocity_limit,
            "motor_casing_temp_limit": motor_casing_temp_limit,
            "motor_winding_temp_limit": motor_winding_temp_limit,
        },
        "checks": checks,
        "failed_checks": failed,
        "recommendation": "continue" if not failed else "call safety_stop before issuing more motion",
    }


def _add_check(checks: list[dict[str, Any]], name: str, ok: bool, message: str, **extra: Any) -> None:
    checks.append({"name": name, "ok": bool(ok), "message": message, **extra})


def _orientation_angle(quaternion: Any) -> float | None:
    if not isinstance(quaternion, list) or len(quaternion) < 4:
        return None
    w, x, y, z = (_to_float(item) for item in quaternion[:4])
    if None in {w, x, y, z}:
        return None
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0:
        return None
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    # Rotate world gravity (0, 0, -1) into the body frame using q.conjugate().
    projected_z = -1.0 + 2.0 * (x * x + y * y)
    projected_z = max(-1.0, min(1.0, projected_z))
    return math.acos(max(-1.0, min(1.0, -projected_z)))


def _max_abs_motor_field(motors: list[Any], field: str) -> float | None:
    values = []
    for motor in motors:
        if isinstance(motor, dict):
            value = _to_float(motor.get(field))
            if value is not None:
                values.append(abs(value))
    return max(values) if values else None


def _max_abs_sequence(value: Any) -> float | None:
    if not isinstance(value, list):
        return None
    values = [_to_float(item) for item in value]
    numbers = [abs(item) for item in values if item is not None]
    return max(numbers) if numbers else None


def _max_temperature(motors: list[Any], index: int) -> float | None:
    values = []
    for motor in motors:
        if not isinstance(motor, dict):
            continue
        temperature = motor.get("temperature")
        if isinstance(temperature, list) and len(temperature) > index:
            value = _to_float(temperature[index])
            if value is not None:
                values.append(value)
    return max(values) if values else None


def _to_float(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
