"""Pure orbit-camera math shared by the physics container and the native renderer.

Operates on a plain dict with keys `lookat`/`distance`/`azimuth`/`elevation` so it
carries no dependency on `G1MujocoState`, `MjData`, or any locking scheme -- callers
own their own state and locks and just call these functions to mutate a snapshot.
"""

import math

import numpy as np


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def default_camera_state(lookat=(0.0, 0.0, 0.72), distance=3.2, azimuth=-130.0, elevation=-18.0):
    return {
        "lookat": [float(value) for value in lookat],
        "distance": float(distance),
        "azimuth": float(azimuth),
        "elevation": float(elevation),
    }


def camera_payload(camera):
    return {
        "cameraId": 0,
        "type": "free",
        "lookat": list(camera["lookat"]),
        "distance": float(camera["distance"]),
        "azimuth": float(camera["azimuth"]),
        "elevation": float(camera["elevation"]),
    }


def orbit(camera, dx=0.0, dy=0.0):
    camera["azimuth"] = float(camera["azimuth"]) + float(dx) * 0.28
    camera["elevation"] = clamp(float(camera["elevation"]) - float(dy) * 0.22, -85.0, 85.0)
    return camera


def pan(camera, dx=0.0, dy=0.0):
    azimuth = math.radians(float(camera["azimuth"]))
    right = np.array([math.cos(azimuth), math.sin(azimuth), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    scale = max(float(camera["distance"]), 0.5) * 0.0018
    lookat = np.array(camera["lookat"], dtype=float) + (
        -float(dx) * scale * right + float(dy) * scale * up
    )
    lookat[2] = clamp(float(lookat[2]), -0.5, 2.5)
    camera["lookat"] = lookat.tolist()
    return camera


def zoom(camera, delta=0.0):
    factor = math.exp(-float(delta) * 0.0018)
    camera["distance"] = clamp(float(camera["distance"]) * factor, 0.45, 12.0)
    return camera


def set_absolute(camera, updates):
    if "lookat" in updates:
        lookat = updates["lookat"]
        if not isinstance(lookat, list) or len(lookat) != 3:
            raise ValueError("camera.lookat must be a 3-element list")
        camera["lookat"] = [float(value) for value in lookat]
    if "distance" in updates:
        camera["distance"] = clamp(float(updates["distance"]), 0.45, 12.0)
    if "azimuth" in updates:
        camera["azimuth"] = float(updates["azimuth"]) % 360.0
    if "elevation" in updates:
        camera["elevation"] = clamp(float(updates["elevation"]), -89.0, 20.0)
    return camera


def apply_to_mjv_camera(camera, mjv_camera, mujoco_module):
    mjv_camera.type = mujoco_module.mjtCamera.mjCAMERA_FREE
    mjv_camera.lookat[:] = camera["lookat"]
    mjv_camera.distance = camera["distance"]
    mjv_camera.azimuth = camera["azimuth"]
    mjv_camera.elevation = camera["elevation"]
