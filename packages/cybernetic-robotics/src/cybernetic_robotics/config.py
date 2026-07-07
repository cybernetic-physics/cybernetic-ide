from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from urllib.parse import urlparse


DEFAULT_GAME_CONTROL_URL = "http://127.0.0.1:38383"
DEFAULT_PHYSICS_URL = "ws://127.0.0.1:8788"


@dataclass(frozen=True)
class RobotEndpoints:
    """Network endpoints for the local Cybernetic G1 simulator."""

    game_control_url: str = DEFAULT_GAME_CONTROL_URL
    physics_url: str = DEFAULT_PHYSICS_URL

    @classmethod
    def from_env(cls) -> "RobotEndpoints":
        game_control_url = os.environ.get("CYBER_G1_GAME_CONTROL_URL", DEFAULT_GAME_CONTROL_URL)
        physics_url = os.environ.get("CYBER_G1_PHYSICS_URL")
        if not physics_url:
            host = os.environ.get("CYBER_G1_WS_HOST", "127.0.0.1")
            port = os.environ.get("CYBER_G1_WS_PORT", "8788")
            physics_url = f"ws://{host}:{port}"
        return cls(
            game_control_url=game_control_url.rstrip("/"),
            physics_url=physics_url,
        )

    @property
    def ws_host(self) -> str:
        parsed = urlparse(self.physics_url)
        return parsed.hostname or "127.0.0.1"

    @property
    def ws_port(self) -> int:
        parsed = urlparse(self.physics_url)
        return parsed.port or 8788

    @property
    def ws_path(self) -> str:
        parsed = urlparse(self.physics_url)
        return parsed.path or "/"


def find_robotics_root(start: str | Path | None = None) -> Path:
    """Find the Cybernetic IDE checkout that owns the simulator harness.

    The package can be installed into arbitrary Python environments. Harness
    operations still need the repo because Docker Compose files, prepare
    scripts, and runtime assets live there.
    """

    env_root = os.environ.get("CYBER_ROBOTICS_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve()

    current = Path(start or os.getcwd()).expanduser().resolve()
    for candidate in [current, *current.parents]:
        if (candidate / "overlays/unitree-g1-mujoco-protocol/Dockerfile").exists():
            return candidate
    return current
