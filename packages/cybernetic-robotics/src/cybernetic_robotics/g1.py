from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any

from .config import RobotEndpoints
from .simulator import SimulatorClient, SimulatorStatus


@dataclass(frozen=True)
class G1Status:
    """Small beginner-friendly view of the simulator status."""

    raw: SimulatorStatus

    @property
    def ready(self) -> bool:
        return self.raw.ready

    @property
    def pose(self) -> str | None:
        return self.raw.pose

    @property
    def paused(self) -> bool:
        return self.raw.paused

    @property
    def speed(self) -> float:
        return self.raw.speed

    @property
    def model_path(self) -> str | None:
        return self.raw.model_path

    @property
    def fallen(self) -> bool:
        return self.raw.fallen

    @property
    def pelvis_height(self) -> float | None:
        return self.raw.pelvis_height


class G1Robot:
    """Beginner-friendly handle for the local Unitree G1 MuJoCo simulator."""

    def __init__(self, simulator: SimulatorClient | None = None):
        self.sim = simulator or SimulatorClient.from_env()

    @classmethod
    def connect(
        cls,
        *,
        wait: bool = True,
        timeout: float = 10.0,
        endpoints: RobotEndpoints | None = None,
    ) -> "G1Robot":
        robot = cls(SimulatorClient(endpoints or RobotEndpoints.from_env()))
        if wait:
            robot.sim.wait_until_ready(timeout=timeout)
        return robot

    def __enter__(self) -> "G1Robot":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.pause()

    def status(self) -> G1Status:
        return G1Status(self.sim.status())

    def reset(self) -> dict[str, Any]:
        return self.sim.reset()

    def pause(self) -> dict[str, Any]:
        return self.sim.pause()

    def resume(self) -> dict[str, Any]:
        return self.sim.resume()

    def step(self, count: int = 1) -> list[dict[str, Any]]:
        return self.sim.step(count)

    def run_for(self, seconds: float) -> G1Status:
        self.resume()
        time.sleep(max(0.0, float(seconds)))
        self.pause()
        return self.status()

    def raise_right_hand(self) -> dict[str, Any]:
        return self.sim.pose("raise_right_hand")

    def release_arm(self) -> dict[str, Any]:
        return self.sim.pose("neutral")

    def neutral(self) -> dict[str, Any]:
        return self.release_arm()

    def pose(self, name: str, **kwargs: Any) -> dict[str, Any]:
        return self.sim.pose(name, **kwargs)

    def hold(self, name: str, teleport: bool = True) -> dict[str, Any]:
        """Physically hold a pose: motors PD-track it while physics runs."""

        return self.sim.hold_pose(name, teleport=teleport)

    def is_fallen(self) -> bool:
        return self.status().fallen

    def snapshot(self, path: str | Path, image_format: str | None = None) -> Path:
        return self.sim.snapshot(path, image_format=image_format)

    def orbit(self, dx: float, dy: float = 0.0):
        return self.sim.orbit(dx, dy)

    def pan(self, dx: float, dy: float = 0.0):
        return self.sim.pan(dx, dy)

    def zoom(self, delta: float):
        return self.sim.zoom(delta)

    def reset_camera(self):
        return self.sim.reset_camera()

    def demo(self, snapshot_dir: str | Path = ".runtime/g1-control-demo") -> dict[str, Any]:
        """Run a tiny, safe demo and save before/after images."""

        out_dir = Path(snapshot_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        before = self.snapshot(out_dir / "before.jpg")
        self.reset()
        self.raise_right_hand()
        after = self.snapshot(out_dir / "right-hand-up.jpg")
        return {
            "status": self.status(),
            "before": before,
            "after": after,
        }


def connect(**kwargs: Any) -> G1Robot:
    """Shortcut for `G1Robot.connect()`."""

    return G1Robot.connect(**kwargs)
