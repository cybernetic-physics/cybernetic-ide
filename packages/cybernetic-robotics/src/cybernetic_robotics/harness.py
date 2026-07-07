from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess

from .config import find_robotics_root


@dataclass(frozen=True)
class DockerHarness:
    """Small wrapper around the repo's Unitree G1 Docker harness."""

    root: Path

    @classmethod
    def discover(cls, start: str | Path | None = None) -> "DockerHarness":
        return cls(find_robotics_root(start))

    @property
    def compose_env(self) -> Path:
        return self.root / ".runtime/unitree-g1-mujoco/compose.env"

    @property
    def compose_file(self) -> Path:
        return self.root / "overlays/unitree-g1-mujoco-container/compose.yaml"

    def prepare(self) -> subprocess.CompletedProcess[str]:
        return self._run(["node", "script/prepare-unitree-g1-mujoco-container.mjs"])

    def start(self) -> subprocess.CompletedProcess[str]:
        return self._run(["docker", *self._compose_args(), "up", "-d"])

    def stop(self) -> subprocess.CompletedProcess[str]:
        return self._run(["docker", *self._compose_args(), "stop", "unitree-g1-mujoco"])

    def logs(self, tail: int = 120) -> subprocess.CompletedProcess[str]:
        return self._run(["docker", "logs", "--tail", str(tail), "unitree-g1-mujoco"])

    def _compose_args(self) -> list[str]:
        return ["compose", "--env-file", str(self.compose_env), "-f", str(self.compose_file)]

    def _run(self, args: list[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            args,
            cwd=self.root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True,
        )
