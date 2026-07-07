from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os
import re
from xml.sax.saxutils import escape

from .config import find_robotics_root


@dataclass(frozen=True)
class SceneWorkspace:
    """File helper for safe MJCF scene-copy edits."""

    root: Path

    @classmethod
    def discover(cls, start: str | Path | None = None) -> "SceneWorkspace":
        return cls(find_robotics_root(start))

    @property
    def compose_env(self) -> Path:
        return self.root / ".runtime/unitree-g1-mujoco/compose.env"

    def env(self) -> dict[str, str]:
        values: dict[str, str] = {}
        if not self.compose_env.exists():
            return values
        for line in self.compose_env.read_text().splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith("#") or "=" not in stripped:
                continue
            key, value = stripped.split("=", 1)
            values[key] = value
        return values

    def active_model_paths(self) -> tuple[Path, str]:
        env = self.env()
        asset_root = Path(
            env.get(
                "UNITREE_G1_MUJOCO_ASSET_ROOT",
                str(self.root / ".runtime/unitree-g1-mujoco/unitree_mujoco"),
            )
        )
        container_model = env.get(
            "UNITREE_G1_MODEL_PATH",
            "/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml",
        )
        prefix = "/opt/unitree_mujoco/"
        if not container_model.startswith(prefix):
            raise ValueError(f"unsupported container model path: {container_model}")
        return asset_root / container_model[len(prefix) :], container_model

    def read_active_mjcf(self) -> str:
        host_path, _container_path = self.active_model_paths()
        return host_path.read_text()

    def add_box(
        self,
        name: str,
        position: tuple[float, float, float],
        size: tuple[float, float, float],
        rgba: tuple[float, float, float, float] = (0.2, 0.7, 1.0, 1.0),
        *,
        activate: bool = False,
    ) -> tuple[Path, str]:
        if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
            raise ValueError("scene object names may contain only letters, numbers, '_' and '-'")

        host_path, _container_path = self.active_model_paths()
        xml = host_path.read_text()
        marker = "</worldbody>"
        if marker not in xml:
            raise ValueError(f"{host_path} does not contain {marker}")

        scene_dir = host_path.parents[2] / "cybernetic_scenes"
        scene_dir.mkdir(parents=True, exist_ok=True)
        output = scene_dir / f"g1_{name}.xml"
        container_output = f"/opt/unitree_mujoco/cybernetic_scenes/{output.name}"
        body = (
            f'  <body name="{escape(name)}" pos="{_numbers(position)}">\n'
            f'    <geom name="{escape(name)}_geom" type="box" '
            f'size="{_numbers(size)}" rgba="{_numbers(rgba)}"/>\n'
            "  </body>"
        )
        output.write_text(xml.replace(marker, f"{body}\n{marker}"))

        if activate:
            self._update_env({"UNITREE_G1_MODEL_PATH": container_output})

        return output, container_output

    def _update_env(self, updates: dict[str, str]) -> None:
        env = self.env()
        env.update(updates)
        order = [
            "UNITREE_G1_MUJOCO_IMAGE",
            "UNITREE_G1_MUJOCO_PLATFORM",
            "UNITREE_G1_MUJOCO_ASSET_ROOT",
            "UNITREE_G1_MODEL_PATH",
            "UNITREE_G1_MODEL_REVISION",
            "UNITREE_G1_ROBOT_NAME",
            "UNITREE_G1_AUTORUN",
            "UNITREE_G1_FRAME_HZ",
            "UNITREE_G1_RENDER_HZ",
            "UNITREE_G1_RENDER_WIDTH",
            "UNITREE_G1_RENDER_HEIGHT",
        ]
        lines = [f"{key}={env[key]}" for key in order if key in env]
        self.compose_env.write_text(os.linesep.join(lines) + os.linesep)


def _numbers(values: tuple[float, ...]) -> str:
    return " ".join(f"{float(value):g}" for value in values)
