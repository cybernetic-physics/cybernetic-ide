#!/usr/bin/env python3
"""Add a box obstacle to a copied Unitree G1 MuJoCo scene.

The script edits a generated MJCF copy, never the pinned upstream Unitree G1
asset. Use `--activate` when you want the local Docker harness to boot the
generated scene on its next restart.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "packages" / "cybernetic-robotics" / "src"

try:
    from cybernetic_robotics import G1Robot, SceneWorkspace
except ModuleNotFoundError:
    sys.path.insert(0, str(PACKAGE_SRC))
    from cybernetic_robotics import G1Robot, SceneWorkspace


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=REPO_ROOT,
        help="Cybernetic IDE checkout root.",
    )
    parser.add_argument("--name", default="agent_obstacle", help="MJCF-safe object name.")
    parser.add_argument(
        "--position",
        nargs=3,
        type=float,
        default=(0.85, 0.0, 0.08),
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--size",
        nargs=3,
        type=float,
        default=(0.12, 0.12, 0.08),
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--rgba",
        nargs=4,
        type=float,
        default=(0.9, 0.18, 0.1, 1.0),
        metavar=("R", "G", "B", "A"),
    )
    parser.add_argument(
        "--activate",
        action="store_true",
        help="Point compose.env at the generated scene copy.",
    )
    parser.add_argument(
        "--snapshot",
        type=Path,
        default=Path(".runtime/g1-control-demo/scene-obstacle.jpg"),
        help="Optional viewer screenshot path. Relative paths are resolved under --root.",
    )
    parser.add_argument(
        "--no-snapshot",
        action="store_true",
        help="Skip viewer screenshot capture.",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=None,
        help="JSON manifest path. Defaults to .runtime/g1-control-demo/scene-obstacle-manifest.json.",
    )
    parser.add_argument("--snapshot-timeout", type=float, default=3.0)
    args = parser.parse_args()

    root = args.root.resolve()
    workspace = SceneWorkspace(root)
    host_model_path, container_model_path = workspace.add_box(
        args.name,
        position=tuple(args.position),
        size=tuple(args.size),
        rgba=tuple(args.rgba),
        activate=args.activate,
    )

    manifest: dict[str, Any] = {
        "ok": True,
        "object": {
            "name": args.name,
            "type": "box",
            "position": list(args.position),
            "size": list(args.size),
            "rgba": list(args.rgba),
        },
        "scene": {
            "host_model_path": str(host_model_path),
            "container_model_path": container_model_path,
            "activated": args.activate,
            "activation_note": "Restart the Unitree G1 MuJoCo harness for an activated scene to load.",
        },
        "snapshot": None,
    }

    if not args.no_snapshot:
        snapshot_path = _resolve_under_root(root, args.snapshot)
        try:
            snapshot_path.parent.mkdir(parents=True, exist_ok=True)
            with G1Robot.connect(timeout=args.snapshot_timeout) as robot:
                robot.reset_camera()
                robot.orbit(dx=35, dy=-10)
                robot.snapshot(snapshot_path)
            manifest["snapshot"] = str(snapshot_path)
        except Exception as exc:  # noqa: BLE001
            manifest["snapshot_error"] = f"{type(exc).__name__}: {exc}"

    manifest_path = _resolve_under_root(
        root,
        args.manifest or Path(".runtime/g1-control-demo/scene-obstacle-manifest.json"),
    )
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print(
        json.dumps(
            {"ok": True, "manifest": str(manifest_path), "scene": manifest["scene"]},
            indent=2,
        )
    )
    return 0


def _resolve_under_root(root: Path, path: Path) -> Path:
    return path if path.is_absolute() else root / path


if __name__ == "__main__":
    raise SystemExit(main())
