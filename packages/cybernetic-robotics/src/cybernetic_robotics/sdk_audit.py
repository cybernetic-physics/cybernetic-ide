from __future__ import annotations

import ast
from dataclasses import dataclass
import importlib
from pathlib import Path
from typing import Any


DEFAULT_UPSTREAM_ROOT = Path("/Users/cuboniks/wagmi/unitree_sdk2_python")
G1_EXAMPLE_GLOBS = [
    "example/g1/high_level/*.py",
    "example/g1/low_level/*.py",
]


@dataclass
class ExampleAudit:
    path: str
    imports: list[dict[str, Any]]
    classes: list[dict[str, Any]]
    method_calls: list[dict[str, Any]]
    topics: list[dict[str, Any]]

    @property
    def supported(self) -> bool:
        return all(item["supported"] for item in self.imports + self.classes + self.method_calls + self.topics)


def audit_official_g1_examples(upstream_root: str | Path = DEFAULT_UPSTREAM_ROOT) -> dict[str, Any]:
    """Compare official Unitree G1 SDK2 examples with Cybernetic's SDK shim."""

    root = Path(upstream_root)
    examples = [_audit_example(path, root) for path in _g1_example_paths(root)]
    supported = sum(1 for item in examples if item.supported)
    return {
        "ok": root.exists(),
        "source": "unitree_sdk2_python_official_examples",
        "upstream_root": str(root),
        "example_count": len(examples),
        "fully_supported_examples": supported,
        "partially_supported_examples": len(examples) - supported,
        "examples": [
            {
                "path": item.path,
                "supported": item.supported,
                "imports": item.imports,
                "classes": item.classes,
                "method_calls": item.method_calls,
                "topics": item.topics,
            }
            for item in examples
        ],
        "summary": _summarize(examples),
    }


def _g1_example_paths(root: Path) -> list[Path]:
    paths: list[Path] = []
    for pattern in G1_EXAMPLE_GLOBS:
        paths.extend(root.glob(pattern))
    return sorted(path for path in paths if path.is_file())


def _audit_example(path: Path, root: Path) -> ExampleAudit:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imports = _import_reports(tree)
    assignments = _client_assignments(tree)
    class_reports = _class_reports(assignments)
    method_reports = _method_reports(tree, assignments)
    topic_reports = _topic_reports(tree)
    return ExampleAudit(
        path=str(path.relative_to(root)),
        imports=imports,
        classes=class_reports,
        method_calls=method_reports,
        topics=topic_reports,
    )


def _import_reports(tree: ast.AST) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module and node.module.startswith("unitree_sdk2py"):
            module_supported = _module_supported(node.module)
            for alias in node.names:
                supported = module_supported and _symbol_supported(node.module, alias.name)
                reports.append(
                    {
                        "module": node.module,
                        "name": alias.name,
                        "supported": supported,
                        "status": "supported" if supported else "missing_or_unsupported",
                    }
                )
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.startswith("unitree_sdk2py"):
                    supported = _module_supported(alias.name)
                    reports.append(
                        {
                            "module": alias.name,
                            "name": alias.asname or alias.name,
                            "supported": supported,
                            "status": "supported" if supported else "missing_or_unsupported",
                        }
                    )
    return reports


def _client_assignments(tree: ast.AST) -> dict[str, str]:
    assignments: dict[str, str] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or not isinstance(node.value, ast.Call):
            continue
        class_name = _call_name(node.value.func)
        if not class_name:
            continue
        for target in node.targets:
            target_name = _owner_name(target)
            if target_name:
                assignments[target_name] = class_name
    return assignments


def _class_reports(assignments: dict[str, str]) -> list[dict[str, Any]]:
    seen = sorted(set(assignments.values()))
    return [
        {
            "class": name,
            "supported": _class_supported(name),
            "status": "supported" if _class_supported(name) else "missing_or_unsupported",
        }
        for name in seen
        if name in _known_official_classes()
    ]


def _method_reports(tree: ast.AST, assignments: dict[str, str]) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    seen: set[tuple[str, str, str]] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        owner = _owner_name(node.func.value)
        if not owner:
            continue
        class_name = assignments.get(owner)
        if not class_name or class_name not in _known_official_classes():
            continue
        key = (owner, class_name, node.func.attr)
        if key in seen:
            continue
        seen.add(key)
        supported = _method_supported(class_name, node.func.attr)
        reports.append(
            {
                "variable": owner,
                "class": class_name,
                "method": node.func.attr,
                "supported": supported,
                "status": "supported" if supported else "missing_or_unsupported",
            }
        )
    return sorted(reports, key=lambda item: (item["class"], item["method"], item["variable"]))


def _topic_reports(tree: ast.AST) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    seen: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        call_name = _call_name(node.func)
        if call_name not in {"ChannelPublisher", "ChannelSubscriber"} or not node.args:
            continue
        topic = _literal_string(node.args[0])
        if topic is None or topic in seen:
            continue
        seen.add(topic)
        supported = topic in _supported_topics()
        reports.append(
            {
                "topic": topic,
                "direction": "publish" if call_name == "ChannelPublisher" else "subscribe",
                "supported": supported,
                "status": "supported" if supported else "missing_or_unsupported",
            }
        )
    return sorted(reports, key=lambda item: (item["topic"], item["direction"]))


def _module_supported(module: str) -> bool:
    try:
        importlib.import_module(module)
    except Exception:
        return False
    return True


def _symbol_supported(module: str, name: str) -> bool:
    if name == "*":
        return _module_supported(module)
    try:
        mod = importlib.import_module(module)
    except Exception:
        return False
    return hasattr(mod, name)


def _class_supported(class_name: str) -> bool:
    return _class_object(class_name) is not None


def _method_supported(class_name: str, method: str) -> bool:
    cls = _class_object(class_name)
    return bool(cls and hasattr(cls, method))


def _class_object(class_name: str):
    for module_name in _shim_class_modules().get(class_name, []):
        try:
            module = importlib.import_module(module_name)
        except Exception:
            continue
        candidate = getattr(module, class_name, None)
        if candidate is not None:
            return candidate
    return None


def _call_name(value: ast.AST) -> str | None:
    if isinstance(value, ast.Name):
        return value.id
    if isinstance(value, ast.Attribute):
        return value.attr
    return None


def _literal_string(value: ast.AST) -> str | None:
    if isinstance(value, ast.Constant) and isinstance(value.value, str):
        return value.value
    return None


def _owner_name(value: ast.AST) -> str | None:
    if isinstance(value, ast.Name):
        return value.id
    if isinstance(value, ast.Attribute):
        parent = _owner_name(value.value)
        return f"{parent}.{value.attr}" if parent else value.attr
    return None


def _known_official_classes() -> set[str]:
    return set(_shim_class_modules())


def _supported_topics() -> set[str]:
    return {
        "rt/arm_sdk",
        "rt/lowcmd",
        "rt/lowstate",
        "rt/sportmodestate",
        "rt/wirelesscontroller",
    }


def _shim_class_modules() -> dict[str, list[str]]:
    return {
        "G1ArmActionClient": ["unitree_sdk2py.g1.arm.g1_arm_action_client"],
        "LocoClient": ["unitree_sdk2py.g1.loco.g1_loco_client"],
        "MotionSwitcherClient": ["unitree_sdk2py.comm.motion_switcher.motion_switcher_client"],
        "ChannelPublisher": ["unitree_sdk2py.core.channel"],
        "ChannelSubscriber": ["unitree_sdk2py.core.channel"],
        "CRC": ["unitree_sdk2py.utils.crc"],
        "RecurrentThread": ["unitree_sdk2py.utils.thread"],
    }


def _summarize(examples: list[ExampleAudit]) -> dict[str, Any]:
    missing_imports = _unique_missing(item for example in examples for item in example.imports)
    missing_classes = _unique_missing(item for example in examples for item in example.classes)
    missing_methods = _unique_missing(item for example in examples for item in example.method_calls)
    missing_topics = _unique_missing(item for example in examples for item in example.topics)
    return {
        "missing_imports": missing_imports,
        "missing_classes": missing_classes,
        "missing_methods": missing_methods,
        "missing_topics": missing_topics,
        "next_steps": _next_steps(missing_imports, missing_classes, missing_methods, missing_topics),
    }


def _unique_missing(items) -> list[dict[str, Any]]:
    seen = set()
    result = []
    for item in items:
        if item.get("supported"):
            continue
        key = tuple((name, item.get(name)) for name in sorted(item))
        if key in seen:
            continue
        seen.add(key)
        result.append(item)
    return result


def _next_steps(
    missing_imports: list[dict[str, Any]],
    missing_classes: list[dict[str, Any]],
    missing_methods: list[dict[str, Any]],
    missing_topics: list[dict[str, Any]],
) -> list[str]:
    steps = []
    if missing_imports:
        steps.append("Add or document missing unitree_sdk2py import surfaces used by official G1 examples.")
    if missing_classes:
        steps.append("Add simulator-safe shims for missing official SDK client classes before porting those examples.")
    if missing_methods:
        steps.append("Implement missing methods as explicit simulator approximations or mark them unsupported with clear errors.")
    if missing_topics:
        steps.append("Route or explicitly reject missing official DDS topics at the UnitreeSession boundary.")
    if not steps:
        steps.append("All inspected official G1 examples import, call, and use DDS topics available through Cybernetic SDK shim surfaces; run behavior-level validation next.")
    return steps
