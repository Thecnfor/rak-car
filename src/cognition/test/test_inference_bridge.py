# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
"""Smoke tests for the slimmed cognition (Python business layer).

The package now hosts only the ROS2 inference bridge. These tests are
deliberately ROS2-free: they must pass on a bare Python (no /opt/ros
sourced, no msgs build needed), so pytest test/
stays usable for fast local iteration.
"""

import ast
import pathlib

import cognition


def test_package_imports() -> None:
    """The top-level package imports and exposes a version."""
    assert cognition.__version__


def test_bridge_module_exists() -> None:
    """The inference bridge source file must be present."""
    bridge = (
        pathlib.Path(cognition.__file__).parent
        / "inference" / "bridge.py"
    )
    assert bridge.is_file(), "inference/bridge.py must exist"


def _console_scripts(setup_tree: ast.AST) -> list[str]:
    """Extract console_scripts from an AST of setup.py (no code execution)."""
    for node in ast.walk(setup_tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "setup"
        ):
            for kw in node.keywords:
                if kw.arg != "entry_points" or not isinstance(kw.value, ast.Dict):
                    continue
                for key, val in zip(kw.value.keys, kw.value.values):
                    if (
                        isinstance(key, ast.Constant)
                        and key.value == "console_scripts"
                        and isinstance(val, ast.List)
                    ):
                        return [
                            item.value
                            for item in val.elts
                            if isinstance(item, ast.Constant)
                        ]
    return []


def test_console_scripts_registered() -> None:
    """setup.py declares inference-bridge and NOT the deleted sidecar entry."""
    setup_py = pathlib.Path(__file__).resolve().parents[1] / "setup.py"
    tree = ast.parse(setup_py.read_text(encoding="utf-8"))

    scripts = _console_scripts(tree)
    assert scripts, "setup() must declare console_scripts"
    assert any("inference-bridge" in s for s in scripts), \
        "inference-bridge entry point must be declared"
    assert not any("sidecar" in s for s in scripts), \
        "deleted sidecar entry point must not be declared"
