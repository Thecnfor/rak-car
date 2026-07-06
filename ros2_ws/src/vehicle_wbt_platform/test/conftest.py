"""Shared pytest fixtures for vehicle_wbt_platform tests.

This conftest makes the test suite runnable on developer machines WITHOUT
real hardware. Two problems are solved here:

1. **MC602 adapter tests must not trigger real `vehicle` import.** The
   `MC602Adapter.open()` method does `from vehicle.base.controller_wrap
   import MC602`. On the Jetson this works (controller wired up). On a
   dev machine, `import vehicle` triggers numpy import + serial port scan
   + firmware probe (per CLAUDE.md "Import-time side effects" warning).
   We pre-inject a fake `vehicle` module into `sys.modules` so the import
   succeeds without touching real hardware.

2. **Orchestrator tests need repo-root absolute path to config_sensors.yml.**
   Tests use `config_path="config_sensors.yml"` (relative). Pytest's default
   cwd is the test directory, but config_sensors.yml lives at repo root.
   We compute the absolute path and pass it via the `repo_config` fixture.
"""

from __future__ import annotations

import os
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest


# ---------------------------------------------------------------------------
# Repo-root path helpers
# ---------------------------------------------------------------------------

_REPO_ROOT = Path(__file__).resolve().parents[4]  # test/ -> vehicle_wbt_platform/ -> src/ -> ros2_ws/ -> <repo>
assert _REPO_ROOT.name == "rak-car", (
    f"conftest.py: expected _REPO_ROOT to be the rak-car repo root, got {_REPO_ROOT}. "
    f"Path depth may have changed — adjust parents[N] if the package moved."
)
_DEFAULT_CONFIG = _REPO_ROOT / "config_sensors.yml"


@pytest.fixture
def repo_config_path() -> str:
    """Absolute path to config_sensors.yml at repo root. Tests should use this instead
    of hard-coding "config_sensors.yml" so they work regardless of pytest cwd."""
    return str(_DEFAULT_CONFIG)


# ---------------------------------------------------------------------------
# Mock the `vehicle` package so MC602Adapter.open() does not trigger real HW
# ---------------------------------------------------------------------------

def _build_fake_vehicle_module() -> types.ModuleType:
    """Build a fake `vehicle.base.controller_wrap` module graph sufficient for
    MC602Adapter to instantiate a MagicMock-backed MC602 without hardware."""
    vehicle_pkg = types.ModuleType("vehicle")
    vehicle_pkg.__path__ = []  # mark as package
    vehicle_base = types.ModuleType("vehicle.base")
    vehicle_base.__path__ = []
    controller_wrap = types.ModuleType("vehicle.base.controller_wrap")

    # MC602 mock — every attribute is itself a MagicMock so adapter can call any
    # method (infrared_read, motor_set_speed, etc.) without raising AttributeError.
    controller_wrap.MC602 = MagicMock(name="MC602")
    # When MC602(port=..., baud=...) is called, return another MagicMock so the
    # adapter can chain methods on the instance.
    controller_wrap.MC602.side_effect = lambda **kwargs: MagicMock(name=f"MC602_instance({kwargs})")

    sys.modules["vehicle"] = vehicle_pkg
    sys.modules["vehicle.base"] = vehicle_base
    sys.modules["vehicle.base.controller_wrap"] = controller_wrap
    return controller_wrap


@pytest.fixture(autouse=True)
def fake_vehicle_module():
    """Pre-inject a fake `vehicle` module into sys.modules before any test runs.

    This is autouse so the MC602 adapter tests do not need to remember to
    request it. On the Jetson (real hardware), this is a no-op because
    `import vehicle` would replace the fake with the real one — but those
    tests are expected to run on the Jetson only via integration tests.
    """
    was_real = "vehicle" in sys.modules
    saved = {k: sys.modules[k] for k in list(sys.modules) if k == "vehicle" or k.startswith("vehicle.")}
    # Remove any pre-existing real `vehicle.*` so our fake takes precedence.
    for k in list(sys.modules):
        if k == "vehicle" or k.startswith("vehicle."):
            del sys.modules[k]
    _build_fake_vehicle_module()
    try:
        yield
    finally:
        # Restore: remove fakes; put back real ones if they existed.
        for k in list(sys.modules):
            if k == "vehicle" or k.startswith("vehicle."):
                del sys.modules[k]
        if was_real:
            for k, v in saved.items():
                sys.modules[k] = v


# ---------------------------------------------------------------------------
# Pre-existing fixtures (kept for forward compatibility)
# ---------------------------------------------------------------------------

@pytest.fixture
def enable_ros2(monkeypatch):
    """Set ENABLE_ROS2=1 for tests that require ROS2 to be active."""
    monkeypatch.setenv("ENABLE_ROS2", "1")
    yield


@pytest.fixture
def disable_ros2(monkeypatch):
    """Ensure ENABLE_ROS2 is unset for tests verifying zero-impact behavior."""
    monkeypatch.delenv("ENABLE_ROS2", raising=False)
    yield
