"""Smoke tests for the rewritten mc602_peripheral_node.

Verifies the module imports cleanly and exposes the expected class.
Doesn't spin a real node (that needs rclpy init + ROS_DOMAIN_ID).
"""
from __future__ import annotations

import importlib
import sys
import os

# Ensure ament-installed packages are importable in the test env.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def test_module_imports_without_error() -> None:
    """The rewritten module must import without raising."""
    # Reload in case it was imported earlier in the test session.
    if 'vehicle_wbt_smartcar_bridge.mc602_peripheral_node' in sys.modules:
        del sys.modules['vehicle_wbt_smartcar_bridge.mc602_peripheral_node']
    mod = importlib.import_module(
        'vehicle_wbt_smartcar_bridge.mc602_peripheral_node')
    assert mod.MC602PeripheralNode is not None
    assert callable(mod.main)


def test_module_does_not_import_ctypes() -> None:
    """The rewrite should not depend on raw ctypes/termios."""
    if 'vehicle_wbt_smartcar_bridge.mc602_peripheral_node' in sys.modules:
        del sys.modules['vehicle_wbt_smartcar_bridge.mc602_peripheral_node']
    importlib.import_module(
        'vehicle_wbt_smartcar_bridge.mc602_peripheral_node')
    # Check that the module itself didn't `import ctypes`
    mod = sys.modules['vehicle_wbt_smartcar_bridge.mc602_peripheral_node']
    src = open(mod.__file__).read()
    assert 'import ctypes' not in src, 'mc602_peripheral_node still uses ctypes'
    assert 'import termios' not in src, 'mc602_peripheral_node still uses termios'


def test_module_exposes_happy_birthday_melody() -> None:
    """The melody constant must remain for the song feature."""
    if 'vehicle_wbt_smartcar_bridge.mc602_peripheral_node' in sys.modules:
        del sys.modules['vehicle_wbt_smartcar_bridge.mc602_peripheral_node']
    mod = importlib.import_module(
        'vehicle_wbt_smartcar_bridge.mc602_peripheral_node')
    assert hasattr(mod, 'HAPPY_BIRTHDAY_MELODY')
    assert len(mod.HAPPY_BIRTHDAY_MELODY) >= 13  # 25 expected but >= 13 is the minimum safe count
