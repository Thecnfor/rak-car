"""Shared pytest fixtures for vehicle_wbt_platform tests."""

import os
import pytest


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
