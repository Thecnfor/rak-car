from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

import pytest

from vehicle_wbt_platform.orchestrator import SidecarOrchestrator


SAMPLE_YAML = """
sensors:
  - {id: ir1, type: ir, port_id: 7, port_physical: P7, topic: /vehicle_wbt/v1/sensors/ir/left, msg_type: std_msgs/Float32, rate_hz: 20}
actuators: []
"""


def test_orchestrator_init_loads_config(tmp_path: Path) -> None:
    cfg = tmp_path / "cfg.yml"
    cfg.write_text(SAMPLE_YAML)
    orch = SidecarOrchestrator(
        config_path=str(cfg),
        serial_port="/dev/ttyUSB0",
        ros_domain_id=42,
    )
    assert orch.registry is not None
    assert "ir1" in orch.registry.sensors


def test_orchestrator_init_creates_adapter() -> None:
    orch = SidecarOrchestrator(
        config_path="config_sensors.yml",
        serial_port="/dev/ttyUSB0",
        ros_domain_id=42,
    )
    assert orch.adapter is not None
    assert orch.adapter.serial_port == "/dev/ttyUSB0"


def test_orchestrator_adapter_not_open_at_init() -> None:
    orch = SidecarOrchestrator(
        config_path="config_sensors.yml",
        serial_port="/dev/ttyUSB0",
        ros_domain_id=42,
    )
    assert not orch.adapter.is_open


def test_orchestrator_shutdown_is_safe_to_call_twice() -> None:
    orch = SidecarOrchestrator(
        config_path="config_sensors.yml",
        serial_port="/dev/ttyUSB0",
        ros_domain_id=42,
    )
    orch.shutdown()
    orch.shutdown()  # must not raise


def test_orchestrator_summary_reports_counts() -> None:
    orch = SidecarOrchestrator(
        config_path="config_sensors.yml",
        serial_port="/dev/ttyUSB0",
        ros_domain_id=42,
    )
    summary = orch.summary()
    assert "sensors=" in summary
    assert "actuators=" in summary
    assert "enabled=" in summary