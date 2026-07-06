from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import pytest

from vehicle_wbt_platform.controller_base import (
    BaseControllerHardwareInterface,
    ControllerError,
)
from vehicle_wbt_platform.mc602_adapter import MC602Adapter


def test_base_cannot_be_instantiated_directly() -> None:
    with pytest.raises(TypeError):
        BaseControllerHardwareInterface()  # type: ignore[abstract]


def test_mc602_adapter_init_with_serial_path() -> None:
    adapter = MC602Adapter(serial_port="/dev/ttyUSB0", baud=1000000)
    assert adapter.serial_port == "/dev/ttyUSB0"
    assert adapter.baud == 1000000
    assert not adapter.is_open


def test_mc602_adapter_open_close() -> None:
    adapter = MC602Adapter(serial_port="/dev/ttyUSB0", baud=1000000)
    adapter.open()
    try:
        assert adapter.is_open
    finally:
        adapter.close()
    assert not adapter.is_open


def test_mc602_adapter_read_ir_returns_float() -> None:
    adapter = MC602Adapter(serial_port="/dev/ttyUSB0", baud=1000000)
    adapter.open()
    try:
        # Stub the underlying MC602 read; we don't want real hardware in tests.
        adapter._mc602 = MagicMock()
        adapter._mc602.infrared_read.return_value = 0.234
        val = adapter.read_sensor(port_id=7, sensor_type="ir")
        assert val == pytest.approx(0.234)
    finally:
        adapter.close()


def test_mc602_adapter_read_unknown_sensor_type_raises() -> None:
    adapter = MC602Adapter(serial_port="/dev/ttyUSB0", baud=1000000)
    adapter.open()
    try:
        with pytest.raises(ControllerError, match="unsupported sensor type"):
            adapter.read_sensor(port_id=1, sensor_type="tachyon_beam")
    finally:
        adapter.close()


def test_mc602_adapter_enumerate_ports() -> None:
    adapter = MC602Adapter(serial_port="/dev/ttyUSB0", baud=1000000)
    ports = adapter.enumerate_ports()
    assert "motor" in ports and ports["motor"] == 6
    assert "servo" in ports and ports["servo"] == 7
    assert "stepper" in ports and ports["stepper"] == 3
    assert "io" in ports and ports["io"] == 8


def test_mc602_adapter_write_actuator_idempotent_under_double_call() -> None:
    adapter = MC602Adapter(serial_port="/dev/ttyUSB0", baud=1000000)
    adapter.open()
    try:
        adapter._mc602 = MagicMock()
        adapter.write_actuator(port_id=5, actuator_type="motor", value=0.5)
        adapter.write_actuator(port_id=5, actuator_type="motor", value=0.5)
        assert adapter._mc602.motor_set_speed.call_count == 2
    finally:
        adapter.close()
