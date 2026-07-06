"""MC602Adapter — concrete adapter for Waveshare MC602 motor controller.

Spec ref: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §硬件接口插件

This adapter wraps the existing `vehicle.controller_wrap.MC602` class — it does
NOT reimplement the protocol. It exposes the BaseControllerHardwareInterface
contract so sidecar components stay controller-agnostic.

Adding a future controller (e.g., MC603) = copy this file, change the wrapped
class, add a new console_script entry in setup.py. No sidecar code changes.
"""

from __future__ import annotations

import logging
import math
from typing import Any, Optional

from vehicle_wbt_platform.controller_base import (
    BaseControllerHardwareInterface,
    ControllerError,
)


_logger = logging.getLogger(__name__)


# Map of sensor_type strings to MC602 method names on the underlying controller_wrap
# wrapper. Extending this map is how new sensor modes get supported.
_SENSOR_TYPE_TO_METHOD = {
    "ir": "infrared_read",
    "analog_input": "analog_input_read",
    "ultrasonic": "ultrasonic_read",
    "touch": "touch_read",
    "ambient_light": "ambient_light_read",
}

# Map of actuator_type strings to MC602 write method names.
_ACTUATOR_TYPE_TO_METHOD = {
    "motor": "motor_set_speed",
    "servo_bus": "servo_bus_set",
    "servo_pwm": "servo_pwm_set",
    "stepper": "stepper_goto",
    "dout": "dout_set",
}


class MC602Adapter(BaseControllerHardwareInterface):
    """Adapter for MC602 over CH340 USB serial."""

    # MC602 physical port counts (per hardware-port-mapping.md)
    _MOTOR_MAX = 6
    _SERVO_MAX = 7
    _STEPPER_MAX = 3
    _IO_MAX = 8

    def __init__(self, *, serial_port: str, baud: int = 1_000_000) -> None:
        if baud not in (380_400, 1_000_000, 115_200):
            raise ControllerError(
                f"unsupported baud {baud}; MC602 supports 380400/1000000/115200"
            )
        self._serial_port = serial_port
        self._baud = baud
        self._mc602: Optional[Any] = None  # lazy — actual MC602 wrapper from vehicle

    # --- BaseControllerHardwareInterface ---

    @property
    def is_open(self) -> bool:
        return self._mc602 is not None

    @property
    def serial_port(self) -> str:
        return self._serial_port

    @property
    def baud(self) -> int:
        return self._baud

    def open(self) -> None:
        if self.is_open:
            return
        # Lazy import to keep this module testable without vehicle package.
        try:
            from vehicle.base.controller_wrap import MC602 as _MC602  # type: ignore
        except ImportError as e:
            raise ControllerError(
                f"cannot import vehicle.base.controller_wrap.MC602 — is vehicle package on PYTHONPATH? {e}"
            )
        self._mc602 = _MC602(port=self._serial_port, baud=self._baud)
        _logger.info("MC602Adapter opened %s @ %d", self._serial_port, self._baud)

    def close(self) -> None:
        if self._mc602 is None:
            return
        try:
            self._mc602.close()
        except Exception as e:  # noqa: BLE001 — close() must not raise on already-broken HW
            _logger.warning("MC602 close raised %r, ignoring", e)
        self._mc602 = None

    def read_sensor(self, *, port_id: int, sensor_type: str) -> float:
        self._require_open()
        method_name = _SENSOR_TYPE_TO_METHOD.get(sensor_type)
        if method_name is None:
            raise ControllerError(
                f"unsupported sensor type {sensor_type!r}; "
                f"supported: {sorted(_SENSOR_TYPE_TO_METHOD)}"
            )
        # All P口 modes share the same physical port space (1..8).
        if not (1 <= int(port_id) <= self._IO_MAX):
            raise ControllerError(
                f"port_id {port_id} out of range for sensor; must be in [1, {self._IO_MAX}]"
            )
        method = getattr(self._mc602, method_name, None)
        if method is None:
            raise ControllerError(
                f"MC602 has no method {method_name!r} (sensor type {sensor_type!r})"
            )
        return float(method(port_id=port_id))

    def write_actuator(self, *, port_id: int, actuator_type: str, value: float) -> None:
        self._require_open()
        method_name = _ACTUATOR_TYPE_TO_METHOD.get(actuator_type)
        if method_name is None:
            raise ControllerError(
                f"unsupported actuator type {actuator_type!r}; "
                f"supported: {sorted(_ACTUATOR_TYPE_TO_METHOD)}"
            )
        # Per-actuator-type port count caps from hardware-port-mapping.md.
        max_ports_for_type = {
            "motor": self._MOTOR_MAX,
            "servo_bus": self._SERVO_MAX,
            "servo_pwm": self._SERVO_MAX,
            "stepper": self._STEPPER_MAX,
            "dout": self._IO_MAX,
        }
        max_ports = max_ports_for_type[actuator_type]
        if not (1 <= int(port_id) <= max_ports):
            raise ControllerError(
                f"port_id {port_id} out of range for actuator type {actuator_type!r}; "
                f"must be in [1, {max_ports}]"
            )
        if not math.isfinite(float(value)):
            raise ControllerError(
                f"value must be a finite number, got {value!r}"
            )
        method = getattr(self._mc602, method_name, None)
        if method is None:
            raise ControllerError(
                f"MC602 has no method {method_name!r} (actuator type {actuator_type!r})"
            )
        method(port_id=port_id, value=float(value))

    def enumerate_ports(self) -> dict[str, int]:
        return {
            "motor": self._MOTOR_MAX,
            "servo": self._SERVO_MAX,
            "stepper": self._STEPPER_MAX,
            "io": self._IO_MAX,
        }

    # --- internals ---

    def _require_open(self) -> None:
        if not self.is_open:
            raise ControllerError("MC602Adapter not open; call open() first")
