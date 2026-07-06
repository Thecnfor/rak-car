"""BaseControllerHardwareInterface — generic controller adapter contract.

Spec ref: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §硬件接口插件

Every controller family (MC601, MC602, future MCxxx) is one adapter subclass.
Adding a new controller = writing one new adapter file in
ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/ and registering it in
the sidecar entry point (Task 6). No changes to business code.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class ControllerError(RuntimeError):
    """Raised when a controller adapter cannot complete read/write."""


class BaseControllerHardwareInterface(ABC):
    """Abstract interface every controller adapter must implement.

    Sidecar components depend only on this interface, not on MCxxx-specific
    classes. Tests can substitute a fake without touching real hardware.
    """

    @property
    @abstractmethod
    def is_open(self) -> bool:
        """True if the underlying serial port / bus is open."""

    @property
    @abstractmethod
    def serial_port(self) -> str:
        """Path to the serial device (e.g., /dev/ttyUSB0) for diagnostics."""

    @property
    @abstractmethod
    def baud(self) -> int:
        """Baud rate the adapter opened at."""

    @abstractmethod
    def open(self) -> None:
        """Open the serial port and run any controller self-test. Idempotent."""

    @abstractmethod
    def close(self) -> None:
        """Close the serial port. Idempotent. Safe to call after exception."""

    @abstractmethod
    def read_sensor(self, *, port_id: int, sensor_type: str) -> float:
        """Read a sensor value. Returns the value in physical units (m, V, ...)."""

    @abstractmethod
    def write_actuator(self, *, port_id: int, actuator_type: str, value: float) -> None:
        """Write a value to an actuator. Idempotent under repeated calls."""

    @abstractmethod
    def enumerate_ports(self) -> dict[str, int]:
        """Return a dict of port-type -> count, e.g. {'motor': 6, 'servo': 7, ...}."""
