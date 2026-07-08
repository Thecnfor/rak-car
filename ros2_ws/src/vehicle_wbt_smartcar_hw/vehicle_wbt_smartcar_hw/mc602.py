"""MC602 hardware protocol layer.

Mirrors the official Baidu SmartCar 2026 SDK mc602_ctl2.py API
surface (Buzzer_2, ServoPwm, PoutD) but uses pyserial instead of
custom ctypes calls. Frame format per SDK serial_wrap.py:

    [0x77, 0x68, length, dev_id, mode, port_id, args..., 0x0A]
    length = len(payload) + 4   # 2 header + 1 length + 1 tail

This module is introduced in Task 2 with just the MC602Serial
wrapper. Frame constants and device classes land in Tasks 3-5.
"""
from __future__ import annotations

from typing import Callable, Optional

import serial


class MC602Serial:
    """pyserial-backed wrapper for the MC602 USB serial link.

    Args:
        port: device path, e.g. '/dev/ttyUSB0'.
        baud: serial baud rate. MC602 default 1_000_000.
        _serial_factory: optional callable(port, baud) -> serial.Serial.
            Used by unit tests to inject a fake. Default: pyserial.
    """

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baud: int = 1_000_000,
        _serial_factory: Optional[Callable[[str, int], object]] = None,
    ) -> None:
        self.port = port
        self.baud = baud
        self._factory = _serial_factory or self._default_factory
        self._ser: Optional[object] = None  # pyserial.Serial or fake

    @staticmethod
    def _default_factory(port: str, baud: int):
        return serial.Serial(port, baud, timeout=0.1)

    def open(self) -> bool:
        try:
            self._ser = self._factory(self.port, self.baud)
            return True
        except (OSError, serial.SerialException):
            self._ser = None
            return False

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    @property
    def is_open(self) -> bool:
        return self._ser is not None and bool(getattr(self._ser, 'is_open', False))

    def write_frame(self, frame: bytes) -> bool:
        if not self.is_open:
            return False
        try:
            self._ser.write(frame)
            return True
        except (OSError, serial.SerialException):
            return False
