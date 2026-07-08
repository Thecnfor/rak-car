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

# MC602 frame constants (per SDK serial_wrap.py).
# Header marks the start of every frame; tail marks the end. Both
# are added automatically by `_DevCmdInterface._send_cmd`.
HEADER = b'\x77\x68'
TAIL = b'\x0A'

# Device IDs and mode opcodes (per SDK mc602_ctl2.py).
# DEV_BEEP = the buzzer peripheral.  MODE_SET = "write set value".
# Task 5 will add DEV_SERVO_PWM and DEV_DOUT alongside these.
DEV_BEEP = 0x0a
MODE_SET = 2


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


# ---------------------------------------------------------------------------
# Frame assembly base class (mirrors SDK mc602_ctl2._DevCmdInterface)
# ---------------------------------------------------------------------------

import struct  # noqa: E402  (placed here so struct isn't needed for serial-only tests)


class _DevCmdInterface:
    """Builds MC602 frames per device-class spec.

    Mirrors `mc602_ctl2._DevCmdInterface` from baidu_smartcar_2026.
    Subclasses set `dev_id`, `mode`, `port_id`, and `fmt` (struct
    format string for the args after port_id).
    """

    def __init__(
        self,
        serial_obj: 'MC602Serial',
        dev_id: int,
        mode: int,
        port_id: int,
        fmt: str,
    ) -> None:
        self.serial = serial_obj
        self.dev_id = dev_id
        self.mode = mode
        self.port_id = port_id
        self.fmt = fmt

    def _send_cmd(self, *args) -> list[int]:
        args_bytes = struct.pack('<' + self.fmt, *args) if self.fmt else b''
        payload = bytes([self.dev_id, self.mode, self.port_id]) + args_bytes
        length = len(payload) + 4  # 2 header + 1 length + 1 tail
        frame = HEADER + bytes([length]) + payload + TAIL
        ok = self.serial.write_frame(frame)
        return list(frame) if ok else []

# ---------------------------------------------------------------------------
# Device classes (mirror baidu_smartcar_2026 SDK names)
# ---------------------------------------------------------------------------

class Buzzer_2(_DevCmdInterface):
    """MC602 buzzer. Mirrors SDK Buzzer_2.

    Frame: dev=0x0a, mode=set, port=0, args=(freq//2, dur*20, 0).
    freq_hz: audible frequency (max raw byte = 510, so fundamental
        up to ~1020 Hz - above hearing, clipped).
    duration_sec: how long the buzzer sounds.
    """

    def __init__(self, serial_obj: 'MC602Serial') -> None:
        super().__init__(serial_obj, dev_id=DEV_BEEP, mode=MODE_SET,
                         port_id=0, fmt='BBB')

    def rings(self, freq_hz: int, duration_sec: float) -> list[int]:
        return self._send_cmd(
            int(freq_hz) // 2 & 0xff,
            int(duration_sec * 20) & 0xff,
            0,
        )
