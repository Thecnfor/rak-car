"""Unit tests for vehicle_wbt_smartcar_hw.mc602 — no real hardware.

Uses an injected fake serial object so frame construction and I/O
plumbing can be tested on a build server without /dev/ttyUSB*.
"""
from __future__ import annotations

import sys
import os

# Allow running pytest from repo root without ament install.
# Add the package root (one level up from test/) so the
# vehicle_wbt_smartcar_hw package is importable.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vehicle_wbt_smartcar_hw.mc602 import MC602Serial  # noqa: E402
from vehicle_wbt_smartcar_hw.mc602 import _DevCmdInterface  # noqa: E402


class FakeSerial:
    """Drop-in replacement for pyserial.Serial that records writes."""

    def __init__(self, port, baud, timeout=None):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.written: list[bytes] = []
        self._open = True

    @property
    def is_open(self) -> bool:
        return self._open

    def write(self, data: bytes) -> int:
        self.written.append(data)
        return len(data)

    def close(self) -> None:
        self._open = False


def _make_serial(port: str = '/dev/ttyUSB0', baud: int = 1_000_000):
    return MC602Serial(port=port, baud=baud,
                       _serial_factory=lambda p, b: FakeSerial(p, b))


def test_mc602serial_open_returns_true_on_success() -> None:
    s = _make_serial()
    assert s.open() is True
    assert s.is_open is True
    s.close()


def test_mc602serial_open_returns_false_on_failure() -> None:
    def boom(port, baud):
        raise OSError("no such device")
    s = MC602Serial(port='/dev/nope', baud=1_000_000,
                    _serial_factory=boom)
    assert s.open() is False
    assert s.is_open is False


def test_mc602serial_write_frame_returns_true() -> None:
    s = _make_serial()
    s.open()
    try:
        assert s.write_frame(b'\x77\x68\x05\x0a\x02\x00\x00\x0a') is True
    finally:
        s.close()


def test_mc602serial_write_frame_returns_false_when_not_open() -> None:
    s = _make_serial()
    assert s.write_frame(b'anything') is False


def test_mc602serial_factory_receives_port_and_baud() -> None:
    captured = {}

    def factory(port, baud):
        captured['port'] = port
        captured['baud'] = baud
        return FakeSerial(port, baud)

    s = MC602Serial(port='/dev/ttyUSB1', baud=500_000,
                    _serial_factory=factory)
    s.open()
    assert captured['port'] == '/dev/ttyUSB1'
    assert captured['baud'] == 500_000


def _make_iface(fmt='BBB', dev_id=0x0a, mode=2, port_id=0):
    ser = _make_serial()
    ser.open()
    return ser, _DevCmdInterface(ser, dev_id=dev_id, mode=mode,
                                 port_id=port_id, fmt=fmt)


def test_dev_cmd_interface_emits_correct_frame() -> None:
    # Buzzer frame: dev=0x0a, mode=2 (set), port=0, args=(freq/2, dur*20, 0)
    ser, iface = _make_iface(fmt='BBB', dev_id=0x0a, mode=2, port_id=0)
    written = iface._send_cmd(100, 4, 0)
    # payload = [0x0a, 0x02, 0x00, 100, 4, 0] = 6 bytes
    # frame   = [0x77, 0x68, 0x0a, 0x0a, 0x02, 0x00, 100, 4, 0, 0x0a]
    expected = bytes([0x77, 0x68, 0x0a, 0x0a, 0x02, 0x00, 100, 4, 0, 0x0a])
    assert written == list(expected)
    ser.close()


def test_dev_cmd_interface_length_includes_overhead() -> None:
    # Verify length byte = len(payload) + 4
    ser, iface = _make_iface(fmt='bb', dev_id=0x10, mode=2, port_id=4)
    iface._send_cmd(1, 0)
    # payload = [0x10, 0x02, 0x04, 1, 0] = 5 bytes → length byte = 9
    fake = ser._ser
    assert fake.written[-1][2] == 9   # index 2 = length
    ser.close()
