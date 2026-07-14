"""串口与字节工具。"""

from __future__ import annotations

import binascii
import time
from contextlib import contextmanager

import serial
from serial.tools import list_ports

from .constants import MC602_BAUDRATE, PORT_KEYWORDS, SERIAL_TIMEOUT_S, USB_PID, USB_VID, WRITE_TIMEOUT_S


class ControllerLabError(RuntimeError):
    """独立测试工具通用异常。"""


class SerialReadTimeout(ControllerLabError):
    """串口读取超时。"""


def hex_bytes(data: bytes) -> str:
    return binascii.hexlify(data, sep=b" ").decode("ascii")


def list_candidate_ports():
    ports = []
    for port_info in list_ports.comports():
        desc = getattr(port_info, "description", "") or ""
        pid = getattr(port_info, "pid", None)
        vid = getattr(port_info, "vid", None)
        if any(keyword in desc for keyword in PORT_KEYWORDS) or (pid == USB_PID and vid == USB_VID):
            ports.append(
                {
                    "device": getattr(port_info, "device", None) or port_info[0],
                    "description": desc,
                    "pid": pid,
                    "vid": vid,
                }
            )
    ports.sort(key=lambda item: "CH340" not in item["description"])
    return ports


def read_exact(serial_obj: serial.Serial, size: int, timeout_s: float) -> bytes:
    deadline = time.time() + timeout_s
    chunks = b""
    while len(chunks) < size and time.time() < deadline:
        chunk = serial_obj.read(size - len(chunks))
        if chunk:
            chunks += chunk
    if len(chunks) != size:
        raise SerialReadTimeout(f"读取 {size} 字节超时，仅收到 {len(chunks)} 字节: {hex_bytes(chunks)}")
    return chunks


def read_available(serial_obj: serial.Serial, timeout_s: float) -> bytes:
    deadline = time.time() + timeout_s
    chunks = b""
    while time.time() < deadline:
        chunk = serial_obj.read(serial_obj.in_waiting or 1)
        if chunk:
            chunks += chunk
            continue
        if chunks:
            break
    return chunks


def checksum8(data: bytes) -> int:
    total = sum(data[:-1]) & 0xFF
    return (~total) & 0xFF


def checksum8_full(data: bytes) -> int:
    total = sum(data) & 0xFF
    return (~total) & 0xFF


def write_slow(serial_obj: serial.Serial, data: bytes, delay_s: float = 0.001) -> None:
    one_byte = bytearray(1)
    for value in data:
        one_byte[0] = value
        serial_obj.write(one_byte)
        time.sleep(delay_s)


@contextmanager
def open_serial(port_name: str, baudrate: int = MC602_BAUDRATE, timeout_s: float = SERIAL_TIMEOUT_S):
    serial_obj = serial.Serial(
        port=port_name,
        baudrate=baudrate,
        timeout=timeout_s,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
        writeTimeout=WRITE_TIMEOUT_S,
    )
    try:
        yield serial_obj
    finally:
        try:
            serial_obj.close()
        except Exception:
            pass

