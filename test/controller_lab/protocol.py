"""MC602 program 模式协议。"""

from __future__ import annotations

import time

from .constants import MC601_BAUDRATE, MC601_PING_FRAME, MC602_BAUDRATE, PROGRAM_FRAME_HEAD, PROGRAM_FRAME_TAIL, PROGRAM_PING_PAYLOAD
from .serial_utils import read_exact


def build_program_frame(payload: bytes) -> bytes:
    return PROGRAM_FRAME_HEAD + bytes([len(payload) + 4]) + payload + PROGRAM_FRAME_TAIL


def read_program_frame(serial_obj, timeout_s: float = 0.2) -> bytes:
    head = read_exact(serial_obj, 3, timeout_s)
    frame_len = head[2]
    body = read_exact(serial_obj, frame_len - 3, timeout_s)
    frame = head + body
    if frame[:2] != PROGRAM_FRAME_HEAD:
        raise ValueError(f"program 帧头错误: {frame[:2].hex()}")
    if frame[-1:] != PROGRAM_FRAME_TAIL:
        raise ValueError(f"program 帧尾错误: {frame[-1:].hex()}")
    return frame[3:-1]


def exchange_program_payload(serial_obj, payload: bytes, timeout_s: float = 0.2) -> bytes:
    serial_obj.baudrate = MC602_BAUDRATE
    serial_obj.reset_input_buffer()
    serial_obj.reset_output_buffer()
    serial_obj.write(build_program_frame(payload))
    return read_program_frame(serial_obj, timeout_s=timeout_s)


def ping_program_mode(serial_obj, timeout_s: float = 0.05) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            exchange_program_payload(serial_obj, PROGRAM_PING_PAYLOAD, timeout_s=0.02)
            return True
        except Exception:
            continue
    return False


def ping_mc601_mode(serial_obj, timeout_s: float = 0.05) -> bool:
    deadline = time.time() + timeout_s
    serial_obj.baudrate = MC601_BAUDRATE
    while time.time() < deadline:
        try:
            serial_obj.reset_input_buffer()
            serial_obj.reset_output_buffer()
            serial_obj.write(MC601_PING_FRAME)
            head = read_exact(serial_obj, 3, 0.03)
            frame_len = head[2] + 7
            body = read_exact(serial_obj, frame_len - 3, 0.03)
            frame = head + body
            if frame[:2] == PROGRAM_FRAME_HEAD and frame[-1:] == PROGRAM_FRAME_TAIL:
                return True
        except Exception:
            continue
    return False

