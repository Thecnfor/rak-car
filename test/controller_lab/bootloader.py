"""bootloader 探测与恢复。"""

from __future__ import annotations

import time

from .constants import (
    BOOT_PING,
    PORT_STABLE_FOR_S,
    PORT_STABLE_TIMEOUT_S,
    PROGRAM_PING_AFTER_RUNCODE_S,
    RECOVERY_COOLDOWN_S,
    RECOVERY_WINDOW_S,
    RUNCODE_ACK_TIMEOUT_S,
    RUNCODE_MAX_ATTEMPTS,
    RUN_CODE,
)
from .protocol import ping_program_mode
from .serial_utils import checksum8, list_candidate_ports, open_serial, read_available, read_exact, write_slow


def is_boot_ack(frame: bytes) -> bool:
    return (
        len(frame) == 10
        and frame[:2] == bytes.fromhex("66 BB")
        and frame[3] == 0x01
        and frame[4] == 0x0A
        and frame[5] == 0x00
        and frame[-1] == checksum8(frame)
    )


def is_runcode_ack(frame: bytes) -> bool:
    return (
        len(frame) == 11
        and frame[:2] == bytes.fromhex("66 BB")
        and frame[3] == 0x41
        and frame[4] == 0x0B
        and frame[5] == 0x00
        and frame[6:10] == RUN_CODE[6:10]
        and frame[-1] == checksum8(frame)
    )


def boot_ping(serial_obj) -> tuple[bool, bytes]:
    serial_obj.reset_input_buffer()
    serial_obj.reset_output_buffer()
    serial_obj.write(BOOT_PING)
    try:
        frame = read_exact(serial_obj, 10, 0.12)
    except Exception:
        frame = b""
    return is_boot_ack(frame), frame


def send_runcode(serial_obj) -> tuple[bool, bytes]:
    serial_obj.reset_input_buffer()
    serial_obj.reset_output_buffer()
    write_slow(serial_obj, RUN_CODE)
    time.sleep(0.02)
    data = read_available(serial_obj, timeout_s=RUNCODE_ACK_TIMEOUT_S)
    if len(data) >= 11:
        frame = data[:11]
    else:
        frame = data
    return (is_runcode_ack(frame) if len(frame) == 11 else False), frame


def wait_port_stable(port_name: str, stable_for_s: float = PORT_STABLE_FOR_S, timeout_s: float = PORT_STABLE_TIMEOUT_S) -> bool:
    start_time = time.time()
    stable_since = None
    while time.time() - start_time < timeout_s:
        ports = list_candidate_ports()
        present = any(item["device"] == port_name for item in ports)
        if present:
            if stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since >= stable_for_s:
                return True
        else:
            stable_since = None
        time.sleep(0.1)
    return False


def wait_program_ready(port_name: str, timeout_s: float = 4.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            with open_serial(port_name) as serial_obj:
                if ping_program_mode(serial_obj, timeout_s=0.4):
                    return True
        except Exception:
            pass
        time.sleep(0.15)
    return False


def recover_to_program(port_name: str) -> dict:
    result = {
        "ok": False,
        "port": port_name,
        "stage": "start",
        "detail": "未开始恢复",
        "boot_ack_hex": "",
        "runcode_ack_hex": "",
    }
    if not wait_port_stable(port_name):
        result.update({"stage": "wait_port", "detail": "串口重枚举未稳定"})
        return result
    deadline = time.time() + RECOVERY_WINDOW_S
    while time.time() < deadline:
        try:
            with open_serial(port_name) as serial_obj:
                if ping_program_mode(serial_obj):
                    result.update({"ok": True, "stage": "program", "detail": "控制器已在 program 模式"})
                    return result
                ok, boot_frame = boot_ping(serial_obj)
                result["boot_ack_hex"] = boot_frame.hex()
                if not ok:
                    result.update({"stage": "boot_ping", "detail": "bootloader 未响应"})
                    time.sleep(0.12)
                    continue
        except Exception as exc:
            result.update({"stage": "boot_ping", "detail": f"bootloader 探测异常: {exc}"})
            time.sleep(0.12)
            continue
        for _attempt in range(1, RUNCODE_MAX_ATTEMPTS + 1):
            try:
                with open_serial(port_name) as serial_obj:
                    ok, run_frame = send_runcode(serial_obj)
                    result["runcode_ack_hex"] = run_frame.hex()
                    if ok and wait_program_ready(port_name, timeout_s=PROGRAM_PING_AFTER_RUNCODE_S):
                        result.update({"ok": True, "stage": "runcode", "detail": "控制器已从 bootloader 拉起到 program 模式"})
                        return result
            except Exception as exc:
                result.update({"stage": "runcode", "detail": f"发送 RUNCODE 异常: {exc}"})
            time.sleep(RECOVERY_COOLDOWN_S)
        result.update({"stage": "runcode", "detail": "bootloader 已响应，但 RUNCODE 未拉起 program"})
        return result
    result.update({"stage": "timeout", "detail": "恢复窗口超时"})
    return result
