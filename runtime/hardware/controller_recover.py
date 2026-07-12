#!/usr/bin/python3
# -*- coding: utf-8 -*-
import threading
import time

import serial


_RECOVERY_LOCK = threading.Lock()

BOOT_PING = bytes.fromhex("55 AA 00 01 08 00 00 F7")
BOOT_PING_ACK = bytes.fromhex("66 BB 01 01 0A 00 5A 02 00 76")
RUN_CODE = bytes.fromhex("55 AA 00 40 0B 00 00 D0 00 08 DD")
RUN_CODE_ACK = bytes.fromhex("66 BB 01 41 0B 00 00 D0 00 08 B9")

MC601_PING = bytes.fromhex("77 68 04 00 01 CA 01 0A")
MC602_PING = bytes.fromhex("77 68 07 02 01 10 0A")

PORT_STABLE_FOR_S = 0.8
PORT_STABLE_TIMEOUT_S = 3.5
RECOVERY_WINDOW_S = 4.5
RUNCODE_MAX_ATTEMPTS = 8
RUNCODE_ACK_TIMEOUT_S = 0.18
PROGRAM_PING_AFTER_RUNCODE_S = 0.8
RECOVERY_COOLDOWN_S = 0.18


def _checksum(frame):
    total = 0
    for byte in frame[:-1]:
        total += byte
    total &= 0xFF
    return (~total) & 0xFF


def _write_slow(serial_obj, data, delay_s=0.001):
    one_byte = bytearray(1)
    for value in data:
        one_byte[0] = value
        serial_obj.write(one_byte)
        time.sleep(delay_s)


def _is_bootloader_ack(frame):
    return (
        len(frame) == 10
        and frame[:2] == bytes.fromhex("66 BB")
        and frame[3] == 0x01
        and frame[4] == 0x0A
        and frame[5] == 0x00
        and frame[-1] == _checksum(frame)
    )


def _is_runcode_ack(frame):
    return (
        len(frame) == 11
        and frame[:2] == bytes.fromhex("66 BB")
        and frame[3] == 0x41
        and frame[4] == 0x0B
        and frame[5] == 0x00
        and frame[6:10] == RUN_CODE[6:10]
        and frame[-1] == _checksum(frame)
    )


def _read_exact(serial_obj, size, timeout_s):
    deadline = time.time() + timeout_s
    chunks = b""
    while len(chunks) < size and time.time() < deadline:
        chunk = serial_obj.read(size - len(chunks))
        if chunk:
            chunks += chunk
    return chunks


def _open_serial(port_name, baudrate=115200, timeout=0.03):
    return serial.Serial(
        port=port_name,
        baudrate=baudrate,
        timeout=timeout,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )


def ping_mc601(serial_obj, timeout_s=0.05):
    serial_obj.baudrate = 380400
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        serial_obj.reset_input_buffer()
        serial_obj.reset_output_buffer()
        serial_obj.write(MC601_PING)
        head = _read_exact(serial_obj, 3, 0.03)
        if len(head) != 3:
            continue
        frame_len = head[2] + 7
        body = _read_exact(serial_obj, frame_len - 3, 0.03)
        response = head + body
        if (
            len(response) == frame_len
            and response[:2] == bytes.fromhex("77 68")
            and response[-1:] == bytes.fromhex("0A")
        ):
            return True
    return False


def ping_mc602(serial_obj, timeout_s=0.05):
    serial_obj.baudrate = 1000000
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        serial_obj.reset_input_buffer()
        serial_obj.reset_output_buffer()
        serial_obj.write(MC602_PING)
        head = _read_exact(serial_obj, 3, 0.02)
        if len(head) != 3:
            continue
        frame_len = head[2]
        body = _read_exact(serial_obj, frame_len - 3, 0.02)
        response = head + body
        if (
            len(response) == frame_len
            and response[:2] == bytes.fromhex("77 68")
            and response[-1:] == bytes.fromhex("0A")
        ):
            return True
    return False


def wait_port_stable(
    port_name,
    port_supplier,
    stable_for_s=PORT_STABLE_FOR_S,
    timeout_s=PORT_STABLE_TIMEOUT_S,
    debug_hook=None,
):
    start_time = time.time()
    stable_since = None
    while time.time() - start_time < timeout_s:
        ports = port_supplier()
        present = any(name == port_name for name, _desc in ports)
        if present:
            if stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since >= stable_for_s:
                if debug_hook is not None:
                    debug_hook(
                        "R",
                        "controller_recover.wait_port_stable",
                        "串口已稳定",
                        {
                            "port": port_name,
                            "stable_for_s": stable_for_s,
                            "waited_s": round(time.time() - start_time, 3),
                        },
                    )
                return True
        else:
            stable_since = None
        time.sleep(0.1)
    if debug_hook is not None:
        debug_hook(
            "R",
            "controller_recover.wait_port_stable",
            "等待串口稳定超时",
            {"port": port_name, "timeout_s": timeout_s},
        )
    return False


def wait_program_ready(port_name, timeout_s=4.0, debug_hook=None):
    deadline = time.time() + timeout_s
    attempts = 0
    while time.time() < deadline:
        attempts += 1
        try:
            with _open_serial(port_name) as serial_obj:
                if ping_mc602(serial_obj, timeout_s=0.4):
                    if debug_hook is not None:
                        debug_hook(
                            "R",
                            "controller_recover.wait_program_ready",
                            "program 握手成功",
                            {"port": port_name, "attempts": attempts, "timeout_s": timeout_s},
                        )
                    return True
        except Exception as exc:
            if debug_hook is not None:
                debug_hook(
                    "R",
                    "controller_recover.wait_program_ready",
                    "等待 program 时串口异常",
                    {"port": port_name, "attempts": attempts, "error": repr(exc)},
                )
        time.sleep(0.15)
    if debug_hook is not None:
        debug_hook(
            "R",
            "controller_recover.wait_program_ready",
            "等待 program 超时",
            {"port": port_name, "attempts": attempts, "timeout_s": timeout_s},
        )
    return False


def _try_runcode_sequence(port_name, debug_hook=None):
    last_detail = "bootloader 已响应，但尚未触发 RUNCODE"
    for attempt in range(1, RUNCODE_MAX_ATTEMPTS + 1):
        try:
            with _open_serial(port_name) as serial_obj:
                serial_obj.baudrate = 1000000
                serial_obj.reset_input_buffer()
                serial_obj.reset_output_buffer()
                _write_slow(serial_obj, RUN_CODE)
                time.sleep(0.05)
                ret = _read_exact(serial_obj, 11, RUNCODE_ACK_TIMEOUT_S)
                if debug_hook is not None:
                    debug_hook(
                        "R",
                        "controller_recover._try_runcode_sequence",
                        "RUNCODE 回包结果",
                        {
                            "port": port_name,
                            "attempt": attempt,
                            "response_hex": ret.hex(),
                        },
                    )
                if _is_runcode_ack(ret):
                    last_detail = "RUNCODE 已确认，等待 program 握手"
                else:
                    last_detail = "bootloader 已响应，但 RUNCODE 未确认"
        except Exception as exc:
            last_detail = "发送 RUNCODE 时串口异常: {}".format(exc)
            if debug_hook is not None:
                debug_hook(
                    "R",
                    "controller_recover._try_runcode_sequence",
                    "发送 RUNCODE 时串口异常",
                    {"port": port_name, "attempt": attempt, "error": repr(exc)},
                )
        if wait_program_ready(
            port_name,
            timeout_s=PROGRAM_PING_AFTER_RUNCODE_S,
            debug_hook=debug_hook,
        ):
            return True, "控制器已从 bootloader 拉起到 program 模式"
        time.sleep(RECOVERY_COOLDOWN_S)
    return False, last_detail


def recover_controller(port_name, port_supplier, debug_hook=None):
    with _RECOVERY_LOCK:
        if not wait_port_stable(port_name, port_supplier, debug_hook=debug_hook):
            return False, "串口重枚举未稳定，跳过本轮恢复"
        deadline = time.time() + RECOVERY_WINDOW_S
        last_detail = "{} 可打开但未收到控制器响应".format(port_name)
        attempt = 0
        while time.time() < deadline:
            attempt += 1
            try:
                with _open_serial(port_name) as serial_obj:
                    if ping_mc602(serial_obj, timeout_s=0.3):
                        return True, "控制器 program 模式在线"
                    if ping_mc601(serial_obj, timeout_s=0.3):
                        return True, "控制器 program 模式在线"

                    serial_obj.baudrate = 1000000
                    serial_obj.reset_input_buffer()
                    serial_obj.reset_output_buffer()
                    _write_slow(serial_obj, BOOT_PING)
                    time.sleep(0.01)
                    ret = _read_exact(serial_obj, 10, 0.12)
                    if debug_hook is not None:
                        debug_hook(
                            "R",
                            "controller_recover.recover_controller",
                            "bootloader 探测结果",
                            {
                                "port": port_name,
                                "attempt": attempt,
                                "response_hex": ret.hex(),
                            },
                        )
                    if _is_bootloader_ack(ret):
                        ok, detail = _try_runcode_sequence(port_name, debug_hook=debug_hook)
                        if ok:
                            return True, detail
                        last_detail = detail
                    else:
                        last_detail = "{} 可打开但未收到控制器响应".format(port_name)
            except Exception as exc:
                last_detail = "{} 恢复时串口异常: {}".format(port_name, exc)
                if debug_hook is not None:
                    debug_hook(
                        "R",
                        "controller_recover.recover_controller",
                        "恢复过程串口异常",
                        {"port": port_name, "attempt": attempt, "error": repr(exc)},
                    )
            time.sleep(RECOVERY_COOLDOWN_S)
        return False, last_detail
