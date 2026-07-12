#!/usr/bin/python3
# -*- coding: utf-8 -*-
import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports
from smartcar.whalesbot.vehicle.base.pydownload import Scratch_Download_MC602P


PORT_KEYWORDS = ("CH340", "USB")


@dataclass
class ControllerProbeResult:
    ready: bool
    port: str = None
    controller: str = None
    detail: str = None


def _get_port_name(port_info):
    return getattr(port_info, "device", None) or port_info[0]


def _get_port_desc(port_info):
    return getattr(port_info, "description", None) or port_info[1]


def list_candidate_ports():
    ports = []
    for port_info in list_ports.comports():
        desc = _get_port_desc(port_info)
        if any(keyword in desc for keyword in PORT_KEYWORDS):
            ports.append((_get_port_name(port_info), desc))
    ports.sort(key=lambda item: "CH340" not in item[1])
    return ports


def _read_exact(serial_obj, size, timeout_s):
    time_end = time.time() + timeout_s
    chunks = b""
    while len(chunks) < size and time.time() < time_end:
        chunk = serial_obj.read(size - len(chunks))
        if chunk:
            chunks += chunk
    return chunks


def _ping_mc601(serial_obj, timeout_s=0.05):
    serial_obj.baudrate = 380400
    time_end = time.time() + timeout_s
    while time.time() < time_end:
        serial_obj.reset_input_buffer()
        serial_obj.reset_output_buffer()
        serial_obj.write(bytes.fromhex("77 68 04 00 01 CA 01 0A"))
        head = _read_exact(serial_obj, 3, 0.03)
        if len(head) != 3:
            continue
        frame_len = head[2] + 7
        body = _read_exact(serial_obj, frame_len - 3, 0.03)
        response = head + body
        if len(response) == frame_len and response[:2] == bytes.fromhex("77 68") and response[-1:] == bytes.fromhex("0A"):
            return True
    return False


def _ping_mc602(serial_obj, timeout_s=0.05):
    serial_obj.baudrate = 1000000
    time_end = time.time() + timeout_s
    while time.time() < time_end:
        serial_obj.reset_input_buffer()
        serial_obj.reset_output_buffer()
        serial_obj.write(bytes.fromhex("77 68 07 02 01 10 0A"))
        head = _read_exact(serial_obj, 3, 0.02)
        if len(head) != 3:
            continue
        frame_len = head[2]
        body = _read_exact(serial_obj, frame_len - 3, 0.02)
        response = head + body
        if len(response) == frame_len and response[:2] == bytes.fromhex("77 68") and response[-1:] == bytes.fromhex("0A"):
            return True
    return False


def _recover_mc602(port_name, serial_obj):
    serial_obj.baudrate = 1000000
    serial_obj.reset_input_buffer()
    serial_obj.reset_output_buffer()
    serial_obj.write(bytes.fromhex("55 AA 00 01 08 00 00 F7"))
    time.sleep(0.01)
    ret = _read_exact(serial_obj, 10, 0.1)
    if ret != bytes.fromhex("66 BB 01 01 0A 00 5A 02 00 76"):
        return False, None

    # 先尝试直接拉起已有的 program，避免每次都重新下载。
    start_time = time.time()
    while time.time() - start_time < 1.0:
        serial_obj.reset_input_buffer()
        serial_obj.reset_output_buffer()
        serial_obj.write(bytes.fromhex("55 AA 00 40 0B 00 00 D0 00 08 DD"))
        time.sleep(0.01)
        ret = _read_exact(serial_obj, 11, 0.1)
        if ret == bytes.fromhex("66 BB 01 41 0B 00 00 D0 00 08 B9"):
            if _ping_mc602(serial_obj, timeout_s=2.0):
                return True, "控制器已从 bootloader 拉起到 program 模式"
            break

    serial_obj.close()
    result, _msg = Scratch_Download_MC602P("RunA", isrun=True)
    if not result:
        return False, "mc602 处于 bootloader，但自动下载 Run.bin 失败"

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        timeout=0.03,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    ) as retry_serial:
        if _ping_mc602(retry_serial, timeout_s=1.5):
            return True, "控制器已自动下载 Run.bin 并进入 program 模式"
    return False, "mc602 自动下载后仍未进入 program 模式"


def probe_controller():
    ports = list_candidate_ports()
    if not ports:
        return ControllerProbeResult(
            ready=False,
            detail="未找到控制器串口，检查 USB 连接和下位机供电",
        )

    last_error = None
    for port_name, _desc in ports:
        try:
            with serial.Serial(
                port=port_name,
                baudrate=115200,
                timeout=0.03,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            ) as serial_obj:
                if _ping_mc602(serial_obj):
                    return ControllerProbeResult(
                        ready=True,
                        port=port_name,
                        controller="mc602",
                        detail="控制器握手成功",
                    )
                if _ping_mc601(serial_obj):
                    return ControllerProbeResult(
                        ready=True,
                        port=port_name,
                        controller="mc601",
                        detail="控制器握手成功",
                    )
                recovered, detail = _recover_mc602(port_name, serial_obj)
                if recovered:
                    return ControllerProbeResult(
                        ready=True,
                        port=port_name,
                        controller="mc602",
                        detail=detail,
                    )
                if detail:
                    last_error = f"{port_name} {detail}"
                    continue
                last_error = f"{port_name} 可打开但未收到控制器响应"
        except Exception as exc:
            last_error = f"{port_name} 探测失败: {exc}"

    return ControllerProbeResult(
        ready=False,
        detail=last_error or "控制器探测失败",
    )
