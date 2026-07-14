#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import time
import urllib.request
from dataclasses import dataclass

import serial
from serial.tools import list_ports

from runtime.hardware.controller_recover import (
    boot_ping,
    ping_mc601,
    ping_mc602,
)


PORT_KEYWORDS = ("CH340", "USB")


@dataclass
class ControllerProbeResult:
    ready: bool
    port: str = None
    controller: str = None
    mode: str = None
    detail: str = None
    bootloader_seen: bool = False
    program_seen: bool = False


# #region debug-point A:report-helper
def _debug_event(hypothesis_id, location, msg, data=None, run_id="pre"):
    env_path = ".dbg/mc602-download-stuck.env"
    server_url = "http://127.0.0.1:7777/event"
    session_id = "mc602-download-stuck"
    try:
        with open(env_path, "r", encoding="utf-8") as env_file:
            for raw_line in env_file:
                line = raw_line.strip()
                if line.startswith("DEBUG_SERVER_URL="):
                    server_url = line.split("=", 1)[1]
                elif line.startswith("DEBUG_SESSION_ID="):
                    session_id = line.split("=", 1)[1]
    except Exception:
        pass
    payload = {
        "sessionId": session_id,
        "runId": run_id,
        "hypothesisId": hypothesis_id,
        "location": location,
        "msg": "[DEBUG] {}".format(msg),
        "data": data or {},
        "ts": int(time.time() * 1000),
    }
    try:
        urllib.request.urlopen(
            urllib.request.Request(
                server_url,
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
            ),
            timeout=0.2,
        ).read()
    except Exception:
        pass


# #endregion


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


def probe_port_mode(port_name, debug_hook=None):
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
            if debug_hook is not None:
                debug_hook(
                    "E",
                    "controller_probe.probe_port_mode",
                    "串口已打开，开始纯探测",
                    {"port": port_name},
                )
            if ping_mc602(serial_obj):
                return ControllerProbeResult(
                    ready=True,
                    port=port_name,
                    controller="mc602",
                    mode="program",
                    detail="mc602 program 握手成功",
                    program_seen=True,
                )
            if ping_mc601(serial_obj):
                return ControllerProbeResult(
                    ready=True,
                    port=port_name,
                    controller="mc601",
                    mode="program",
                    detail="mc601 program 握手成功",
                    program_seen=True,
                )
            serial_obj.baudrate = 1000000
            ok, _frame = boot_ping(serial_obj)
            if ok:
                return ControllerProbeResult(
                    ready=False,
                    port=port_name,
                    controller="mc602",
                    mode="bootloader",
                    detail="bootloader 在线，等待拉起 program",
                    bootloader_seen=True,
                )
            return ControllerProbeResult(
                ready=False,
                port=port_name,
                controller=None,
                mode="unknown",
                detail="串口可打开，但既非 program 也非 bootloader",
            )
    except Exception as exc:
        if debug_hook is not None:
            debug_hook(
                "E",
                "controller_probe.probe_port_mode",
                "探测串口失败",
                {"port": port_name, "error": repr(exc)},
            )
        return ControllerProbeResult(
            ready=False,
            port=port_name,
            controller=None,
            mode="unknown",
            detail=f"{port_name} 探测失败: {exc}",
        )


def probe_controller():
    ports = list_candidate_ports()
    _debug_event(
        "E",
        "controller_probe.probe_controller",
        "开始探测控制器",
        {"ports": [{"name": name, "desc": desc} for name, desc in ports]},
    )
    if not ports:
        return ControllerProbeResult(
            ready=False,
            mode="no_port",
            detail="未找到控制器串口，检查 USB 连接和下位机供电",
        )

    last_result = None
    for port_name, _desc in ports:
        result = probe_port_mode(port_name, debug_hook=_debug_event)
        last_result = result
        if result.mode == "program":
            _debug_event(
                "E",
                "controller_probe.probe_controller",
                "program 探测成功",
                {"port": port_name, "controller": result.controller},
            )
            return result
        if result.mode == "bootloader":
            _debug_event(
                "E",
                "controller_probe.probe_controller",
                "bootloader 探测成功",
                {"port": port_name},
            )
            return result
    return last_result or ControllerProbeResult(
        ready=False,
        mode="unknown",
        detail="控制器探测失败",
    )
