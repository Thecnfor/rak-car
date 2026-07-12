#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import time
import urllib.request
from dataclasses import dataclass

import serial
from serial.tools import list_ports

from runtime.hardware.controller_recover import (
    ping_mc601,
    ping_mc602,
    recover_controller,
)


PORT_KEYWORDS = ("CH340", "USB")


@dataclass
class ControllerProbeResult:
    ready: bool
    port: str = None
    controller: str = None
    detail: str = None


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

def probe_controller():
    ports = list_candidate_ports()
    # #region debug-point E:probe-entry
    _debug_event(
        "E",
        "controller_probe.probe_controller",
        "开始探测控制器",
        {"ports": [{"name": name, "desc": desc} for name, desc in ports]},
    )
    # #endregion
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
                # #region debug-point E:probe-port-open
                _debug_event(
                    "E",
                    "controller_probe.probe_controller",
                    "串口已打开，开始握手",
                    {"port": port_name, "desc": _desc},
                )
                # #endregion
                if ping_mc602(serial_obj):
                    # #region debug-point E:probe-mc602-ok
                    _debug_event(
                        "E",
                        "controller_probe.probe_controller",
                        "mc602 program 握手成功",
                        {"port": port_name},
                    )
                    # #endregion
                    return ControllerProbeResult(
                        ready=True,
                        port=port_name,
                        controller="mc602",
                        detail="控制器握手成功",
                    )
                if ping_mc601(serial_obj):
                    # #region debug-point E:probe-mc601-ok
                    _debug_event(
                        "E",
                        "controller_probe.probe_controller",
                        "mc601 program 握手成功",
                        {"port": port_name},
                    )
                    # #endregion
                    return ControllerProbeResult(
                        ready=True,
                        port=port_name,
                        controller="mc601",
                        detail="控制器握手成功",
                    )
                serial_obj.close()
                recovered, detail = recover_controller(
                    port_name,
                    port_supplier=list_candidate_ports,
                    debug_hook=_debug_event,
                )
                if recovered:
                    # #region debug-point E:probe-recovered
                    _debug_event(
                        "E",
                        "controller_probe.probe_controller",
                        "bootloader 恢复成功",
                        {"port": port_name, "detail": detail},
                    )
                    # #endregion
                    return ControllerProbeResult(
                        ready=True,
                        port=port_name,
                        controller="mc602",
                        detail=detail,
                    )
                if detail:
                    if str(detail).startswith(port_name):
                        last_error = str(detail)
                    else:
                        last_error = f"{port_name} {detail}"
                    # #region debug-point E:probe-detail
                    _debug_event(
                        "E",
                        "controller_probe.probe_controller",
                        "端口恢复失败",
                        {"port": port_name, "detail": detail},
                    )
                    # #endregion
                    continue
                last_error = f"{port_name} 可打开但未收到控制器响应"
                # #region debug-point E:probe-no-response
                _debug_event(
                    "E",
                    "controller_probe.probe_controller",
                    "串口可打开但无控制器响应",
                    {"port": port_name},
                )
                # #endregion
        except Exception as exc:
            last_error = f"{port_name} 探测失败: {exc}"
            # #region debug-point E:probe-open-error
            _debug_event(
                "E",
                "controller_probe.probe_controller",
                "探测串口失败",
                {"port": port_name, "error": repr(exc)},
            )
            # #endregion

    return ControllerProbeResult(
        ready=False,
        detail=last_error or "控制器探测失败",
    )
