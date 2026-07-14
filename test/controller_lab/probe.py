"""控制器探测。"""

from __future__ import annotations

from .protocol import ping_mc601_mode, ping_program_mode
from .serial_utils import list_candidate_ports, open_serial


def probe_port(port_name: str) -> dict:
    result = {
        "ready": False,
        "port": port_name,
        "controller": None,
        "mode": None,
        "detail": "未收到控制器响应",
    }
    try:
        with open_serial(port_name) as serial_obj:
            if ping_program_mode(serial_obj):
                result.update({"ready": True, "controller": "mc602", "mode": "program", "detail": "MC602 program 握手成功"})
                return result
            if ping_mc601_mode(serial_obj):
                result.update({"ready": True, "controller": "mc601", "mode": "program", "detail": "MC601 program 握手成功"})
                return result
    except Exception as exc:
        result["detail"] = f"串口探测失败: {exc}"
    return result


def probe_controller() -> dict:
    ports = list_candidate_ports()
    if not ports:
        return {"ready": False, "port": None, "controller": None, "mode": None, "detail": "未找到候选串口", "ports": []}
    last_result = None
    for port in ports:
        last_result = probe_port(port["device"])
        if last_result["ready"]:
            last_result["ports"] = ports
            return last_result
    if last_result is None:
        last_result = {"ready": False, "detail": "未执行探测"}
    last_result["ports"] = ports
    return last_result

