#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""runtime.services 共享纯工具（从 runtime_service.py 拆出的无状态部分）。

- `normalize_value`：把 numpy 标量/数组递归转成 JSON 可序列化的 Python 类型
- `_debug_emit`：debug-point runtime-init-queue-session 的埋点上报

独立成模块是为了避免 mixin 之间循环 import：car_runtime_service 要继承
各 mixin，而 mixin 又要用 normalize_value —— 放中间层即可两头 import。
"""
import json
import os
import urllib.request

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


def normalize_value(value):
    if np is not None:
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, np.ndarray):
            return value.tolist()
    if isinstance(value, (list, tuple)):
        return [normalize_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): normalize_value(val) for key, val in value.items()}
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


#region debug-point runtime-init-queue-session
def _debug_emit(hypothesis_id, location, msg, data=None):
    api_url = os.environ.get("DEBUG_SERVER_URL") or os.environ.get("TRAE_DEBUG_API_URL")
    if not api_url:
        return
    payload = {
        "sessionId": "runtime-init-queue",
        "hypothesisId": hypothesis_id,
        "location": location,
        "msg": msg,
        "data": data or {},
    }
    try:
        req = urllib.request.Request(
            api_url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        urllib.request.urlopen(req, timeout=0.2).read()
    except Exception:
        pass
#endregion debug-point runtime-init-queue-session
