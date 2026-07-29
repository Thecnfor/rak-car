"""底盘组 examples 目录。只放核心装配逻辑：不写调参默认值、不写内层循环、
不写每帧打印、不写 ``__main__``。对应那些事分别落在 config / loops/telemetry / cli。

文件名 ``05_subscribe_lane_state.py`` 以数字开头，没法直接 ``import``，用
importlib 兜一下。
"""
from __future__ import annotations

import importlib

_mod = importlib.import_module("main.chassis.examples.05_subscribe_lane_state")
subscribe_lane_state = _mod.subscribe_lane_state

__all__ = ["subscribe_lane_state"]
