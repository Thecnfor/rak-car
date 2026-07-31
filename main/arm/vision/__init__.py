"""main/arm/vision/__init__.py — ArmVisionClient 聚合类.

MRO = (ServoLoop, RealtimeLoop) — HTTP 路径先 mixin, WS 路径后 mixin.
"""
from __future__ import annotations

from typing import List

from ..labels import LABEL_GROUPS, LABELS, Label, LabelInfo
from .parsers import _parse_cache, _parse_sync
from .realtime import RealtimeLoop
from .selector import SelectionStrategy, TargetSelector
from .servo import ServoLoop
from .types import BBoxNorm, BBoxPixels, Detection, ServoResult, ServoTrace

__all__ = [
    "ArmVisionClient",
    "BBoxNorm", "BBoxPixels", "Detection",
    "ServoResult", "ServoTrace",
    "SelectionStrategy", "TargetSelector",
    "LabelInfo", "LABELS", "LABEL_GROUPS", "Label",
]


class ArmVisionClient(ServoLoop, RealtimeLoop):
    """末端摄像头视觉伺服客户端. 主路径 task_feed 30Hz cache; WS 路径走 push."""

    def __init__(self, http, *, default_timeout_s: float = 10.0):
        self.http = http
        self.default_timeout_s = default_timeout_s

    @staticmethod
    def labels():
        return LABELS

    @staticmethod
    def group(name: str):
        return LABEL_GROUPS[name]

    def get_state(self) -> List[Detection]:
        return _parse_cache(self.http.get_vision_task_cache())

    def get_state_filtered(self, selector) -> List[Detection]:
        return [d for d in self.get_state() if selector.matches(d)]

    def snap(self, *, sort_pos=(0.0, 0.0), limit_x: float = 1.0,
             limit_y: float = 1.0, timeout: float = 20.0) -> List[Detection]:
        return _parse_sync(self.http.request_vision_task(
            sort_pos=sort_pos, limit_x=limit_x, limit_y=limit_y, timeout=timeout))
