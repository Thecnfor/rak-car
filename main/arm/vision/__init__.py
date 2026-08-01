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
from .velocity import VelocityLoop, VelocityResult, VelocityTrace

__all__ = [
    "ArmVisionClient",
    "BBoxNorm", "BBoxPixels", "Detection",
    "ServoResult", "ServoTrace",
    "VelocityResult", "VelocityTrace",
    "SelectionStrategy", "TargetSelector",
    "LabelInfo", "LABELS", "LABEL_GROUPS", "Label",
]


class ArmVisionClient(ServoLoop, RealtimeLoop, VelocityLoop):
    """末端摄像头视觉伺服客户端. 主路径 task_feed 30Hz cache; WS 路径走 push;
    velocity 模式 (07/08) 走 /v1/realtime/arm-velocity 直发 (免 queue)."""

    DEFAULT_FOCAL_LENGTH_PX = 600.0
    DEFAULT_REF_DEPTH_M = 0.30

    def __init__(self, http, *, default_timeout_s: float = 10.0):
        self.http = http
        self.default_timeout_s = default_timeout_s

    @staticmethod
    def labels():
        return LABELS

    @staticmethod
    def group(name: str):
        return LABEL_GROUPS[name]

    @staticmethod
    def compute_depth(bbox_pixels, target_real_height_m: float,
                      focal_length_px: float = 600.0) -> float:
        """从 bbox 像素高反推物理距离 (m).

        depth_m = (target_real_height_m * focal_length_px) / bbox_height_px.
        bbox_height=0 / target_real_height=0 / bbox_pixels=None 时走 fallback (0.30m).
        """
        if bbox_pixels is None or bbox_pixels.height <= 0:
            return ArmVisionClient.DEFAULT_REF_DEPTH_M
        if target_real_height_m <= 0:
            return ArmVisionClient.DEFAULT_REF_DEPTH_M
        return (target_real_height_m * focal_length_px) / bbox_pixels.height

    def get_state(self) -> List[Detection]:
        return _parse_cache(self.http.get_vision_task_cache())

    def get_state_filtered(self, selector) -> List[Detection]:
        return [d for d in self.get_state() if selector.matches(d)]

    def snap(self, *, sort_pos=(0.0, 0.0), limit_x: float = 1.0,
             limit_y: float = 1.0, timeout: float = 20.0) -> List[Detection]:
        return _parse_sync(self.http.request_vision_task(
            sort_pos=sort_pos, limit_x=limit_x, limit_y=limit_y, timeout=timeout))
