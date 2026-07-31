"""main/arm/api/vis_servo.py — 视觉伺服懒构造 mixin.

arm_client.vision 第一次访问时建 ArmVisionClient.
_make_vision_with_move 返回带 _safe_move 注入的 client (PR#13 HIGH gate-bypass 修复).
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from ..vision import ArmVisionClient


class VisServoMixin:
    """vision property + _make_vision_with_move 入口."""

    @property
    def vision(self) -> "ArmVisionClient":
        """懒构造: 首次访问时建 ArmVisionClient."""
        if getattr(self, "_vision", None) is None:
            from ..vision import ArmVisionClient
            self._vision = ArmVisionClient(self.http)
        return self._vision

    def _make_vision_with_move(self) -> "ArmVisionClient":
        from ..vision import ArmVisionClient
        client = ArmVisionClient(self.http)
        original_find = client.find_target
        original_find_realtime = client.find_target_realtime

        def _safe_move(nx: float, ny: float) -> dict:
            self._check_y_protected("find_target")
            self._check_safe(y_mm=ny)
            return self.move_xy(nx, ny, timeout=5.0)

        def _safe_wrap(original, label: str):
            def safe_fn(selector, *, x_mm, y_mm, **kwargs):
                move_fn = kwargs.pop("move_fn", None) or _safe_move
                return original(selector, x_mm=x_mm, y_mm=y_mm,
                                move_fn=move_fn, **kwargs)
            safe_fn.__name__ = label
            return safe_fn

        client.find_target = _safe_wrap(original_find, "safe_find_target")  # type: ignore[method-assign]
        client.find_target_realtime = _safe_wrap(original_find_realtime,  # type: ignore[method-assign]
                                                  "safe_find_target_realtime")
        return client
