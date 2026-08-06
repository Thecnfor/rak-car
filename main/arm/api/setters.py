"""main/arm/api/setters.py — 大臂 / 手爪角度设置 mixin.

依赖 SafetyMixin (由聚合类统一 mixin).
"""
from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


class SettersMixin:
    """set_arm_angle / set_hand_angle"""

    def set_arm_angle(self, angle: float, speed: int, timeout: float) -> dict:
        try:
            a = float(angle)
        except (TypeError, ValueError):
            raise ValueError(f"set_arm_angle angle 必须是数字, 收到: {angle!r}")
        if a > self._ARM_ANGLE_MAX or a < self._ARM_ANGLE_MIN:
            raise ValueError(
                f"set_arm_angle({a}) 超出业务硬限 [{self._ARM_ANGLE_MIN}, "
                f"{self._ARM_ANGLE_MAX}]°。\n"
                f"  规则: 大臂角度 ∈ [+150, -150]° (对称硬限, 2026-08-05 用户放宽 +90→+150)"
            )
        skip_y_protect = self._is_arm_safe_position()
        if skip_y_protect:
            logger.info("set_arm_angle: 大臂已 <= -30 或 >= +30, 跳过 y 保护区")
        self._check_y_protected(
            "set_arm_angle",
            allow_init_position=(a == 90.0 or a == 0.0),
            skip=skip_y_protect,
        )
        return self._call_arm("set_arm_angle", timeout=timeout, angle=a, speed=speed)

    def set_hand_angle(self, angle: float, speed: int, timeout: float) -> dict:
        """2026-08-07 优化: arm_angle 安全判断改用 ``_read_arm_angle_realtime``
        (1 HTTP GET → arm_feed 缓存), 替代 ``get_state()`` (3 HTTP calls)."""
        try:
            a = float(angle)
        except (TypeError, ValueError):
            raise ValueError(f"set_hand_angle angle 必须是数字, 收到: {angle!r}")
        if a > self._HAND_ANGLE_MAX or a < self._HAND_ANGLE_MIN:
            raise ValueError(
                f"set_hand_angle({a}) 超出业务硬限 [{self._HAND_ANGLE_MIN}, "
                f"{self._HAND_ANGLE_MAX}]°。\n"
                f"  规则: 手爪角度 ∈ [-90, +10]° (UP=-90, DOWN=0, P 姿态上限 +10, 2026-08-05)"
            )
        try:
            cur_arm = self._read_arm_angle_realtime()  # fast-path: 1 HTTP GET
        except Exception:
            cur_arm = None
        if cur_arm is not None and self._ARM_SAFE_BAND_MIN <= cur_arm <= self._ARM_SAFE_BAND_MAX:
            if a != self._HAND_ANGLE_MIN:
                raise ValueError(
                    f"set_hand_angle({a}) 拒绝: 当前大臂在 [{self._ARM_SAFE_BAND_MIN}, "
                    f"{self._ARM_SAFE_BAND_MAX}]° 展开区, 手爪只允许 init (UP=-90°)."
                )
            self._check_y_protected("set_hand_angle", allow_init_position=True)
        else:
            self._check_y_protected("set_hand_angle", allow_init_position=(a == -90.0))
        return self._call_arm("set_hand_angle", timeout=timeout, angle=a, speed=speed)
