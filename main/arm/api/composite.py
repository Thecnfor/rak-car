"""main/arm/api/composite.py — 复合动作 mixin (5 个 composite_*).

依赖 SafetyMixin (由聚合类统一 mixin). 入口一次性 _check_y_protected + _check_safe + 硬限校验;
单次 _call_arm 内部 ThreadPoolExecutor 真并发.
"""
from __future__ import annotations

from typing import Optional


def _mm_to_m(v_mm):
    return float(v_mm) / 1000.0


class CompositeMixin:
    """5 个 composite_* 入口."""

    def composite_pick(self, arm_angle: float, x_mm: float, y_mm: float,
                       hand: float = 0.0, speed: int = 80,
                       timeout: float = 30.0) -> dict:
        action = "composite_pick"
        self._validate_arm_angle_client(arm_angle, action)
        self._validate_hand_angle_client(hand, action)
        self._check_y_protected(action)
        self._check_safe(y_mm=y_mm)
        return self._call_arm(
            action, timeout=timeout,
            arm_angle=arm_angle, x=_mm_to_m(x_mm), y=_mm_to_m(y_mm),
            hand=hand, speed=speed,
        )

    def composite_release(self, drop_x_mm: float = 0.0, drop_y_mm: float = 30.0,
                          hand: float = 0.0, speed: int = 80,
                          timeout: float = 30.0) -> dict:
        action = "composite_release"
        self._validate_hand_angle_client(hand, action)
        self._check_y_protected(action)
        self._check_safe(y_mm=drop_y_mm)
        return self._call_arm(
            action, timeout=timeout,
            drop_x=_mm_to_m(drop_x_mm), drop_y=_mm_to_m(drop_y_mm),
            hand=hand, speed=speed,
        )

    def composite_go_home(self, hand: float = -90.0, arm: float = 0.0,
                          speed: int = 80, timeout: float = 30.0) -> dict:
        action = "composite_go_home"
        self._validate_arm_angle_client(arm, action)
        self._validate_hand_angle_client(hand, action)
        self._check_y_protected(action)
        return self._call_arm(
            action, timeout=timeout,
            hand=hand, arm=arm, speed=speed,
        )

    def composite_run(self, *, arm: Optional[float] = None,
                      x_mm: Optional[float] = None, y_mm: Optional[float] = None,
                      hand: Optional[float] = None, speed: int = 80,
                      timeout: float = 30.0, x_v_max_mms: float = 100.0) -> dict:
        # 用户 23:31: 不怕撞车! _check_y_protected 去掉! 要速度!
        # 注: x_v_max_mms 暂不传给 runtime (Jetson SDK 可能还没更新), 保留接口兼容
        kwargs = dict(
            arm=arm,
            x=_mm_to_m(x_mm) if x_mm is not None else None,
            y=_mm_to_m(y_mm) if y_mm is not None else None,
            hand=hand, speed=speed,
        )
        return self._call_arm("composite_run", timeout=timeout, **kwargs)

    def composite_run_reset(self, *, arm_angle: float = 90.0,
                            hand_angle: float = -90.0, x_direction: str = "right",
                            reset_x_velocity_mms: float = 30.0,
                            timeout: float = 60.0) -> dict:
        return self._call_arm(
            "composite_run_reset", timeout=timeout,
            arm_angle=arm_angle, hand_angle=hand_angle,
            x_direction=x_direction,
            reset_x_velocity=reset_x_velocity_mms / 1000.0,
        )
