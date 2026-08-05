"""main/arm/api/safety.py — 业务层安全门 mixin.

从 api.py 拆出: 软限位校验 / y 保护区 / 大臂手爪硬限 / 丢步核对。
所有 mixin 在调用 _check_safe / _check_y_protected 之前必须先 mixin SafetyMixin。
"""
from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class ArmSafetyError(ValueError):
    """机械臂安全门拦截时抛的异常。

    业务层入口（move_x / move_y / set_arm_angle / set_hand_angle）目前的
    保护区检查仍统一抛 ``ValueError``；本类作为 ``ValueError`` 的子类提供
    显式语义——调用方可以用 ``except ArmSafetyError`` 单独拦截安全门拦截，
    而不必 match 字符串。当前所有安全检查仍抛 ValueError 的实现在此统一
    收敛前，本类可被后续代码直接 raise / 作基类使用。
    """
    pass


class SafetyMixin:
    """ArmClient 的安全门行为."""

    # ---- 软限位(y only;x 轴已取消 2026-07-16) ----

    def _check_safe(self, x_mm: Optional[float] = None,
                    y_mm: Optional[float] = None) -> None:
        """y 业务坐标: 触底=0, 向下为正, 向上为负; 区间 [-soft_y_max_mm, 0]."""
        from ..state import ArmOrigin
        origin = self.origin or ArmOrigin()
        if y_mm is not None and not (-origin.soft_y_max_mm <= y_mm <= 0.0):
            raise ValueError(
                f"y_mm={y_mm} 超出软区间 [-{origin.soft_y_max_mm:.0f}, 0] mm"
                f" (触底=0, 顶部=-{origin.soft_y_max_m:.0f}mm)"
            )

    @staticmethod
    def _check_step_loss(axis: str, target_mm: float, actual_mm: float,
                         threshold_mm: float) -> None:
        try:
            err = abs(float(actual_mm) - float(target_mm))
        except (TypeError, ValueError):
            return
        if err > threshold_mm:
            print(
                f"[move_{axis}] 警告: 目标={target_mm:.1f}mm 实际={actual_mm:.1f}mm "
                f"偏差={err:.1f}mm > {threshold_mm:.1f}mm (步进/电机可能丢步或堵转)",
                flush=True,
            )

    # ---- y 保护区 (fail-closed 2026-07-31) ----

    _Y_PROTECTED_THRESHOLD_MM = -30.0

    def _check_y_protected(self, action: str, *,
                           allow_init_position: bool = False,
                           skip: bool = False) -> None:
        if skip:
            return
        try:
            st = self.get_state()
            y_mm = float(st.y_mm)
        except Exception as exc:
            logger.warning(
                "_check_y_protected: 读不到 state, 保守拒绝 (action=%s, err=%s)",
                action, exc,
            )
            raise ValueError(
                f"[{action}] 无法读取 y 状态, 保守拒绝。runtime 是否在线?"
            ) from exc
        if y_mm > self._Y_PROTECTED_THRESHOLD_MM:
            if allow_init_position:
                return
            raise ValueError(
                f"[{action}] y={y_mm:.1f}mm ∈ [0, -30] 安全保护区, 禁止动。\n"
                f"  规则: 接近触底时舵机摆动会撞车\n"
                f"  解决: 先 ArmClient.move_y(-150) 或更低, 再试。\n"
                f"  例外: set_hand('UP'/-90) / set_arm_angle('MID'/0) 初始化姿态允许。"
            )

    # ---- 大臂 / 手爪硬限(业务层 2026-07-27 v3) ----

    _ARM_ANGLE_MIN = -150.0
    _ARM_ANGLE_MAX = 90.0
    _HAND_ANGLE_MIN = -90.0
    _HAND_ANGLE_MAX = 10.0
    _ARM_SAFE_BAND_MIN = -30.0
    _ARM_SAFE_BAND_MAX = 30.0

    def _validate_arm_angle_client(self, angle, action):
        try:
            a = float(angle)
        except (TypeError, ValueError):
            raise ValueError(f"{action} arm_angle 必须是数字, 收到: {angle!r}")
        if a > self._ARM_ANGLE_MAX or a < self._ARM_ANGLE_MIN:
            raise ValueError(
                f"{action} arm_angle({a}) 超出业务硬限 [{self._ARM_ANGLE_MIN}, "
                f"{self._ARM_ANGLE_MAX}]°。\n"
                f"  规则: 大臂角度 ∈ [+90, -150]° (+90 是复位位, -150 是结构极限)"
            )

    def _validate_hand_angle_client(self, angle, action):
        try:
            a = float(angle)
        except (TypeError, ValueError):
            raise ValueError(f"{action} hand 必须是数字, 收到: {angle!r}")
        if a > self._HAND_ANGLE_MAX or a < self._HAND_ANGLE_MIN:
            raise ValueError(
                f"{action} hand({a}) 超出业务硬限 [{self._HAND_ANGLE_MIN}, "
                f"{self._HAND_ANGLE_MAX}]°。\n"
                f"  规则: 手爪角度 ∈ [-90, 0]° (DOWN=0, UP=-90)"
            )

    def _is_arm_safe_position(self) -> bool:
        try:
            st = self.get_state()
        except Exception:
            return False
        cur = st.arm_angle
        if cur is None:
            return False
        return cur <= self._ARM_SAFE_BAND_MIN or cur >= self._ARM_SAFE_BAND_MAX
