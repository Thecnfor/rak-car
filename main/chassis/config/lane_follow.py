"""main/chassis/config/lane_follow.py
巡线外环的**参数值**集中地。examples / tasks 里不再写默认值，只引用这里的 profile。

一个 profile = 一套现场标定好的参数 + 从参数造控制律/软化器的工厂方法。
换场地只改这个文件（或传一个 replace() 出来的变体），控制律与循环代码不动。
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Optional

if TYPE_CHECKING:  # 避免 config 反向依赖控制律实现
    from ..controllers.base import OuterLoop, WheelSmoother


@dataclass(frozen=True)
class LaneFollowProfile:
    """curvature-adaptive 巡线的全部可调量。字段语义见 controllers/curvature_adaptive.py。"""

    # --- 循环节律 ---
    hz: float = 50.0
    max_seconds: float = 85.0
    # 兜底：None = 关掉该项检查
    watchdog_ms: Optional[float] = 500.0
    lost_line_ms: Optional[float] = None  # 笔直居中路段误差本来就会齐 0，默认不按丢线处理

    # --- 速度曲线 ---
    v_max: float = 0.24
    v_min: float = 0.064
    kappa_full: float = 0.6
    dkappa_full: float = 1.5

    # --- P 项 ---
    kp_y: float = 0.80
    kp_theta: float = 1.5

    # --- 横向 I 项（消除直行稳态偏差）---
    ki_y: float = 0.40
    ey_int_cap: float = 0.10
    ey_int_decay: float = 0.80

    # --- 弧度变化驱动的 omega 增益 + 软上限 ---
    omega_gain: float = 0.35
    k_curvature: float = 0.25
    omega_cap: float = 1.8

    # --- 恢复门控 ---
    ey_release: float = 0.02
    ea_release: float = 0.05
    hold_ms: float = 250.0

    # --- 轴向互斥（直线走 vy / 弯道走 omega）+ 弯道 vy 保底 ---
    kappa_axis_center: float = 1.0
    kappa_axis_width: float = 0.5
    vy_floor: float = 0.15

    # --- 麦轮几何 ---
    r_eff: float = 0.30

    # --- 下发软化（饱和 + slew rate）---
    wheel_max_abs: float = 0.55
    wheel_max_accel: float = 0.4
    wheel_max_decel: float = 0.6

    def build_outer(self) -> "OuterLoop":
        from ..controllers.curvature_adaptive import CurvatureAdaptiveOuterLoop

        return CurvatureAdaptiveOuterLoop(
            v_max=self.v_max,
            v_min=self.v_min,
            kappa_full=self.kappa_full,
            dkappa_full=self.dkappa_full,
            kp_y=self.kp_y,
            kp_theta=self.kp_theta,
            ki_y=self.ki_y,
            ey_int_cap=self.ey_int_cap,
            ey_int_decay=self.ey_int_decay,
            omega_gain=self.omega_gain,
            k_curvature=self.k_curvature,
            omega_cap=self.omega_cap,
            ey_release=self.ey_release,
            ea_release=self.ea_release,
            hold_ms=self.hold_ms,
            kappa_axis_center=self.kappa_axis_center,
            kappa_axis_width=self.kappa_axis_width,
            vy_floor=self.vy_floor,
            r_eff=self.r_eff,
        )

    def build_smoother(self) -> "WheelSmoother":
        from ..controllers.base import WheelSmoother

        return WheelSmoother(
            max_abs=self.wheel_max_abs,
            max_accel=self.wheel_max_accel,
            max_decel=self.wheel_max_decel,
        )

    def tuned(self, **overrides) -> "LaneFollowProfile":
        """临时改几个参数跑一把：``LANE_FOLLOW.tuned(v_max=0.2, ki_y=0.0)``。"""
        return replace(self, **overrides)


# 现场默认 profile。调参先改这里，别散回 examples。
LANE_FOLLOW = LaneFollowProfile()

# dry-run 看数用：不下发也降速，避免误按到实车时冲出去
LANE_FOLLOW_SLOW = LANE_FOLLOW.tuned(v_max=0.18, v_min=0.06)
