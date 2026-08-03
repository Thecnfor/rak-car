"""main/chassis/config/lane_follow.py
巡线外环的**参数值**集中地。examples / tasks 里不再写默认值，只引用这里的 profile。

一个 profile = 一套现场标定好的参数 + 从参数造控制律/软化器的工厂方法。
换场地只改这个文件（或传一个 tuned() 出来的变体），控制律与循环代码不动。
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Optional

if TYPE_CHECKING:  # 避免 config 反向依赖控制律实现
    from ..controllers.base import OuterLoop, WheelSmoother


class ControllerType(str, Enum):
    """profile 支持的控制律。``build_outer()`` 按这个字段分发。"""

    STRAIGHT = "straight"  # 直道底层：vx 巡航 + vy 横移 + ω 视觉航向（mission 默认）
    CURVATURE_ADAPTIVE = "curvature_adaptive"
    STANLEY = "stanley"
    P = "p"
    ORTHOGONAL = "orthogonal"


# ─── Magic-number 集中点（#8）─────────────────────────────────
# 这些值以前散落在 curvature_adaptive / stanley / safety 里，
# 现在统一到这里，方便跨控制器统一调。
# 含义见 controllers/ 下各文件。

# _update_curvature 中过滤 lane_feed 异常间隔（秒）
DT_VALID_MIN_S: float = 0.005
DT_VALID_MAX_S: float = 0.5

# omega_raw 的硬截上限（弧度变化率）—— curvature_adaptive 的 min(kappa, 1.5) 等价物
KAPPA_HARD_CAP: float = 1.5

# Stanley 的 vx 防除零下限
STANLEY_VX_FLOOR: float = 0.05

# LostLineDetector 的零误差阈值（m / rad）
LOST_LINE_ZERO_EPS: float = 1e-3


@dataclass(frozen=True)
class LaneFollowProfile:
    """巡线外环全部可调量。字段语义见 controllers/curvature_adaptive.py 等。"""

    # --- 控制律选择（#6） ---
    # 默认 straight：StraightOuterLoop 是底盘巡线直行的底层（vx 巡航 + vy 横移 + ω 航向）。
    # 场地有急弯时切回 curvature_adaptive / stanley。
    controller_type: ControllerType = ControllerType.STRAIGHT

    # --- 循环节律 ---
    hz: float = 50.0
    max_seconds: float = 85.0
    # 兜底：None = 关掉该项检查
    watchdog_ms: Optional[float] = 500.0
    lost_line_ms: Optional[float] = None  # 笔直居中路段误差本来就会齐 0，默认不按丢线处理

    # 控制器参数全部走 curvature_adaptive.py 默认值，这里不再持有一份。
    # CLI --tune 仍可覆盖 profile 字段，但 build_outer() 不再透传这些字段。

    # --- 误差标定（视觉零漂）---
    # 实车放车道正中读 trace ey，把这个读数（带符号）填到这里 → 正中即标成 0。
    # 0 = 不标定。CLI 未显式传 --error-offset-y 时用它。
    error_offset_y: float = 0.0

    # --- 下发软化（饱和 + slew rate）---
    wheel_max_abs: float = 0.70
    wheel_max_accel: float = 0.4
    wheel_max_decel: float = 0.6

    def build_outer(self) -> "OuterLoop":
        # 控制器参数全部走各自 __init__ 默认值，这里不再透传。
        if self.controller_type == ControllerType.STANLEY:
            from ..controllers.stanley import StanleyOuterLoop
            return StanleyOuterLoop()
        if self.controller_type == ControllerType.P:
            from ..controllers.p_controller import POuterLoop
            return POuterLoop()
        if self.controller_type == ControllerType.STRAIGHT:
            from ..controllers.straight import StraightOuterLoop
            return StraightOuterLoop()
        if self.controller_type == ControllerType.ORTHOGONAL:
            from ..controllers.orthogonal import OrthogonalOuterLoop
            # 默认 vx=0："原地水平稳定"；
            # 如果想切"正交巡航"，传 tuned(controller_type=ORTHOGONAL)
            # 后再在调用侧 outer.vx_target = 0.30（或新 CLI --vx-target 传）
            return OrthogonalOuterLoop()

        from ..controllers.curvature_adaptive import CurvatureAdaptiveOuterLoop
        return CurvatureAdaptiveOuterLoop()

    def build_smoother(self) -> "WheelSmoother":
        from ..controllers.base import WheelSmoother

        return WheelSmoother(
            max_abs=self.wheel_max_abs,
            max_accel=self.wheel_max_accel,
            max_decel=self.wheel_max_decel,
        )

    def tuned(self, **overrides) -> "LaneFollowProfile":
        """临时改几个 profile 自有字段跑一把：``LANE_FOLLOW.tuned(hz=20.0, wheel_max_abs=0.5)``。

        只能覆盖循环节律 / 下发软化 / controller_type。
        控制器增益（v_max / kp_y / ki_y …）不在这里——直接构造
        ``CurvatureAdaptiveOuterLoop(...)`` 并传给 ``subscribe_lane_state(outer=...)``。
        """
        return replace(self, **overrides)


# 现场默认 profile。现在只持有循环节律 + 下发软化参数，
# 控制器参数全部走 curvature_adaptive.py 默认值。
LANE_FOLLOW = LaneFollowProfile()

# dry-run 看数用：降 hz 减少打印量
LANE_FOLLOW_SLOW = LANE_FOLLOW.tuned(hz=20.0)