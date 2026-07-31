"""main/chassis/config/lane_follow.py
巡线外环 profile：只描述循环节律和控制器选型。
参数值统一从 controllers/ 里的默认值取，profile 不再重复持有。
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Optional

if TYPE_CHECKING:  # 避免 config 反向依赖控制律实现
    from ..controllers.base import OuterLoop, WheelSmoother


class ControllerType(str, Enum):
    """profile 支持的控制律。``build_outer()`` 按这个字段分发。"""

    CURVATURE_ADAPTIVE = "curvature_adaptive"
    STANLEY = "stanley"
    P = "p"


# ─── Magic-number 集中点（#8）─────────────────────────────────
# 这些值以前散落在 curvature_adaptive / stanley / safety 里，
# 现在统一到这里，方便跨控制器统一调。
# 含义见 controllers/ 下各文件。

# _update_curvature 中过滤 lane_feed 异常间隔（秒）
DT_VALID_MIN_S: float = 0.005
DT_VALID_MAX_S: float = 0.5

# omega_raw 的硬截上限（弧度变化率）
KAPPA_HARD_CAP: float = 1.5

# Stanley 的 vx 防除零下限
STANLEY_VX_FLOOR: float = 0.05

# LostLineDetector 的零误差阈值（m / rad）
LOST_LINE_ZERO_EPS: float = 1e-3


@dataclass(frozen=True)
class LaneFollowProfile:
    """巡线外环 profile：只定义循环行为，不持有控制器参数。"""

    # --- 控制律选择 ---
    controller_type: ControllerType = ControllerType.CURVATURE_ADAPTIVE

    # --- 循环节律 ---
    hz: float = 50.0
    max_seconds: float = 85.0
    # 兜底：None = 关掉该项检查
    watchdog_ms: Optional[float] = 500.0
    lost_line_ms: Optional[float] = None

    def build_outer(self) -> "OuterLoop":
        if self.controller_type == ControllerType.STANLEY:
            from ..controllers.stanley import StanleyOuterLoop
            return StanleyOuterLoop()
        if self.controller_type == ControllerType.P:
            from ..controllers.p_controller import POuterLoop
            return POuterLoop()

        from ..controllers.curvature_adaptive import CurvatureAdaptiveOuterLoop
        return CurvatureAdaptiveOuterLoop()

    def build_smoother(self) -> "WheelSmoother":
        from ..controllers.base import WheelSmoother
        return WheelSmoother()


# 现场默认 profile
LANE_FOLLOW = LaneFollowProfile()
