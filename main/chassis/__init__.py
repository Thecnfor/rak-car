# main/chassis 子包：底盘组独占目录
# 外部 import 只允许指向 main.*，不接触 runtime / smartcar
from .api import ChassisClient
from .state import LaneState
from .loops.closed_loop import DoubleLoopRunner
from .loops.safety import EmergencyWatchdog, LostLineDetector
from .controllers.base import OuterLoop, WheelSmoother
from .controllers.p_controller import POuterLoop
from .controllers.stanley import StanleyOuterLoop
from .controllers.pure_pursuit import PurePursuitOuterLoop
from .controllers.curvature_adaptive import CurvatureAdaptiveOuterLoop
from .tasks import auto_navigate  # 2026-07-16: 自动导航任务（外环 + 视觉 + 安全）

__all__ = [
    "ChassisClient",
    "LaneState",
    "DoubleLoopRunner",
    "EmergencyWatchdog",
    "LostLineDetector",
    "OuterLoop",
    "WheelSmoother",
    "POuterLoop",
    "StanleyOuterLoop",
    "PurePursuitOuterLoop",
    "CurvatureAdaptiveOuterLoop",
    "auto_navigate",
]
