# main/chassis 子包：底盘组独占目录
# 外部 import 只允许指向 main.*，不接触 runtime / smartcar
from .api import ChassisClient
from .state import LaneState
from .controllers.base import OuterLoop, WheelSmoother
from .controllers.p_controller import POuterLoop
from .controllers.stanley import StanleyOuterLoop
from .controllers.pure_pursuit import PurePursuitOuterLoop
from .controllers.curvature_adaptive import CurvatureAdaptiveOuterLoop
from .loops.closed_loop import DoubleLoopRunner
from .loops.safety import EmergencyWatchdog, LostLineDetector
from .loops.telemetry import lane_trace
from .config import LANE_FOLLOW, LANE_FOLLOW_SLOW, LaneFollowProfile

__all__ = [
    "ChassisClient",
    "LaneState",
    "OuterLoop",
    "WheelSmoother",
    "POuterLoop",
    "StanleyOuterLoop",
    "PurePursuitOuterLoop",
    "CurvatureAdaptiveOuterLoop",
    "DoubleLoopRunner",
    "EmergencyWatchdog",
    "LostLineDetector",
    "lane_trace",
    "LaneFollowProfile",
    "LANE_FOLLOW",
    "LANE_FOLLOW_SLOW",
]