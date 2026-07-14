"""main/arm 子包：机械臂业务层。

外部 import 只允许指向 main.*，不接触 runtime / smartcar。
"""
from .api import ArmClient
from .state import ArmState, ArmOrigin, SIDES, HANDS
from .origin import OriginCalibrator, run_calibrator
from .trajectory import (
    TrajectoryGenerator,
    TrajectoryPlan,
    TrajectorySample,
)
from .loops.runner import ArmRunner

__all__ = [
    "ArmClient",
    "ArmState",
    "ArmOrigin",
    "SIDES",
    "HANDS",
    "OriginCalibrator",
    "run_calibrator",
    "TrajectoryGenerator",
    "TrajectoryPlan",
    "TrajectorySample",
    "ArmRunner",
]
