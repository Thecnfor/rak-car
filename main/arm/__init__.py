"""main/arm 子包：机械臂业务层。

外部 import 只允许指向 main.*，不接触 runtime / smartcar。
"""
from .api import ArmClient, ArmSafetyError
from .state import (
    ArmState,
    ArmOrigin,
    SIDES,
    HANDS,
    STORAGE_SIDES,
    STORAGE_DEFAULT_LEFT_ANGLE,
    STORAGE_DEFAULT_RIGHT_ANGLE,
)
from .origin import OriginCalibrator, run_calibrator
from .trajectory import (
    TrajectoryGenerator,
    TrajectoryPlan,
    TrajectorySample,
)
from .loops.runner import ArmRunner
# 2026-07-31 视觉伺服封装（VISION_SERVO_DESIGN.md）：
from .labels import (
    Label, LabelInfo, LABELS, LABEL_GROUPS,
    get_label_info, is_in_group,
)
from .vision import (
    ArmVisionClient,
    Detection, BBoxNorm, BBoxPixels,
    TargetSelector, SelectionStrategy,
    ServoTrace, ServoResult,
)

__all__ = [
    "ArmClient", "ArmSafetyError", "ArmRunner", "ArmState", "ArmOrigin",
    "SIDES", "HANDS", "STORAGE_SIDES",
    "STORAGE_DEFAULT_LEFT_ANGLE", "STORAGE_DEFAULT_RIGHT_ANGLE",
    "OriginCalibrator", "run_calibrator",
    "TrajectoryGenerator", "TrajectoryPlan", "TrajectorySample",
    # 视觉伺服
    "Label", "LabelInfo", "LABELS", "LABEL_GROUPS",
    "get_label_info", "is_in_group",
    "ArmVisionClient",
    "Detection", "BBoxNorm", "BBoxPixels",
    "TargetSelector", "SelectionStrategy",
    "ServoTrace", "ServoResult",
]