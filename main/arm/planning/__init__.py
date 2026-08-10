"""main/arm/planning —— 机械臂轨迹规划（纯 Python，无硬件依赖）。

- `joint_trajectory`：4-DOF 多关键点平滑（goal → waypoint → … → goal），
  每轴带终端速度梯形 + 四轴同步，关键点精确经过，中间点可选不停车穿越。
  输入来自示教器标定姿势（见 `main/arm/postures.py` 姿势库）。
- 离线可用 FakeRobotSim 仿真（`runtime/services/fake_robot.py`），
  真机把 `JointTrajectory.dense_waypoints()` 喂给 composite_run。
"""
from main.arm.planning.joint_trajectory import (
    ARM_MAX_DEG, ARM_MIN_DEG, HAND_MAX_DEG, HAND_MIN_DEG,
    Y_MAX_MM, Y_MIN_MM,
    JointPose, JointSegment, JointTrajectory, plan_joint_trajectory,
)

__all__ = [
    "JointPose", "JointSegment", "JointTrajectory", "plan_joint_trajectory",
    "ARM_MIN_DEG", "ARM_MAX_DEG", "HAND_MIN_DEG", "HAND_MAX_DEG",
    "Y_MIN_MM", "Y_MAX_MM",
]
