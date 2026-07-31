"""main/chassis/config — 调参 profile 集中地。

只放「参数值 + 工厂方法」，不放控制律实现。tasks/examples 都从这里 import profile。
"""
from .lane_follow import LANE_FOLLOW, ControllerType, LaneFollowProfile

__all__ = ["LaneFollowProfile", "ControllerType", "LANE_FOLLOW"]
