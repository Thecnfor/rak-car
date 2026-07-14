"""main/chassis/tasks/follow_lane.py
"沿当前车道线持续外环巡线到目标距离或超时"。
"""
from typing import Optional

from ..api import ChassisClient
from ..controllers.base import OuterLoop
from ..loops.closed_loop import DoubleLoopRunner


def follow_lane(
    api: ChassisClient,
    outer: OuterLoop,
    *,
    dis_hold: float = 1.5,
    timeout_s: float = 30.0,
    hz: float = 50.0,
    start_lane_feed_hz: float = 20.0,
) -> None:
    """起 lane feed + DoubleLoopRunner；按 dis_hold 退出的逻辑由调用方决定（本任务交给 timeout 兜底）。"""
    try:
        api.start_lane_feed(hz=start_lane_feed_hz)
        runner = DoubleLoopRunner(api=api, outer=outer, hz=hz)
        runner.run(max_seconds=float(timeout_s))
    finally:
        try:
            api.stop_lane_feed()
        except Exception:
            pass
