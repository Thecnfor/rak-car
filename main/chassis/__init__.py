# main/chassis 子包：底盘组独占目录
# 外部 import 只允许指向 main.*，不接触 runtime / smartcar
from __future__ import annotations

from typing import Callable, Optional

from .api import ChassisClient
from .state import LaneState
from .controllers.base import OuterLoop, WheelSmoother
from .controllers.p_controller import POuterLoop
from .controllers.stanley import StanleyOuterLoop
from .controllers.curvature_adaptive import CurvatureAdaptiveOuterLoop
from .loops.closed_loop import DoubleLoopRunner
from .loops.safety import EmergencyWatchdog, LostLineDetector
from .loops.telemetry import lane_trace
from .tasks.monitor_ir import monitor_ir, IRAlertCallback, IRTickCallback
from .tasks.read_dis import read_dis, DisTickCallback
from .tasks.read_ir import read_ir
from .config import LANE_FOLLOW, ControllerType, LaneFollowProfile


def subscribe_lane_state(
    *,
    profile: LaneFollowProfile = LANE_FOLLOW,
    hz: Optional[float] = None,
    max_seconds: Optional[float] = None,
    dry_run: bool = False,
    with_trace: bool = True,
    on_tick: Optional[Callable[[LaneState, list[float]], None]] = None,
) -> None:
    """巡线外环的**一健装配**：profile → outer / smoother → DoubleLoopRunner。

    等价于手动写::

        api = ChassisClient.connect()
        outer = profile.build_outer()
        smoother = profile.build_smoother()
        on_tick = lane_trace(outer) if with_trace else None
        runner = DoubleLoopRunner(api=api, outer=outer, hz=..., smoother=smoother, on_tick=on_tick)
        runner.run(max_seconds=...)

    用法::

        from main.chassis import subscribe_lane_state, LANE_FOLLOW
        subscribe_lane_state(profile=LANE_FOLLOW, max_seconds=10.0)

    参数：
        profile    - 调参 profile，默认 LANE_FOLLOW
        hz         - 循环频率，默认用 profile.hz
        max_seconds - 最大运行时间，默认用 profile.max_seconds
        dry_run    - True 时只跑控制律不下发轮速
        with_trace - True 时每帧打印 lane 误差 + 轮速
        on_tick    - 覆盖 with_trace 的自定义回调
    """
    api = ChassisClient.connect()
    effective_hz = profile.hz if hz is None else hz

    try:
        api.start_lane_feed(hz=effective_hz)
    except Exception:
        pass

    outer = profile.build_outer()
    smoother = profile.build_smoother()

    if on_tick is None and with_trace:
        on_tick = lane_trace(outer)

    runner = DoubleLoopRunner(
        api=api,
        outer=outer,
        hz=effective_hz,
        watchdog_ms=profile.watchdog_ms,
        lost_line_ms=profile.lost_line_ms,
        dry_run=dry_run,
        smoother=smoother,
        on_tick=on_tick,
    )
    try:
        runner.run(max_seconds=profile.max_seconds if max_seconds is None else max_seconds)
    finally:
        try:
            api.stop_lane_feed()
        except Exception:
            pass

__all__ = [
    "subscribe_lane_state",
    "ChassisClient",
    "LaneState",
    "OuterLoop",
    "WheelSmoother",
    "POuterLoop",
    "StanleyOuterLoop",
    "CurvatureAdaptiveOuterLoop",
    "DoubleLoopRunner",
    "EmergencyWatchdog",
    "LostLineDetector",
    "lane_trace",
    "LaneFollowProfile",
    "ControllerType",
    "LANE_FOLLOW",
    "monitor_ir",
    "IRAlertCallback",
    "IRTickCallback",
    "read_dis",
    "DisTickCallback",
    "read_ir",
]