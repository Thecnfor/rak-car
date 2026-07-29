"""main/chassis/examples/05_subscribe_lane_state.py
巡线外环的**核心装配**：profile → outer / smoother → DoubleLoopRunner。

不含：
- 调参默认值（→ main/chassis/config/lane_follow.py）
- 每帧 trace 打印（→ main/chassis/loops/telemetry.py）
- 命令行入口 / argparse（→ main/chassis/cli/run_lane_follow.py）

用法（程序化）：
    from main.chassis.examples import subscribe_lane_state
    from main.chassis.config import LANE_FOLLOW
    subscribe_lane_state(profile=LANE_FOLLOW.tuned(v_max=0.2), max_seconds=10.0)
"""
from __future__ import annotations

from typing import Optional

from ..api import ChassisClient
from ..config import LANE_FOLLOW, LaneFollowProfile
from ..loops.closed_loop import DoubleLoopRunner
from ..loops.telemetry import lane_trace


def subscribe_lane_state(
    *,
    profile: LaneFollowProfile = LANE_FOLLOW,
    hz: Optional[float] = None,
    max_seconds: Optional[float] = None,
    dry_run: bool = False,
    with_trace: bool = True,
) -> None:
    """按 profile 装配外环 runner 并跑完。`None` 的字段自动用 profile 内的值。"""
    api = ChassisClient.connect()
    outer = profile.build_outer()
    smoother = profile.build_smoother()
    on_tick = lane_trace(outer) if with_trace else None

    runner = DoubleLoopRunner(
        api=api,
        outer=outer,
        hz=profile.hz if hz is None else hz,
        watchdog_ms=profile.watchdog_ms,
        lost_line_ms=profile.lost_line_ms,
        dry_run=dry_run,
        smoother=smoother,
        on_tick=on_tick,
    )
    runner.run(max_seconds=profile.max_seconds if max_seconds is None else max_seconds)
