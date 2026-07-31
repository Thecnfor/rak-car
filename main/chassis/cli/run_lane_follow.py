"""main/chassis/cli/run_lane_follow.py
巡线外环的命令行入口。

用法：
    python3 -m main.chassis.cli.run_lane_follow
    python3 -m main.chassis.cli.run_lane_follow --dry-run --max-seconds 5
    python3 -m main.chassis.cli.run_lane_follow --no-trace --controller stanley
"""
from __future__ import annotations

import argparse
from dataclasses import replace

from ..api import ChassisClient
from ..config.lane_follow import LANE_FOLLOW, ControllerType, LaneFollowProfile
from ..loops.closed_loop import DoubleLoopRunner
from ..loops.telemetry import lane_trace


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="main.chassis.cli.run_lane_follow",
        description="跑一行底盘巡线外环，参数走控制器默认值。",
    )
    parser.add_argument(
        "--controller",
        choices=[c.value for c in ControllerType],
        default=None,
        help="覆盖控制器类型（curvature_adaptive / stanley / p）",
    )
    parser.add_argument("--dry-run", action="store_true", help="只跑控制律不下发轮速")
    parser.add_argument("--no-trace", action="store_true", help="关掉每帧打印")
    parser.add_argument("--hz", type=float, default=None, help="循环频率，默认用 profile.hz")
    parser.add_argument("--max-seconds", type=float, default=None, help="最大运行时间，默认用 profile.max_seconds")
    parser.add_argument("--watchdog-ms", type=float, default=None, help="数据过期急停阈值 ms")
    parser.add_argument("--lost-line-ms", type=float, default=None, help="丢线检测阈值 ms")
    args = parser.parse_args(argv)

    profile = LANE_FOLLOW
    if args.controller:
        profile = replace(profile, controller_type=ControllerType(args.controller))

    effective_hz = profile.hz if args.hz is None else args.hz
    effective_max_seconds = profile.max_seconds if args.max_seconds is None else args.max_seconds
    effective_watchdog = profile.watchdog_ms if args.watchdog_ms is None else args.watchdog_ms
    effective_lost_line = profile.lost_line_ms if args.lost_line_ms is None else args.lost_line_ms

    api = ChassisClient.connect()

    # 守护线程启动只做一次(幂等): lane_feed 是常驻生产者,本脚本只是消费者,
    # 生命周期完全解耦 — 脚本退出不影响守护线程,下一个客户端立即拿到最新缓存。
    try:
        api.start_lane_feed(hz=effective_hz)
    except Exception:
        pass

    outer = profile.build_outer()
    smoother = profile.build_smoother()
    on_tick = None if args.no_trace else lane_trace(outer)

    runner = DoubleLoopRunner(
        api=api,
        outer=outer,
        hz=effective_hz,
        watchdog_ms=effective_watchdog,
        lost_line_ms=effective_lost_line,
        dry_run=args.dry_run,
        smoother=smoother,
        on_tick=on_tick,
    )
    runner.run(max_seconds=effective_max_seconds)


if __name__ == "__main__":
    main()
