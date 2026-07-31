"""main/chassis/cli/run_lane_follow.py
巡线外环的命令行入口。

用法：
    python3 -m main.chassis.cli.run_lane_follow
    python3 -m main.chassis.cli.run_lane_follow --dry-run --max-seconds 5
    python3 -m main.chassis.cli.run_lane_follow --profile slow --tune v_max=0.2 --tune ki_y=0.0
    python3 -m main.chassis.cli.run_lane_follow --no-trace --controller stanley
"""
from __future__ import annotations

import argparse
import inspect

from ..api import ChassisClient
from ..config.lane_follow import (
    LANE_FOLLOW,
    LANE_FOLLOW_SLOW,
    ControllerType,
    LaneFollowProfile,
)
from ..loops.closed_loop import DoubleLoopRunner
from ..loops.telemetry import lane_trace


# 内置 profile 列表（#3）：CLI 与 subscribe_lane_state 共用同一份装配逻辑
_PROFILE_CHOICES = {
    "default": LANE_FOLLOW,
    "slow": LANE_FOLLOW_SLOW,
}


def _parse_kv_pairs(items: list[str]) -> dict:
    """把 `--tune key=value` 解成 dict。"""
    out: dict = {}
    for raw in items:
        if "=" not in raw:
            raise SystemExit(f"--tune 参数必须是 key=value，实际: {raw!r}")
        k, v = raw.split("=", 1)
        k = k.strip()
        try:
            out[k] = float(v)
        except ValueError:
            raise SystemExit(f"--tune 字段 {k!r} 解析失败: {v!r}（应为数字）")
    return out


def _build_profile(args: argparse.Namespace) -> LaneFollowProfile:
    """根据 CLI 参数选 profile，再 apply --tune overrides。

    --controller 切换控制律（#6），--tune 仍然按字段名直接覆盖 profile 字段
    —— 比 inspect.signature 拆 outer/smoother 更简单，且不会因 outer 字段重命名
    而失效。
    """
    base = _PROFILE_CHOICES[args.profile]

    # 控制律切换：单独字段，不走 --tune（避免 enum 字符串解析歧义）
    if args.controller:
        base = base.tuned(controller_type=ControllerType(args.controller))

    # --tune 走 dataclasses.replace（frozen dataclass），任意字段都能覆盖
    tune = _parse_kv_pairs(args.tune)
    if tune:
        base = base.tuned(**tune)
    return base


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="main.chassis.cli.run_lane_follow",
        description="跑一行底盘巡线外环，参数走 LaneFollowProfile。",
    )
    parser.add_argument(
        "--profile",
        choices=sorted(_PROFILE_CHOICES.keys()),
        default="default",
        help="内置 profile：default（实车）/ slow（dry-run 看数）",
    )
    parser.add_argument(
        "--controller",
        choices=[c.value for c in ControllerType],
        default=None,
        help="覆盖 profile.controller_type（curvature_adaptive / stanley / p）",
    )
    parser.add_argument("--dry-run", action="store_true", help="只跑控制律不下发轮速")
    parser.add_argument("--no-trace", action="store_true", help="关掉每帧打印")
    parser.add_argument("--hz", type=float, default=None, help="循环频率，默认用 profile.hz")
    parser.add_argument("--max-seconds", type=float, default=None, help="最大运行时间，默认用 profile.max_seconds")
    parser.add_argument("--watchdog-ms", type=float, default=None, help="数据过期急停阈值 ms")
    parser.add_argument("--lost-line-ms", type=float, default=None, help="丢线检测阈值 ms")
    parser.add_argument(
        "--tune",
        action="append",
        default=[],
        metavar="key=value",
        help="覆盖任意 profile 字段（直接走 dataclasses.replace）",
    )
    args = parser.parse_args(argv)

    profile = _build_profile(args)
    effective_hz = profile.hz if args.hz is None else args.hz
    effective_max_seconds = profile.max_seconds if args.max_seconds is None else args.max_seconds
    effective_watchdog = profile.watchdog_ms if args.watchdog_ms is None else args.watchdog_ms
    effective_lost_line = profile.lost_line_ms if args.lost_line_ms is None else args.lost_line_ms

    api = ChassisClient.connect()

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
    try:
        runner.run(max_seconds=effective_max_seconds)
    finally:
        try:
            api.stop_lane_feed()
        except Exception:
            pass


if __name__ == "__main__":
    main()