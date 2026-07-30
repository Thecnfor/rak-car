"""main/chassis/cli/run_lane_follow.py
巡线外环的命令行入口。参数全部走 controllers 默认值

用法：
    python3 -m main.chassis.cli.run_lane_follow
    python3 -m main.chassis.cli.run_lane_follow --dry-run --max-seconds 5
    python3 -m main.chassis.cli.run_lane_follow --tune v_max=0.2 --tune ki_y=0.0
    python3 -m main.chassis.cli.run_lane_follow --no-trace
"""
from __future__ import annotations

import argparse
import inspect

from ..api import ChassisClient
from ..controllers.base import WheelSmoother
from ..controllers.curvature_adaptive import CurvatureAdaptiveOuterLoop
from ..loops.closed_loop import DoubleLoopRunner
from ..loops.telemetry import lane_trace


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


def _split_tune_params(tune: dict) -> tuple[dict, dict]:
    """把 --tune 拆分为 outer 参数和 smoother 参数。"""
    outer_params = set(inspect.signature(CurvatureAdaptiveOuterLoop).parameters.keys())
    smoother_params = set(inspect.signature(WheelSmoother).parameters.keys())

    outer_kw: dict = {}
    smoother_kw: dict = {}
    unknown: list[str] = []

    for k, v in tune.items():
        if k in outer_params:
            outer_kw[k] = v
        elif k in smoother_params:
            smoother_kw[k] = v
        else:
            unknown.append(k)

    if unknown:
        raise SystemExit(
            f"--tune 未知字段: {sorted(unknown)}，"
            f"可选 outer: {sorted(outer_params)}，smoother: {sorted(smoother_params)}"
        )
    return outer_kw, smoother_kw


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="main.chassis.cli.run_lane_follow",
        description="跑一行底盘巡线外环，参数全部走 controllers 默认值。",
    )
    parser.add_argument("--dry-run", action="store_true", help="只跑控制律不下发轮速")
    parser.add_argument("--no-trace", action="store_true", help="关掉每帧打印")
    parser.add_argument("--hz", type=float, default=50.0, help="循环频率，默认 50Hz")
    parser.add_argument("--max-seconds", type=float, default=85.0, help="最大运行时间，默认 85s")
    parser.add_argument("--watchdog-ms", type=float, default=500.0, help="数据过期急停阈值 ms")
    parser.add_argument("--lost-line-ms", type=float, default=None, help="丢线检测阈值 ms")
    parser.add_argument(
        "--tune",
        action="append",
        default=[],
        metavar="key=value",
        help="覆盖任意 outer/smoother 参数，自动识别归属",
    )
    args = parser.parse_args(argv)

    tune = _parse_kv_pairs(args.tune)
    outer_kw, smoother_kw = _split_tune_params(tune)

    api = ChassisClient.connect()
    effective_hz = args.hz

    try:
        api.start_lane_feed(hz=effective_hz)
    except Exception:
        pass

    outer = CurvatureAdaptiveOuterLoop(**outer_kw)
    smoother = WheelSmoother(**smoother_kw)
    on_tick = None if args.no_trace else lane_trace(outer)

    runner = DoubleLoopRunner(
        api=api,
        outer=outer,
        hz=effective_hz,
        watchdog_ms=args.watchdog_ms,
        lost_line_ms=args.lost_line_ms,
        dry_run=args.dry_run,
        smoother=smoother,
        on_tick=on_tick,
    )
    try:
        runner.run(max_seconds=args.max_seconds)
    finally:
        try:
            api.stop_lane_feed()
        except Exception:
            pass


if __name__ == "__main__":
    main()
