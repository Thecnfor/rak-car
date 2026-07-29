"""main/chassis/cli/run_lane_follow.py
巡线外环的命令行入口。

用法：
    python3 -m main.chassis.cli.run_lane_follow
    python3 -m main.chassis.cli.run_lane_follow --dry-run --max-seconds 5
    python3 -m main.chassis.cli.run_lane_follow --tune v_max=0.2 --tune ki_y=0.0
    python3 -m main.chassis.cli.run_lane_follow --profile slow --no-trace
"""
from __future__ import annotations

import argparse
from dataclasses import fields

from ..config import LANE_FOLLOW, LANE_FOLLOW_SLOW, LaneFollowProfile
from main.chassis import subscribe_lane_state


_PROFILES = {
    "default": LANE_FOLLOW,
    "slow": LANE_FOLLOW_SLOW,
}


def _parse_kv_pairs(items: list[str]) -> dict:
    """把 `--tune v_max=0.2 --tune ki_y=0.0` 解成 dict；字段名/类型不合法直接报错退出。"""
    out: dict = {}
    valid = {f.name: f.type for f in fields(LaneFollowProfile)}
    for raw in items:
        if "=" not in raw:
            raise SystemExit(f"--tune 参数必须是 key=value，实际: {raw!r}")
        k, v = raw.split("=", 1)
        k = k.strip()
        if k not in valid:
            raise SystemExit(
                f"--tune 未知字段 {k!r}，可选: {sorted(valid)}"
            )
        try:
            out[k] = float(v)
        except ValueError:
            raise SystemExit(f"--tune 字段 {k!r} 解析失败: {v!r}（应为数字）")
    return out


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="main.chassis.cli.run_lane_follow",
        description="按指定 profile 跑一行底盘巡线外环。",
    )
    parser.add_argument("--dry-run", action="store_true", help="只跑控制律不下发轮速")
    parser.add_argument("--no-trace", action="store_true", help="关掉每帧打印")
    parser.add_argument("--hz", type=float, default=None, help="覆盖 profile.hz")
    parser.add_argument("--max-seconds", type=float, default=None, help="覆盖 profile.max_seconds")
    parser.add_argument(
        "--profile",
        choices=sorted(_PROFILES),
        default="default",
        help="选一个起始 profile（再叠加 --tune）",
    )
    parser.add_argument(
        "--tune",
        action="append",
        default=[],
        metavar="key=value",
        help="在 profile 基础上覆盖任意字段，可重复",
    )
    args = parser.parse_args(argv)

    profile = _PROFILES[args.profile]
    overrides = _parse_kv_pairs(args.tune)
    if overrides:
        profile = profile.tuned(**overrides)

    subscribe_lane_state(
        profile=profile,
        hz=args.hz,
        max_seconds=args.max_seconds,
        dry_run=args.dry_run,
        with_trace=not args.no_trace,
    )


if __name__ == "__main__":
    main()
