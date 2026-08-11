#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/chassis/cli/run_rak_lane.py
直接调 rak/car_wrap_2026.MyCar.lane_base 的巡线入口（绕过 runtime HTTP 外环）。

lane_base(speed, end_fuction) 内部闭环：
    get_lane_results() → lane_pid(y/angle 双 PID) → set_velocity(speed, y_speed, angle_speed)
直到 end_fuction() 为 True。

停止条件（end_fuction）：
    - 按键 3 急停（key 线程置 _stop_flag）
    - --max-distance：odom distance（get_distance）超过阈值
    - --max-seconds：定时
    都不设则一直巡到按键 3 / Ctrl+C。

用法：
    /usr/bin/python3 -m main.chassis.cli.run_rak_lane --speed 0.3 --max-distance 30
    /usr/bin/python3 main/chassis/cli/run_rak_lane.py --speed 0.3 --max-seconds 5
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# 把 rak/ 插到 sys.path 首位：car_wrap_2026 里的 `from smartcar import ...`
# 才能解析到 rak/smartcar（仓库根也有 smartcar/，同名包会撞）。
_RAK_DIR = str(Path(__file__).resolve().parents[3] / "rak")
sys.path.insert(0, _RAK_DIR)

from car_wrap_2026 import MyCar  # noqa: E402


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="main.chassis.cli.run_rak_lane",
        description="用 rak/car_wrap_2026.MyCar.lane_base 沿车道中心线巡线。",
    )
    parser.add_argument("--speed", type=float, default=0.3, help="巡线前向速度 (m/s)")
    parser.add_argument(
        "--max-distance", type=float, default=30.0,
        help="odom distance (get_distance) 达到此值 (m) 即停；0/负=不用距离停",
    )
    parser.add_argument(
        "--max-seconds", type=float, default=None, help="定时停止（秒）；None=不限时"
    )
    args = parser.parse_args(argv)

    car = MyCar()
    try:
        # 里程计清零，本次 distance 从 0 记
        car.reset_position(0, 0, 0, 0)
        deadline = time.monotonic() + args.max_seconds if args.max_seconds else None

        def end_fuction() -> bool:
            if car._stop_flag:  # 按键 3 急停
                return True
            if deadline is not None and time.monotonic() > deadline:
                return True
            return bool(
                args.max_distance and args.max_distance > 0
                and car.get_distance() > args.max_distance
            )

        print(f"巡线开始 speed={args.speed} max_distance={args.max_distance}")
        car.lane_base(args.speed, end_fuction)
    finally:
        try:
            car.stop()
        finally:
            car.close()


if __name__ == "__main__":
    main()
