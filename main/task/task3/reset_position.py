#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/reset_position.py - 把车摆回校准起点的便利脚本

**目的**:跑 shoot_by_calib / shoot_4_targets / auto_calibrate_target 前,
把车摆回校准起点(odom x=+0.05, y=+0.03, yaw=-9°),避免脚本
因「起点偏移」失效。

**用法**:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.reset_position
    # 或自定义起点:
    python -m main.tasks.task333.reset_position --x 0.10 --y 0.05 --yaw -10

**前提**:
- 校准起点已知(默认 x=+0.05, y=+0.03, yaw=-9°,实测自 manual_calibrate)
- 每次只摆 1 次车,**不循环收敛**(避免 yaw 误差积累)

**安全机制**:
- 如果 yaw 差 > 90° → 不自动转,要求用户手动(避免「车头转 3 圈」)
- 如果直行距离 > 5m → 不自动直行,要求用户手动
- 每次摆车前读 odom → 提示差距 → 按 Enter 确认
"""
from __future__ import annotations

import argparse
import math
import sys

from main.api_client import RuntimeApiClient
from main.tasks.task333.manual_calibrate import car_call, read_odom


# 校准初始 odom(实测自 manual_calibrate)
DEFAULT_TARGET_X = 0.05
DEFAULT_TARGET_Y = 0.03
DEFAULT_TARGET_YAW_DEG = -9.0

# 安全阈值
MAX_AUTO_DRIVE_M = 5.0    # 直行超过 5m 不自动
MAX_AUTO_YAW_DEG = 90.0   # yaw 差超过 90° 不自动


def main():
    ap = argparse.ArgumentParser(description="把车摆回校准起点")
    ap.add_argument("--x", type=float, default=DEFAULT_TARGET_X,
                    help=f"目标 odom x(m,默认 {DEFAULT_TARGET_X})")
    ap.add_argument("--y", type=float, default=DEFAULT_TARGET_Y,
                    help=f"目标 odom y(m,默认 {DEFAULT_TARGET_Y})")
    ap.add_argument("--yaw", type=float, default=DEFAULT_TARGET_YAW_DEG,
                    help=f"目标 yaw(度,默认 {DEFAULT_TARGET_YAW_DEG})")
    ap.add_argument("--no-confirm", action="store_true",
                    help="跳过确认提示,直接摆车(脚本调用)")
    ap.add_argument("--only-print", action="store_true",
                    help="只打印差距,不摆车")
    args = ap.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()

    odo = read_odom(client)
    if odo is None:
        print("[err] 读不到 odom", file=sys.stderr)
        return 1

    cur_x, cur_y, cur_yaw = odo
    cur_yaw_deg = math.degrees(cur_yaw)

    target_x = args.x
    target_yaw_deg = args.yaw

    # 计算差距
    dx = target_x - cur_x
    dyaw = target_yaw_deg - cur_yaw_deg
    while dyaw > 180:
        dyaw -= 360
    while dyaw < -180:
        dyaw += 360

    print(f"\n[reset] 当前 odom:")
    print(f"  x={cur_x:+.3f}m y={cur_y:+.3f}m yaw={cur_yaw_deg:+.2f}°",
          flush=True)
    print(f"\n[reset] 目标 odom:")
    print(f"  x={target_x:+.3f}m (直行 dx={dx:+.3f}m)"
          f" yaw={target_yaw_deg:+.2f}° (转 dyaw={dyaw:+.2f}°)",
          flush=True)

    if args.only_print:
        return 0

    # 安全检查
    safety_ok = True
    if abs(dx) > MAX_AUTO_DRIVE_M:
        print(f"\n  ⚠ 直行距离 {abs(dx):.2f}m > {MAX_AUTO_DRIVE_M}m,"
              f"不自动摆车", file=sys.stderr)
        safety_ok = False
    if abs(dyaw) > MAX_AUTO_YAW_DEG:
        print(f"\n  ⚠ yaw 差 {abs(dyaw):.1f}° > {MAX_AUTO_YAW_DEG}°,"
              f"不自动转", file=sys.stderr)
        safety_ok = False

    if not safety_ok:
        print(f"\n  请手动摆车到目标 odom 后再跑 shoot_by_calib",
              file=sys.stderr)
        return 1

    # 确认
    if not args.no_confirm:
        print(f"\n  按 Enter 确认摆车(或 Ctrl+C 退出):", flush=True)
        try:
            input()
        except EOFError:
            pass

    # 摆车:先直行,再转 yaw(避免转 yaw 后直行方向错)
    if abs(dx) > 0.02:
        print(f"\n[reset] 直行 dx={dx:+.3f}m...", flush=True)
        try:
            car_call(client, "move_for", [dx, 0.0, 0.0],
                     timeout=max(10, abs(dx) * 20))
            print(f"  ✓ 直行完成", flush=True)
        except Exception as e:
            print(f"  [move err] {e}", file=sys.stderr)
            return 1
    else:
        print(f"  [reset] |dx|={abs(dx):.3f}m < 2cm,跳过直行",
              flush=True)

    if abs(dyaw) > 0.5:
        print(f"\n[reset] 转 yaw dyaw={dyaw:+.2f}°...", flush=True)
        try:
            car_call(client, "move_for", [0.0, 0.0, math.radians(dyaw)],
                     timeout=10)
            print(f"  ✓ 转 yaw 完成", flush=True)
        except Exception as e:
            print(f"  [yaw err] {e}", file=sys.stderr)
            return 1
    else:
        print(f"  [reset] |dyaw|={abs(dyaw):.2f}° < 0.5°,跳过转 yaw",
              flush=True)

    # 验证
    odo = read_odom(client)
    if odo is not None:
        new_x, new_y, new_yaw = odo
        new_yaw_deg = math.degrees(new_yaw)
        err_x = abs(new_x - target_x)
        err_yaw = abs(((target_yaw_deg - new_yaw_deg + 180) % 360) - 180)
        print(f"\n[reset] 摆车后 odom:")
        print(f"  x={new_x:+.3f}m y={new_y:+.3f}m yaw={new_yaw_deg:+.2f}°",
              flush=True)
        print(f"  误差:dx={err_x:.3f}m dyaw={err_yaw:.2f}°",
              flush=True)
        if err_x > 0.05 or err_yaw > 5.0:
            print(f"  ⚠ 误差较大,可能 chassis 反馈有累积误差",
                  file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())