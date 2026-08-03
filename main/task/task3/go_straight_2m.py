#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/task/task3/go_straight_2m.py - 走 2 米直线

极简实现:沿用户指定的轴走 2m,每步前进 15cm 后做横向纠偏(yaw 反向)。

用法:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.task.task3.go_straight_2m
    python -m main.task.task3.go_straight_2m --axis -x   # 默认
    python -m main.task.task3.go_straight_2m --axis -y   # 走 odom -y 方向

约束:
    - 总前进距离 = 2m(--total-distance 可调)
    - 每步 15cm(--step 可调)
    - 每步后做横向纠偏(用 yaw 反向,把累积的横向漂移打掉)
    - 直线行走,不能左右偏移(用户硬约束)
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from main.api_client import RuntimeApiClient


# ============================================================================
# 常量
# ============================================================================
DRIFT_TOL_M = 0.01             # 横向漂移容差 1cm
YAW_PER_STEP_CAP_DEG = 15.0    # 单次纠偏 yaw 上限
YAW_BUDGET_TOTAL_DEG = 90.0    # 累计纠偏 yaw 预算


# ============================================================================
# helpers
# ============================================================================

def car_call(client, name, *args, timeout=20.0, **kwargs):
    """对齐 shoot_target.py 的写法:`*args` 让外层 list 整体作为位置参数
    传到 runtime,runtime 内部 _dispatch_car 再 *args 解包一次,最终 SDK
    car.move_for(position_offset=[x,y,z]) 就对了。

    用 sync=True,避免 wait_job polling 卡住或超时太短。
    """
    job = client.execute_car_action(name, *args, timeout=timeout,
                                    sync=True, **kwargs)
    if job.get("status") != "succeeded":
        raise RuntimeError(f"car.{name} failed: {job.get('error')}")
    return job.get("result")


def read_odom(client):
    """读 odom 返回 (x, y, yaw_rad)。失败 None。"""
    try:
        odo = (client.get_runtime() or {}).get("runtime", {}).get("odometry") or [0, 0, 0]
        return float(odo[0]), float(odo[1]), float(odo[2])
    except Exception:
        return None


def axis_offset(axis, step):
    """把 axis + step 转成 move_for 的 [x, y, z] 偏移量。"""
    if axis == "y":
        return [0.0, +step, 0.0]
    if axis == "-y":
        return [0.0, -step, 0.0]
    if axis == "x":
        return [+step, 0.0, 0.0]
    if axis == "-x":
        return [-step, 0.0, 0.0]
    raise ValueError(f"未知 axis: {axis}")


def correct_yaw(client, axis, start_x, start_y, drift_budget):
    """横向纠偏:读累积的横向漂移,发一个反向 yaw 把它打掉。

    axis = 前进轴(y / -y / x / -x)
    横向漂移轴 = 垂直于前进轴的那一个

    yaw 符号系数 hardcode = -1(实测,这台 chassis 的 +yaw 是把车头转右,
    跟纠偏"消除右偏需要把车头转左"的方向相反,所以前面要乘 -1)。
    """
    odo = read_odom(client)
    if odo is None:
        return 0.0, drift_budget
    cur_x, cur_y, _ = odo

    if axis in ("y", "-y"):
        # 沿 y 轴走,横向漂移是 x
        drift = cur_x - start_x
        advance = abs(cur_y - start_y)
    else:
        # 沿 x 轴走,横向漂移是 y
        drift = cur_y - start_y
        advance = abs(cur_x - start_x)

    if abs(drift) <= DRIFT_TOL_M:
        return 0.0, drift_budget

    # 几何精确:atan2(drift, advance) 给出从当前位置看回起点的精确反向角
    advance = max(advance, 0.05)
    yaw_deg = -math.degrees(math.atan2(drift, advance))   # × -1 = yaw_sign

    # cap
    if abs(yaw_deg) > YAW_PER_STEP_CAP_DEG:
        yaw_deg = YAW_PER_STEP_CAP_DEG if yaw_deg > 0 else -YAW_PER_STEP_CAP_DEG
    if abs(yaw_deg) > drift_budget:
        yaw_deg = drift_budget if yaw_deg > 0 else -drift_budget
    if abs(yaw_deg) < 0.05:
        return 0.0, drift_budget

    car_call(client, "move_for", [0.0, 0.0, math.radians(yaw_deg)], timeout=10)
    print(f"  [纠偏] drift={drift*1000:+.1f}mm advance={advance*100:.0f}cm "
          f"→ yaw {yaw_deg:+.2f}° (预算剩余 "
          f"{drift_budget - abs(yaw_deg):.1f}°)", flush=True)

    # 验证(可选,不阻塞)
    time.sleep(0.2)
    odo2 = read_odom(client)
    if odo2 is not None:
        if axis in ("y", "-y"):
            drift2 = odo2[0] - start_x
        else:
            drift2 = odo2[1] - start_y
        print(f"  [纠偏-验] 纠偏后 drift={drift2*1000:+.1f}mm "
              f"(变化 {(drift2-drift)*1000:+.1f}mm)", flush=True)

    return yaw_deg, drift_budget - abs(yaw_deg)


# ============================================================================
# main
# ============================================================================

def main():
    ap = argparse.ArgumentParser(
        description="直行 --total-distance 米(默认 2m)"
    )
    ap.add_argument("--axis", choices=["y", "-y", "x", "-x"], default="x",
                    help="前进轴(默认 x = 用户视觉的'前')。"
                         "实测:把车往视觉前推,odom x 从 0 → +1.21m")
    ap.add_argument("--total-distance", type=float, default=2.0,
                    help="总前进距离 m(默认 2.0)")
    ap.add_argument("--step", type=float, default=0.15,
                    help="单步前进 m(默认 0.15 = 15cm)")
    ap.add_argument("--step-delay", type=float, default=0.4,
                    help="步间 sleep s(默认 0.4)")
    ap.add_argument("--drift-tol", type=float, default=DRIFT_TOL_M,
                    help=f"横向漂移容差 m(默认 {DRIFT_TOL_M})")
    ap.add_argument("--no-correction", action="store_true",
                    help="关闭横向纠偏(纯直线不纠偏)")
    ap.add_argument("--no-reset", action="store_true",
                    help="不重置 odom(默认 ON:跑前 reset_position 清零,防止"
                         "历史漂移污染起点)")
    args = ap.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()

    # 跑前自动 reset odom — 清掉历史累积漂移,起点从 0 开始
    # (用户实测:不 reset 起点已经 x=+1m,纠偏永远追不上)
    if not args.no_reset:
        try:
            car_call(client, "reset_position", timeout=5)
            print("[reset] odom 已清零", flush=True)
            time.sleep(0.5)
        except Exception as e:
            print(f"[reset err] {e} (继续跑,起点可能不干净)", file=sys.stderr)

    # 起点 odom
    start = read_odom(client)
    if start is None:
        print("[err] 读不到 odom,退出", file=sys.stderr)
        return 1
    sx, sy, syaw = start
    print(f"[start] x={sx:+.3f}m y={sy:+.3f}m yaw={math.degrees(syaw):+.2f}°",
          flush=True)
    print(f"[plan] 沿 {args.axis} 轴走 {args.total_distance:.2f}m "
          f"(步长 {args.step*100:.0f}cm),"
          f"漂移容差 {args.drift_tol*1000:.0f}mm,纠偏 = "
          f"{'OFF' if args.no_correction else 'ON'}",
          flush=True)

    n_steps = max(1, int(round(args.total_distance / args.step)))
    drift_budget = YAW_BUDGET_TOTAL_DEG

    for step_i in range(1, n_steps + 1):
        offset = axis_offset(args.axis, args.step)
        print(f"\n[step {step_i}/{n_steps}] 前进 "
              f"{abs(args.step)*100:.0f}cm 沿 {args.axis} → "
              f"offset={offset}", flush=True)
        try:
            car_call(client, "move_for", offset, timeout=5)
        except Exception as e:
            print(f"  [step err] {e}", file=sys.stderr)
            continue
        time.sleep(args.step_delay)

        # 横向纠偏
        if not args.no_correction:
            _, drift_budget = correct_yaw(
                client, args.axis, sx, sy, drift_budget
            )
            if drift_budget <= 0.5:
                print("  [warn] 纠偏预算耗尽,后续不再纠偏", flush=True)
                drift_budget = 0.0

        # 检查走了多少
        cur = read_odom(client)
        if cur is not None:
            if args.axis in ("y", "-y"):
                advanced = abs(cur[1] - sy)
            else:
                advanced = abs(cur[0] - sx)
            print(f"  [进度] 已前进 {advanced*100:.1f}cm / "
                  f"{args.total_distance*100:.0f}cm", flush=True)
            if advanced >= args.total_distance:
                print(f"\n[done] 已到达 {args.total_distance:.2f}m", flush=True)
                break

    end = read_odom(client)
    if end is not None:
        if args.axis in ("y", "-y"):
            dx = end[0] - sx
            advanced = abs(end[1] - sy)
        else:
            dx = end[1] - sy
            advanced = abs(end[0] - sx)
        print(f"\n[end] x={end[0]:+.3f}m y={end[1]:+.3f}m "
              f"yaw={math.degrees(end[2]):+.2f}°", flush=True)
        print(f"[end] 实际前进 {advanced*100:.1f}cm,横向漂移 "
              f"{dx*1000:+.1f}mm", flush=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
