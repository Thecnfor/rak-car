#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task4 / dipan —— 底盘单步直行 (单动作脚本, 不进 arm 长流程)

职责单一: 只做 "底盘前进 N mm" 这一动作, 其他什么都不管。

调用入口 (按使用频率):
  1. step_chassis_forward(client, distance_mm=80)   # 程序里调用
  2. CLI:
       python -m main.arm.each_task.task4.dipan                 # 默认前 80mm
       python -m main.arm.each_task.task4.dipan --distance 200   # 前 200mm
       python -m main.arm.each_task.task4.dipan --sign -1        # 后退 80mm
       python -m main.arm.each_task.task4.dipan --axis y         # 用 y 轴前进 (如果你的
                                                                  # chassis 约定是 y 而非 x)
       python -m main.arm.each_task.task4.dipan --dry-run        # 只 print 参数

forward 方向约定 (现场跑过的基线, 来自 main/arm/each_task/task4/target4.py:
                 forward 模式每轮调 car.move_for([0.05, 0, 0])):
  - 默认 axis='x', 即 position_offset[0] = 前进轴 (m, 正=前进, 负=后退)
  - 跟 SDK docstring 写的 [x偏移, y偏移, 角度偏移] 不完全一致 —— 现场以
    target4.py 的实测为准。
  - 若现场发现方向反了, 先用 --sign -1 翻转; 真要换 axis 用 --axis y。

底层走 runtime CAR_ACTIONS["move_for"] (runtime/core/actions.py:15), 透传
到车端 car.move_for, 车端走完自动停 (timeout 由 max_velocities 隐式约束)。

不依赖 main/chassis/ 的外环 (DoubleLoopRunner), 不下轮速, 也不开 lane_feed;
是「一点到点」的一次性位置闭环, 跟 task4 业务脚本 (target4.py / open_storage.py)
风格一致 —— 调一次等结果, 不并发。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# 项目根 → sys.path, 允许 `python main/arm/each_task/task4/dipan.py` 直接跑
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.api_client import RuntimeApiClient  # noqa: E402
from main.arm import ArmClient  # noqa: E402  # 复用 .http, 顺便走 preflight
from main.arm.test._runtime_guard import preflight, postflight  # noqa: E402


# ---- 默认参数 (与 task4/target4.py: DEFAULT_CHASSIS_STEP_M = 0.05m 对齐) ----
DEFAULT_DISTANCE_MM: float = 80.0     # 默认前 80mm (用户 2026-07-30 要求)
DEFAULT_AXIS: str = "x"               # 默认用 SDK 的 position_offset[0] 当前进轴
                                      # (task4/target4.py:33 现场验证约定)
DEFAULT_SIGN: int = +1                # +1=前进, -1=后退
DEFAULT_TIMEOUT_S: float = 30.0       # move_for 一次性位置闭环, 80mm 1-3s 就够, 留余量
DEFAULT_MAX_VELOCITY_MS: float = 0.20 # m/s, 与 smartcar/whalesbot/.../mecanum.py:632 默认 max_velocities[0] 对齐


def _stop_chassis_quietly(http: RuntimeApiClient) -> None:
    """兜底停底盘 —— best-effort, 失败静默, 用于 finally 块。

    2026-07-31 加: target4.py 用 step_chassis_forward (一次性 move_for, 走完自动停)
    串行多轮跑, 正常不需要兜底; 但保险起见 still 调一次 car.stop()
    以防最后一轮 move_for 失败 / 中断时车没停。

    Args:
        http: RuntimeApiClient 实例。
    """
    try:
        http.execute_car_action(
            "stop",                     # runtime CAR_ACTIONS["stop"] -> car.stop()
            timeout=5.0,
            sync=True,
        )
    except Exception:
        # 静默: finally 块不允许抛
        pass


def step_chassis_forward(
    http: RuntimeApiClient,
    *,
    distance_mm: float = DEFAULT_DISTANCE_MM,
    axis: str = DEFAULT_AXIS,
    sign: int = DEFAULT_SIGN,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    max_velocity_ms: float = DEFAULT_MAX_VELOCITY_MS,
) -> dict:
    """底盘单步直行 distance_mm (mm, sign=+1=前进 / -1=后退)。

    走 runtime CAR_ACTIONS["move_for"] -> 车端 car.move_for, 一次性位置闭环,
    走完自动停。返回 /v1/execute 的同步 job dict。

    Args:
        http: RuntimeApiClient 实例 (用 ArmClient.http 也行, 这里不强求 ArmClient)
        distance_mm: 目标距离, 单位 mm。> 0
        axis: 'x' (默认, position_offset[0] = 前进轴) 或 'y' (position_offset[1])
        sign: +1 / -1, 决定正走还是反走
        timeout_s: HTTP 同步超时秒
        max_velocity_ms: 最大线速度 m/s, 默认 0.20 = 与 SDK move_for 默认 max_velocities 对齐

    Returns:
        dict: /v1/execute 的同步 job dict, 含 status/result/error。

    Raises:
        ValueError: 参数不合法 (distance_mm<=0, axis 不在 {'x','y'}, sign 不在 {+1,-1})
        RuntimeError: job failed 时带车端 error 字段抛出
    """
    # ---- 参数校验 ----
    if distance_mm <= 0:
        raise ValueError(f"distance_mm 必须 > 0, 收到: {distance_mm}")
    if axis not in ("x", "y"):
        raise ValueError(f"axis 必须是 'x' 或 'y', 收到: {axis!r}")
    if sign not in (+1, -1):
        raise ValueError(f"sign 必须是 +1 或 -1, 收到: {sign}")

    d_m = float(distance_mm) / 1000.0
    offset = [0.0, 0.0, 0.0]   # [x_offset, y_offset, theta_offset]
    if axis == "x":
        offset[0] = sign * d_m
    else:
        offset[1] = sign * d_m

    direction_zh = "前进" if sign > 0 else "后退"
    print(
        f"[task4/dipan] {direction_zh} {distance_mm:.0f}mm  "
        f"(axis={axis}, sign={sign:+d}, offset={offset}, v_max={max_velocity_ms:.2f}m/s, "
        f"timeout={timeout_s:.0f}s)"
    )

    # ---- 下发 car.move_for, sync=True 阻塞等结果 ----
    t0 = time.monotonic()
    job = http.execute_car_action(
        "move_for",
        offset,                          # 位置参: position_offset list
        timeout=timeout_s,
        sync=True,
        max_velocities=[max_velocity_ms, max_velocity_ms, 3.14159 / 3],
    )
    took_s = time.monotonic() - t0

    ok = isinstance(job, dict) and job.get("status") == "succeeded"
    status = job.get("status") if isinstance(job, dict) else type(job).__name__
    err = job.get("error") if isinstance(job, dict) else None
    flag = "OK  " if ok else "FAIL"
    print(f"  [{flag}] move_for status={status!r}  took={took_s:.2f}s  err={err}")
    if not ok:
        raise RuntimeError(
            f"[task4/dipan] move_for failed: status={status!r}, error={err!r}, "
            f"raw={job if isinstance(job, dict) else str(job)[:200]}"
        )
    return job


def main() -> int:
    parser = argparse.ArgumentParser(
        description="底盘单步直行 (默认前进 80mm, sync 阻塞到完成)",
    )
    parser.add_argument("--distance", type=float, default=DEFAULT_DISTANCE_MM,
                        help=f"距离 mm (默认 {DEFAULT_DISTANCE_MM})")
    parser.add_argument("--axis", choices=["x", "y"], default=DEFAULT_AXIS,
                        help=f"前进轴 ('x' 默认, 与 task4/target4.py 一致; 'y' 用 SDK docstring 约定)")
    parser.add_argument("--sign", type=int, choices=[1, -1], default=DEFAULT_SIGN,
                        help="+1=前进 / -1=后退 (默认 +1)")
    parser.add_argument("--velocity", type=float, default=DEFAULT_MAX_VELOCITY_MS,
                        help=f"最大线速度 m/s (默认 {DEFAULT_MAX_VELOCITY_MS})")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help=f"HTTP 同步超时秒 (默认 {DEFAULT_TIMEOUT_S})")
    parser.add_argument("--dry-run", action="store_true",
                        help="只 print 参数, 不实际调 move_for")
    args = parser.parse_args()

    # ---- 1. preflight (跟 aaa_origin 同款守卫, 网络/服务层问题早暴露) ----
    client = ArmClient.connect()
    if not preflight(client):
        return 1
    print()

    if args.dry_run:
        d_m = args.distance / 1000.0
        offset = [0.0, 0.0, 0.0]
        if args.axis == "x":
            offset[0] = args.sign * d_m
        else:
            offset[1] = args.sign * d_m
        print(f"[DRY-RUN] move_for offset={offset}  "
              f"v_max={args.velocity:.2f}m/s  timeout={args.timeout:.0f}s")
        return 0

    # ---- 2. 起点 odo (best-effort, 失败不阻断) ----
    print("=== 起点 odo ===")
    try:
        odo = client.http.execute_car_action("get_odometry", timeout=5.0, sync=True)
        if isinstance(odo, dict) and odo.get("status") == "succeeded":
            r = odo.get("result") or {}
            print(f"  x={r.get('x', '?'):+.3f}m  y={r.get('y', '?'):+.3f}m  "
                  f"theta={r.get('theta', '?'):+.3f}rad")
        else:
            print(f"  [WARN] get_odometry 失败: {odo}")
    except Exception as exc:
        print(f"  [WARN] get_odometry 异常: {type(exc).__name__}: {str(exc)[:80]}")
    print()

    # ---- 3. 直行 ----
    print("=== 直行 ===")
    try:
        step_chassis_forward(
            client.http,
            distance_mm=args.distance,
            axis=args.axis,
            sign=args.sign,
            timeout_s=args.timeout,
            max_velocity_ms=args.velocity,
        )
    except (ValueError, RuntimeError) as exc:
        print(f"\n[ABORT] {exc}")
        postflight(client, "after")
        return 1
    print()

    # ---- 4. 终点 odo ----
    print("=== 终点 odo ===")
    try:
        odo = client.http.execute_car_action("get_odometry", timeout=5.0, sync=True)
        if isinstance(odo, dict) and odo.get("status") == "succeeded":
            r = odo.get("result") or {}
            print(f"  x={r.get('x', '?'):+.3f}m  y={r.get('y', '?'):+.3f}m  "
                  f"theta={r.get('theta', '?'):+.3f}rad")
        else:
            print(f"  [WARN] get_odometry 失败: {odo}")
    except Exception as exc:
        print(f"  [WARN] get_odometry 异常: {type(exc).__name__}: {str(exc)[:80]}")
    print()

    postflight(client, "after")
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())