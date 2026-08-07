#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/task/task3/arm_seq_v9.py - sequenced arm motion v9

动作序列(每步独立,失败抛错停):
  1. arm.move_y_position(y1)             -> 先到中间 y 位(如 -100mm = -0.100m)
  2. arm.set_hand_angle(hand, speed)     -> 在 y1 处调整手爪舵机角度
  3. arm.set_arm_angle(arm, speed)       -> 在 y1 处调整机械臂(大臂)角度
  4. arm.move_x_position(x)              -> 在 y1 处调整 x 位置
  5. arm.move_y_position(y2)             -> 最后走到目标 y 位(如 -40mm = -0.040m)

每步后读 arm_state 打印。

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤"
    python -m main.task.task3.arm_seq_v9
    python -m main.task.task3.arm_seq_v9 --y1 -0.100 --y2 -0.040 --x -0.270 --arm-angle 90 --hand-angle -70
"""
from __future__ import annotations

import argparse
import time

from main.api_client import RuntimeApiClient


def arm_call(client, name, *a, timeout=20.0, **k):
    job = client.execute_arm_action(name, *a, timeout=timeout, sync=False, **k)
    done = client.wait_job(job["id"], timeout=timeout + 10)
    if done.get("status") != "succeeded":
        raise RuntimeError(f"arm.{name} failed: {done.get('error')}")
    return done.get("result")


def read_arm(client):
    return (client.get_arm_state() or {}).get("arm_state") or {}


# 机械臂移动速度上限(m/s)。y / x 两种轴都按这个走。
# - y(vert)默认 PID output_limits = 0.1 m/s → SDK 没有暴露临时收紧 y 速度的
#   runtime action,只能走默认 0.1 m/s。如果后续 runtime 加 'arm_set_y_speed_limit'
#   可以包成 try/finally 临时收紧。本脚本保持默认 0.1 m/s(走 150mm 约 1.5s,够柔和)。
# - x(horiz)默认 PID output_limits = 0.4 m/s,通过 move_x_position(target, out_time,
#   v_max_mms=100) 临时收紧到 0.1 m/s(走 100mm 约 1.0s)。
ARM_SPEED_MPS = 0.1


def _slow_arm_y(client, target_m: float, timeout: float) -> None:
    """y 步进电机移动。SDK 无速度参数 → 走 yaml 默认 0.1 m/s。

    业务目标 0.1 m/s 与 SDK 默认一致 → 直接调 move_y_position 即可。
    备选:后续可加 'arm_set_y_pid_limits' runtime action 临时收紧。
    """
    print(f"       (y 默认 0.1 m/s,SDK 默认就是 0.1 m/s,无需收紧)",
          flush=True)
    arm_call(client, "move_y_position", target_m, timeout=timeout)


def _slow_arm_x(client, target_m: float, timeout: float) -> None:
    """move_x_position 通过 v_max_mms 参数临时收紧 x 速度到 0.1 m/s。

    ARM_SPEED_MPS = 0.1 → v_max_mms = 100 mm/s。
    out_time 给 15s 兜底(走 100mm 大约 1.0s,但给余量)。
    """
    arm_call(client, "move_x_position", target_m, timeout=timeout,
             v_max_mms=ARM_SPEED_MPS * 1000.0, out_time=15.0)


def main():
    ap = argparse.ArgumentParser(description="arm sequence v9 "
                                              "(y1 中间位 → 调整手爪/大臂/x → y2 目标位)")
    ap.add_argument("--y1", type=float, default=-0.100,
                    help="step 1 中间 y 位 (m, 默认 -0.100 = -100mm),在此调整手爪/大臂/x")
    ap.add_argument("--y2", type=float, default=-0.040,
                    help="step 5 最终 y 位 (m, 默认 -0.040 = -40mm)")
    ap.add_argument("--x", type=float, default=-0.270,
                    help="step 4 x target (m, 默认 -0.270 = -270mm)")
    ap.add_argument("--arm-angle", type=float, default=90.0,
                    help="step 3 机械臂(大臂)角度 (deg, default +90)")
    ap.add_argument("--hand-angle", type=float, default=-70.0,
                    help="step 2 手爪舵机角度 (deg, default -70)")
    ap.add_argument("--angle-speed", type=int, default=100,
                    help="set_*_angle speed (default 100)")
    ap.add_argument("--speed", type=float, default=ARM_SPEED_MPS,
                    help=f"机械臂 y/x 移动速度 m/s (默认 {ARM_SPEED_MPS})")
    ap.add_argument("--settle", type=float, default=0.3, dest="settle",
                    help="settle delay after each step (s, default 0.3)")
    ap.add_argument("--timeout", type=float, default=30.0)
    args = ap.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()

    print("[seq] starting arm sequence v9", flush=True)
    b = read_arm(client)
    print(f"[before] y_m={b.get('y_m')} x_m={b.get('x_m')} "
          f"ref_encoder={b.get('ref_encoder')} "
          f"arm_angle={b.get('arm_angle')} "
          f"hand_angle={b.get('hand_angle')}", flush=True)

    # 1) y -> 中间位(如 -100mm;速度限制 0.1 m/s;y SDK 默认就是 0.1,见 _slow_arm_y)
    print(f"\n[1/5] move_y_position({args.y1}) 速度={args.speed} m/s ...",
          flush=True)
    _slow_arm_y(client, args.y1, timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    # 2) 手爪舵机角度(在 y1 中间位调整)
    print(f"\n[2/5] set_hand_angle({args.hand_angle}, {args.angle_speed}) ...",
          flush=True)
    arm_call(client, "set_hand_angle", args.hand_angle, args.angle_speed,
             timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> hand_angle={s.get('hand_angle')} "
          f"arm_angle={s.get('arm_angle')}", flush=True)

    # 3) 机械臂(大臂)角度(在 y1 中间位调整)
    print(f"\n[3/5] set_arm_angle({args.arm_angle}, {args.angle_speed}) ...",
          flush=True)
    arm_call(client, "set_arm_angle", args.arm_angle, args.angle_speed,
             timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> arm_angle={s.get('arm_angle')} "
          f"hand_angle={s.get('hand_angle')}", flush=True)

    # 4) x 位置(在 y1 中间位调整;速度限制 0.1 m/s)
    print(f"\n[4/5] move_x_position({args.x}) 速度={args.speed} m/s ...",
          flush=True)
    _slow_arm_x(client, args.x, timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    # 5) y -> 目标位(如 -40mm)
    print(f"\n[5/5] move_y_position({args.y2}) 速度={args.speed} m/s ...",
          flush=True)
    _slow_arm_y(client, args.y2, timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    print("\n[seq] done", flush=True)


if __name__ == "__main__":
    main()
