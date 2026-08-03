#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/arm_seq_v9.py - sequenced arm motion v9

动作序列(按用户列出顺序,每步独立,失败抛错停):
  1. arm.move_y_position(-0.150)        -> y = -150mm
  2. arm.move_x_position(-0.100)        -> x = -100mm
  3. arm.set_arm_angle(+90, speed)      -> 机械臂(大臂)角度 +90 度
  4. arm.set_hand_angle(-90, speed)     -> 手爪舵机角度 -90 度

每步后读 arm_state 打印。

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.arm_seq_v9
    python -m main.tasks.task333.arm_seq_v9 --y1 -0.150 --x -0.100 --arm-angle 90 --hand-angle -90
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
                                              "(y=-150 → x=-100 → arm=+90 → hand=-90)")
    ap.add_argument("--y1", type=float, default=-0.150,
                    help="step 1 y target (m, default -0.150 = -150mm)")
    ap.add_argument("--x", type=float, default=-0.100,
                    help="step 2 x target (m, default -0.100 = -100mm)")
    ap.add_argument("--arm-angle", type=float, default=90.0,
                    help="step 3 机械臂(大臂)角度 (deg, default +90)")
    ap.add_argument("--hand-angle", type=float, default=-90.0,
                    help="step 4 手爪舵机角度 (deg, default -90)")
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

    # 1) y -> -150mm(速度限制 0.1 m/s;y SDK 默认就是 0.1,见 _slow_arm_y)
    print(f"\n[1/4] move_y_position({args.y1}) 速度={args.speed} m/s ...",
          flush=True)
    _slow_arm_y(client, args.y1, timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    # 2) x -> -100mm(速度限制 0.1 m/s)
    print(f"\n[2/4] move_x_position({args.x}) 速度={args.speed} m/s ...",
          flush=True)
    _slow_arm_x(client, args.x, timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    # 3) 机械臂(大臂)角度 +90 度(set_arm_angle)
    print(f"\n[3/4] set_arm_angle({args.arm_angle}, {args.angle_speed}) ...",
          flush=True)
    arm_call(client, "set_arm_angle", args.arm_angle, args.angle_speed,
             timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> arm_angle={s.get('arm_angle')} "
          f"hand_angle={s.get('hand_angle')}", flush=True)

    # 4) 手爪舵机角度 -90 度(set_hand_angle)
    print(f"\n[4/4] set_hand_angle({args.hand_angle}, {args.angle_speed}) ...",
          flush=True)
    arm_call(client, "set_hand_angle", args.hand_angle, args.angle_speed,
             timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> hand_angle={s.get('hand_angle')} "
          f"arm_angle={s.get('arm_angle')}", flush=True)

    print("\n[seq] done", flush=True)


if __name__ == "__main__":
    main()