#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/arm_seq_v6.py - sequenced arm motion v6

动作序列(每步独立,失败抛错停):
  1. arm.move_y_position(-0.150)        -> y = -150mm
  2. arm.set_hand_angle(-90, speed)     -> 手爪舵机角度 -90 度
  3. arm.move_x_position(-0.270)        -> x = -270mm
     + arm.set_arm_angle(90, speed)      -> 机械臂角度 90 度
  4. arm.move_y_position(-0.020)        -> y = -20mm

每步后读 arm_state 打印。

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.arm_seq_v6
    python -m main.tasks.task333.arm_seq_v6 --hand-angle -90 --y2 -0.020
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


def main():
    ap = argparse.ArgumentParser(description="arm sequence v6 "
                                              "(y=-150 → hand=-90 → x=-270+arm=90 → y=-20)")
    ap.add_argument("--y1", type=float, default=-0.150,
                    help="step 1 y target (m, default -0.150 = -150mm)")
    ap.add_argument("--hand-angle", type=float, default=-90.0,
                    help="step 2 hand servo angle (deg, default -90)")
    ap.add_argument("--x", type=float, default=-0.270,
                    help="step 3 x target (m, default -0.270 = -270mm)")
    ap.add_argument("--arm-angle", type=float, default=90.0,
                    help="step 3 arm angle (deg, default 90)")
    ap.add_argument("--y2", type=float, default=-0.020,
                    help="step 4 y target (m, default -0.020 = -20mm)")
    ap.add_argument("--angle-speed", type=int, default=100,
                    help="set_*_angle speed (default 100)")
    ap.add_argument("--settle", type=float, default=0.3, dest="settle",
                    help="settle delay after each step (s, default 0.3)")
    ap.add_argument("--timeout", type=float, default=30.0)
    args = ap.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()

    print("[seq] starting arm sequence v6", flush=True)
    b = read_arm(client)
    print(f"[before] y_m={b.get('y_m')} x_m={b.get('x_m')} "
          f"ref_encoder={b.get('ref_encoder')} "
          f"arm_angle={b.get('arm_angle')} "
          f"hand_angle={b.get('hand_angle')}", flush=True)

    # 1) y -> -150mm
    print(f"\n[1/4] move_y_position({args.y1}) ...", flush=True)
    arm_call(client, "move_y_position", args.y1, timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    # 2) 手爪舵机角度 -90 度(set_hand_angle)
    print(f"\n[2/4] set_hand_angle({args.hand_angle}, {args.angle_speed}) ...",
          flush=True)
    arm_call(client, "set_hand_angle", args.hand_angle, args.angle_speed,
             timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> hand_angle={s.get('hand_angle')} "
          f"arm_angle={s.get('arm_angle')}", flush=True)

    # 3) x -> -270mm + 机械臂角度 90 度
    print(f"\n[3/4] move_x_position({args.x}) + set_arm_angle({args.arm_angle}, "
          f"{args.angle_speed}) ...", flush=True)
    arm_call(client, "move_x_position", args.x, timeout=args.timeout)
    arm_call(client, "set_arm_angle", args.arm_angle, args.angle_speed,
             timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')} "
          f"arm_angle={s.get('arm_angle')}", flush=True)

    # 4) y -> -20mm
    print(f"\n[4/4] move_y_position({args.y2}) ...", flush=True)
    arm_call(client, "move_y_position", args.y2, timeout=args.timeout)
    time.sleep(args.settle)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    print("\n[seq] done", flush=True)


if __name__ == "__main__":
    main()