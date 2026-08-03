#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/arm_seq_v2.py - sequenced arm motion v2

动作序列(每步独立,失败抛错停):
  1. arm.move_y_position(-0.100)       -> y = -100mm
  2. arm.set_arm_pose(UP)              -> 大臂抬 UP(摄像头抬起)
  3. arm.move_x_position(-0.100)       -> x = -100mm
     + arm.set_arm_angle(0, speed)      -> 角度归 0 度
  4. arm.move_y_position(0.0)          -> y 回到 0

每步后读 arm_state 打印位置。

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.arm_seq_v2
    python -m main.tasks.task333.arm_seq_v2 --raise-angle 30 --final-angle 0
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


def try_arm_pose_up(client, x, y, timeout=30):
    """尝试 set_arm_pose UP;失败 fallback 到 set_arm_angle(raise_angle)。"""
    try:
        arm_call(client, "set_arm_pose", x, y, "UP", "UP", timeout=timeout)
        return "set_arm_pose(UP)"
    except Exception as exc:
        print(f"[warn] set_arm_pose failed: {exc}", flush=True)
        # 用 raise_angle 代替
        return None


def main():
    ap = argparse.ArgumentParser(description="arm sequence v2")
    ap.add_argument("--y-start", type=float, default=-0.100,
                    help="step 1 target y (m, default -0.100 = -100mm)")
    ap.add_argument("--x-target", type=float, default=-0.100,
                    help="step 3 target x (m, default -0.100 = -100mm)")
    ap.add_argument("--y-final", type=float, default=0.0,
                    help="step 4 final y (m, default 0.0)")
    ap.add_argument("--raise-angle", type=float, default=30.0,
                    help="angle to raise camera (deg, default 30)")
    ap.add_argument("--final-angle", type=float, default=0.0,
                    help="step 3 angle to set (deg, default 0)")
    ap.add_argument("--angle-speed", type=int, default=100,
                    help="set_arm_angle speed (default 100)")
    ap.add_argument("--timeout", type=float, default=30.0)
    args = ap.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()

    print("[seq] starting arm sequence v2", flush=True)
    b = read_arm(client)
    print(f"[before] y_m={b.get('y_m')} x_m={b.get('x_m')} "
          f"ref_encoder={b.get('ref_encoder')}", flush=True)

    # 1) y -> -0.100 (-100mm)
    print(f"\n[1/5] move_y_position({args.y_start}) ...", flush=True)
    arm_call(client, "move_y_position", args.y_start, timeout=args.timeout)
    time.sleep(0.3)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    # 2) 大臂抬 UP(摄像头抬起)
    print(f"\n[2/5] raise camera: set_arm_pose(UP) ...", flush=True)
    used = try_arm_pose_up(client, args.y_start, args.y_start, timeout=args.timeout)
    if used is None:
        print(f"       fallback: set_arm_angle({args.raise_angle}, {args.angle_speed}) ...",
              flush=True)
        try:
            arm_call(client, "set_arm_angle", args.raise_angle, args.angle_speed,
                     timeout=args.timeout)
        except Exception as e:
            print(f"[err] set_arm_angle failed: {e}", flush=True)
    time.sleep(0.3)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    # 3) x -> -0.100 + 角度归 0
    print(f"\n[3/5] move_x_position({args.x_target}) + set_arm_angle(0) ...",
          flush=True)
    arm_call(client, "move_x_position", args.x_target, timeout=args.timeout)
    arm_call(client, "set_arm_angle", args.final_angle, args.angle_speed,
             timeout=args.timeout)
    time.sleep(0.3)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    # 4) y -> 0
    print(f"\n[4/5] move_y_position({args.y_final}) ...", flush=True)
    arm_call(client, "move_y_position", args.y_final, timeout=args.timeout)
    time.sleep(0.3)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    print("\n[seq] done", flush=True)


if __name__ == "__main__":
    main()