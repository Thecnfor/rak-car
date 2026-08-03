#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/arm_sequence.py - sequenced arm motion

动作序列(每步独立,失败抛错停):
  1. arm.move_y_position(-0.100)   -> y = -100mm (-0.1m)
  2. arm.set_arm_pose(UP)          -> 大臂抬 UP(摄像头抬起)
  3. arm.move_x_position(-0.100)   -> x = -100mm (-0.1m)
  4. arm.move_y_position(0.0)      -> y 回到 0

每步后读 arm_state 打印位置。

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.arm_sequence
"""
from __future__ import annotations

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
    client = RuntimeApiClient()
    client.wait_until_ready()

    print("[seq] starting arm sequence", flush=True)
    b = read_arm(client)
    print(f"[before] y_m={b.get('y_m')} x_m={b.get('x_m')} "
          f"ref_encoder={b.get('ref_encoder')}", flush=True)

    # 1) y -> -0.100 (-100mm)
    print("[1/4] move_y_position(-0.100) ...", flush=True)
    arm_call(client, "move_y_position", -0.100, timeout=30)
    time.sleep(0.3)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    # 2) 大臂抬 UP(摄像头抬起)
    print("[2/4] set_arm_pose(x=-0.100, y=-0.100, arm='UP', hand='UP') ...",
          flush=True)
    try:
        arm_call(client, "set_arm_pose", -0.100, -0.100, "UP", "UP", timeout=30)
    except Exception as exc:
        print(f"[warn] set_arm_pose failed: {exc}", flush=True)
        print("       fallback: set_arm_angle(30, 100) ...", flush=True)
        try:
            arm_call(client, "set_arm_angle", 30, 100, timeout=30)
        except Exception as e2:
            print(f"[err] set_arm_angle failed: {e2}", flush=True)

    time.sleep(0.3)
    s = read_arm(client)
    print(f"       -> arm pose applied (y_m={s.get('y_m')})", flush=True)

    # 3) x -> -0.100
    print("[3/4] move_x_position(-0.100) ...", flush=True)
    arm_call(client, "move_x_position", -0.100, timeout=30)
    time.sleep(0.3)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    # 4) y -> 0
    print("[4/4] move_y_position(0.0) ...", flush=True)
    arm_call(client, "move_y_position", 0.0, timeout=30)
    time.sleep(0.3)
    s = read_arm(client)
    print(f"       -> y_m={s.get('y_m')} x_m={s.get('x_m')}", flush=True)

    print("[seq] done", flush=True)


if __name__ == "__main__":
    main()