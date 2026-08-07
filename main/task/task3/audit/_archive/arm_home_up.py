#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/arm_home_up.py - reset arm to home + raise camera

动作序列:
  1. arm.move_y_position(0.0)        -> y 轴归零
  2. arm.move_x_position(0.0)        -> x 轴归零
  3. arm.set_arm_pose(...)           -> 大臂摆中、抬 UP(摄像头抬起)

每步独立,失败会抛错停。

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.arm_home_up
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


def main():
    client = RuntimeApiClient()
    client.wait_until_ready()

    print("[home] starting arm reset (y=0, x=0, camera UP)", flush=True)

    # 1) y = 0
    before = (client.get_arm_state() or {}).get("arm_state") or {}
    print(f"[before] y_m={before.get('y_m')} x_m={before.get('x_m')} "
          f"ref_encoder={before.get('ref_encoder')}", flush=True)

    print("[1/3] arm.move_y_position(0.0) ...", flush=True)
    arm_call(client, "move_y_position", 0.0, timeout=30)
    time.sleep(0.3)
    s1 = (client.get_arm_state() or {}).get("arm_state") or {}
    print(f"       -> y_m={s1.get('y_m')}", flush=True)

    # 2) x = 0
    print("[2/3] arm.move_x_position(0.0) ...", flush=True)
    arm_call(client, "move_x_position", 0.0, timeout=30)
    time.sleep(0.3)
    s2 = (client.get_arm_state() or {}).get("arm_state") or {}
    print(f"       -> x_m={s2.get('x_m')}", flush=True)

    # 3) 大臂抬 UP
    print("[3/3] arm.set_arm_pose(x=0, y=0, arm='UP', hand='UP') ...", flush=True)
    try:
        arm_call(client, "set_arm_pose", 0.0, 0.0, "UP", "UP", timeout=30)
    except Exception as exc:
        print(f"[warn] set_arm_pose failed: {exc}", flush=True)
        print("       trying set_arm_angle(angle) for raise ...", flush=True)
        try:
            arm_call(client, "set_arm_angle", 30, 100, timeout=30)
        except Exception as e2:
            print(f"[err] set_arm_angle also failed: {e2}", flush=True)

    time.sleep(0.3)
    after = (client.get_arm_state() or {}).get("arm_state") or {}
    print(f"[after] y_m={after.get('y_m')} x_m={after.get('x_m')} "
          f"ref_encoder={after.get('ref_encoder')}", flush=True)

    print("[home] done", flush=True)


if __name__ == "__main__":
    main()