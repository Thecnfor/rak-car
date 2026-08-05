#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/test_yaw_direction.py - 原地 yaw 后看 cam 视野怎么变

用法:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.test_yaw_direction
    # 脚本会 yaw +15° / -15° / +30° / -30°,每次都打印 cam 视野里 4 只板子的 xc
    # 这样可以确定:
    # - cam 视野左/右 对应车体左/右 还是反过来
    # - yaw 正方向 让 cam 视野朝哪边偏
"""
from __future__ import annotations

import math
import sys
import time

from main.api_client import RuntimeApiClient


def car_call(client, name, *args, timeout=10.0, **kwargs):
    job = client.execute_car_action(name, *args, timeout=timeout,
                                    sync=True, **kwargs)
    if job.get("status") != "succeeded":
        raise RuntimeError(f"car.{name} failed: {job.get('error')}")
    return job.get("result")


def read_odom(client):
    try:
        odo = (client.get_runtime() or {}).get("runtime", {}).get("odometry") or [0, 0, 0]
        return float(odo[0]), float(odo[1]), float(odo[2])
    except Exception:
        return None


def get_animals(client, min_score=0.3):
    """读 cam 视野里所有动物 detection 的 xc。"""
    try:
        # 用 task_state 接口
        ts = client.get_task_state() or {}
        dets = ts.get("task_state", {}).get("detections", []) or []
        out = []
        for d in dets:
            if d.get("label") == "animal" and d.get("score", 0.0) >= min_score:
                bbox = d.get("bbox_norm") or {}
                out.append({
                    "xc": float(bbox.get("x_center", 0.0)),
                    "yc": float(bbox.get("y_center", 0.5)),
                    "score": float(d.get("score", 0.0)),
                })
        return out
    except Exception as e:
        print(f"  [get_animals err] {e}", file=sys.stderr)
        return []


def main():
    client = RuntimeApiClient()
    client.wait_until_ready()

    # 不 reset odom — 但要先归位(如果有漂移)
    print("[yaw-test] 读初始 cam 视野...")
    init_animals = get_animals(client)
    init_animals_sorted = sorted(init_animals, key=lambda a: a["xc"])
    print(f"  初始 4 只(按 xc 左→右):")
    for i, a in enumerate(init_animals_sorted):
        print(f"    #{i+1}: xc={a['xc']:+.2f} score={a['score']:.2f}")
    odo_init = read_odom(client)
    print(f"  初始 odom: x={odo_init[0]:+.3f} y={odo_init[1]:+.3f} "
          f"yaw={math.degrees(odo_init[2]):+.2f}°")

    # 测试 yaw
    yaw_steps = [0, +15, +30, -15, -30, 0]   # 度数(回到 0)
    yaw_total_sent = 0.0

    for tgt_deg in yaw_steps:
        delta = tgt_deg - yaw_total_sent
        if abs(delta) < 0.5:
            continue
        print(f"\n[test yaw={tgt_deg:+.0f}°] 发 yaw {delta:+.2f}°...")
        try:
            car_call(client, "move_for",
                     [0.0, 0.0, math.radians(delta)], timeout=10)
        except Exception as e:
            print(f"  err: {e}", file=sys.stderr)
            continue
        yaw_total_sent = tgt_deg
        time.sleep(1.0)

        # 读 cam 视野
        animals = get_animals(client)
        odo = read_odom(client)
        if odo:
            print(f"  odom: x={odo[0]:+.3f} y={odo[1]:+.3f} "
                  f"yaw={math.degrees(odo[2]):+.2f}°")
        if animals:
            animals_sorted = sorted(animals, key=lambda a: a["xc"])
            xcs = [f"{a['xc']:+.2f}" for a in animals_sorted]
            print(f"  cam 视野 xc 左→右: {xcs}")
        else:
            print(f"  cam 视野: 空")
        time.sleep(0.5)

    print(f"\n[done] yaw 测试完,告诉 Claude:")
    print(f"  - yaw +15° 时,#1 xc 是增大还是减小?(即正 yaw 让 cam 视野朝哪边偏)")
    print(f"  - yaw -15° 时,#1 xc 怎么变?")


if __name__ == "__main__":
    sys.exit(main() or 0)