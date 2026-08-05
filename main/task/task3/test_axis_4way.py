#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/test_axis_4way.py - 4 个 axis 各走 5cm,确定 cam 视野前方对应哪个 offset

用法:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.test_axis_4way
    # 脚本会按顺序发 [+0.05,0,0] / [-0.05,0,0] / [0,+0.05,0] / [0,-0.05,0] 各一次
    # 每步后等 1s,打印 odom
    # 用户手动把车头方向跟 cam 视野方向对齐,观察哪个 axis 让车朝 cam 视野前方走
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


def main():
    client = RuntimeApiClient()
    client.wait_until_ready()

    # reset odom
    try:
        car_call(client, "reset_position", timeout=5)
        print("[reset] odom 已清零", flush=True)
        time.sleep(0.5)
    except Exception as e:
        print(f"[reset err] {e}", file=sys.stderr)

    # 4 个 axis 测试
    axes = [
        ("+x(车头方向)", [+0.05, 0.0, 0.0]),
        ("-x(车尾方向)", [-0.05, 0.0, 0.0]),
        ("+y(车体左 / 板子方向?)", [0.0, +0.05, 0.0]),
        ("-y(车体右)", [0.0, -0.05, 0.0]),
    ]

    for label, offset in axes:
        # 读起点
        odo_before = read_odom(client)
        print(f"\n[{label}] offset={offset} "
              f"before=({odo_before[0]:+.3f},{odo_before[1]:+.3f})" if odo_before else "",
              flush=True)
        try:
            car_call(client, "move_for", offset, timeout=5)
        except Exception as e:
            print(f"  err: {e}", file=sys.stderr)
            continue
        time.sleep(1.0)
        odo_after = read_odom(client)
        if odo_after:
            dx = odo_after[0] - odo_before[0]
            dy = odo_after[1] - odo_before[1]
            print(f"  after=({odo_after[0]:+.3f},{odo_after[1]:+.3f}) "
                  f"Δx={dx*100:+.1f}cm Δy={dy*100:+.1f}cm",
                  flush=True)
        time.sleep(0.5)

    print(f"\n[done] 4 个 axis 测试完,告诉 Claude 哪个是 cam 视野前方", flush=True)


if __name__ == "__main__":
    sys.exit(main() or 0)