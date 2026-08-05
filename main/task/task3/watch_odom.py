#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/watch_odom.py - 持续打印 odom,辅助 axis 诊断

启动后每 0.2s 打印当前 x/y/yaw。
你手动推车 / 拉车,看 odom 怎么变 → 确定 axis 对应的"前"是哪个方向。

用法:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.watch_odom
    # 然后在物理上把车往视觉"前"推一下
    # 看到 odom 哪个轴增加 / 减少
"""
from __future__ import annotations

import math
import sys
import time

from main.api_client import RuntimeApiClient


def main():
    client = RuntimeApiClient()
    client.wait_until_ready()
    print("[watch] 持续读 odom,Ctrl+C 退出")
    print("[watch] 把车往视觉'前'推一下,看 odom 变化:")
    while True:
        try:
            odo = (client.get_runtime() or {}).get("runtime", {}).get("odometry") or [0, 0, 0]
            x, y, yaw = float(odo[0]), float(odo[1]), float(odo[2])
            print(f"  x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+.2f}°",
                  flush=True)
        except Exception as e:
            print(f"  [err] {e}", file=sys.stderr)
        time.sleep(0.2)


if __name__ == "__main__":
    sys.exit(main())
