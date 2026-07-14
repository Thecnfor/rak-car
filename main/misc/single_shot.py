"""main/misc/single_shot.py

单发点射测试。

目的：
- 验证 PoutD(4) 数字口硬件通路
- 验证 car.shooting 走 /v1/execute 路径完整
- 验证收尾拉低（finally 段）正常

硬件依赖：
- 控制器已上电，PoutD(4) 接枪口继电器
- 蜂鸣器接好（用于人耳确认触发）

跑前准备：
- 让小车静止停好，远离人和易碎物
- 不需要对准目标，纯电信号测试
- 弹药可不装

参数：
- count: 连发次数（默认 3，每次间隔 1.0s）
- interval: 两次射击间隔秒数（默认 1.0）
- timeout: 单次 shooting 的 HTTP timeout（默认 5s，足够覆盖内置 500ms）

跑法：
    python3 -m main.misc.single_shot
    python3 -m main.misc.single_shot --count 5 --interval 0.5
"""
from __future__ import annotations

import argparse
import time

from main.api_client import RuntimeApiClient


def main():
    parser = argparse.ArgumentParser(description="单发点射测试")
    parser.add_argument("--count", type=int, default=3, help="连发次数（默认 3）")
    parser.add_argument("--interval", type=float, default=1.0, help="两次射击间隔秒数（默认 1.0）")
    parser.add_argument("--timeout", type=float, default=5.0, help="单次 shooting 的 HTTP timeout 秒数（默认 5）")
    args = parser.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()
    print(f"[ready] count={args.count} interval={args.interval}s timeout={args.timeout}s")

    client.call("car", "beep", timeout=args.timeout)
    print("[beep] 起始提示")

    for i in range(args.count):
        t0 = time.time()
        client.call("car", "shooting", timeout=args.timeout)
        dt = time.time() - t0
        print(f"[shot {i + 1}/{args.count}] ok in {dt:.2f}s")
        if i < args.count - 1:
            time.sleep(args.interval)

    client.call("car", "beep", timeout=args.timeout)
    print("[beep] 完成提示，全部 done")


if __name__ == "__main__":
    main()
