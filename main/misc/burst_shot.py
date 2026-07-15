"""main/misc/burst_shot.py

连发测试。

目的：
- 验证连发场景下收尾拉低是否稳定
- 验证每次触发间隔不会因为 sleep 被截断
- 验证数字口不会因为频繁 set/reset 出现粘连

注意：
- 这是纯电信号测试，不一定每次都成功击发（取决于弹药装填/弹道）
- 跟比赛任务里的"射击除害"区别：这里不巡线、不视觉对齐
- 内部 sleep 已经由 car.shooting 处理，业务层只需要控制调用间隔

跑前准备：
- 让小车静止停好
- 弹药装不装都行（如果要真打，注意安全距离）

参数：
- count: 射击次数（默认 5）
- interval: 两次 shooting 之间的额外 sleep（默认 0.3s；car.shooting 内部约 500ms，所以真实间隔 ≈ interval + 0.5）
- timeout: 单次 shooting 的 HTTP timeout（默认 8s）

跑法：
    python3 -m main.misc.burst_shot
    python3 -m main.misc.burst_shot --count 10 --interval 0.0
"""
from __future__ import annotations

import argparse
import time

from main.api_client import RuntimeApiClient


def main():
    parser = argparse.ArgumentParser(description="连发测试")
    parser.add_argument("--count", type=int, default=5, help="射击次数（默认 5）")
    parser.add_argument("--interval", type=float, default=0.3, help="两次 shooting 之间的额外 sleep 秒数（默认 0.3）")
    parser.add_argument("--timeout", type=float, default=8.0, help="单次 shooting 的 HTTP timeout 秒数（默认 8）")
    args = parser.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()
    print(f"[ready] count={args.count} interval={args.interval}s timeout={args.timeout}s")

    for i in range(args.count):
        t0 = time.time()
        client.call("car", "shooting", timeout=args.timeout)
        dt = time.time() - t0
        print(f"[shot {i + 1}/{args.count}] ok in {dt:.2f}s")
        if i < args.count - 1 and args.interval > 0:
            time.sleep(args.interval)

    print("[done] 全部完成")


if __name__ == "__main__":
    main()
