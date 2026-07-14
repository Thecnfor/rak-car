"""main/misc/drive_and_shoot.py

边巡线边周期性射击。

目的：
- 验证底盘动作不会阻塞射击调用
- 验证 shoot 调用频次稳定（每 N 米一发）
- 给真实比赛流程（target_shooting）做一个最小可跑的 smoke test

模型：
- 巡线跑到总距离 dis_total
- 期间每 step_dis 米打一发
- 终点后再补一发

注意：
- 不做视觉对齐，只验证"边走边打"的链路
- 真实比赛里视觉对齐后再打，这里只用 lane_dis_offset 跑距离

跑前准备：
- 车放在有赛道线的地面，前摄能看到车道线
- 弹药装好

参数：
- speed: 巡线速度 m/s（默认 0.25）
- dis_total: 总行程米数（默认 1.5）
- step_dis: 每隔多少米打一发（默认 0.5）
- shot_timeout: 单发 shooting 的 HTTP timeout（默认 5s）

跑法：
    python3 -m main.misc.drive_and_shoot
    python3 -m main.misc.drive_and_shoot --speed 0.3 --dis-total 2.0 --step-dis 0.5
"""
from __future__ import annotations

import argparse
import math
import time

from main.api_client import RuntimeApiClient


def main():
    parser = argparse.ArgumentParser(description="边巡线边周期性射击")
    parser.add_argument("--speed", type=float, default=0.25, help="巡线速度 m/s（默认 0.25）")
    parser.add_argument("--dis-total", type=float, default=1.5, dest="dis_total", help="总行程米数（默认 1.5）")
    parser.add_argument("--step-dis", type=float, default=0.5, dest="step_dis", help="每隔多少米打一发（默认 0.5）")
    parser.add_argument("--shot-timeout", type=float, default=5.0, dest="shot_timeout", help="单发 shooting 的 HTTP timeout 秒数（默认 5）")
    parser.add_argument("--lane-timeout", type=float, default=120.0, dest="lane_timeout", help="单次 lane_dis_offset 的 HTTP timeout 秒数（默认 120）")
    args = parser.parse_args()

    if args.step_dis <= 0:
        raise ValueError("--step-dis 必须 > 0")
    if args.dis_total <= 0:
        raise ValueError("--dis-total 必须 > 0")

    # 预计打几发：起点一发 + 中间每 step_dis 一发 + 终点一发
    n_shots = max(1, int(math.ceil(args.dis_total / args.step_dis)) + 1)
    print(
        f"[plan] speed={args.speed} dis_total={args.dis_total} step_dis={args.step_dis} "
        f"预计 {n_shots} 发"
    )

    client = RuntimeApiClient()
    client.wait_until_ready()

    start_distance = client.call("car", "get_distance", timeout=5)["result"]
    print(f"[start] distance={start_distance:.3f}m")

    shot_count = 0

    # 起点先来一发（让弹道先稳定）
    client.call("car", "shooting", timeout=args.shot_timeout)
    shot_count += 1
    print(f"[shot {shot_count}] 起点")

    remaining = args.dis_total
    while remaining > 1e-3:
        step = min(args.step_dis, remaining)
        client.call("car", "lane_dis_offset", timeout=args.lane_timeout, speed=args.speed, dis_hold=step)

        now_distance = client.call("car", "get_distance", timeout=5)["result"]
        print(f"[run] 已累计 {now_distance - start_distance:.3f}m")

        remaining -= step
        if remaining > 1e-3:
            client.call("car", "shooting", timeout=args.shot_timeout)
            shot_count += 1
            print(f"[shot {shot_count}] 中间一发")

    # 终点再补一发
    client.call("car", "shooting", timeout=args.shot_timeout)
    shot_count += 1
    print(f"[shot {shot_count}] 终点")

    end_distance = client.call("car", "get_distance", timeout=5)["result"]
    print(
        f"[done] 共 {shot_count} 发，总行程 {end_distance - start_distance:.3f}m"
    )


if __name__ == "__main__":
    main()
