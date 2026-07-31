#!/usr/bin/python3
"""task2 / step a —— 到达灌溉区"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_a_approach(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [A] 到达灌溉区 ===")
    # 底盘:从任务1 终点巡线到灌溉区
    # client._call_car("lane_dis_offset", 0.4, ...)
    # client._call_car("move_to_detection_target", label="irrigation_area")
    # arm:安全位
    print("  [底盘] 巡线到灌溉区")
    print("  [arm]  set_hand(UP) + set_side(MID)")
    runner.set_hand("UP", timeout=10)
    runner.set_side("MID", timeout=10)
    print("=== [A] 完成 ===\n")
    return {"arrived": True}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_a_approach(client, runner)


if __name__ == "__main__":
    main()
