#!/usr/bin/python3
"""task7 / step a —— 到达配送区"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_a_approach(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [A] 到达配送区 ===")
    # 从 task6 终点巡线到配送区
    # client._call_car("lane_dis_offset", 0.4, ...)
    # 货物还在 arm 上吸着(从 task6 来的状态)
    print("  [note] 货物从 task6 继续带着")
    print("=== [A] 完成 ===\n")
    return {"arrived": True, "holding_goods": True}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_a_approach(client, runner)


if __name__ == "__main__":
    main()
