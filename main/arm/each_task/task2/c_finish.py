#!/usr/bin/python3
"""task2 / step c —— 灌溉任务完成,收尾"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_c_finish(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [C] 灌溉完成收尾 ===")
    print("  [arm] go_home()")
    runner.go_home()
    print("  [底盘] 巡线到下一任务")
    # client._call_car("lane_dis_offset", 0.4, ...)
    print("=== [C] 完成 ===\n")
    return {"done": True, "next": "task3_shoot"}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_c_finish(client, runner)


if __name__ == "__main__":
    main()
