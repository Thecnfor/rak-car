#!/usr/bin/python3
"""task4 / step c —— 采收完成收尾"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_c_finish(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [C] 采收完成收尾 ===")
    print("  [arm] go_home()")
    runner.go_home()
    print("  [底盘] 巡线到分拣区")
    # client._call_car("lane_dis_offset", 0.4, ...)
    print("=== [C] 完成 ===\n")
    return {"done": True, "next": "task5_sort"}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_c_finish(client, runner)


if __name__ == "__main__":
    main()
