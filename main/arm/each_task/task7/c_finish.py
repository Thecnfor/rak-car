#!/usr/bin/python3
"""task7 / step c —— 配送完成收尾(8 个任务全跑完)"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_c_finish(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [C] 配送完成收尾(8 任务完结) ===")
    print("  [arm] go_home()")
    runner.go_home()
    # 任务3/8 是底盘任务,这里不展开
    print("  [底盘] 巡线到任务3 射击区(task8 巡逻可选)")
    # client._call_car("lane_dis_offset", 0.4, ...)
    print("=== [C] 任务7 完成 ===\n")
    print("\n*** 至此 8 任务全部结束 ***\n")
    return {"done": True, "next": "task3_shoot"}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_c_finish(client, runner)


if __name__ == "__main__":
    main()
