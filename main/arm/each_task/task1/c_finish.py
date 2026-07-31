#!/usr/bin/python3
"""task1 / step c —— 全部完成收尾

动作:
  - 机械臂回原点(go_home): y=0, x=0, hand=UP, side=MID
  - 底盘巡线到下一任务区(任务2 智能灌溉)

依赖:3 个种子都已放好
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_c_finish(client: ArmClient, runner: ArmRunner) -> dict:
    """C:收尾。"""
    print("=== [C] 全部完成收尾 ===")

    # ---- 机械臂归位 ----
    print("  [arm] go_home()  y=0, x=0, hand=UP, side=MID")
    runner.go_home()

    # ---- 底盘去下一任务 ----
    print("  [底盘] 巡线到任务2 智能灌溉区")
    # client._call_car("lane_dis_offset", speed=0.4, dis_hold=...)

    print("=== [C] 完成 ===\n")
    return {"done": True, "next_task": "task2_irrigation"}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_c_finish(client, runner)


if __name__ == "__main__":
    main()
