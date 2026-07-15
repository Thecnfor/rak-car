#!/usr/bin/python3
"""task6 / step c —— 接单完成,货物暂存底盘上,准备 task7 配送"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_c_finish(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [C] 接单完成,准备配送 ===")
    # 货物先临时放在一个安全位(臂中,吸着,等 task7)
    print("  [arm]  set_hand(UP) + set_side(MID) 货物暂存")
    runner.set_hand("UP", timeout=10)
    runner.set_side("MID", timeout=10)
    runner.move_y(y_mm=60.0)  # 抬到一个安全高度
    print("  [note] 货物吸着进入 task7")
    print("=== [C] 完成 ===\n")
    return {"done": True, "next": "task7_deliver"}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_c_finish(client, runner)


if __name__ == "__main__":
    main()
