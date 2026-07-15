#!/usr/bin/python3
"""task2 / step b2 —— 吸取水块(从水块堆)

水块是 5cm 正方体,堆在起点区。底盘开到水块堆,arm 吸起 1 块。
"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b2_pick_water(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [B2] 吸取水块 ===")
    # 底盘开到水块堆
    print("  [底盘] 走到水块堆起点")
    # client._call_car("move_to_position", 0.0, 0.0, 0.0)
    # arm 准备
    print("  [arm]  set_hand(DOWN) + set_side(MID)")
    runner.set_side("MID", timeout=10)
    runner.set_hand("DOWN", timeout=10)
    # 移到水块上方
    print("  [arm]  move_xy(x=60, y=5)")
    runner.move_xy(x_mm=60.0, y_mm=-5.0)
    # 下降 + 吸取
    print("  [arm]  move_y(0) + grasp(True)")
    runner.move_y(y_mm=0.0)
    time.sleep(0.3)
    runner.grasp(True, timeout=10)
    # 抬起
    print("  [arm]  move_y(80) 抬起")
    runner.move_y(y_mm=-80.0)
    print("=== [B2] 完成 ===\n")
    return {"picked": True}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b2_pick_water(client, runner)


if __name__ == "__main__":
    main()
