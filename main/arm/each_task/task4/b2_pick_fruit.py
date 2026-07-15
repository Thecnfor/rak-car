#!/usr/bin/python3
"""task4 / step b2 —— 吸取果实

果实是 4cm 球,放在任务模型上(模型有一定高度)。
model_height_mm: 任务模型的高度,需要先把 y 降到模型上,再吸取。
"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b2_pick_fruit(client: ArmClient, runner: ArmRunner,
                      fruit_x_mm: float, model_height_mm: float = -30.0) -> dict:
    print("=== [B2] 吸取果实 ===")
    print(f"  [arm]  set_hand(DOWN) + set_side(MID)")
    runner.set_side("MID", timeout=10)
    runner.set_hand("DOWN", timeout=10)
    print(f"  [arm]  move_xy(x={fruit_x_mm:.0f}, y={model_height_mm+5:.0f})")
    runner.move_xy(x_mm=fruit_x_mm, y_mm=model_height_mm - 5.0)
    print(f"  [arm]  move_y({model_height_mm:.0f})  下降到模型面")
    runner.move_y(y_mm=model_height_mm)
    time.sleep(0.3)
    print("  [arm]  grasp(True) 吸取")
    runner.grasp(True, timeout=10)
    print("  [arm]  move_y(80) 抬起")
    runner.move_y(y_mm=-80.0)
    print("=== [B2] 完成 ===\n")
    return {"picked": True}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b2_pick_fruit(client, runner, fruit_x_mm=60.0)


if __name__ == "__main__":
    main()
