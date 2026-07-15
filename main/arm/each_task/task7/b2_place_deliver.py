#!/usr/bin/python3
"""task7 / step b2 —— 把货物放到对应配送点

arm 吸着货物(从 task6 来的) → 走到配送点 → 放下到平板上
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b2_place_deliver(client: ArmClient, runner: ArmRunner,
                         house_x_mm: float, house_y_mm: float = -30.0) -> dict:
    print("=== [B2] 放下货物到配送点 ===")
    # 底盘走到配送点前方
    print(f"  [底盘] 走到配送点 x={house_x_mm:.0f}")
    # client._call_car("move_to_position", house_x_mm / 1000, 0, 0)
    # arm 移到配送点上方
    print(f"  [arm]  move_xy(x={house_x_mm:.0f}, y={house_y_mm+5:.0f})")
    runner.move_xy(x_mm=house_x_mm, y_mm=-house_y_mm - 5.0)
    print(f"  [arm]  move_y({-house_y_mm:.0f})  放到平板上")
    runner.move_y(y_mm=-house_y_mm)
    print("  [arm]  grasp(False) 释放货物")
    runner.grasp(False, timeout=10)
    print("  [arm]  move_y(80) 抬起脱离")
    runner.move_y(y_mm=-80.0)
    print("=== [B2] 完成 ===\n")
    return {"delivered": True}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b2_place_deliver(client, runner, house_x_mm=80.0)


if __name__ == "__main__":
    main()
