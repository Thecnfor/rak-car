#!/usr/bin/python3
"""task2 / step b3 —— 把水块放到水塔平板上

水塔平板有一定高度,所以要先把水塔定位,然后 arm 把水块放上去。
得分:水块在水塔中间平板上方。
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b3_place_water(client: ArmClient, runner: ArmRunner,
                        tower_x_mm: float, tower_y_mm: float = -60.0) -> dict:
    """tower_y_mm 是水塔平板高度(假设 60mm),不是 0。"""
    print("=== [B3] 放到水塔 ===")
    # 底盘走到水塔前方
    print(f"  [底盘] 走到水塔 x={tower_x_mm:.0f}mm")
    # client._call_car("move_to_position", tower_x_mm / 1000, 0, 0)
    # arm 移到水塔正上方
    print(f"  [arm]  move_xy(x={tower_x_mm:.0f}, y={tower_y_mm:.0f})")
    runner.move_xy(x_mm=tower_x_mm, y_mm=tower_y_mm)
    # 放下(放到平板上,不需要 y 触底)
    print("  [arm]  grasp(False) 释放水块")
    runner.grasp(False, timeout=10)
    # 抬起脱离
    print(f"  [arm]  move_y(80) 抬起来")
    runner.move_y(y_mm=-80.0)
    print("=== [B3] 完成 ===\n")
    return {"placed": True, "at": (tower_x_mm, tower_y_mm)}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b3_place_water(client, runner, tower_x_mm=80.0)


if __name__ == "__main__":
    main()
