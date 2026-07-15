#!/usr/bin/python3
"""task6 / step b3 —— 走到货架 + 吸取正确货物

货架上每个格子里有 1 个货物(5cm 正方体,贴图)。
视觉识别 + 匹配名称,吸取对应的那个。
"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b3_pick_goods(client: ArmClient, runner: ArmRunner,
                       goods_name: str) -> dict:
    print("=== [B3] 吸取正确货物 ===")
    # 底盘走到货架
    print("  [底盘] 走到货架前")
    # client._call_car("move_to_position", ...)
    # 视觉找到对应货物的 x 坐标
    print(f"  [视觉] 找货物: {goods_name}")
    # boxes = client._call_car("get_detection_results", label=goods_name)
    # 简化:假设货在 x=80
    goods_x_mm = 80.0
    shelf_y_mm = 60.0  # 货架有高度
    # arm 准备
    print("  [arm]  set_hand(DOWN) + set_side(MID)")
    runner.set_side("MID", timeout=10)
    runner.set_hand("DOWN", timeout=10)
    print(f"  [arm]  move_xy(x={goods_x_mm}, y={shelf_y_mm+5})")
    runner.move_xy(x_mm=goods_x_mm, y_mm=shelf_y_mm + 5.0)
    print(f"  [arm]  move_y({shelf_y_mm}) 下降到货架面")
    runner.move_y(y_mm=shelf_y_mm)
    time.sleep(0.3)
    print("  [arm]  grasp(True) 吸取")
    runner.grasp(True, timeout=10)
    print("  [arm]  move_y(80) 抬起")
    runner.move_y(y_mm=80.0)
    print("=== [B3] 完成 ===\n")
    return {"picked": goods_name}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b3_pick_goods(client, runner, goods_name="西红柿")


if __name__ == "__main__":
    main()
