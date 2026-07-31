#!/usr/bin/python3
"""task1 / step b3 —— 机械臂移到种子正上方

动作:
  - move_xy(seed.x_mm, 5.0)
    x = 种子水平位置(由 B1 视觉给出)
    y = 5mm(几乎触底,但还没吸住)

依赖:B1 给了 seed.x_mm;x 轴电机硬件要正常
⚠️ 当前 x 轴电机(id=6)不响应,这一步硬件会卡
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b3_arm_to_seed(client: ArmClient, runner: ArmRunner,
                       seed_x_mm: float, hover_y_mm: float = -5.0) -> dict:
    """B3:移到种子上方。"""
    print("=== [B3] 机械臂移到种子正上方 ===")
    print(f"  [arm] move_xy(x={seed_x_mm:.1f}mm, y={hover_y_mm:.1f}mm)")
    runner.move_xy(x_mm=seed_x_mm, y_mm=hover_y_mm)
    print("=== [B3] 完成 ===\n")
    return {"moved_to": (seed_x_mm, hover_y_mm)}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    # 演示:把 x=120 改成 B1 实际给的数
    step_b3_arm_to_seed(client, runner, seed_x_mm=120.0)


if __name__ == "__main__":
    main()
