#!/usr/bin/python3
"""task1 / step b4 —— 下降 + 真空吸取

动作:
  - move_y(0)        y 触底,吸盘贴紧种子
  - 短延迟等真空建立
  - grasp(True)      开真空泵

依赖:B3 已把吸盘移到种子上方
⚠️ grasp() 当前 api.py:234 有 bug,会抛 TypeError,需要修
"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b4_pick(client: ArmClient, runner: ArmRunner,
                ground_y_mm: float = 0.0, settle_s: float = 0.5) -> dict:
    """B4:下降 + 吸取。"""
    print("=== [B4] 下降 + 吸取 ===")
    print(f"  [arm] move_y({ground_y_mm:.1f}mm)  y 触底")
    runner.move_y(y_mm=ground_y_mm)
    print(f"  [延时] 等待 {settle_s}s,真空建立")
    time.sleep(settle_s)
    print("  [arm] grasp(True)  开真空泵")
    runner.grasp(True, timeout=10)
    print("=== [B4] 完成 ===\n")
    return {"picked": True}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b4_pick(client, runner)


if __name__ == "__main__":
    main()
