#!/usr/bin/python3
"""task5 / run_full —— 分拣入库完整流程

对储存仓每个果实: 拿出 → 识别颜色 → 放入对应仓
假设仓里有 4 个果实(2 红 2 绿,顺序按取出顺序)
"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402

from a_approach import step_a_approach  # noqa: E402
from b1_take_fruit import step_b1_take_fruit  # noqa: E402
from b2_detect_color import step_b2_detect_color, COLOR_TO_BIN  # noqa: E402
from b3_place_bin import step_b3_place_bin  # noqa: E402
from c_finish import step_c_finish  # noqa: E402


# 仓里果实数量和顺序(赛前公布)
FRUITS_IN_STORAGE = ["red", "green", "red", "green"]


def main() -> None:
    t0 = time.time()
    client = ArmClient.connect()
    runner = ArmRunner(client)

    step_a_approach(client, runner)

    for i, expected_color in enumerate(FRUITS_IN_STORAGE):
        print(f"\n### 果实 #{i+1} ({expected_color}) ###")
        step_b1_take_fruit(client, runner)
        # 简化:跳过视觉,直接用 expected_color
        target = COLOR_TO_BIN[expected_color]
        print(f"  [决策] {expected_color} -> {target['bin']} 仓")
        step_b3_place_bin(client, runner,
                         target_x_mm=target["x_mm"],
                         target_y_mm=target["y_mm"],
                         is_high=(target["bin"] == "high"))

    step_c_finish(client, runner)
    print(f"\n=== 任务5 完成 {time.time()-t0:.1f}s ===")


if __name__ == "__main__":
    main()
