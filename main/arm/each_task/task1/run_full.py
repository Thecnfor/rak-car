#!/usr/bin/python3
"""task1 / run_full —— 任务1 完整流程

阶段A 到达 → 3 轮 (B1→B7) → 阶段C 收尾
10cm → 8cm → 6cm

⚠️ 前置:任务1 完整跑通需要
  1) x 轴电机(id=6)修好
  2) api.py:234 grasp bug 修好
  3) arm_origin.yaml 已标定

用法:
  python3 main/arm/each_task/task1/run_full.py
"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402

from a_approach import step_a_approach  # noqa: E402
from c_finish import step_c_finish  # noqa: E402
from run_one import run_one_seed  # noqa: E402


# 从大到小:10cm, 8cm, 6cm
SEED_SIZES_CM = [10, 8, 6]


def main() -> None:
    t_total = time.time()

    client = ArmClient.connect()
    runner = ArmRunner(client)

    # 阶段A
    step_a_approach(client, runner)

    # 3 个种子(大→小,放左→右)
    for i, size_cm in enumerate(SEED_SIZES_CM):
        run_one_seed(client, runner, seed_index=i, seed_size_mm=size_cm)

    # 阶段C
    step_c_finish(client, runner)

    dt = time.time() - t_total
    print(f"\n=== 任务1 完成,总耗时 {dt:.1f}s ===")


if __name__ == "__main__":
    main()
