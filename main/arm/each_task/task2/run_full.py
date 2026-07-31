#!/usr/bin/python3
"""task2 / run_full —— 智能灌溉完整流程

对每个水塔:走到 → 吸一块水 → 放到塔上 (重复 need 次)
"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402

from a_approach import step_a_approach  # noqa: E402
from b1_detect_tower import step_b1_detect_tower  # noqa: E402
from b2_pick_water import step_b2_pick_water  # noqa: E402
from b3_place_water import step_b3_place_water  # noqa: E402
from c_finish import step_c_finish  # noqa: E402


def main() -> None:
    t0 = time.time()
    client = ArmClient.connect()
    runner = ArmRunner(client)

    step_a_approach(client, runner)
    info = step_b1_detect_tower(client, runner)

    for tower in info["towers"]:
        for i in range(tower["need"]):
            print(f"\n### {tower['id']} 水塔 第 {i+1}/{tower['need']} 块 ###")
            step_b2_pick_water(client, runner)
            step_b3_place_water(client, runner, tower_x_mm=tower["x_mm"])

    step_c_finish(client, runner)
    print(f"\n=== 任务2 完成 {time.time()-t0:.1f}s ===")


if __name__ == "__main__":
    main()
