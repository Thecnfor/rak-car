#!/usr/bin/python3
"""task4 / run_full —— 采收完整流程

对每个果实:b1 识别 → b2 吸取 → b3 存入储存仓
"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402

from a_approach import step_a_approach  # noqa: E402
from b1_detect_fruit import step_b1_detect_fruit  # noqa: E402
from b2_pick_fruit import step_b2_pick_fruit  # noqa: E402
from b3_store_fruit import step_b3_store_fruit  # noqa: E402
from c_finish import step_c_finish  # noqa: E402


def main() -> None:
    t0 = time.time()
    client = ArmClient.connect()
    runner = ArmRunner(client)

    step_a_approach(client, runner)
    info = step_b1_detect_fruit(client, runner)

    for i, fruit in enumerate(info["fruits"]):
        print(f"\n### 果实 #{i+1} {fruit['color']} ###")
        step_b2_pick_fruit(client, runner, fruit_x_mm=fruit["x_mm"])
        step_b3_store_fruit(client, runner)

    step_c_finish(client, runner)
    print(f"\n=== 任务4 完成 {time.time()-t0:.1f}s ===")


if __name__ == "__main__":
    main()
