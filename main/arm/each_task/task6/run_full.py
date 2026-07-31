#!/usr/bin/python3
"""task6 / run_full —— 智能接单完整流程"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402

from a_approach import step_a_approach  # noqa: E402
from b1_push_rod import step_b1_push_rod  # noqa: E402
from b2_ocr_lm import step_b2_ocr_lm  # noqa: E402
from b3_pick_goods import step_b3_pick_goods  # noqa: E402
from c_finish import step_c_finish  # noqa: E402


def main() -> None:
    t0 = time.time()
    client = ArmClient.connect()
    runner = ArmRunner(client)

    step_a_approach(client, runner)
    step_b1_push_rod(client, runner)
    parsed = step_b2_ocr_lm(client, runner)
    step_b3_pick_goods(client, runner, goods_name=parsed["goods_name"])
    step_c_finish(client, runner)
    print(f"\n=== 任务6 完成 {time.time()-t0:.1f}s ===")


if __name__ == "__main__":
    main()
