#!/usr/bin/python3
"""task7 / run_full —— 产品配送完整流程"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402

from a_approach import step_a_approach  # noqa: E402
from b1_detect_house import step_b1_detect_house  # noqa: E402
from b2_place_deliver import step_b2_place_deliver  # noqa: E402
from c_finish import step_c_finish  # noqa: E402


# 从 task6 解析出来的地址(实际应跨任务传递)
ADDRESS = "1单元"


def main() -> None:
    t0 = time.time()
    client = ArmClient.connect()
    runner = ArmRunner(client)

    step_a_approach(client, runner)
    house = step_b1_detect_house(client, runner, address=ADDRESS)
    step_b2_place_deliver(client, runner, house_x_mm=house["x_mm"])
    step_c_finish(client, runner)
    print(f"\n=== 任务7 完成 {time.time()-t0:.1f}s ===")


if __name__ == "__main__":
    main()
