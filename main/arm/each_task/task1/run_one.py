#!/usr/bin/python3
"""task1 / run_one —— 跑 1 个种子的完整循环 B1→B7

把单个种子的抓取+放置串起来,方便调单个循环的时序/参数。

用法:
  python3 main/arm/each_task/task1/run_one.py
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402

from b1_detect import step_b1_detect  # noqa: E402
from b2_arm_prep import step_b2_arm_prep  # noqa: E402
from b3_arm_to_seed import step_b3_arm_to_seed  # noqa: E402
from b4_pick import step_b4_pick  # noqa: E402
from b5_lift import step_b5_lift  # noqa: E402
from b6_drive_to_target import step_b6_drive_to_target  # noqa: E402
from b7_place import step_b7_place  # noqa: E402


def run_one_seed(client: ArmClient, runner: ArmRunner,
                 seed_index: int, seed_size_mm: int) -> bool:
    """跑 1 个种子:B1→B7。返回是否成功。"""
    print(f"\n########## Seed #{seed_index} ({seed_size_mm}cm) ##########")

    r1 = step_b1_detect(client, runner)
    seed = r1["seed"]
    # 按 size 模拟不同 x 位置(实际由视觉给)
    seed["x_mm"] = 80.0 + seed_index * 40.0  # 临时占位
    seed["diameter_mm"] = float(seed_size_mm) * 10

    # 按种子的水平位置决定大臂方向(粗略阈值:中间用 MID)
    side = "MID"
    if seed["x_mm"] < 80:
        side = "LEFT"
    elif seed["x_mm"] > 160:
        side = "RIGHT"

    step_b2_arm_prep(client, runner, side=side, hand="DOWN")
    step_b3_arm_to_seed(client, runner, seed_x_mm=seed["x_mm"])
    step_b4_pick(client, runner)
    step_b5_lift(client, runner, safe_y_mm=-80.0)
    step_b6_drive_to_target(client, runner,
                            target_x_mm=200.0 + seed_index * 30.0)
    step_b7_place(client, runner,
                 target_x_mm=200.0 + seed_index * 30.0)

    return True


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)

    # 演示:跑 1 个 10cm 的种子
    run_one_seed(client, runner, seed_index=0, seed_size_mm=10)

    print("\n=== run_one done ===")


if __name__ == "__main__":
    main()
