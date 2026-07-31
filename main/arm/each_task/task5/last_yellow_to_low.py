"""task5 / last_yellow_to_low —— 检测黄球 → 循环 N 次 test3_from_yellow_to_low。

(2026-07-29 从 last_yellow_to_high 拆分出来, 通过 _last_loop 共享骨架。)

接线:
  - **prep_pose 以 get_yellow.py 为主** (用户 2026-07-29 要求: last 带 blue/yellow
    的抓球全部以 get_* 为主):
      y → GET_YELLOW_Y_DOWN_MM (-130) → x → GET_YELLOW_X_MM (-68) →
      大臂 85° → 手爪 0°
    prep_pose 跟实际抓取位姿完全一致 (get_yellow 5 步的前 4 步)。
  - 检测: target_yellow.detect_balls (DETECT_COLOR_FILTER="yellow", task5 专属阈值)
  - 循环: test3_from_yellow_to_low.run() (get_yellow → grasp+sleep → low_tower → release → reset_x)

⚠️ **prep_pose 改用 get_yellow 而非 target_yellow 的取舍**: 同 last_yellow_to_high,
   **prep+grab 一次到位**; 检测不准时退回 target_yellow prep 仅需改 prep_arm_deg
   为 TARGET1_ARM_DEG (默认 90°)。

跑法:
    python main/arm/each_task/task5/last_yellow_to_low.py
    python main/arm/each_task/task5/last_yellow_to_low.py --balls 2
    python main/arm/each_task/task5/last_yellow_to_low.py --no-prep --balls 1
"""
from __future__ import annotations

import argparse
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm.each_task.task5.test3_from_yellow_to_low import (  # noqa: E402
    run as test3_run,
    LOG_PREFIX as TEST3_LOG_PREFIX,
)
import main.arm.each_task.task5.target_yellow as target_module  # noqa: E402
import main.arm.each_task.task5.get_yellow as grab_module  # noqa: E402 (用于 prep_pose 参数)
from main.arm.each_task.task5._last_loop import (  # noqa: E402
    build_last_parser, main_with_args,
)

LOG_PREFIX: str = "[task5/last_yellow_to_low]"
COLOR_LABEL: str = "yellow"


def build_parser() -> argparse.ArgumentParser:
    return build_last_parser(LOG_PREFIX, COLOR_LABEL, TEST3_LOG_PREFIX)


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return main_with_args(
        args,
        log_prefix=LOG_PREFIX,
        target_module=target_module,
        test_run_fn=test3_run,
        test_log_prefix=TEST3_LOG_PREFIX,
        color_label=COLOR_LABEL,
        # ⚠️ prep_pose 统一用 target_blue 的位姿 (用户 2026-07-29 要求)
        prep_y_mm=target_module.TARGET1_Y_MM,    # -200
        prep_x_mm=target_module.TARGET1_X_MM,    # -40
        prep_arm_deg=target_module.TARGET1_ARM_DEG,   # 90°
        prep_hand_deg=target_module.TARGET1_HAND_DEG, # 0°
    )


if __name__ == "__main__":
    sys.exit(main())