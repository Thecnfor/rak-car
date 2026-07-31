"""task5 / last_yellow_to_high —— 检测黄球 → 循环 N 次 test1_from_yellow_to_high。

(2026-07-29 重构成 thin wrapper, 通过 _last_loop 共享骨架, 跟 last_blue_* 一致。)

接线:
  - **prep_pose 以 get_yellow.py 为主** (用户 2026-07-29 要求: last 带 blue/yellow
    的抓球全部以 get_* 为主, prep+grab 一次到位):
      y → GET_YELLOW_Y_DOWN_MM (-130) → x → GET_YELLOW_X_MM (-68) →
      大臂 85° → 手爪 0°
    prep_pose 跟实际抓取位姿完全一致 (get_yellow 5 步的前 4 步)。
  - 检测: target_yellow.detect_balls (DETECT_COLOR_FILTER="yellow", task5 专属阈值)
  - 循环: test1_from_yellow_to_high.run() (get_yellow → grasp+sleep → high_tower → release → reset_x)

⚠️ **prep_pose 改用 get_yellow 而非 target_yellow 的取舍 (2026-07-29)**:
  - target_yellow 的 prep 姿态 (y=-200 / x=-68 / arm=90°) 是"专门为球在画面里
    好认"设计的, 臂更展开 (90°) 把吸盘抬离画面, 球在 cx_norm/cy_norm 表现更稳。
  - get_yellow 的 prep 姿态 (y=-130 / x=-68 / arm=85°) 是"实际抓取位姿", 臂没
    完全展开, 球检测时的 cx/cy 数值会跟 target_yellow 位姿下略有不同。
  - 当前选择 get_yellow 的 prep: **prep+grab 一次到位, 进 test1_run 时已就位**,
    省掉 prep→grab 的过渡动作 (~0.5s 舵机臂 5° 调整)。如果发现球检测不准, 退
    回 target_yellow prep 仅需改 prep_arm_deg=grab_module.GET_YELLOW_ARM_DEG
    → TARGET1_ARM_DEG (默认 90°) 即可。

跑法:
    python main/arm/each_task/task5/last_yellow_to_high.py
    python main/arm/each_task/task5/last_yellow_to_high.py --balls 3
    python main/arm/each_task/task5/last_yellow_to_high.py --no-prep --balls 1
"""
from __future__ import annotations

import argparse
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm.each_task.task5.test1_from_yellow_to_high import (  # noqa: E402
    run as test1_run,
    LOG_PREFIX as TEST1_LOG_PREFIX,
)
import main.arm.each_task.task5.target_yellow as target_module  # noqa: E402 (用于 _move_x_with_split + detect)
import main.arm.each_task.task5.get_yellow as grab_module  # noqa: E402 (用于 prep_pose 参数)
from main.arm.each_task.task5._last_loop import (  # noqa: E402
    build_last_parser, main_with_args,
)

LOG_PREFIX: str = "[task5/last_yellow_to_high]"
COLOR_LABEL: str = "yellow"


def build_parser() -> argparse.ArgumentParser:
    return build_last_parser(LOG_PREFIX, COLOR_LABEL, TEST1_LOG_PREFIX)


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return main_with_args(
        args,
        log_prefix=LOG_PREFIX,
        target_module=target_module,
        test_run_fn=test1_run,
        test_log_prefix=TEST1_LOG_PREFIX,
        color_label=COLOR_LABEL,
        # ⚠️ prep_pose 统一用 target_blue 的位姿 (用户 2026-07-29 要求, 4 个
        #    last_* 都用同一组观察坐标, 颜色靠 DETECT_COLOR_FILTER 区分)
        prep_y_mm=target_module.TARGET1_Y_MM,    # -200
        prep_x_mm=target_module.TARGET1_X_MM,    # -40
        prep_arm_deg=target_module.TARGET1_ARM_DEG,   # 90°
        prep_hand_deg=target_module.TARGET1_HAND_DEG, # 0°
    )


if __name__ == "__main__":
    sys.exit(main())