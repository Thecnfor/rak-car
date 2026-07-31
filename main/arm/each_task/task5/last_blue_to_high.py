"""task5 / last_blue_to_high —— 检测蓝球 → 循环 N 次 test2_from_blue_to_high。

(2026-07-29 从 last_yellow_to_high 拆分出来, 通过 _last_loop 共享骨架。)

接线:
  - **prep_pose 以 get_blue.py 为主** (用户 2026-07-29 要求: last 带blue的抓球
    全部以 get_blue 为主):
      y → GET_BLUE_Y_DOWN_MM (-130) → x → 0 (reset_x 撞墙) → 大臂 85° → 手爪 0°
    也就是 prep_pose 跟实际抓取位姿完全一致 (get_blue 5 步的前 4 步);
    prep_pose 跟 detection 不再分开, 检测时已经在抓取位姿上。
  - 检测: target_blue.detect_balls (DETECT_COLOR_FILTER="blue", task5 专属阈值)
  - 循环: test2_from_blue_to_high.run() (get_blue → grasp+sleep → high_tower → release → reset_x)

⚠️ **prep_pose 改用 get_blue 而非 target_blue 的取舍 (2026-07-29)**:
  - target_blue 的 prep 姿态 (y=-200 / x=-40 / arm=90°) 是"专门为球在画面里好认"
    设计的, 臂更展开 (90°) 把吸盘抬离画面, 球在 cx_norm/cy_norm 表现更稳。
  - get_blue 的 prep 姿态 (y=-130 / x=0 / arm=85°) 是"实际抓取位姿", 臂没完全
    展开, 球检测时的 cx/cy 数值会跟 target_blue 位姿下的 BALL_VERIFIED_* 不同。
  - 当前选择 get_blue 的 prep: **prep+grab 一次到位, 进 test2_run 时已就位**, 省
    掉 prep→grab 的过渡动作 (~0.5s 舵机臂 5° 调整)。如果发现球检测不准, 退回
    target_blue prep 仅需去掉下面对 get_blue.* 的 import 即可 (回退到默认行为)。

跑法:
    python main/arm/each_task/task5/last_blue_to_high.py
    python main/arm/each_task/task5/last_blue_to_high.py --balls 2
    python main/arm/each_task/task5/last_blue_to_high.py --no-prep --balls 1
"""
from __future__ import annotations

import argparse
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm.each_task.task5.test2_from_blue_to_high import (  # noqa: E402
    run as test2_run,
    LOG_PREFIX as TEST2_LOG_PREFIX,
)
import main.arm.each_task.task5.target_blue as target_module  # noqa: E402 (用于 _move_x_with_split + detect)
import main.arm.each_task.task5.get_blue as grab_module  # noqa: E402 (用于 prep_pose 参数 + GRAB_POSE_LABEL)
from main.arm.each_task.task5._last_loop import (  # noqa: E402
    build_last_parser, main_with_args,
)

LOG_PREFIX: str = "[task5/last_blue_to_high]"
COLOR_LABEL: str = "blue"
GRAB_POSE_LABEL: str = "get_blue"  # 显示用


def build_parser() -> argparse.ArgumentParser:
    return build_last_parser(LOG_PREFIX, COLOR_LABEL, TEST2_LOG_PREFIX)


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return main_with_args(
        args,
        log_prefix=LOG_PREFIX,
        target_module=target_module,
        test_run_fn=test2_run,
        test_log_prefix=TEST2_LOG_PREFIX,
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