"""task5 / test1_from_yellow_to_high —— 黄球 → 高位仓 (薄 wrapper)。

(2026-08-03 重构: 主体抽到 pick_and_place.py, 本文件只保留接线 + CLI 兼容。)

接线:
  - pick : get_yellow.run   (y=-130 → x=-68 → arm=85° → hand=0° → y=-70)
  - tower: high_tower.run   (y=-180 → arm=90° → hand=-90° → x=-160)
  - label: ball_yellow (仅 --vision 模式用)

两种模式 (见 pick_and_place.py 模块注释):
  - 默认开环: get_yellow → grasp(True)+sleep(hold_s) → high_tower → grasp(False) → reset_x
  - --vision: get_yellow → track_velocity_pick("ball_yellow") 视觉伺服 → high_tower → ...

跑法:
    python main/arm/each_task/task5/test1_from_yellow_to_high.py            # 开环 (旧行为)
    python main/arm/each_task/task5/test1_from_yellow_to_high.py --hold 10  # 改保持秒
    python main/arm/each_task/task5/test1_from_yellow_to_high.py --vision   # 视觉闭环取球
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.each_task.task5.get_yellow import run as get_yellow_run  # noqa: E402
from main.arm.each_task.task5.high_tower import run as high_tower_run  # noqa: E402
from main.arm.each_task.task5.pick_and_place import (  # noqa: E402
    run_pick_and_place, build_pick_place_parser,
    GRASP_HOLD_S_DEFAULT, DEFAULT_GRASP_Y_MM,
)


LOG_PREFIX: str = "[task5/test1_from_yellow_to_high]"

VISION_LABEL: str = "ball_yellow"
"""--vision 模式的视觉伺服 label (labels.py 20 项之一)。"""


def run(client: ArmClient, runner: ArmRunner,
        hold_s: float = GRASP_HOLD_S_DEFAULT,
        vision: bool = False,
        grasp_y_mm: float = DEFAULT_GRASP_Y_MM,
        vision_fallback: bool = True,
        sign_arm: float = 1.0,
        sign_x: float = -1.0,
        vision_timeout: float = 20.0) -> dict:
    """黄球 → 高位仓。详见 pick_and_place.run_pick_and_place。"""
    return run_pick_and_place(
        client, runner,
        log_prefix=LOG_PREFIX,
        pick_fn=get_yellow_run, pick_name="get_yellow",
        tower_fn=high_tower_run, tower_name="high_tower",
        vision=vision, vision_label=VISION_LABEL,
        grasp_y_mm=grasp_y_mm,
        hold_s=hold_s,
        vision_fallback=vision_fallback,
        sign_arm=sign_arm, sign_x=sign_x,
        vision_timeout=vision_timeout,
    )


def build_parser() -> argparse.ArgumentParser:
    return build_pick_place_parser(
        "task5 test1: get_yellow -> 取球(开环盲吸/--vision 视觉闭环) -> high_tower -> release"
    )


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    t_total_start = time.perf_counter()
    run(client, runner,
        hold_s=args.hold,
        vision=args.vision,
        grasp_y_mm=args.grasp_y,
        vision_fallback=args.vision_fallback,
        sign_arm=args.sign_arm, sign_x=args.sign_x,
        vision_timeout=args.vision_timeout)
    elapsed = time.perf_counter() - t_total_start
    print(f"========== {LOG_PREFIX} 总耗时: {elapsed:.3f} s ==========")
    return 0


if __name__ == "__main__":
    sys.exit(main())
