"""task5 / test3_from_yellow_to_low —— 黄球 → 低位仓 (薄 wrapper)。

(2026-08-03 重构: 主体抽到 pick_and_place.py, 本文件只保留接线 + CLI 兼容。)

接线:
  - pick : get_yellow.run  (y=-130 → x=-68 → arm=85° → hand=0° → y=-70)
  - tower: low_tower.run   (y=-200 → arm=90° → hand=0° → x=-169)
  - label: ball_yellow (仅 --vision 模式用)

跑法:
    python main/arm/each_task/task5/test3_from_yellow_to_low.py            # 开环 (旧行为)
    python main/arm/each_task/task5/test3_from_yellow_to_low.py --hold 10  # 改保持秒
    python main/arm/each_task/task5/test3_from_yellow_to_low.py --vision   # 视觉闭环取球
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
from main.arm.each_task.task5.low_tower import run as low_tower_run  # noqa: E402
from main.arm.each_task.task5.pick_and_place import (  # noqa: E402
    run_pick_and_place, build_pick_place_parser,
    GRASP_HOLD_S_DEFAULT, DEFAULT_GRASP_Y_MM,
)


LOG_PREFIX: str = "[task5/test3_from_yellow_to_low]"

<<<<<<< Updated upstream
VISION_LABEL: str = "ball_yellow"
"""--vision 模式的视觉伺服 label (labels.py 20 项之一)。"""
=======
GRASP_HOLD_S_DEFAULT: float = 5.0
"""吸气独立保持秒数 (用户 2026-07-23 要求 5s)。在 grasp(True) 之后, low_tower 之前执行。
设 0 可跳过保持 (退化为 v3 逻辑)。"""

# reset_x 撞墙定原点参数 (内联, 跟 get_blue.py 的 _reset_x_wall 同款, ARM_API §9.2)
RESET_X_DIRECTION: str = "right"
"""终点 reset_x 方向: low_tower 在 x=-169mm, 取蓝墙在 x 增大方向 (get_blue.py
测试结论 2026-07-22), 故走 right (正速度) 撞取蓝墙 → 撞到点定义为 x=0。"""

RESET_X_VELOCITY_MMS: float = 50.0
"""撞墙速度 (mm/s)。§9.2 建议 50mm/s 比 wrapper 默认 20 稳。"""

RESET_X_PROBE_TIME: float = 0.3
"""arm_base.py 默认值: probe_time=0 在 '车刚好在 selected 方向的墙上' 场景下
会立即误判 stall → calibrate 失败/撞错位置。留 0.3 让反向探针先验证 motor 工作。"""

RESET_X_TIMEOUT: float = 30.0


# ---------- reset_x 撞墙 (内联, 跟 get_blue.py 的 _reset_x_wall 同款) ----------

def _reset_x_wall(client: ArmClient) -> dict:
    """撞墙定 x 原点, 一步到位。走 ArmClient.reset_x wrapper (2026-08-01 wrapper 已透传
    probe_time, 不再需要 escape hatch)。

    Returns:
        {"reset": job dict, "x_mm_after": float | None}
    """
    print(f"  {LOG_PREFIX} reset_x(direction={RESET_X_DIRECTION}, v={RESET_X_VELOCITY_MMS}mm/s, "
          f"probe_time={RESET_X_PROBE_TIME})  撞墙一步到位")
    job = client.reset_x(
        direction=RESET_X_DIRECTION,
        reset_velocity_mms=RESET_X_VELOCITY_MMS,   # mm/s, wrapper 内部转 m/s
        probe_time=RESET_X_PROBE_TIME,
        timeout=RESET_X_TIMEOUT,
    )
    x_after = client._read_x_mm_realtime()
    print(f"  {LOG_PREFIX} reset_x 完成, realtime x={x_after}")
    return {"reset": job, "x_mm_after": x_after}
>>>>>>> Stashed changes


def run(client: ArmClient, runner: ArmRunner,
        hold_s: float = GRASP_HOLD_S_DEFAULT,
        vision: bool = False,
        grasp_y_mm: float = DEFAULT_GRASP_Y_MM,
        vision_fallback: bool = True,
        sign_arm: float = 1.0,
        sign_x: float = -1.0,
        vision_timeout: float = 20.0) -> dict:
    """黄球 → 低位仓。详见 pick_and_place.run_pick_and_place。"""
    return run_pick_and_place(
        client, runner,
        log_prefix=LOG_PREFIX,
        pick_fn=get_yellow_run, pick_name="get_yellow",
        tower_fn=low_tower_run, tower_name="low_tower",
        vision=vision, vision_label=VISION_LABEL,
        grasp_y_mm=grasp_y_mm,
        hold_s=hold_s,
        vision_fallback=vision_fallback,
        sign_arm=sign_arm, sign_x=sign_x,
        vision_timeout=vision_timeout,
    )


def build_parser() -> argparse.ArgumentParser:
    return build_pick_place_parser(
        "task5 test3: get_yellow -> 取球(开环盲吸/--vision 视觉闭环) -> low_tower -> release"
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
