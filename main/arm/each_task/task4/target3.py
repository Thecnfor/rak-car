#!/usr/bin/python3
"""task4 / target3 —— 吸气下降取球，再抬回安全高度。

动作顺序：
  1. grasp(True)       开启吸气泵
  2. move_y(-58)       y 下降到 -58 mm，期间保持吸气
  3. move_y(-133)      y 抬回 -133 mm，期间保持吸气
  4. 默认 grasp(False)；组合流程可传 release_after_return=False 继续保持吸气

⚠️ 本脚本不做 x 轴移动、目标检测或舵机姿态调整；运行前应确保吸盘已经
   对准球，且当前大臂/手爪姿态适合垂直抓取。
⚠️ 任一 move_y 抛错时不会继续执行放气，避免机械臂尚未回到 -133 mm 就
   把球释放到场地；此时需人工检查位置和吸附状态。

跑法：
    python main/arm/each_task/task4/target3.py
    python -m main.arm.each_task.task4.target3
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


LOG_PREFIX: str = "[task4/target3]"
PICK_Y_MM: float = -58.0
RETURN_Y_MM: float = -133.0
"""抬回 y (用户 2026-07-28 从 -150 改 -133, 跟 target1 / target4 对齐)。"""
MOVE_Y_TIMEOUT_S: float = 30.0
GRASP_TIMEOUT_S: float = 10.0


def step_target3(
    client: ArmClient,
    runner: ArmRunner,
    *,
    pick_y_mm: float = PICK_Y_MM,
    return_y_mm: float = RETURN_Y_MM,
    release_after_return: bool = True,
) -> dict:
    """吸气下降到 pick_y_mm，再保持吸气抬回 return_y_mm。

    release_after_return=True 时抬回后放气；False 时保持吸气，交给后续流程放气。
    """
    print(f"\n========== {LOG_PREFIX} step_target3 ==========")
    finish_action = "放气" if release_after_return else "继续保持吸气"
    print(
        f"  动作: 吸气 → y={pick_y_mm:+.1f}mm → "
        f"y={return_y_mm:+.1f}mm → {finish_action}"
    )

    print("  [1/4] grasp(True)  开启吸气泵")
    runner.grasp(True, timeout=GRASP_TIMEOUT_S)

    print(f"  [2/4] move_y({pick_y_mm:+.1f}mm)  下降，保持吸气")
    runner.move_y(pick_y_mm, timeout=MOVE_Y_TIMEOUT_S)

    print(f"  [3/4] move_y({return_y_mm:+.1f}mm)  抬回，保持吸气")
    runner.move_y(return_y_mm, timeout=MOVE_Y_TIMEOUT_S)

    if release_after_return:
        print("  [4/4] grasp(False)  已回安全高度，放气")
        runner.grasp(False, timeout=GRASP_TIMEOUT_S)
        pump_state = "released_after_return"
    else:
        print("  [4/4] 保持吸气，交给后续流程放气")
        pump_state = "holding"

    print(f"========== {LOG_PREFIX} 完成 ==========\n")
    return {
        "ok": True,
        "pick_y_mm": pick_y_mm,
        "return_y_mm": return_y_mm,
        "pump": pump_state,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="task4 target3: 吸气下降到 -58mm，抬回 -133mm 后放气",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--pick-y", type=float, default=PICK_Y_MM,
                        help="下降抓取位置 (mm)")
    parser.add_argument("--return-y", type=float, default=RETURN_Y_MM,
                        help="抬回后放气位置 (mm)")
    return parser


def main(argv=None) -> int:
    t_start = time.perf_counter()
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_target3(
        client,
        runner,
        pick_y_mm=args.pick_y,
        return_y_mm=args.return_y,
    )
    elapsed = time.perf_counter() - t_start
    print(f"========== {LOG_PREFIX} 总耗时: {elapsed:.3f} s ==========")
    return 0


if __name__ == "__main__":
    sys.exit(main())
