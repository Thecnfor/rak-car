"""task5 / grasp_5 —— 简单吸气 5s + 放气。

最简冒烟脚本: 接通 ArmClient → 吸气 → 保持 5s → 放气。

⚠️ 本脚本**不**做位姿 setup, 调用前必须先保证:
  - y 出 y 保护区 [0, -30] (move_y 任意 ≤ -30 即可, grasp 不挑 y)
  - arm / hand / x 任意 (grasp 不挑姿态)
  - 想真测吸力: 在吸盘正下方放块小纸片 / 球 (但本脚本不挪臂, 只测泵)

⚠️ 本文件**自包含**: 只依赖 `main.arm` (ArmClient/ArmRunner), 不 import task5
   包内其它模块。原因: task5 辅助文件曾被外部动作清空过, 自包含保证直接跑不受影响。

跑法:
    python main/arm/each_task/task5/grasp_5.py
    python -m main.arm.each_task.task5.grasp_5
    python main/arm/each_task/task5/grasp_5.py --hold 8   # 改保持秒数
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


LOG_PREFIX: str = "[task5/grasp_5]"

GRASP_5_HOLD_S: float = 5.0
"""默认吸气保持秒数 (用户 2026-07-22 要求 5s)。"""


def run(client: ArmClient, runner: ArmRunner,
        hold_s: float = GRASP_5_HOLD_S) -> dict:
    """吸气 hold_s 秒 + 放气。

    Args:
        client: ArmClient
        runner: ArmRunner
        hold_s: 吸气保持秒数 (默认 GRASP_5_HOLD_S = 5.0)

    Returns:
        {"ok": True, "pump": "tested_<hold_s>s_on_off", "hold_s": float}
    """
    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  [1/3] grasp(True)   吸气")
    runner.grasp(True, timeout=10.0)
    print(f"  [2/3] sleep({hold_s:.1f}s)  保持")
    time.sleep(hold_s)
    print(f"  [3/3] grasp(False)  放气")
    runner.grasp(False, timeout=10.0)
    print(f"========== {LOG_PREFIX} 完成 (抽气 {hold_s:.1f}s + 放气) ==========\n")
    return {
        "ok": True,
        "pump": f"tested_{hold_s:.1f}s_on_off",
        "hold_s": hold_s,
    }


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task5 grasp_5: 简单吸气 5s + 放气 (不挪臂)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--hold", type=float, default=GRASP_5_HOLD_S,
                   help="吸气保持秒数 (默认 5.0)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner, hold_s=args.hold)
    return 0


if __name__ == "__main__":
    sys.exit(main())
