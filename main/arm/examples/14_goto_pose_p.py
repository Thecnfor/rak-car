#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/arm/examples/14_goto_pose_p.py

一次性脚本: 把臂摆到 P 姿态 (Pose-P), 供现场手动标定 grasp 抓取位姿。

P 姿态 (单一真相源, 与 main/arm/each_task/common.py 同步):
  x = -300 mm
  y = -120 mm
  大臂 = +90°  (MID / 复位位)
  手爪 = +10°  (2026-08-05 用户拍板)

用法:
  export RAK_CAR_API_BASE=http://192.168.5.230:5050
  /usr/bin/python3 main/arm/examples/14_goto_pose_p.py

⚠️ 本脚本**只做摆位姿** (composite_run 4 轴并行), 不动底盘, 不动 grasp。
   适合在用户手动标定 "吸嘴吸到球的那一刻位置" 之前, 把臂摆过去。

⚠️ 配套标定脚本: main/arm/examples/13_nozzle_align_pose_p.py (标定吸嘴
   在画面里的 setpoint 偏移 (sx, sy)); 摆好姿态后由用户手动放球, 然后
   启动 13 采集采样。
"""
from __future__ import annotations

import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.each_task.common import (  # noqa: E402
    POSE_P_Y_MM, POSE_P_X_MM,
    POSE_P_ARM_DEG, POSE_P_HAND_DEG,
    goto_pose_p,
)


def main() -> int:
    client = ArmClient.connect()
    runner = ArmRunner(client)

    print(f"\n========== [14_goto_pose_p] 摆到 P 姿态 ==========")
    print(f"  目标姿态 (单一真相源 main/arm/each_task/common.py):")
    print(f"    y = {POSE_P_Y_MM:+.0f} mm")
    print(f"    x = {POSE_P_X_MM:+.0f} mm")
    print(f"    大臂 = {POSE_P_ARM_DEG:.0f}°")
    print(f"    手爪 = {POSE_P_HAND_DEG:.0f}°")
    print(f"  → composite_run 4 轴并行 (arm/hand/y/x 同步到位)")
    print()

    result = goto_pose_p(
        client, runner,
        log_prefix="[14_goto_pose_p]",
    )
    print(f"\n========== [14_goto_pose_p] 完成 ==========")
    print(f"  实际 y = {result.get('actual_y_mm')} mm")
    print(f"  实际 x = {result.get('actual_x_mm')} mm")
    print()
    print("现在可以手动:")
    print("  1. 把球放到吸嘴正下方")
    print("  2. 跑 13_nozzle_align_pose_p.py 标定吸嘴 setpoint (sx, sy)")
    print("  3. 或者直接用 grasp.py / 调 sdk 测 grasp 抓取的 y 高度")
    return 0


if __name__ == "__main__":
    sys.exit(main())