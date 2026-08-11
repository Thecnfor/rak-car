#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/task/task3/set_recognition_pose.py — 到达识别区时, 把机械臂摆到识别姿态.

2026-08-12 用户需求: 车到达 task3 识别区后, 手动/调试下把臂调到识别位置。
编排模式下 orchestrator 已在 task1 结束后巡线途中预摆 (本脚本会因"已在位"跳过);
单任务 / 手动调试时跑本脚本补摆。

识别姿态 (RECOGNITION_ARM, 见 arm_poses.py):
    y1=-100mm(中间位) → y2=-40mm(识别位), x=-270mm, 大臂=90°, 手爪=-70°

用法 (仓库根目录, 车已停在识别区):
    python -m main.task.task3.set_recognition_pose
    python -m main.task.task3.set_recognition_pose --force    # 无视已在位, 强制重摆
"""
from __future__ import annotations

import argparse
import subprocess
import sys

from main.api_client import RuntimeApiClient
from main.task.task3.arm_poses import RECOGNITION_ARM, arm_at_pose


def main() -> int:
    ap = argparse.ArgumentParser(description="到达识别区时摆识别姿态 (RECOGNITION_ARM)")
    ap.add_argument("--force", action="store_true",
                    help="已在识别姿态也强制重摆 (默认: 已在位则跳过)")
    args = ap.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()

    # 2026-08-12: 编排下 orchestrator 已在 task1→识别区巡线途中预摆 → 已在位则跳过.
    if not args.force and arm_at_pose(client, RECOGNITION_ARM):
        print(f"[arm] 已在识别姿态 {RECOGNITION_ARM}, 跳过摆臂", flush=True)
        return 0

    command = [
        sys.executable, "-m", "main.task.task3.arm_seq_v9",
        "--y1", RECOGNITION_ARM[0], "--y2", RECOGNITION_ARM[1],
        "--x", RECOGNITION_ARM[2], "--arm-angle", RECOGNITION_ARM[3],
        "--hand-angle", RECOGNITION_ARM[4],
    ]
    print(f"[arm] 摆识别姿态: {' '.join(command)}", flush=True)
    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
