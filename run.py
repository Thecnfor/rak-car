#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""run.py — 全流程入口（极薄壳）。

真正的调度逻辑在 main.start.orchestrator.Orchestrator。
本文件只做：① 加 sys.path ② 解析 CLI ③ Orchestrator().run()。
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.start.orchestrator import Orchestrator


def main() -> None:
    p = argparse.ArgumentParser(prog="run.py", description="rak-car 全流程入口")
    p.add_argument("--lane-hz", type=float, default=50.0)
    p.add_argument("--ir-interval-s", type=float, default=0.02)
    p.add_argument(
        "--task", type=int, default=None, choices=range(1, 8), metavar="1-7",
        help=(
            "只跑单个任务（1-7）: 巡线到该任务点位 → IR/里程计触发 → 执行任务 → 停止。"
            "不指定时跑全流程 8 任务。"
        ),
    )
    p.add_argument(
        "--wait-key", action="store_true",
        help=(
            "一键启动（比赛用）: 先完成全部初始化（车不挪动，MC602 屏幕显示 READY），"
            "按 MC602 板上键后立即开始完整任务。按下即开始计 时。"
        ),
    )
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(message)s")
    orch = Orchestrator(lane_hz=args.lane_hz,
                        ir_interval_s=args.ir_interval_s)
    if args.wait_key:
        orch.wait_key_then_run()
    elif args.task is not None:
        orch.run_single_task(args.task)
    else:
        orch.run()


if __name__ == "__main__":
    main()