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
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.start.orchestrator import Orchestrator
from main.api_client import RuntimeApiClient


def probe_keys() -> None:
    """--probe-keys 標定模式：連續讀 BoardKey raw bytes 印出來，按 Ctrl+C 退出。

    用法：在 Jetson 跑 `python run.py --probe-keys`，逐顆按/放板上按鈕，
    觀察每個 byte 的變化，找出「按下時非零」的 byte index。
    然後填到 config_car.yml io.key 的 button_index（mode=specific）。
    不啟動 lane runner，不擾動任務邏輯——純離線讀鍵。
    """
    client = RuntimeApiClient()
    if not client.wait_until_ready(timeout=10.0):
        sys.stderr.write("runtime not ready (pm2 logs rak-car-api)\n")
        sys.exit(2)
    sys.stderr.write(
        "[probe-keys] 按板上每顆按鈕，按 Ctrl+C 退出。\n"
        "找到「按下時 byte N 從 0 變非零」的 N，填 config_car.yml io.key.button_index。\n"
        "若所有 byte 全程都是 0 或 wrapper 看不到你按的鈕，告訴我——可能要繞 wrapper。\n"
    )
    sys.stderr.flush()
    last_print = 0.0
    try:
        while True:
            try:
                resp = client.get(f"{client.api_prefix}/realtime/key/state", timeout=1.0)
                raw = resp.get("raw") if isinstance(resp, dict) and resp.get("ok") else None
            except Exception as exc:
                raw = None
                now = time.time()
                if now - last_print >= 1.0:
                    sys.stderr.write(f"[probe-keys] read error: {exc}\n")
                    sys.stderr.flush()
                    last_print = now
                time.sleep(0.1)
                continue
            now = time.time()
            if now - last_print >= 0.2:
                vals = list(raw) if isinstance(raw, (tuple, list)) else ([raw] if raw is not None else [])
                tags = [f"b{i}={'PRESS' if v else '....'}" for i, v in enumerate(vals)]
                sys.stderr.write(
                    f"[probe-keys] raw={vals!r:24}  " + "  ".join(tags) + "\n"
                )
                sys.stderr.flush()
                last_print = now
            time.sleep(0.05)
    except KeyboardInterrupt:
        sys.stderr.write("[probe-keys] exit\n")


def main() -> None:
    p = argparse.ArgumentParser(prog="run.py", description="rak-car 全流程入口")
    p.add_argument("--lane-hz", type=float, default=50.0)
    p.add_argument("--ir-interval-s", type=float, default=0.02)
    p.add_argument(
        "--task", type=int, default=None, choices=range(1, 8), metavar="1-7",
        help="只跑单个任务（1-7）: 巡线到该任务点位 → IR/里程计触发 → 执行任务 → 停止。"
             "不指定时跑全流程 8 任务。",
    )
    p.add_argument(
        "--tasks", type=str, default=None, metavar="1,2,3",
        help="只跑指定任务（逗号分隔）: 例如 --tasks 1,3,5。"
             "按给定顺序巡线触发执行，跳过未列出的任务。",
    )
    p.add_argument(
        "--wait-key", action="store_true",
        help=(
            "一键启动（比赛用）: 先完成全部初始化（车不挪动，MC602 屏幕显示 READY），"
            "按 MC602 板上键后立即开始完整任务。按下即开始计 时。"
        ),
    )
    p.add_argument(
        "--probe-keys", action="store_true",
        help=(
            "标定模式：连续读 MC602 BoardKey raw bytes 印出（按 Ctrl+C 退出）。"
            "找出「按下时 byte N 非零」的 N 填 config_car.yml io.key.button_index。"
        ),
    )
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(message)s")
    if args.probe_keys:
        probe_keys()
        return
    orch = Orchestrator(lane_hz=args.lane_hz,
                        ir_interval_s=args.ir_interval_s)
    if args.wait_key:
        orch.wait_key_then_run()
    elif args.tasks is not None:
        # 解析 --tasks 逗号分隔列表，如 "1,3,5"
        try:
            task_ids = [int(t.strip()) for t in args.tasks.split(",") if t.strip()]
        except ValueError:
            sys.stderr.write(f"--tasks 格式错误，请用逗号分隔数字，例如: --tasks 1,3,5\n")
            sys.exit(2)
        # 校验 task_id 范围
        invalid = [t for t in task_ids if t not in range(1, 8)]
        if invalid:
            sys.stderr.write(f"task_id 必须在 1-7 之间，非法值: {invalid}\n")
            sys.exit(2)
        orch.run_tasks(task_ids)
    elif args.task is not None:
        orch.run_single_task(args.task)
    else:
        orch.run()


if __name__ == "__main__":
    main()