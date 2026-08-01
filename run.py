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
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(message)s")
    Orchestrator(lane_hz=args.lane_hz,
                 ir_interval_s=args.ir_interval_s).run()


if __name__ == "__main__":
    main()