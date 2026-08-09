#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务四: 作物抓取 (收割).

薄封装 main.arm.each_task.task4.target4.step_target4.
CLI 只暴露最常用的开关, 其余走 target4 默认值.
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient
from main.arm import ArmClient, ArmRunner


def run(client: Optional[RuntimeApiClient] = None, **kwargs: Any) -> Dict[str, Any]:
    """任务四入口: 组装 client/runner, 透传参数给 step_target4."""
    cli = client or RuntimeApiClient()
    arm = ArmClient(http=cli)
    runner = ArmRunner(arm)

    from main.arm.each_task.task4.target4 import step_target4
    # step_target4 的预算参数已于 2026-08-10 冻结为 constants 默认值, 不再接收
    # max_picks/creep_speed_mps/max_creep_m/max_seconds/return_x_mm; 透传时必须
    # 过滤掉这些键 (CLI --max-picks 等已是 no-op), 否则全量 **kwargs 会 TypeError。
    _ALLOWED = {
        "defer_task5_handoff", "dry_run", "debug_recognition",
        "pose_p_y_mm", "pose_p_x_mm", "pose_p_arm_deg", "pose_p_hand_deg",
        "pick_x_mm", "pick_y_mm", "transit_y_mm", "transit_x_mm", "put_y_mm",
        "bin_x_blue_mm", "bin_x_yellow_mm", "bin_y_blue_mm", "bin_y_yellow_mm",
        "bin_hand_blue_deg", "bin_hand_yellow_deg",
    }
    detail = step_target4(
        arm_client=arm,
        http_client=cli,
        runner=runner,
        **{k: v for k, v in kwargs.items() if k in _ALLOWED},
    )

    ok = bool(detail.get("ok")) if isinstance(detail, dict) else bool(detail)
    return {"ok": ok, "task": "task4_harvest", "detail": detail}


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task4_harvest: 作物抓取",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--max-picks", type=int, default=1000,
                   help="最多抓取数")
    p.add_argument("--creep-speed", type=float, default=0.12,
                   help="creep 前移速度 (m/s)")
    p.add_argument("--max-creep-m", type=float, default=0.58,
                   help="累计前移预算 (m), 耗尽无球=采区走完")
    p.add_argument("--max-seconds", type=float, default=9999.0,
                   help="任务总时长预算 (s)")
    p.add_argument("--dry-run", action="store_true",
                   help="dry-run 模式 (只打印不动硬件)")
    p.add_argument("--no-return", action="store_true",
                   help="放 bin 后 x 不回位")
    p.add_argument("--debug-recognition", action="store_true",
                   help="fetch_balls 打印每条检测的过滤原因")
    return p


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)

    from main.arm.each_task.task4.target4 import DEFAULT_RETURN_X_MM
    return_x_mm: Optional[float] = None if args.no_return else DEFAULT_RETURN_X_MM

    t0 = time.monotonic()
    result = run(
        max_picks=args.max_picks,
        creep_speed_mps=args.creep_speed,
        max_creep_m=args.max_creep_m,
        max_seconds=args.max_seconds,
        dry_run=args.dry_run,
        debug_recognition=args.debug_recognition,
        return_x_mm=return_x_mm,
    )
    elapsed = time.monotonic() - t0

    print(f"\n[task4_harvest] 完成 ok={result['ok']} elapsed={elapsed:.1f}s")
    print(f"  detail: {result['detail']}")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
