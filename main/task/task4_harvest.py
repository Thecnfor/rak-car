#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务四: 作物抓取 (收割).

实际业务逻辑委托至: main.arm.each_task.task4.target4.step_target4
(移植自 am 分支; 2026-08 P1 重写为 pick_by_vision 视觉伺服 + composite_run 并行放 bin).

完整业务流程概览:
  1. 摆臂到 P 姿态 (y=-120, x=-300, arm=90°, hand=10°)
  2. 进入循环: 底盘前移 → 视觉识别作物 → 抓取作物 → 放入存储
  3. 多轮抓取完毕后, 机械臂回到 P 姿态 + 关仓

本文件职责: 薄封装 + 参数透传. 暴露 step_target4 核心参数,
  让 orchestrator / 手动调用都能一眼看到可调项.
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient
from main.arm import ArmClient, ArmRunner

# 所有参数默认值统一从 target4.py 读取，单一配置入口
from main.arm.each_task.task4.target4 import (
    DEFAULT_MAX_PICKS,
    DEFAULT_MAX_CREEP_M,
    DEFAULT_MAX_SECONDS,
    DEFAULT_TRACK_MAX_SECONDS,
    DEFAULT_MAX_CONSECUTIVE_PICK_FAILURES,
    DEFAULT_RETURN_X_MM,
    DEFAULT_PICK_TIMEOUT_S,
    DEFAULT_CREEP_SPEED_MPS,
    TASK4_POSE_P_Y_MM,
    TASK4_POSE_P_X_MM,
    TASK4_POSE_P_ARM_DEG,
    TASK4_POSE_P_HAND_DEG,
    X_PICK_MM,
    Y_PICK_MM,
    Y_TRANSIT_MM,
    X_TRANSIT_MM,
    Y_PUT_MM,
    BIN_X_MM,
    COLOR_BLUE,
    COLOR_YELLOW,
)


def run(
    client: Optional[RuntimeApiClient] = None,
    *,
    # 抓取预算
    max_picks: int = DEFAULT_MAX_PICKS,
    max_seconds: float = DEFAULT_MAX_SECONDS,
    max_creep_m: float = DEFAULT_MAX_CREEP_M,
    # 搜索速度 (m/s)
    creep_speed_mps: float = DEFAULT_CREEP_SPEED_MPS,
    # 底盘视觉伺服收敛预算 (s)
    track_max_seconds: float = DEFAULT_TRACK_MAX_SECONDS,
    # 连续 pick 失败容忍 (命中即退出, 防死循环)
    max_consecutive_pick_failures: int = DEFAULT_MAX_CONSECUTIVE_PICK_FAILURES,
    # 放 bin 后 x 回位 (mm); None = 不回; 默认 = P 姿态 x
    return_x_mm: Optional[float] = DEFAULT_RETURN_X_MM,
    # pick_by_vision 总超时 (s)
    pick_timeout_s: float = DEFAULT_PICK_TIMEOUT_S,
    # 准备位姿 (当前 target4 已内嵌 P 姿态恢复, 此参数保留兼容)
    do_prep: bool = False,
    # 调试
    dry_run: bool = False,
    debug_recognition: bool = False,
    # ---- 姿态参数 (默认值来自 target4.py, 单一配置入口) ----
    pose_p_y_mm: float = TASK4_POSE_P_Y_MM,
    pose_p_x_mm: float = TASK4_POSE_P_X_MM,
    pose_p_arm_deg: float = TASK4_POSE_P_ARM_DEG,
    pose_p_hand_deg: float = TASK4_POSE_P_HAND_DEG,
    pick_y_mm: float = Y_PICK_MM,
    pick_x_mm: float = X_PICK_MM,
    transit_y_mm: float = Y_TRANSIT_MM,
    transit_x_mm: float = X_TRANSIT_MM,
    put_y_mm: float = Y_PUT_MM,
    bin_x_blue_mm: float = BIN_X_MM[COLOR_BLUE],
    bin_x_yellow_mm: float = BIN_X_MM[COLOR_YELLOW],
    defer_task5_handoff: bool = False,
) -> Dict[str, Any]:
    """任务四主入口: 薄封装 step_target4, 参数全透传.

    Args:
        client: 复用 RuntimeApiClient; None 时内部新建连接 (orchestrator 场景走复用).
        max_picks: 最多抓取数 (距离优先模式下默认 1000, 实际不限制).
        max_seconds: 任务总时长预算 (s) (距离优先模式下默认 9999, 实际不限制).
        max_creep_m: 累计前移距离预算 (m), 耗尽无球 = 采区走完 (唯一实际生效的终止条件).
        creep_speed_mps: creep 前移速度 (m/s), 第一球用此值, 后续减半.
        track_max_seconds: 单球底盘视觉伺服收敛预算 (s).
        max_consecutive_pick_failures: 连续 pick 失败容忍 (距离优先模式下默认 1000, 实际不限制).
        return_x_mm: 放 bin 后 x 回位 (mm); None = 不回; 默认 -300 (P 姿态 x).
        pick_timeout_s: pick_by_vision 总超时 (s).
        do_prep: True 时开头跑 target1 准备位姿 (当前 target4 已删, 保留兼容).
        dry_run: True 不动硬件 (仍轮询视觉排练流程).
        debug_recognition: fetch_balls 打印每条检测的过滤原因.
        defer_task5_handoff: 仅 orchestrator 调度时置 True —— IR+odom 正常结束后
            关仓 + task5 Phase 1 姿态交给巡航后台线程; 独立运行保持 False 由本任务收尾.

    Returns:
        Dict: {
            "ok": bool,                    # step_target4 成功与否
            "task": "task4_harvest",      # 固定任务名
            "detail": step_target4 原始返回值  # 业务层详细数据
        }
    """
    cli = client or RuntimeApiClient()
    arm = ArmClient(http=cli)
    runner = ArmRunner(arm)

    # lazy import: each_task 包体积较大, 不进入 cold path
    from main.arm.each_task.task4.target4 import step_target4

    detail = step_target4(
        arm_client=arm,
        http_client=cli,
        runner=runner,
        max_picks=max_picks,
        max_seconds=max_seconds,
        max_creep_m=max_creep_m,
        creep_speed_mps=creep_speed_mps,
        track_max_seconds=track_max_seconds,
        max_consecutive_pick_failures=max_consecutive_pick_failures,
        return_x_mm=return_x_mm,
        pick_timeout_s=pick_timeout_s,
        do_prep=do_prep,
        dry_run=dry_run,
        debug_recognition=debug_recognition,
        # 姿态参数 (从顶部常量透传)
        pose_p_y_mm=pose_p_y_mm,
        pose_p_x_mm=pose_p_x_mm,
        pose_p_arm_deg=pose_p_arm_deg,
        pose_p_hand_deg=pose_p_hand_deg,
        pick_x_mm=pick_x_mm,
        pick_y_mm=pick_y_mm,
        transit_y_mm=transit_y_mm,
        transit_x_mm=transit_x_mm,
        put_y_mm=put_y_mm,
        bin_x_blue_mm=bin_x_blue_mm,
        bin_x_yellow_mm=bin_x_yellow_mm,
        defer_task5_handoff=defer_task5_handoff,
    )

    ok = bool(detail.get("ok")) if isinstance(detail, dict) else bool(detail)
    return {"ok": ok, "task": "task4_harvest", "detail": detail}


# ---- CLI (方便 python -m main.task.task4_harvest 快速调试) ----

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task4_harvest: 作物抓取 (薄封装, 参数全透传)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--max-picks", type=int, default=DEFAULT_MAX_PICKS,
                   help="最多抓取数")
    p.add_argument("--max-seconds", type=float, default=DEFAULT_MAX_SECONDS,
                   help="任务总时长预算 (s)")
    p.add_argument("--max-creep-m", type=float, default=DEFAULT_MAX_CREEP_M,
                   help="累计前移距离预算 (m)")
    p.add_argument("--creep-speed", type=float, default=DEFAULT_CREEP_SPEED_MPS,
                   help="creep 前移速度 (m/s), 第一球用此值, 后续减半")
    p.add_argument("--track-max-seconds", type=float, default=DEFAULT_TRACK_MAX_SECONDS,
                   help="单球底盘视觉伺服收敛预算 (s)")
    p.add_argument("--max-consecutive-pick-failures", type=int,
                   default=DEFAULT_MAX_CONSECUTIVE_PICK_FAILURES,
                   help="连续 pick 失败容忍 (命中即退出)")
    p.add_argument("--return-x", type=float, default=DEFAULT_RETURN_X_MM,
                   help=f"放 bin 后 x 回位 (mm); {DEFAULT_RETURN_X_MM} = P 姿态 x; None = 不回 (传 --no-return)")
    p.add_argument("--no-return", action="store_true",
                   help="放 bin 后 x 不回位")
    p.add_argument("--pick-timeout", type=float, default=DEFAULT_PICK_TIMEOUT_S,
                   help="pick_by_vision 总超时 (s)")
    p.add_argument("--do-prep", action="store_true",
                   help="开头跑 target1 准备位姿 (当前 target4 已删, 保留兼容)")
    p.add_argument("--dry-run", action="store_true",
                   help="dry-run 模式 (只打印不动硬件)")
    p.add_argument("--debug-recognition", action="store_true",
                   help="fetch_balls 打印每条检测的过滤原因")
    return p


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)

    if args.no_return:
        return_x_mm: Optional[float] = None
    else:
        return_x_mm = args.return_x

    t0 = time.monotonic()
    result = run(
        max_picks=args.max_picks,
        max_seconds=args.max_seconds,
        max_creep_m=args.max_creep_m,
        creep_speed_mps=args.creep_speed,
        track_max_seconds=args.track_max_seconds,
        max_consecutive_pick_failures=args.max_consecutive_pick_failures,
        return_x_mm=return_x_mm,
        pick_timeout_s=args.pick_timeout,
        do_prep=args.do_prep,
        dry_run=args.dry_run,
        debug_recognition=args.debug_recognition,
    )
    elapsed = time.monotonic() - t0

    print(f"\n[{__name__}] 完成: ok={result['ok']}  elapsed={elapsed:.1f}s")
    print(f"  detail: {result['detail']}")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
