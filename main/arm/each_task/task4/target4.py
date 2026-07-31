#!/usr/bin/python3
"""task4 / target4 —— target1 起手 + 循环 7 次 (底盘前移 80mm → 识别 → 抓球)。

2026-07-31 重写 (替代 v7 状态机, 用户原话 "识别到就停"):
  旧 v7: 二段式状态机 (SEARCH → found) + 复杂 x 扫描 + ADJUST PD 闭环 + 闪烁缓冲
  新版: 简单线性循环, 每轮 1 次 chassis 前移 + 1 次识别 + 0/1 次抓球

流程 (用户 2026-07-31 指定):
  1. target1.step_target1(client, runner)  —— 摆到 target1 位姿 (y=-133, x=-260)
  2. 初始识别 (站在 target1 位姿上, 不前移) + 抓球 (蓝→pick_up_blue / 黄→pick_up_yellow)
  3. 重复 7 次: 底盘前移 80mm + 识别 + 抓球 (无球则跳过)
  4. 返回 summary

总计: 1× target1 + 1× 初始识别/抓球 + 7× (前移 + 识别 + 抓球)
     = 1× target1 + 7× chassis 前移 + 8× 识别 + 8× 抓球-or-跳过

⚠️ 硬约束 (跟旧 v7 一致, 不动舵机 / y / hand / grasp / storage, 只用业务层现成封装):
  - 不调 move_y / set_arm_angle / set_hand_angle / grasp / set_storage
  - x 移动由 pick_up_*.py 自己负责 (return_x_mm 默认回 -260)
  - 底盘走 task4.dipan.step_chassis_forward (HTTP car action "move_for")
  - 走完 try/finally 必须 stop_wheel_speeds (但本脚本只下 move_for, 不开连续 set_wheel_speeds,
    兜底仍调一次确保清场)

⚠️ 抓球策略:
  - 一次 fetch_balls 可能返回多球 (蓝 + 黄并存罕见但理论可能)
  - 选 **score 最高** 的 1 球 (用户隐含语义: "如果有球", 单数, 不并行抓两个)
  - 颜色 unknown / 无球 → 跳过本轮, 但前移仍执行 (按用户 spec)

⚠️ 抓球异常处理:
  - 抓球失败 (move_x stall / grasp 异常) → log + 继续下一轮, 不让一次失败
    把整轮 7 次循环打掉。失败次数 > tolerance 时建议人工介入, 但本脚本不强制 stop。
  - 底盘前移失败 (move_for 异常) → log + break 退出循环 (后续识别无意义, 车没动)

CLI 跑法 (默认 execute; --dry-run 仅打印):
    # 1. 真跑 7 轮 (默认)
    python -m main.arm.each_task.task4.target4
    # 2. dry-run: 不动硬件, 只 print 每轮动作
    python -m main.arm.each_task.task4.target4 --dry-run
    # 3. 改底盘前移距离 (调试用)
    python -m main.arm.each_task.task4.target4 --chassis-step-mm 50
    # 4. 改循环次数
    python -m main.arm.each_task.task4.target4 --rounds 3
    # 5. 跳过 target1 起手 (假设已在 target1 位姿)
    python -m main.arm.each_task.task4.target4 --no-prep
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Callable, Optional

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

# ---- task4 内部模块 ----
try:  # noqa: E402
    from . import target1, target2  # noqa: E402
    from . import pick_up_blue, pick_up_yellow  # noqa: E402
    from . import dipan as _dipan  # noqa: E402
    from .constants import (  # noqa: E402
        LOG_PREFIX_TASK4,
        COLOR_BLUE, COLOR_YELLOW,
    )
except ImportError:  # pragma: no cover —— 直接 python target4.py 时无包上下文
    from main.arm.each_task.task4 import (  # type: ignore # noqa: E402
        target1, target2, pick_up_blue, pick_up_yellow, dipan as _dipan,
    )
    from main.arm.each_task.task4.constants import (  # type: ignore # noqa: E402
        LOG_PREFIX_TASK4, COLOR_BLUE, COLOR_YELLOW,
    )

from main.arm import ArmClient, ArmRunner  # noqa: E402


LOG_PREFIX: str = LOG_PREFIX_TASK4 + "/target4"


# ---- 默认参数 ----
DEFAULT_ROUNDS: int = 7
"""循环次数: 重复 (前移 + 识别 + 抓球) 几次 (用户 2026-07-31 指定 7)。"""

DEFAULT_CHASSIS_STEP_MM: float = 80.0
"""每轮底盘前移距离 (mm, 用户 2026-07-31 指定 80)。"""

DEFAULT_RETURN_X_MM: Optional[float] = -260.0
"""抓球后 x 回的目标位置 (mm), 走 trust 模式 (绝对位置指令)。
   默认 -260 = target1 抓取位, 业务流推荐 (target1→pick→target1→pick 循环)。
   改 None = 不回 (给手动多轮跑但下一阶段不依赖 x 的场景)。"""

DEFAULT_PICK_TIMEOUT_S: float = 60.0
"""单次 pick_up 序列 HTTP 总超时 (兜底, 正常 9 步序列 ~10-20s)。
   当前未透传给 pick_up_*.py (它们内部 move_y/grasp 各自有 timeout), 仅作未来扩展位。"""

DEFAULT_PICK_ERROR_TOLERANCE: int = 99
"""连续 pick 失败超过此数 → break (默认 99 = 不强制 stop, 让业务层自己决定何时停)。
   7 轮中失败 1-2 次正常 (现场 stall / 噪声), 不应让 1 次失败打掉整轮。"""


# ---------- 内部 helper ----------

def _pick_best_ball(balls: list[dict]) -> Optional[dict]:
    """从 fetch_balls 返回的球列表中选 1 球: score 最高, 平局取第一个。

    Args:
        balls: list[dict], 字段见 target2.fetch_balls docstring (color / score / ...)

    Returns:
        选中的 ball dict, 或 None (balls 为空 / 全是 unknown / 全部 score=None)。
    """
    candidates = [b for b in balls if b.get("color") in (COLOR_BLUE, COLOR_YELLOW)]
    if not candidates:
        return None
    return max(candidates, key=lambda b: float(b.get("score", 0.0)))


def _identify_and_pick(
    arm_client: ArmClient,
    http_client,
    runner: ArmRunner,
    round_idx: int,
    *,
    return_x_mm: Optional[float],
    verify_target1_pose: bool,
    pick_timeout_s: float,
    dry_run: bool,
) -> dict:
    """单轮 "识别 + 抓球" 序列。

    1. fetch_balls (调 task_feed 守护线程, 走 realtime 不抢 car_lock)
    2. 选 score 最高的 1 球 (蓝/黄)
    3. 蓝色 → pick_up_blue; 黄色 → pick_up_yellow; 无球 → skip
    4. 抓球失败 → log + 返 result["ok"]=False, **不抛** (让外层循环继续)

    Args:
        round_idx: 当前轮次 (用于日志, 0 = 初始轮)
        其他: 见 step_target4 docstring

    Returns:
        dict:
          - ok: bool (成功抓到球 / 跳过无球 都算 ok, 抓球失败才 False)
          - ball: dict | None
          - color: str | None ("blue" / "yellow" / None)
          - action: str ("picked" / "skipped_no_ball" / "skipped_unknown_color"
                       / "pick_failed" / "dry_run")
          - pick_result: dict | None (pick_up_*.py 的返值, 失败时 None)
          - error: str | None
    """
    phase = "初始" if round_idx == 0 else f"第{round_idx}轮"
    print(f"\n--- [{LOG_PREFIX}] {phase}: 识别 + 抓球 ---")

    # 1. 视觉检测
    try:
        balls = target2.fetch_balls(
            http_client,
            color_filter=None,                # 不按颜色过滤, 选 score 最高时再判
            verify_target1_pose=verify_target1_pose,
        )
    except Exception as e:
        print(f"  [{LOG_PREFIX}] ⚠️ fetch_balls 异常: "
              f"{type(e).__name__}: {str(e)[:80]}; 视作无球")
        balls = []

    print(f"  [{LOG_PREFIX}] fetch_balls 返回 {len(balls)} 球")

    # 2. 选球
    best = _pick_best_ball(balls)
    if best is None:
        print(f"  [{LOG_PREFIX}] ❌ 无蓝/黄球, 跳过本轮抓球")
        return {
            "ok": True,
            "ball": None,
            "color": None,
            "action": "skipped_no_ball",
            "pick_result": None,
            "error": None,
        }

    color = best["color"]
    score = float(best.get("score", 0.0))
    cx = float(best.get("cx_norm", 0.0))
    cy = float(best.get("cy_norm", 0.0))
    print(f"  [{LOG_PREFIX}] ✓ 选中 {color} 球  "
          f"score={score:.3f}  cx={cx:+.3f}  cy={cy:+.3f}")

    # 3. dry-run 短路
    if dry_run:
        pick_label_dry = ('pick_up_blue' if color == COLOR_BLUE
                          else 'pick_up_yellow')
        print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过实际抓球 "
              f"(would call {pick_label_dry})")
        return {
            "ok": True,
            "ball": best,
            "color": color,
            "action": "dry_run",
            "pick_result": None,
            "error": None,
        }

    # 4. 选 pick_up_*.py
    if color == COLOR_BLUE:
        pick_fn: Callable = pick_up_blue.step_pick_up_blue
        pick_label = "pick_up_blue"
    elif color == COLOR_YELLOW:
        pick_fn = pick_up_yellow.step_pick_up_yellow
        pick_label = "pick_up_yellow"
    else:
        # 兜底 (理论上 _pick_best_ball 已经过滤了)
        print(f"  [{LOG_PREFIX}] ❌ 颜色 {color!r} 既不是蓝也不是黄, 跳过")
        return {
            "ok": True,
            "ball": best,
            "color": color,
            "action": "skipped_unknown_color",
            "pick_result": None,
            "error": None,
        }

    # 5. 执行抓球
    print(f"  [{LOG_PREFIX}] 调 {pick_label}.step_pick_up_{color}("
          f"return_x_mm={return_x_mm})")
    try:
        pick_result = pick_fn(
            arm_client, runner,
            return_x_mm=return_x_mm,
        )
        # pick_up_*.py 内部异常会自己抛; 这里仅做 best-effort 包装
        # 注意: pick_up_*.py 当前不传 timeout 参数 (内部 move_y/grasp 各自有 timeout),
        # 这里的 pick_timeout_s 仅作未来扩展位
        _ = pick_timeout_s
    except Exception as e:
        err = f"{type(e).__name__}: {str(e)[:120]}"
        print(f"  [{LOG_PREFIX}] ❌ {pick_label} 失败: {err}")
        return {
            "ok": False,
            "ball": best,
            "color": color,
            "action": "pick_failed",
            "pick_result": None,
            "error": err,
        }

    print(f"  [{LOG_PREFIX}] ✅ {pick_label} 完成")
    return {
        "ok": True,
        "ball": best,
        "color": color,
        "action": "picked",
        "pick_result": pick_result,
        "error": None,
    }


# ---------- 核心 step ----------

def step_target4(
    arm_client: ArmClient,
    http_client,
    *,
    runner: Optional[ArmRunner] = None,
    rounds: int = DEFAULT_ROUNDS,
    chassis_step_mm: float = DEFAULT_CHASSIS_STEP_MM,
    return_x_mm: Optional[float] = DEFAULT_RETURN_X_MM,
    verify_target1_pose: bool = True,
    pick_timeout_s: float = DEFAULT_PICK_TIMEOUT_S,
    pick_error_tolerance: int = DEFAULT_PICK_ERROR_TOLERANCE,
    do_prep: bool = True,
    dry_run: bool = False,
) -> dict:
    """target1 起手 + 循环 (前移 + 识别 + 抓球)。

    Args:
        arm_client: ArmClient 实例 (调 move_x / grasp / _read_x_mm_realtime)。
        http_client: RuntimeApiClient (给 target2.fetch_balls + dipan + pick_up_*.py 共用)。
        runner: ArmRunner (None 时自动建一个)。
        rounds: 循环次数, 每轮 = (chassis 前移 + 识别 + 抓球), 默认 7 (用户指定)。
        chassis_step_mm: 底盘前移距离 (mm, 正数 = 前进), 默认 80 (用户指定)。
        return_x_mm: 抓球后 x 回的目标位置 (mm), 走 trust 模式。默认 -260。
                     None = 不回 (v5 兼容)。
        verify_target1_pose: fetch_balls 是否用 BALL_VERIFIED_* 7 项验证 (target1 位姿下)。
                            本流程始终在 target1 位姿上 (抓球后 y/x 自动回), 建议 True。
        pick_timeout_s: 单次 pick_up 序列超时 (秒, 兜底用, 当前未透传)。
        pick_error_tolerance: 连续 pick 失败超过此数 → break (默认 99 = 不强制停)。
        do_prep: True (默认) 开头跑 target1.step_target1; False 跳过 (假设已在 target1 位姿)。
        dry_run: True 只 print 不动硬件。

    Returns:
        dict:
          - ok: bool (整体成功, 中途 chassis 异常 → False)
          - rounds_done: int (实际完成轮数, 含初始轮)
          - rounds_planned: int (= rounds, 不含初始)
          - picks: int (成功抓球次数)
          - skips: int (无球跳过次数)
          - pick_failures: int (抓球失败次数)
          - history: list[dict] (每轮的 _identify_and_pick 返值; 索引 0 = 初始轮)
          - reason: str (退出原因: "completed" / "chassis_error" / "pick_error_exceeded"
                       / "keyboard_interrupt")
    """
    print(f"\n========== {LOG_PREFIX} step_target4 ==========")
    print(f"  模式: {'DRY-RUN (不动硬件)' if dry_run else 'EXECUTE (动硬件)'}")
    print(f"  循环: {rounds} 轮 × (前移 {chassis_step_mm:.0f}mm + 识别 + 抓球) "
          f"+ 1 初始识别/抓球 (在 target1 位姿上)")
    print(f"  抓球后 x 回: {return_x_mm} mm (None = 不回)")
    print(f"  准备位姿: {'跑 target1' if do_prep and not dry_run else '跳过'}")

    if rounds < 0:
        raise ValueError(f"rounds 必须 ≥ 0, 收到: {rounds}")
    if chassis_step_mm < 0:
        raise ValueError(f"chassis_step_mm 必须 ≥ 0, 收到: {chassis_step_mm}")

    # ---- 0. 准备 runner ----
    if runner is None:
        runner = ArmRunner(arm_client)

    history: list[dict] = []
    n_picks = 0
    n_skips = 0
    n_pick_failures = 0
    n_consecutive_pick_failures = 0
    final_reason: str = "unknown"
    t_start = time.monotonic()

    try:
        # ---- 1. 准备位姿 (target1) ----
        if do_prep and not dry_run:
            print(f"\n--- [{LOG_PREFIX}] 准备位姿: target1 ---")
            prep_res = target1.step_target1(arm_client, runner)
            print(f"  [{LOG_PREFIX}] target1 完成: {prep_res}")
        elif do_prep and dry_run:
            print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过 target1 (避免误触硬件)")

        # ---- 2. 初始识别 + 抓球 (在 target1 位姿上, 不前移) ----
        init_res = _identify_and_pick(
            arm_client, http_client, runner,
            round_idx=0,
            return_x_mm=return_x_mm,
            verify_target1_pose=verify_target1_pose,
            pick_timeout_s=pick_timeout_s,
            dry_run=dry_run,
        )
        history.append(init_res)
        if init_res["action"] == "picked":
            n_picks += 1
            n_consecutive_pick_failures = 0
        elif init_res["action"] == "pick_failed":
            n_pick_failures += 1
            n_consecutive_pick_failures += 1
        else:
            n_skips += 1
            n_consecutive_pick_failures = 0

        # ---- 3. 主循环: rounds 轮 (前移 + 识别 + 抓球) ----
        for r in range(1, rounds + 1):
            elapsed = time.monotonic() - t_start
            print(f"\n========== [{LOG_PREFIX}] 第 {r}/{rounds} 轮 "
                  f"(t={elapsed:.1f}s) ==========")

            # 3.1 底盘前移
            if not dry_run:
                print(f"  [{LOG_PREFIX}] 底盘前移 {chassis_step_mm:.0f}mm")
                try:
                    _dipan.step_chassis_forward(
                        http_client,
                        distance_mm=chassis_step_mm,
                        timeout_s=30.0,
                    )
                except Exception as e:
                    final_reason = "chassis_error"
                    print(f"  [{LOG_PREFIX}] ❌ 底盘前移失败 "
                          f"({type(e).__name__}: {str(e)[:80]}); 退出循环")
                    break
            else:
                print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过底盘前移 "
                      f"(would move_for({chassis_step_mm/1000:.3f}, 0, 0))")

            # 3.2 识别 + 抓球
            round_res = _identify_and_pick(
                arm_client, http_client, runner,
                round_idx=r,
                return_x_mm=return_x_mm,
                verify_target1_pose=verify_target1_pose,
                pick_timeout_s=pick_timeout_s,
                dry_run=dry_run,
            )
            history.append(round_res)
            if round_res["action"] == "picked":
                n_picks += 1
                n_consecutive_pick_failures = 0
            elif round_res["action"] == "pick_failed":
                n_pick_failures += 1
                n_consecutive_pick_failures += 1
            else:
                n_skips += 1
                n_consecutive_pick_failures = 0

            # 3.3 连续 pick 失败兜底
            if n_consecutive_pick_failures > pick_error_tolerance:
                final_reason = "pick_error_exceeded"
                print(f"  [{LOG_PREFIX}] ❌ 连续 {n_consecutive_pick_failures} 轮 "
                      f"pick 失败, 超过 tolerance={pick_error_tolerance}, 退出")
                break

        else:
            # for-else: 循环正常跑完 (没 break)
            final_reason = "completed"

    except KeyboardInterrupt:
        final_reason = "keyboard_interrupt"
        print(f"\n  [{LOG_PREFIX}] Ctrl-C 中断")
    finally:
        # 兜底: stop_wheel_speeds (move_for 一次性闭环, 通常车端会自动停, 但保险起见)
        if not dry_run:
            try:
                _dipan._stop_chassis_quietly(http_client)
            except Exception:
                pass

    elapsed = time.monotonic() - t_start
    print(f"\n========== {LOG_PREFIX} 完成 ==========")
    print(f"  reason={final_reason}  rounds_done={len(history)-1}/{rounds}  "
          f"picks={n_picks}  skips={n_skips}  pick_failures={n_pick_failures}  "
          f"elapsed={elapsed:.1f}s")

    return {
        "ok": final_reason in ("completed", "keyboard_interrupt"),
        "rounds_done": max(0, len(history) - 1),   # 减去初始轮
        "rounds_planned": rounds,
        "picks": n_picks,
        "skips": n_skips,
        "pick_failures": n_pick_failures,
        "history": history,
        "reason": final_reason,
        "elapsed_s": elapsed,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task4 target4: target1 起手 + 循环 (前移 + 识别 + 抓球) "
                    "(默认 7 轮 × 80mm 前移, --dry-run 只打印)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--dry-run", action="store_true",
                   help="dry-run 模式 (默认 execute, 真动硬件; "
                        "加此参数只打印不动)")
    p.add_argument("--rounds", type=int, default=DEFAULT_ROUNDS,
                   help=f"循环次数 (每轮 = 前移 + 识别 + 抓球), 默认 {DEFAULT_ROUNDS}")
    p.add_argument("--chassis-step-mm", type=float, default=DEFAULT_CHASSIS_STEP_MM,
                   help=f"每轮底盘前移距离 (mm), 默认 {DEFAULT_CHASSIS_STEP_MM}")
    p.add_argument("--return-x", dest="return_x", type=float, default=None,
                   help=f"抓球后 x 回的目标位置 (mm, 绝对位置指令)。"
                        f" 默认 {DEFAULT_RETURN_X_MM} (target1 抓取位)。"
                        f" 设 0=撞墙; 跟 --no-return 互斥。")
    p.add_argument("--no-return", dest="no_return", action="store_true",
                   help="抓球后不回 x (v5 行为兼容)")
    p.add_argument("--no-verify-pose", dest="verify_target1_pose",
                   action="store_false", default=True,
                   help="fetch_balls 不做 BALL_VERIFIED_* 验证 (调试噪声框用)")
    p.add_argument("--no-prep", dest="no_prep", action="store_true",
                   help="跳过开头 target1.step_target1 (假设已在 target1 位姿)")
    p.add_argument("--pick-error-tolerance", type=int,
                   default=DEFAULT_PICK_ERROR_TOLERANCE,
                   help=f"连续 pick 失败超过此数 → 退出循环 (默认 "
                        f"{DEFAULT_PICK_ERROR_TOLERANCE} = 不强制停)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    # CLI → return_x_mm 转换
    if args.no_return:
        return_x_mm: Optional[float] = None
    elif args.return_x is not None:
        return_x_mm = float(args.return_x)
    else:
        return_x_mm = DEFAULT_RETURN_X_MM   # -260.0 默认

    from main.api_client import RuntimeApiClient  # noqa: E402
    http = RuntimeApiClient()
    arm = ArmClient.connect()
    runner = ArmRunner(arm)

    result = step_target4(
        arm, http,
        runner=runner,
        rounds=args.rounds,
        chassis_step_mm=args.chassis_step_mm,
        return_x_mm=return_x_mm,
        verify_target1_pose=args.verify_target1_pose,
        pick_error_tolerance=args.pick_error_tolerance,
        do_prep=not args.no_prep,
        dry_run=args.dry_run,
    )

    print(f"\n[{LOG_PREFIX}] 最终结果: {result}")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())