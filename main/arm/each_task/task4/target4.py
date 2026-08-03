#!/usr/bin/python3
"""task4 / target4 —— 慢速前移搜索 + 底盘视觉定位 (最左球) + 吸嘴中心抓取。

2026-08-03 二次重写 (用户指定 "新方法", 替代离散底盘控制):
  旧版 (v7/P1): 离散 80mm 步进 × 7 轮开环 → 识别 → 抓球, 底盘定位无反馈,
               只在车子理想到位时成立。
  新版: 到任务点后持续慢速前移扫球; 见到球即用底盘视觉伺服把**画面最左**
        那颗球拉到画面中心 (track_chassis, chassis 两速度 DOF vx/vy,
        2026-08-02 现场标定参数); 然后 pick_by_vision 臂视觉伺服 (吸嘴中心
        对准球, setpoint 自动读 origin 标定) → 吸气 → 按颜色放 bin
        (复用 P1 的 composite_run 并行序列)。预算式收尾: 累计前移距离 /
        总时长 / 最大抓取数 任一命中即结束。

流程:
  1. target1.step_target1(client, runner)  —— 识别/抓取准备位姿 (y=-133, x=-260, arm=90°, hand=0°)
  2. 循环 (直到预算耗尽):
       ① creep 搜索    /v1/realtime/chassis-velocity 慢速前移, 10Hz 轮询 fetch_balls, 见到即停
       ② track_chassis  select_mode="leftmost", 把最左球拉到画面中心
       ③ pick_by_vision 吸嘴中心对准球 (粗定位 → PID 精调 → composite_pick → 吸)
       ④ 放 bin        composite_run (抬 y=-190 ∥ 移 bin) → 降 y=-155 → 放气 → 回识别位姿
  3. 返回 summary

终止条件 (任一命中):
  - 累计前移 ≥ max_creep_m (默认 0.8m; 旧版总行程 7×80mm=0.56m + 余量)
  - picks ≥ max_picks (默认 8; 比赛正常 6-8 球)
  - 总耗时 ≥ max_seconds (默认 180s)
  - 连续 pick 失败 ≥ max_consecutive_pick_failures (默认 3)
  - Ctrl-C

⚠️ 底盘通道: /v1/realtime/chassis-velocity (realtime 门, 免 job_queue, 与
   track_chassis 同通道); 不再用 move_for 离散步进。orchestrator 派发 task4
   前已暂停 lane 外环, 不冲突。
⚠️ 视觉伺服前置: 跑本脚本前必须先停 arm_feed (20Hz 轮询会饿 arm_queue,
   pick_by_vision 走位置环); 跑完恢复。
⚠️ 抓取异常: 单次失败只 log + 恢复搜索, 连续失败超容忍才退出;
   try/finally 底盘速度清零 + stop_wheel_speeds 兜底。

CLI 跑法:
    python -m main.arm.each_task.task4.target4                    # 真跑 (默认预算)
    python -m main.arm.each_task.task4.target4 --dry-run          # 只打印不动硬件
    python -m main.arm.each_task.task4.target4 --max-picks 3      # 最多抓 3 个
    python -m main.arm.each_task.task4.target4 --creep-speed 0.02 --max-creep-m 0.6
    python -m main.arm.each_task.task4.target4 --no-prep          # 跳过 target1 起手 (已在位姿)
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

# ---- task4 内部模块 ----
try:  # noqa: E402
    from . import target1, target2  # noqa: E402
    from . import dipan as _dipan  # noqa: E402
    from .constants import (  # noqa: E402
        LOG_PREFIX_TASK4,
        COLOR_BLUE, COLOR_YELLOW,
    )
except ImportError:  # pragma: no cover —— 直接 python target4.py 时无包上下文
    from main.arm.each_task.task4 import (  # type: ignore # noqa: E402
        target1, target2, dipan as _dipan,
    )
    from main.arm.each_task.task4.constants import (  # type: ignore # noqa: E402
        LOG_PREFIX_TASK4, COLOR_BLUE, COLOR_YELLOW,
    )

from main.arm import (  # noqa: E402
    ArmClient, ArmRunner, TargetSelector, SelectionStrategy,
)
from main.chassis import track_chassis  # noqa: E402


LOG_PREFIX: str = LOG_PREFIX_TASK4 + "/target4"


# ---- 默认参数 (2026-08-03 底盘视觉伺服版) ----

DEFAULT_MAX_PICKS: int = 8
"""最多抓取数 (比赛正常 6-8 球, 给 buffer)。"""

DEFAULT_CREEP_SPEED_MPS: float = 0.045
"""creep 搜索前移速度 (m/s)。慢 = 帧覆盖密, 不漏球; 快 = 省时间。
2026-08-03 用户现场反馈 0.03 太慢, ×1.5 → 0.045。"""

DEFAULT_MAX_CREEP_M: float = 0.8
"""累计前移距离预算 (m, 开环 速度×时间 记账)。旧版总行程 0.56m + 余量。"""

DEFAULT_MAX_SECONDS: float = 180.0
"""任务总时长预算 (s)。"""

DEFAULT_TRACK_MAX_SECONDS: float = 12.0
"""单球底盘视觉伺服收敛预算 (s)。超时但目标仍在画面 → 照样试抓 (臂伺服补残差)。"""

DEFAULT_MAX_CONSECUTIVE_PICK_FAILURES: int = 3
"""连续 pick 失败超过此数 → 退出 (单个球抓不起不应拖垮全场)。"""

DEFAULT_RETURN_X_MM: Optional[float] = -260.0
"""放 bin 后 x 回的目标位置 (mm)。默认 -260 = target1 抓取位。None = 不回。"""

DEFAULT_PICK_TIMEOUT_S: float = 60.0
"""pick_by_vision 总超时 (s)。"""

CREEP_POLL_HZ: float = 10.0
"""creep 期间 fetch_balls 轮询频率。"""

# ---- 放 bin 参数 (沿用 P1 版, 现场已调) ----

BIN_X_MM = {COLOR_BLUE: 0.0, COLOR_YELLOW: -65.0}
"""蓝 bin x=0, 黄 bin x=-65。"""

Y_PICK_MM: float = -58.0
"""抓球 y (吸盘贴近球面)。"""

Y_TRANSIT_MM: float = -190.0
"""中转 y (持球抬升, 出 gate)。"""

Y_PUT_MM: float = -155.0
"""放球 y (命中开仓 y gate 上界)。"""

Y_FINAL_MM: float = -133.0
"""最终 y (识别位姿)。"""

BALL_LABELS = ["ball_blue", "ball_yellow"]
"""track_chassis 目标集 (PaddleDet 模型标签)。"""


# ---------- 底盘速度 helper ----------

def _set_chassis_vel(http_client, vx: float, vy: float = 0.0) -> None:
    """下一次 chassis 速度 (realtime 门, 与 track_chassis 同通道)。

    异常只 warn 不抛 —— creep 是搜索阶段, 单次下发失败下一帧自愈。
    """
    try:
        http_client.post(
            "/v1/realtime/chassis-velocity",
            {"vx": float(vx), "vy": float(vy), "wz": 0.0},
            timeout=1.5,
        )
    except Exception as e:
        print(f"  [{LOG_PREFIX}] ⚠️ chassis 速度下发失败 "
              f"({type(e).__name__}: {str(e)[:60]})")


def _creep_search(
    http_client,
    *,
    speed_mps: float,
    max_distance_m: float,
    max_seconds_s: float,
    dry_run: bool = False,
    debug: bool = False,
) -> dict:
    """慢速前移 + 轮询 fetch_balls; 见到蓝/黄球立即停车返回。

    Args:
        speed_mps: 前移速度 (m/s)。
        max_distance_m: 剩余前移预算 (m, 开环记账)。
        max_seconds_s: 本段 creep 时间上限 (s)。
        dry_run: True 不下发速度, 但仍轮询视觉 (流程排练)。
        debug: 透传 fetch_balls 打印过滤原因。

    Returns:
        {"balls": list|None,    # 见到球时的 fetch_balls 全量返回; None=预算耗尽没见球
         "distance_m": float,   # 实际累计前移 (速度×时间)
         "elapsed_s": float}
    """
    period = 1.0 / CREEP_POLL_HZ
    t0 = time.monotonic()
    dist = 0.0
    balls_seen: Optional[list] = None
    try:
        while (time.monotonic() - t0) < max_seconds_s and dist < max_distance_m:
            if not dry_run:
                _set_chassis_vel(http_client, speed_mps)
            time.sleep(period)
            dist += speed_mps * period
            try:
                balls = target2.fetch_balls(
                    http_client, color_filter=None, debug=debug,
                )
            except Exception as e:
                print(f"  [{LOG_PREFIX}] ⚠️ fetch_balls 异常: "
                      f"{type(e).__name__}: {str(e)[:80]}")
                continue
            if any(b.get("color") in (COLOR_BLUE, COLOR_YELLOW) for b in balls):
                balls_seen = balls
                break
    finally:
        if not dry_run:
            _set_chassis_vel(http_client, 0.0)
    return {
        "balls": balls_seen,
        "distance_m": dist,
        "elapsed_s": time.monotonic() - t0,
    }


def _track_leftmost_ball(*, max_seconds: float, dry_run: bool):
    """底盘视觉伺服: 把画面最左 (cx 最小) 的球拉到画面中心。

    走 main.chassis.track_chassis (2026-08-02 现场标定的 sign/kp/v_max/slew),
    内部 finally 自动零速。返回 TrackChassisResult
    (arrived / reason / final_frame.label=cx 最小的球 label)。
    """
    return track_chassis(
        target=BALL_LABELS,
        select_mode="leftmost",
        setpoint_cxcy=(0.0, 0.0),
        max_seconds=max_seconds,
        dry_run=dry_run,
    )


def _color_from_track(track_res) -> Optional[str]:
    """从 track 结果 final_frame.label 提球色 (ball_blue → "blue")。"""
    ff = getattr(track_res, "final_frame", None)
    label = getattr(ff, "label", None) if ff is not None else None
    if label in BALL_LABELS:
        return label.split("_", 1)[1]
    return None


def _pick_best_ball(balls: list) -> Optional[dict]:
    """从 fetch_balls 结果选 1 球: score 最高, 平局取第一个 (兜底判色用)。"""
    candidates = [b for b in balls if b.get("color") in (COLOR_BLUE, COLOR_YELLOW)]
    if not candidates:
        return None
    return max(candidates, key=lambda b: float(b.get("score", 0.0)))


def _pick_and_store(
    arm_client,
    runner: ArmRunner,
    *,
    color: str,
    return_x_mm: Optional[float],
    pick_timeout_s: float,
) -> dict:
    """pick_by_vision 吸嘴中心抓取 + composite_run 并行放 bin。

    Returns:
        {"ok": bool, "error": str|None}
    """
    label = f"ball_{color}"
    bin_x = BIN_X_MM[color]
    # 底盘已把球拉到画面中心, 臂伺服选最靠中心的那个 (残差交给 PID + nozzle setpoint)
    selector = TargetSelector.for_label(
        label, strategy=SelectionStrategy.CLOSEST_TO_CENTER.value,
    )
    print(f"  [{LOG_PREFIX}] move_to_vision_target(label={label!r}, x=-260, "
          f"y={Y_PICK_MM}, arm=90°, hand=0°) → composite_pick")
    try:
        # ⚠️ 不直接用 runner.pick_by_vision: 它粗定位阶段默认 hand=-90°(抬起),
        #    会把手爪抬进侧摄视野挡住球 → find_target 连续 5 帧未检出 abort
        #    (2026-08-03 现场实测)。这里拆成两步, 伺服阶段 hand=0° DOWN
        #    (与 prep/track 阶段一致, 球保持可见), 收敛后再 composite_pick。
        runner.move_to_vision_target(
            selector,
            x_mm=-260.0,
            y_mm=Y_PICK_MM,
            arm_angle=90.0,
            hand=0.0,
            settle_tol_norm=0.05,
            timeout=pick_timeout_s,
        )
        arm_client.composite_pick(
            arm_angle=90.0, x_mm=-260.0, y_mm=Y_PICK_MM,
            hand=0.0, speed=80, timeout=30.0,
        )
    except Exception as e:
        return {"ok": False,
                "error": f"pick_vision: {type(e).__name__}: {str(e)[:120]}"}

    try:
        # 抬升 + 横移到 bin 上方 (并行)
        arm_client.composite_run(y_mm=Y_TRANSIT_MM, x_mm=bin_x, speed=80, timeout=30.0)
        # 降到放球位 → 放气
        runner.move_y(Y_PUT_MM)
        runner.grasp(False, timeout=10.0)
        time.sleep(0.2)  # 球稳定掉进 bin
        # 回识别位姿 (y + x 并行)
        if return_x_mm is not None:
            arm_client.composite_run(
                y_mm=Y_FINAL_MM, x_mm=return_x_mm, speed=80, timeout=30.0,
            )
        else:
            runner.move_y(Y_FINAL_MM)
    except Exception as e:
        return {"ok": False,
                "error": f"store: {type(e).__name__}: {str(e)[:120]}"}
    return {"ok": True, "error": None}


# ---------- 核心 step ----------

def step_target4(
    arm_client: ArmClient,
    http_client,
    *,
    runner: Optional[ArmRunner] = None,
    max_picks: int = DEFAULT_MAX_PICKS,
    creep_speed_mps: float = DEFAULT_CREEP_SPEED_MPS,
    max_creep_m: float = DEFAULT_MAX_CREEP_M,
    max_seconds: float = DEFAULT_MAX_SECONDS,
    track_max_seconds: float = DEFAULT_TRACK_MAX_SECONDS,
    max_consecutive_pick_failures: int = DEFAULT_MAX_CONSECUTIVE_PICK_FAILURES,
    return_x_mm: Optional[float] = DEFAULT_RETURN_X_MM,
    pick_timeout_s: float = DEFAULT_PICK_TIMEOUT_S,
    do_prep: bool = True,
    dry_run: bool = False,
    debug_recognition: bool = False,
) -> dict:
    """慢速前移搜索 + 底盘视觉定位 (最左球) + 吸嘴中心抓取 + 放 bin。

    Args:
        arm_client: ArmClient 实例。
        http_client: RuntimeApiClient (creep 速度下发 / fetch_balls / track_chassis 共用)。
        runner: ArmRunner (None 时自动建)。
        max_picks: 最多抓取数, 默认 8。
        creep_speed_mps: creep 前移速度 (m/s), 默认 0.03。
        max_creep_m: 累计前移距离预算 (m), 默认 0.8。耗尽无球 → 视作采区走完。
        max_seconds: 任务总时长预算 (s), 默认 180。
        track_max_seconds: 单球底盘伺服收敛预算 (s), 默认 12。
        max_consecutive_pick_failures: 连续 pick 失败容忍, 默认 3。
        return_x_mm: 放 bin 后 x 回位 (mm), 默认 -260; None = 不回。
        pick_timeout_s: pick_by_vision 超时 (s), 默认 60。
        do_prep: True 开头跑 target1.step_target1 摆准备位姿。
        dry_run: True 不动硬件 (仍轮询视觉排练流程)。
        debug_recognition: fetch_balls 打印每条检测的过滤原因。

    Returns:
        dict:
          - ok: bool (completed / zone_cleared / time_budget / keyboard_interrupt 为 True)
          - picks / skips / pick_failures: 计数
          - total_creep_m: 累计前移距离
          - history: list[dict] 每球记录
          - reason: "completed" / "zone_cleared" / "time_budget" /
                    "pick_error_exceeded" / "keyboard_interrupt"
          - elapsed_s: 总耗时
    """
    print(f"\n========== {LOG_PREFIX} step_target4 (底盘视觉伺服版) ==========")
    print(f"  模式: {'DRY-RUN (不动硬件)' if dry_run else 'EXECUTE (动硬件)'}")
    print(f"  预算: 前移 ≤{max_creep_m:.2f}m @ {creep_speed_mps:.2f}m/s | "
          f"picks ≤{max_picks} | 总时 ≤{max_seconds:.0f}s")
    print(f"  单球底盘伺服预算: {track_max_seconds:.0f}s | 连续失败容忍: "
          f"{max_consecutive_pick_failures}")
    print(f"  放 bin 后 x 回: {return_x_mm} mm | 准备位姿: "
          f"{'跑 target1' if do_prep and not dry_run else '跳过'}")

    for name, val in (("max_picks", max_picks), ("max_seconds", max_seconds),
                      ("max_creep_m", max_creep_m), ("creep_speed_mps", creep_speed_mps),
                      ("track_max_seconds", track_max_seconds)):
        if val < 0:
            raise ValueError(f"{name} 必须 ≥ 0, 收到: {val}")

    if runner is None:
        runner = ArmRunner(arm_client)

    history: list = []
    n_picks = 0
    n_skips = 0
    n_pick_failures = 0
    n_consecutive_failures = 0
    total_creep_m = 0.0
    final_reason = "unknown"
    t_start = time.monotonic()

    try:
        # ---- 1. 准备位姿 (target1) ----
        if do_prep and not dry_run:
            print(f"\n--- [{LOG_PREFIX}] 准备位姿: target1 ---")
            prep_res = target1.step_target1(arm_client, runner)
            print(f"  [{LOG_PREFIX}] target1 完成: {prep_res}")
        elif do_prep and dry_run:
            print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过 target1 (避免误触硬件)")

        # ---- 2. 主循环: creep → track → pick → store ----
        ball_idx = 0
        while n_picks < max_picks:
            elapsed = time.monotonic() - t_start
            if elapsed >= max_seconds:
                final_reason = "time_budget"
                print(f"\n  [{LOG_PREFIX}] ⏱  总时长 {elapsed:.1f}s 达预算 "
                      f"{max_seconds:.0f}s, 收尾")
                break

            remaining_m = max_creep_m - total_creep_m
            if remaining_m <= 0.02:
                final_reason = "zone_cleared"
                print(f"\n  [{LOG_PREFIX}] 🏁 前移预算 {max_creep_m:.2f}m 耗尽, "
                      f"视作采区走完")
                break

            ball_idx += 1
            print(f"\n========== [{LOG_PREFIX}] 第 {ball_idx} 球 "
                  f"(t={elapsed:.1f}s, 已采 {n_picks}, "
                  f"剩余前移 {remaining_m:.2f}m) ==========")

            # 2.1 creep 搜索 (慢速前移直到见球)
            print(f"  [{LOG_PREFIX}] 🔍 creep 搜索 ({creep_speed_mps:.2f} m/s, "
                  f"≤{remaining_m:.2f}m)")
            creep = _creep_search(
                http_client,
                speed_mps=creep_speed_mps,
                max_distance_m=remaining_m,
                max_seconds_s=max(1.0, max_seconds - elapsed),
                dry_run=dry_run,
                debug=debug_recognition,
            )
            total_creep_m += creep["distance_m"]
            print(f"  [{LOG_PREFIX}] creep 结束: 前移 {creep['distance_m']:.3f}m "
                  f"/ {creep['elapsed_s']:.1f}s → "
                  f"{'见球' if creep['balls'] is not None else '未见球'}")
            if creep["balls"] is None:
                final_reason = "zone_cleared"
                print(f"  [{LOG_PREFIX}] 🏁 前移预算内未见球, 视作采区走完")
                break

            # 2.2 底盘视觉定位 (最左球 → 画面中心)
            print(f"  [{LOG_PREFIX}] 🎯 track_chassis(leftmost, "
                  f"≤{track_max_seconds:.0f}s)")
            track_res = _track_leftmost_ball(
                max_seconds=track_max_seconds, dry_run=dry_run,
            )
            print(f"  [{LOG_PREFIX}] track 结束: arrived={track_res.arrived} "
                  f"reason={track_res.reason}")

            # 2.3 定颜色 (track label 优先; 兜底再识别一帧)
            color = _color_from_track(track_res)
            if color is None and not dry_run:
                try:
                    balls = target2.fetch_balls(
                        http_client, color_filter=None, debug=debug_recognition,
                    )
                except Exception:
                    balls = []
                best = _pick_best_ball(balls)
                color = best["color"] if best else None
            if dry_run and color is None:
                color = COLOR_BLUE  # dry-run 无视觉时的占位, 走通流程
            if color not in (COLOR_BLUE, COLOR_YELLOW):
                n_skips += 1
                n_consecutive_failures = 0
                history.append({"ball": ball_idx, "action": "skipped_no_color",
                                "color": None, "error": None})
                print(f"  [{LOG_PREFIX}] ❌ 无法确定球色, 跳过继续搜索")
                continue
            print(f"  [{LOG_PREFIX}] ✓ 锁定 {color} 球, 智能抓取")

            # 2.4 抓取 + 放 bin
            if dry_run:
                print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过 pick_by_vision + 放 bin "
                      f"(would pick {color}, bin x={BIN_X_MM[color]})")
                history.append({"ball": ball_idx, "action": "dry_run",
                                "color": color, "error": None})
                n_picks += 1
                n_consecutive_failures = 0
                continue

            res = _pick_and_store(
                arm_client, runner,
                color=color,
                return_x_mm=return_x_mm,
                pick_timeout_s=pick_timeout_s,
            )
            history.append({"ball": ball_idx,
                            "action": "picked" if res["ok"] else "pick_failed",
                            "color": color, "error": res["error"]})
            if res["ok"]:
                n_picks += 1
                n_consecutive_failures = 0
                print(f"  [{LOG_PREFIX}] ✅ {color} 球完成 (累计 {n_picks})")
            else:
                n_pick_failures += 1
                n_consecutive_failures += 1
                print(f"  [{LOG_PREFIX}] ❌ 抓取失败 ({res['error']}); "
                      f"连续 {n_consecutive_failures} 次")
                if n_consecutive_failures > max_consecutive_pick_failures:
                    final_reason = "pick_error_exceeded"
                    print(f"  [{LOG_PREFIX}] ❌ 连续失败超过 "
                          f"{max_consecutive_pick_failures}, 退出")
                    break
        else:
            # while-else: 没 break = picks 达到 max_picks
            final_reason = "completed"

    except KeyboardInterrupt:
        final_reason = "keyboard_interrupt"
        print(f"\n  [{LOG_PREFIX}] Ctrl-C 中断")
    finally:
        # 兜底清场: 速度清零 + stop_wheel_speeds (track_chassis 自己会停, 这是保险)
        try:
            _set_chassis_vel(http_client, 0.0)
        except Exception:
            pass
        if not dry_run:
            try:
                _dipan._stop_chassis_quietly(http_client)
            except Exception:
                pass

    elapsed = time.monotonic() - t_start
    print(f"\n========== {LOG_PREFIX} 完成 ==========")
    print(f"  reason={final_reason}  picks={n_picks}/{max_picks}  "
          f"skips={n_skips}  pick_failures={n_pick_failures}  "
          f"前移={total_creep_m:.3f}m  elapsed={elapsed:.1f}s")

    return {
        "ok": final_reason in (
            "completed", "zone_cleared", "time_budget", "keyboard_interrupt",
        ),
        "picks": n_picks,
        "skips": n_skips,
        "pick_failures": n_pick_failures,
        "total_creep_m": total_creep_m,
        "history": history,
        "reason": final_reason,
        "elapsed_s": elapsed,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task4 target4: 慢速前移搜索 + 底盘视觉定位 (最左球) + "
                    "吸嘴中心抓取 (--dry-run 只打印)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--dry-run", action="store_true",
                   help="dry-run 模式 (默认 execute, 真动硬件)")
    p.add_argument("--max-picks", dest="max_picks", type=int,
                   default=DEFAULT_MAX_PICKS,
                   help="最多抓取数")
    p.add_argument("--creep-speed", dest="creep_speed", type=float,
                   default=DEFAULT_CREEP_SPEED_MPS,
                   help="creep 前移速度 (m/s)")
    p.add_argument("--max-creep-m", dest="max_creep_m", type=float,
                   default=DEFAULT_MAX_CREEP_M,
                   help="累计前移距离预算 (m), 耗尽无球=采区走完")
    p.add_argument("--max-seconds", dest="max_seconds", type=float,
                   default=DEFAULT_MAX_SECONDS,
                   help="任务总时长预算 (s)")
    p.add_argument("--track-max-seconds", dest="track_max_seconds", type=float,
                   default=DEFAULT_TRACK_MAX_SECONDS,
                   help="单球底盘视觉伺服收敛预算 (s)")
    p.add_argument("--max-consecutive-pick-failures",
                   dest="max_consecutive_pick_failures", type=int,
                   default=DEFAULT_MAX_CONSECUTIVE_PICK_FAILURES,
                   help="连续 pick 失败超过此数 → 退出")
    p.add_argument("--return-x", dest="return_x", type=float, default=None,
                   help=f"放 bin 后 x 回位 (mm), 默认 {DEFAULT_RETURN_X_MM}; "
                        f"跟 --no-return 互斥")
    p.add_argument("--no-return", dest="no_return", action="store_true",
                   help="放 bin 后不回 x")
    p.add_argument("--no-prep", dest="no_prep", action="store_true",
                   help="跳过开头 target1.step_target1 (假设已在准备位姿)")
    p.add_argument("--debug-recognition", dest="debug_recognition",
                   action="store_true", default=False,
                   help="fetch_balls 打印每条 detection 过滤原因")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.no_return:
        return_x_mm: Optional[float] = None
    elif args.return_x is not None:
        return_x_mm = float(args.return_x)
    else:
        return_x_mm = DEFAULT_RETURN_X_MM

    from main.api_client import RuntimeApiClient  # noqa: E402
    http = RuntimeApiClient()
    arm = ArmClient.connect()
    runner = ArmRunner(arm)

    result = step_target4(
        arm, http,
        runner=runner,
        max_picks=args.max_picks,
        creep_speed_mps=args.creep_speed,
        max_creep_m=args.max_creep_m,
        max_seconds=args.max_seconds,
        track_max_seconds=args.track_max_seconds,
        max_consecutive_pick_failures=args.max_consecutive_pick_failures,
        return_x_mm=return_x_mm,
        do_prep=not args.no_prep,
        dry_run=args.dry_run,
        debug_recognition=args.debug_recognition,
    )

    print(f"\n[{LOG_PREFIX}] 最终结果: {result}")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
