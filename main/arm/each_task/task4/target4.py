#!/usr/bin/python3
"""task4 / target4 —— 编排门面: 慢速前移搜索 + 机械臂智能抓取对齐 + 放 bin。

2026-08-10 拆分: 原 1599 行单文件按职责拆成 4 个单向依赖模块, 本文件保留
**编排骨架** (`step_target4` 主循环 + 预算/清场 + CLI), 业务实现下沉:
  - ``creep_thread.py``   后台保前移线程 + 跨球 IR 生命周期 + 底盘速度 helper
                          (`_CreepThread` / `_Task4SearchState` / `_set_chassis_vel`)
  - ``pick_store.py``     选球判色 + 机械臂智能抓取 + 放 bin
                          (`_pick_by_arm_servo` / `_pick_and_store` / `_pick_best_ball`)
  - ``constants.py``      所有位姿/预算/IR/蠕动/臂伺服常量 + 时间戳辅助 (校准表面)

对外符号全部 re-export 于此 (step_target4 / _CreepThread /
_Task4SearchState), 外部 import 无需改动。

流程 (2026-08-10 起用机械臂智能抓取, 替换底盘对齐):
  - 到任务点后持续慢速前移扫球 (creep, cam2 侧摄); 见到球即用**机械臂智能抓取**
    对齐 (track_velocity_pick: 大臂控 cx + x 十字控 cy, y 锁 0, 用户标定 setpoint),
    高位伺服收敛后 y 盲降 → 吸气 → 按颜色放 bin。
  - 用户拍板 (2026-08-10): 底盘对齐打滑不准 → 改臂伺服; setpoint 硬编码到
    constants.TASK4_SETPOINT_*; 两种球同尺寸共用一份; 高位伺服→最后盲降。
  - 预算式收尾: IR 丢失+0.3m 是主终止; 累计前移距离 / 总时长 / 最大抓取数兜底。
  - 终止条件 (任一): IR 离开区后走满 0.3m / 前移 ≥ max_creep_m / picks ≥ max_picks /
    总耗时 ≥ max_seconds / 连续 pick 失败 ≥ max_consecutive_pick_failures / Ctrl-C。

⚠️ 底盘通道: 仅 creep 用 /v1/realtime/chassis-velocity (realtime 门); 对齐在臂上
   (velocity 模式 /v1/realtime/arm-velocity), 底盘不打滑。orchestrator 派发
   task4 前已暂停 lane 外环, 不冲突。
⚠️ 视觉伺服前置: 跑本脚本前必须先停 arm_feed (20Hz 轮询会饿 arm_queue); 跑完恢复。

CLI 跑法:
    python -m main.arm.each_task.task4.target4                    # 真跑 (默认预算)
    python -m main.arm.each_task.task4.target4 --dry-run          # 只打印不动硬件
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

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.each_task.task4 import dipan as _dipan  # noqa: E402
from main.arm.each_task.task4.constants import (  # noqa: E402
    # 预算 / 终止
    DEFAULT_MAX_PICKS, DEFAULT_MAX_CREEP_M, DEFAULT_MAX_SECONDS,
    DEFAULT_CREEP_SPEED_MPS, DEFAULT_TRACK_MAX_SECONDS,
    DEFAULT_MAX_CONSECUTIVE_PICK_FAILURES,
    CREEP_POLL_HZ, CREEP_MAX_SECONDS_S,
    # 姿态 / 放仓
    COLOR_BLUE, COLOR_YELLOW, BIN_X_MM, BIN_Y_MM, BIN_HAND_DEG,
    TASK4_POSE_P_Y_MM, TASK4_POSE_P_X_MM, TASK4_POSE_P_ARM_DEG, TASK4_POSE_P_HAND_DEG,
    X_PICK_MM, Y_PICK_MM, Y_TRANSIT_MM, X_TRANSIT_MM, Y_PUT_MM,
    STORAGE_OPEN_ANGLE_DEG, STORAGE_CLOSE_ANGLE_DEG, STORAGE_OPEN_SPEED,
    # 时间戳辅助
    _ts_str, reset_ts,
    LOG_PREFIX_TARGET4,
)
from main.arm.each_task.task4.creep_thread import (  # noqa: E402
    _Task4SearchState, _CreepThread, _set_chassis_vel,
)
from main.arm.each_task.task4.pick_store import (  # noqa: E402
    _pick_best_ball, _pick_and_store,
)


LOG_PREFIX: str = LOG_PREFIX_TARGET4


# ---------- 核心 step ----------

def step_target4(
    arm_client: ArmClient,
    http_client,
    *,
    runner: Optional[ArmRunner] = None,
    defer_task5_handoff: bool = False,
    dry_run: bool = False,
    # ---- 姿态参数 (默认值在 constants.py, 可由外部覆盖) ----
    pose_p_y_mm: float = TASK4_POSE_P_Y_MM,
    pose_p_x_mm: float = TASK4_POSE_P_X_MM,
    pose_p_arm_deg: float = TASK4_POSE_P_ARM_DEG,
    pose_p_hand_deg: float = TASK4_POSE_P_HAND_DEG,
    pick_y_mm: float = Y_PICK_MM,  # 盲降抓球目标 y (臂伺服 grasp_y)
    transit_x_mm: float = X_TRANSIT_MM,
    bin_x_blue_mm: float = BIN_X_MM[COLOR_BLUE],
    bin_x_yellow_mm: float = BIN_X_MM[COLOR_YELLOW],
    bin_y_blue_mm: float = BIN_Y_MM.get(COLOR_BLUE, Y_PUT_MM),
    bin_y_yellow_mm: float = Y_PUT_MM,
    bin_hand_blue_deg: float = BIN_HAND_DEG.get(COLOR_BLUE, TASK4_POSE_P_HAND_DEG),
    bin_hand_yellow_deg: float = TASK4_POSE_P_HAND_DEG,
) -> dict:
    """慢速前移搜索 + 机械臂智能抓取 + 放 bin。

    Args:
        arm_client: ArmClient 实例。
        http_client: RuntimeApiClient (creep 速度下发 / fetch_balls 共用)。
        runner: ArmRunner (None 时自动建)。
        defer_task5_handoff: task4 由 orchestrator 调度时，IR+odom 结束后将
            关仓 + task5 Phase 1 姿态交给巡航线程并行执行；独立运行保持 False。
        dry_run: True 不动硬件 (仍轮询视觉排练流程)。
        姿态参数: 各轴目标 (mm / °), 默认值 = constants.py 的 task4 校准常量。

    Returns:
        dict:
          - ok: bool (completed / zone_cleared / time_budget / ir_odom_exit /
                keyboard_interrupt 为 True)
          - picks / skips / pick_failures: 计数
          - total_creep_m: 累计前移距离
          - history: list[dict] 每球记录
          - reason: "completed" / "zone_cleared" / "time_budget" /
                    "ir_odom_exit" / "pick_error_exceeded" / "keyboard_interrupt"
          - elapsed_s: 总耗时
    """
    # 预算 / 动作参数全部走 constants 模块默认值，只保留姿态可调 (2026-08-10 冻结)。
    max_picks = DEFAULT_MAX_PICKS
    max_seconds = DEFAULT_MAX_SECONDS
    max_creep_m = DEFAULT_MAX_CREEP_M
    creep_speed_mps = DEFAULT_CREEP_SPEED_MPS
    track_max_seconds = DEFAULT_TRACK_MAX_SECONDS
    max_consecutive_pick_failures = DEFAULT_MAX_CONSECUTIVE_PICK_FAILURES
    print(f"\n========== {LOG_PREFIX} step_target4 (机械臂智能抓取版) ==========")
    print(f"  模式: {'DRY-RUN (不动硬件)' if dry_run else 'EXECUTE (动硬件)'}")
    print(f"  预算: 前移 ≤{max_creep_m:.2f}m @ {creep_speed_mps:.2f}m/s | "
          f"picks ≤{max_picks} | 总时 ≤{max_seconds:.0f}s")
    print(f"  单球臂伺服预算: {track_max_seconds:.0f}s | 连续失败容忍: "
          f"{max_consecutive_pick_failures}")

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
    # orchestrator 已经用左 IR 触发 task4；该状态跨每一球 creep worker 保留。
    search_state = _Task4SearchState(ir_started=True)
    t_start = time.monotonic()
    reset_ts(t_start)

    try:
        # ---- 0. 停 arm_feed 守护线程 (2026-08-06 修):
        #    模块 docstring 自己写了"视觉伺服前置必须停 arm_feed (20Hz 轮询
        #    会饿 arm_queue / composite_run 4 路并发会跟 arm_feed 抢
        #    SerialEngine share_key)", 但代码一直没真停. 实际现场跑过的
        #    "composite_run 卡 200-500ms" 大概率就是 arm_feed 抢 share_key 引起.
        #    stop_arm_feed(force=True) 是真停守护线程; force=False 默认 NOOP
        #    (消费者生命周期不能杀生产者守护线程的设计原则), 所以必须 force=True.
        arm_feed_was_running = False
        if not dry_run:
            try:
                stop_res = http_client.call(
                    "car", "stop_arm_feed", force=True, timeout=5.0, sync=True,
                )
                # sync=True 会阻塞到 job 结束, 但 stop_arm_feed 在 runtime
                # 是注册到 car target 的 sync action (actions.py:29), 立刻返回.
                # sync 路径返回结构: {"ok": bool, "job": {"status":..., "result":...}}
                # (见 runtime/api/routers/_helpers.py:242), 不是 {"result": ...} 直接外层.
                stop_job = (
                    (stop_res or {}).get("job", {})
                    if isinstance(stop_res, dict) else {}
                )
                stop_result = stop_job.get("result") or {}
                if not isinstance(stop_result, dict):
                    stop_result = {}
                arm_feed_was_running = bool(
                    stop_result.get("stopped", False)
                    or stop_result.get("reason")
                       not in ("noop_without_force", "never_started")
                )
                print(f"  [{LOG_PREFIX}] �� stop_arm_feed(force=True) "
                      f"result={stop_result} "
                      f"ok={(stop_res or {}).get('ok') if isinstance(stop_res, dict) else None}")
            except Exception as e:
                print(f"  [{LOG_PREFIX}] ⚠️ stop_arm_feed 失败 "
                      f"({type(e).__name__}: {str(e)[:80]}), 继续")

        # ---- 0.b 打开存储仓 (task4 开始开仓, 结束关仓) ----
        #    纯舵机动作 (set_storage_angle 75°), 不碰臂/底盘, 边采边存常开。
        if not dry_run:
            try:
                print(f"  [{LOG_PREFIX}] 打开存储仓 (angle={STORAGE_OPEN_ANGLE_DEG}°)")
                arm_client.set_storage_angle(
                    STORAGE_OPEN_ANGLE_DEG, speed=STORAGE_OPEN_SPEED, timeout=10.0)
            except Exception as e:
                print(f"  [{LOG_PREFIX}] ⚠️ 开仓失败 ({type(e).__name__}: {str(e)[:80]}), 继续")
        else:
            print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过开仓")

        # ---- 0.c 初始 P 姿态恢复 (task4 开始前确保臂在 P 姿态) ----
        #    主循环第一球也会恢复 P 姿态, 但此处提前一次确保起始位姿正确。
        #    2026-08-09: orchestrator 已在 task3 射击结束后巡线途中预摆 TASK4_P_ARM
        #    → 已在位则跳过, 省重复摆臂 (~2s).
        if not dry_run:
            try:
                from main.task.task3.arm_poses import arm_at_pose
                p_pose_tuple = ("0", str(pose_p_y_mm / 1000.0),
                                str(pose_p_x_mm / 1000.0),
                                str(pose_p_arm_deg), str(pose_p_hand_deg))
                if arm_at_pose(arm_client, p_pose_tuple):
                    print(f"  [{LOG_PREFIX}] 初始 P 姿态已在位, 跳过恢复")
                else:
                    print(f"  [{LOG_PREFIX}] 初始 P 姿态恢复 "
                          f"(arm={pose_p_arm_deg}° x={pose_p_x_mm}mm "
                          f"y={pose_p_y_mm}mm hand={pose_p_hand_deg}°)")
                    arm_client.composite_run(
                        arm=pose_p_arm_deg,
                        x_mm=pose_p_x_mm,
                        y_mm=pose_p_y_mm,
                        hand=pose_p_hand_deg,
                        speed=60,
                        timeout=30.0,
                    )
                    print(f"  [{LOG_PREFIX}] 初始 P 姿态恢复完成")
            except Exception as e:
                print(f"  [{LOG_PREFIX}] ⚠️ 初始 P 姿态恢复失败: "
                      f"{type(e).__name__}: {str(e)[:120]}")

        # ---- 1. 准备位姿: 已删 ----
        #    开头 target1 准备位姿 (~8s) 是冗余 legacy — 主循环第一球立刻 goto_pose_p 覆盖。
        #    直接从 P 姿态开始 creep, 省 ~8s。

        # ---- 2. 主循环: P 姿态 + creep 并发 → 见球 → 抓取 → 放仓 → 循环 ----
        #    现场拍板:
        #      a) P 姿态 + creep 并发 (背景线程保前移, 主线程摆臂)
        #      b) 见球 → creep 停 → track → pick → 放仓固定两位置
        #      c) 再恢复 P 姿态 + creep 继续, 循环到 max_picks
        #      d) 失败一次就退出, 不死循环
        ball_idx = 0
        release_thread = None  # 上一球放仓后台线程, 回 P 前需 join
        while n_picks < max_picks:
            elapsed = time.monotonic() - t_start
            if elapsed >= max_seconds:
                final_reason = "time_budget"
                print(f"\n  [{LOG_PREFIX}] ⏱  总时长 {elapsed:.1f}s 达预算 "
                      f"{max_seconds:.0f}s, 收尾")
                break

            remaining_m = max_creep_m - total_creep_m
            # max_creep_m 仅保留为日志/安全参数；正常收尾由 IR 丢失+0.3m 决定。

            ball_idx += 1
            print(f"\n========== [{LOG_PREFIX}] 第 {ball_idx} 球 "
                  f"(t={elapsed:.1f}s, 已采 {n_picks}, "
                  f"剩余前移 {remaining_m:.2f}m) ==========")

            # 上一球放仓线程收尾 (防 composite_run 期间臂还在动)
            if release_thread is not None and release_thread.is_alive():
                release_thread.join(timeout=15.0)

            # 每球先恢复 P 姿态, 再启动 creep (顺序执行; 曾试过并发, 抢串口会错过球)。
            creep_thread = None

            pose_ok = True
            if not dry_run:
                try:
                    # P 姿態的四个目标一次并发下发；每个轴仍必须到位，
                    # 但不再让 x 完成后的同步 job 返回阻塞 arm/hand 下一步。
                    print(f"\n[{_ts_str()}] ========== {LOG_PREFIX}/球{ball_idx} 恢复 P 姿態 "
                          f"(x={pose_p_x_mm:+.0f}, y={pose_p_y_mm:+.0f}, "
                          f"arm={pose_p_arm_deg:+.0f}, hand={pose_p_hand_deg:+.0f}) ==========")
                    pose_result = arm_client.composite_run(
                        arm=pose_p_arm_deg,
                        x_mm=pose_p_x_mm,
                        y_mm=pose_p_y_mm,
                        hand=pose_p_hand_deg,
                        speed=100,
                        timeout=30.0,
                    )
                    if isinstance(pose_result, dict) and not pose_result.get("ok", True):
                        raise RuntimeError(f"P 姿態 composite_run 未全部完成: {pose_result}")
                    print(f"========== {LOG_PREFIX}/球{ball_idx} 恢复 P 姿態完成 ==========\n")
                except Exception as e:
                    print(f"  [{LOG_PREFIX}] ❌ 恢复 P 姿態失敗: "
                          f"{type(e).__name__}: {str(e)[:120]}")
                    pose_ok = False
            else:
                print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳過 P 姿態")

            if not pose_ok:
                final_reason = "pick_error_exceeded"
                print(f"  [{LOG_PREFIX}] ❌ P 姿態恢復失敗, 終止循环")
                if creep_thread is not None:
                    creep_thread.stop_and_join()
                break

            if creep_thread is None:
                # 非首球: P 姿态到位后才启动 creep (顺序执行, 不抢串口)
                creep_thread = _CreepThread(
                    http_client, state=search_state,
                    speed_mps=creep_speed_mps,
                    max_distance_m=remaining_m,
                    poll_hz=CREEP_POLL_HZ,
                    max_seconds_s=CREEP_MAX_SECONDS_S,
                )
                creep_thread.start()

            # 2.0.b 等待 creep 后台见球 / 见球即停 + 累计前移记账
            #    wait 上限必须有限: creep 线程自带墙钟兜底 (odom 卡死也会退出),
            #    这里给个 buffer, 绝不 inherit max_seconds=9999 干等。
            creep_res = creep_thread.wait_for_ball(
                timeout_s=min(max(1.0, max_seconds - elapsed),
                              CREEP_MAX_SECONDS_S + 10.0))
            creep_thread.stop_and_join()
            total_creep_m += creep_res["distance_m"]
            if creep_res.get("finished_by_ir_odom"):
                creep_status = "IR+里程计完成"
            elif creep_res["balls"] is not None:
                creep_status = "见球"
            else:
                creep_status = "未见球"
            print(f"  [{LOG_PREFIX}] creep 结束: 前移 {creep_res['distance_m']:.3f}m "
                  f"/ {creep_res['elapsed_s']:.1f}s → {creep_status}")
            if creep_res.get("finished_by_ir_odom"):
                final_reason = "ir_odom_exit"
                print(f"  [{LOG_PREFIX}] IR 触发后里程计走满 0.3m，"
                      f"立即结束 task4 搜索")
                break
            if creep_res["balls"] is None:
                final_reason = "zone_cleared"
                print(f"  [{LOG_PREFIX}] 🏁 前移预算内未见球, 视作采区走完")
                break

            # 2.2 定颜色 (机械臂智能抓取, 2026-08-10 移除底盘对齐)
            #    creep 阶段已把当前帧 balls 拿到 (creep_res['balls']), 直接复用判色,
            #    不再 track_chassis (底盘打滑不准) → 不 track 完再 fetch_balls。
            #    颜色锁定后, 由 _pick_and_store 内的臂伺服 (大臂+x轴) 智能抓取。
            color = None
            if creep_res.get("balls"):
                best = _pick_best_ball(creep_res["balls"])
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
            print(f"  [{LOG_PREFIX}] ✓ 锁定 {color} 球, 机械臂智能抓取 (大臂+x轴)")

            # 2.3 抓取 + 放 bin (臂伺服 pick + store)
            if dry_run:
                print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过臂伺服抓取 + 放 bin "
                      f"(would pick {color}, bin x={BIN_X_MM[color]})")
                history.append({"ball": ball_idx, "action": "dry_run",
                                "color": color, "error": None})
                n_picks += 1
                n_consecutive_failures = 0
                continue

            res = _pick_and_store(
                arm_client, runner,
                color=color,
                pick_y_mm=pick_y_mm,
                transit_x_mm=transit_x_mm,
                bin_x_mm=bin_x_blue_mm if color == COLOR_BLUE else bin_x_yellow_mm,
                bin_y_mm=bin_y_blue_mm if color == COLOR_BLUE else bin_y_yellow_mm,
                bin_hand_deg=bin_hand_blue_deg if color == COLOR_BLUE else bin_hand_yellow_deg,
                # 臂伺服 pick 起始位姿 = P 姿态 (主循环已 composite_run 到位)
                servo_x_start_mm=pose_p_x_mm,
                servo_y_start_mm=pose_p_y_mm,
                servo_arm_start_deg=pose_p_arm_deg,
                servo_hand_start_deg=pose_p_hand_deg,
            )
            release_thread = res.get("release_thread")  # 记录放仓线程, 下轮回P前 join
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
                # pick 失败后下一轮又 goto_pose_p → creep → track → pick,
                # 但 vision 阈值/PID 还没调通时, 每球都失败会无限循环在 '恢复P → 找球 →
                # 抓 → 失败 → 恢复P'。失败一次就退出, 避免循环浪费现场时间。
                if n_consecutive_failures >= max_consecutive_pick_failures:
                    final_reason = "pick_error_exceeded"
                    print(f"  [{LOG_PREFIX}] ❌ 连续失败达到 "
                          f"{max_consecutive_pick_failures}, 退出循环")
                    break
        else:
            # while-else: 没 break = picks 达到 max_picks
            final_reason = "completed"

    except KeyboardInterrupt:
        final_reason = "keyboard_interrupt"
        print(f"\n  [{LOG_PREFIX}] Ctrl-C 中断")
    finally:
        # 兜底清场: 速度清零 + stop_wheel_speeds (creep/臂伺服内已停, 这是保险)
        try:
            _set_chassis_vel(http_client, 0.0)
        except Exception:
            pass
        if not dry_run:
            try:
                _dipan._stop_chassis_quietly(http_client)
            except Exception:
                pass
            # 最后一球的放仓线程收尾, 再关仓 (防臂还在动就关舵机)
            if release_thread is not None and release_thread.is_alive():
                release_thread.join(timeout=2.0)
            # task4→task5 正常交接时关仓/P 姿态/arm_feed 都由 orchestrator 的
            # 巡航后台线程负责; 异常/独立运行仍由本 finally 收尾。
            handoff_deferred = defer_task5_handoff and final_reason == "ir_odom_exit"
            # ---- 关闭存储仓 (task4 结束关仓) ----
            if not handoff_deferred:
                try:
                    print(f"  [{LOG_PREFIX}] 关闭存储仓 (angle={STORAGE_CLOSE_ANGLE_DEG}°)")
                    arm_client.set_storage_angle(
                        STORAGE_CLOSE_ANGLE_DEG, speed=STORAGE_OPEN_SPEED, timeout=10.0)
                except Exception as e:
                    print(f"  [{LOG_PREFIX}] ⚠️ 关仓失败 ({type(e).__name__}: {str(e)[:80]})")
            else:
                print(f"  [{LOG_PREFIX}] task4→task5 交接: 关仓交给巡航后台线程")
            # ---- 结束回到 bin/P 姿态 ----
            # 正常 task4→task5 交接由 orchestrator 后台送到 task5 Phase 1，
            # 不再启动旧的 daemon P 归位，避免和交接动作抢串口。
            if not handoff_deferred:
                try:
                    import threading as _th
                    def _return_to_pose_p():
                        try:
                            print(f"  [{LOG_PREFIX}] 后台回到 P 姿态 "
                                  f"(x={pose_p_x_mm} y={pose_p_y_mm} "
                                  f"arm={pose_p_arm_deg} hand={pose_p_hand_deg})")
                            arm_client.composite_run(
                                arm=pose_p_arm_deg,
                                x_mm=pose_p_x_mm,
                                y_mm=pose_p_y_mm,
                                hand=pose_p_hand_deg,
                                speed=80,
                                timeout=30.0,
                            )
                        except Exception as e:
                            print(f"  [{LOG_PREFIX}] ⚠️ 回 P 姿态失败 "
                                  f"({type(e).__name__}: {str(e)[:80]})")
                    _th.Thread(target=_return_to_pose_p, daemon=True).start()
                except Exception:
                    pass
            # ---- 恢复 arm_feed (2026-08-06 修) ----
            #    start_arm_feed 幂等, 任务前 stop 后 start 不丢状态.
            #    没在任务前 stop 的场景 (force=False 静默 noop, 或本来就没启)
            #    start 也无害 (同 hz 的 already_running fast-path).
            #    sync 路径返回结构: {"ok": bool, "job": {"status":..., "result":...}}
            #    (见 runtime/api/routers/_helpers.py:242), 不是 {"result": ...} 直接外层.
            if not handoff_deferred:
                try:
                    start_res = http_client.call(
                        "car", "start_arm_feed", hz=20.0, timeout=5.0, sync=True,
                    )
                    start_job = (
                        (start_res or {}).get("job", {})
                        if isinstance(start_res, dict) else {}
                    )
                    start_result = start_job.get("result") or {}
                    if not isinstance(start_result, dict):
                        start_result = {}
                    started = start_result.get("started", None)
                    print(f"  [{LOG_PREFIX}] ▶️ start_arm_feed(hz=20) started={started} "
                          f"ok={(start_res or {}).get('ok') if isinstance(start_res, dict) else None}")
                except Exception as e:
                    print(f"  [{LOG_PREFIX}] ⚠️ start_arm_feed 失败 "
                          f"({type(e).__name__}: {str(e)[:80]})")
            else:
                print(f"  [{LOG_PREFIX}] task4→task5 交接: arm_feed 由 handoff 完成后恢复")

    elapsed = time.monotonic() - t_start
    print(f"\n========== {LOG_PREFIX} 完成 ==========")
    print(f"  reason={final_reason}  picks={n_picks}/{max_picks}  "
          f"skips={n_skips}  pick_failures={n_pick_failures}  "
          f"前移={total_creep_m:.3f}m  elapsed={elapsed:.1f}s")

    return {
        "ok": final_reason in (
            "completed", "zone_cleared", "time_budget", "keyboard_interrupt",
            "ir_odom_exit",
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
        description="task4 target4: 慢速前移搜索 + 机械臂智能抓取 + 放 bin "
                    "(--dry-run 只打印)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--dry-run", action="store_true",
                   help="dry-run 模式 (默认 execute, 真动硬件)")
    p.add_argument("--pose-x", type=float, default=TASK4_POSE_P_X_MM,
                   help=f"P 姿态 x (mm), 默认 {TASK4_POSE_P_X_MM}")
    p.add_argument("--pose-y", type=float, default=TASK4_POSE_P_Y_MM,
                   help=f"P 姿态 y (mm), 默认 {TASK4_POSE_P_Y_MM}")
    p.add_argument("--pose-arm", type=float, default=TASK4_POSE_P_ARM_DEG,
                   help=f"P 姿态 arm 角度 (°), 默认 {TASK4_POSE_P_ARM_DEG}")
    p.add_argument("--pose-hand", type=float, default=TASK4_POSE_P_HAND_DEG,
                   help=f"P 姿态 hand 角度 (°), 默认 {TASK4_POSE_P_HAND_DEG}")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    from main.api_client import RuntimeApiClient  # noqa: E402
    http = RuntimeApiClient()
    arm = ArmClient.connect()
    runner = ArmRunner(arm)

    result = step_target4(
        arm, http,
        runner=runner,
        dry_run=args.dry_run,
        pose_p_x_mm=args.pose_x,
        pose_p_y_mm=args.pose_y,
        pose_p_arm_deg=args.pose_arm,
        pose_p_hand_deg=args.pose_hand,
    )

    print(f"\n[{LOG_PREFIX}] 最终结果: {result}")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
