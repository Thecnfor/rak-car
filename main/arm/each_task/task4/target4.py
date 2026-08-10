#!/usr/bin/python3
"""task4 / target4 —— 编排门面: 慢速前移搜索 + 底盘视觉定位 (最左球) + 吸嘴中心抓取。

2026-08-10 拆分: 原 1599 行单文件按职责拆成 4 个单向依赖模块, 本文件保留
**编排骨架** (`step_target4` 主循环 + 预算/清场 + CLI), 业务实现下沉:
  - ``creep_thread.py``   后台保前移线程 + 底盘速度 helper
                          (`_CreepThread` / `_set_chassis_vel`)
  - ``track_align.py``    底盘视觉对齐最左球 (4s → 超时加时 3s)
                          (`_track_leftmost_ball`)
  - ``pick_store.py``     选球判色 + 机械臂视觉伺服 + 盲降抓取放 bin
                          (`_servo_and_pick` / `_color_from_track` / `_pick_best_ball`)
  - ``constants.py``      所有位姿/伺服/扫描/放仓常量 + 时间戳辅助 (校准表面)

对外符号全部 re-export 于此 (step_target4 / _CreepThread / _track_leftmost_ball),
外部 import 无需改动。

流程 (2026-08-11 用户确认新版):
  - 开始阶段 (触发后立即): 四轴联动到初始姿势 (x=-250/y=-150/hand=-10/arm=90,
    已到位跳过) ∥ lane 前进 0.1m ∥ 开仓 75° 三步并发。
  - 第一球: creep 慢扫找球 → 底盘对齐 (4s, 超时加时 3s, 失败也继续) →
    机械臂视觉伺服 (run_arm_servo, 只动 x+大臂, setpoint 0.045/-0.083) → 盲降抓+放 bin。
  - 后续球 (最多 8): move_for 前进 0.1m ∥ 臂回初始 → 找球 ≤3s (可抓窗口+最左) →
    臂伺服 → 抓放 → 循环。
  - 退出: 左 IR > 0.75 (离区) 或 picks ≥ 8 (封顶)。
  - 旧机制 (0.58m 蠕动预算 / zone_cleared / 五段式 track) 已废除 (2026-08-11)。

⚠️ 底盘通道: /v1/realtime/chassis-velocity (realtime 门, 免 job_queue, 与
   track_chassis 同通道); orchestrator 派发 task4 前已暂停 lane 外环, 不冲突。
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
    DEFAULT_MAX_SECONDS, DEFAULT_CREEP_SPEED_MPS, DEFAULT_TRACK_MAX_SECONDS,
    CREEP_POLL_HZ, CREEP_MAX_SECONDS_S,
    # 姿态 / 放仓
    COLOR_BLUE, COLOR_YELLOW, BIN_X_MM, BIN_HAND_DEG,
    TASK4_POSE_P_Y_MM, TASK4_POSE_P_X_MM, TASK4_POSE_P_ARM_DEG, TASK4_POSE_P_HAND_DEG,
    STORAGE_OPEN_ANGLE_DEG, STORAGE_CLOSE_ANGLE_DEG, STORAGE_OPEN_SPEED,
    # 开始阶段 (三步并发)
    START_LANE_FORWARD_M, START_LANE_FORWARD_VX_MPS,
    P_POSE_SKIP_TOL_X_MM, P_POSE_SKIP_TOL_Y_MM,
    P_POSE_SKIP_TOL_ARM_DEG, P_POSE_SKIP_TOL_HAND_DEG,
    # 新版流程 (2026-08-11)
    FIRST_CREEP_MAX_M, TRACK_EXTEND_SECONDS,
    SCAN_MAX_PICKS, SCAN_ADVANCE_M, SCAN_LOOK_S, SCAN_GRAB_CX_HALF,
    SCAN_GRAB_CX_HALF_LATE, SCAN_GRAB_TIER_ADVANCES,
    MIN_SCAN_ADVANCES, SCAN_EMPTY_ROUNDS,
    ARM_SERVO_SETPOINT_CX, ARM_SERVO_SETPOINT_CY, PICK_RELEASE_HAND_DEG,
    # 临时调试开关
    ALIGN_ONLY,
    # 时间戳辅助
    _ts_str, reset_ts,
    LOG_PREFIX_TARGET4,
)
from main.arm.each_task.task4.creep_thread import (  # noqa: E402
    _CreepThread, _set_chassis_vel,
)
from main.arm.each_task.task4.track_align import (  # noqa: E402
    _track_leftmost_ball,
)
from main.arm.each_task.task4.pick_store import (  # noqa: E402
    _color_from_track, _pick_best_ball, _servo_and_pick,
)


LOG_PREFIX: str = LOG_PREFIX_TARGET4


# ---------- 新版流程 helpers (2026-08-11) ----------

def _advance_and_arm_init(arm_client, http_client, *, pose_p_x_mm, pose_p_y_mm,
                          pose_p_arm_deg, pose_p_hand_deg, dry_run=False) -> None:
    """沿车道线前进 SCAN_ADVANCE_M ∥ 并发 臂回初始姿势。

    前进走 move_along_lane (lane_follow, 视觉对齐车道中心不偏), 不用 move_for。
    串口抢占风险已接受 (2026-08-11 用户) —— 前进与臂回初始同时进行。
    """
    if dry_run:
        print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过 前进{SCAN_ADVANCE_M:.2f}m ∥ 臂回初始")
        return
    import threading as _th

    def _arm_init():
        try:
            arm_client.composite_run(arm=pose_p_arm_deg, x_mm=pose_p_x_mm,
                                     y_mm=pose_p_y_mm, hand=pose_p_hand_deg,
                                     speed=80, timeout=30.0)
        except Exception as e:
            print(f"  [{LOG_PREFIX}] ⚠️ 臂回初始失败 ({type(e).__name__}: {str(e)[:80]})")

    t = _th.Thread(target=_arm_init, name="task4-arm-init", daemon=True)
    t.start()
    try:
        from main.chassis.controllers import move_along_lane
        move_along_lane(vx=0.05, distance_m=SCAN_ADVANCE_M)
        print(f"  [{LOG_PREFIX}] 前进 {SCAN_ADVANCE_M:.2f}m 完成 (lane_follow, 臂回初始并发)")
    except Exception as e:
        print(f"  [{LOG_PREFIX}] ⚠️ lane_follow 前进失败 ({type(e).__name__}: {str(e)[:80]})")
    t.join(timeout=35.0)


def _look_grabbable_ball(http_client, *, timeout_s: float = SCAN_LOOK_S,
                         grab_half: float = SCAN_GRAB_CX_HALF,
                         dry_run: bool = False) -> Optional[dict]:
    """找球 ≤timeout_s: 轮询 fetch_balls, 过滤到可抓窗口
    (|cx_norm - ARM_SERVO_SETPOINT_CX| ≤ grab_half), 取最左 (cx 最小)。

    窗口内无球 → None (没球 / 球在窗口外 = 下一轮的球, 上层继续前进)。
    grab_half 由主循环按梯度 (前 N 次窄窗, 之后宽窗) 传入。
    """
    if dry_run:
        return {"color": COLOR_BLUE, "cx_norm": 0.0, "score": 1.0}
    from . import target2  # noqa: E402
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() < deadline:
        try:
            balls = target2.fetch_balls(
                http_client, color_filter=None,
                score_min=0.35, aspect_tol=1.0,
                area_min=0.03, area_max=0.90, debug=False,
            )
        except Exception:
            balls = []
        in_window = [
            b for b in balls
            if b.get("color") in (COLOR_BLUE, COLOR_YELLOW)
            and abs(float(b.get("cx_norm", 0.0)) - ARM_SERVO_SETPOINT_CX) <= grab_half
        ]
        if in_window:
            return min(in_window, key=lambda b: float(b.get("cx_norm", 0.0)))
        time.sleep(0.1)
    return None


# ---------- 核心 step ----------

def step_target4(
    arm_client: ArmClient,
    http_client,
    *,
    runner: Optional[ArmRunner] = None,
    defer_task5_handoff: bool = False,
    dry_run: bool = False,
    debug_recognition: bool = False,
    # ---- 初始姿势参数 (默认值在 constants.py, 可由外部覆盖) ----
    pose_p_y_mm: float = TASK4_POSE_P_Y_MM,
    pose_p_x_mm: float = TASK4_POSE_P_X_MM,
    pose_p_arm_deg: float = TASK4_POSE_P_ARM_DEG,
    pose_p_hand_deg: float = TASK4_POSE_P_HAND_DEG,
    bin_x_blue_mm: float = BIN_X_MM[COLOR_BLUE],
    bin_x_yellow_mm: float = BIN_X_MM[COLOR_YELLOW],
    bin_hand_blue_deg: float = BIN_HAND_DEG.get(COLOR_BLUE, PICK_RELEASE_HAND_DEG),
    bin_hand_yellow_deg: float = BIN_HAND_DEG.get(COLOR_YELLOW, PICK_RELEASE_HAND_DEG),
) -> dict:
    """慢速前移搜索 + 底盘视觉定位 (最左球) + 吸嘴中心抓取 + 放 bin。

    Args:
        arm_client: ArmClient 实例。
        http_client: RuntimeApiClient (creep 速度下发 / fetch_balls / track_chassis 共用)。
        runner: ArmRunner (None 时自动建)。
        defer_task5_handoff: task4 由 orchestrator 调度时，IR+odom 结束后将
            关仓 + task5 Phase 1 姿态交给巡航线程并行执行；独立运行保持 False。
        dry_run: True 不动硬件 (仍轮询视觉排练流程)。
        姿态参数: 各轴目标 (mm / °), 默认值 = constants.py 的 task4 校准常量。

    Returns:
        dict:
          - ok: bool (completed / zone_cleared / time_budget /
                keyboard_interrupt 为 True)
          - picks / skips / pick_failures: 计数
          - total_creep_m: 累计前移距离
          - history: list[dict] 每球记录
          - reason: "completed" / "zone_cleared" / "time_budget" /
                    "pick_error_exceeded" / "keyboard_interrupt"
          - elapsed_s: 总耗时
    """
    # 参数全部走 constants 模块默认值，只保留初始姿势可调 (2026-08-11)。
    max_seconds = DEFAULT_MAX_SECONDS
    creep_speed_mps = DEFAULT_CREEP_SPEED_MPS
    track_max_seconds = DEFAULT_TRACK_MAX_SECONDS
    print(f"\n========== {LOG_PREFIX} step_target4 (新版: 蠕动+底盘对齐+臂伺服) ==========")
    print(f"  模式: {'DRY-RUN (不动硬件)' if dry_run else 'EXECUTE (动硬件)'}")
    print(f"  初始姿势: x={pose_p_x_mm} y={pose_p_y_mm} arm={pose_p_arm_deg}° hand={pose_p_hand_deg}°")
    print(f"  第一球 creep {creep_speed_mps:.2f}m/s | 后续 前进 {SCAN_ADVANCE_M}m × "
          f"{SCAN_LOOK_S:.0f}s 找球 (窗口梯度: 前{SCAN_GRAB_TIER_ADVANCES}次 "
          f"{SCAN_GRAB_CX_HALF:.1f}, 之后 {SCAN_GRAB_CX_HALF_LATE:.1f})")
    print(f"  底盘对齐 ≤{track_max_seconds:.0f}s (+超时加时 {TRACK_EXTEND_SECONDS:.0f}s, "
          f"失败也继续) | 臂伺服 setpoint=({ARM_SERVO_SETPOINT_CX},{ARM_SERVO_SETPOINT_CY})")
    print(f"  退出: 连续{SCAN_EMPTY_ROUNDS}轮无球 且 前进≥{MIN_SCAN_ADVANCES}次 "
          f"| 无抓球数封顶 | ALIGN_ONLY={ALIGN_ONLY}")

    for name, val in (("max_seconds", max_seconds), ("creep_speed_mps", creep_speed_mps),
                      ("track_max_seconds", track_max_seconds)):
        if val < 0:
            raise ValueError(f"{name} 必须 ≥ 0, 收到: {val}")

    if runner is None:
        runner = ArmRunner(arm_client)

    history: list = []
    n_picks = 0
    n_skips = 0
    n_pick_failures = 0
    total_creep_m = 0.0
    final_reason = "unknown"
    t_start = time.monotonic()
    reset_ts(t_start)

    try:
        # ---- 0. 起始臂姿态检测 (2026-08-10): orchestrator 途中已摆好 TASK4_P_ARM 时
        #    跳过开始阶段的四轴联动。此刻 arm_feed 还在跑, get_state fast-path 可用,
        #    必须放在 stop_arm_feed(force=True) 之前读。----
        arm_at_p_pose = False
        if not dry_run:
            try:
                _st = arm_client.get_state()
                if _st is not None:
                    _vals = (_st.x_mm, _st.y_mm, _st.arm_angle, _st.hand_angle)
                    arm_at_p_pose = (
                        all(v is not None for v in _vals)
                        and abs(_st.x_mm - pose_p_x_mm) <= P_POSE_SKIP_TOL_X_MM
                        and abs(_st.y_mm - pose_p_y_mm) <= P_POSE_SKIP_TOL_Y_MM
                        and abs(float(_st.arm_angle) - pose_p_arm_deg) <= P_POSE_SKIP_TOL_ARM_DEG
                        and abs(float(_st.hand_angle) - pose_p_hand_deg) <= P_POSE_SKIP_TOL_HAND_DEG
                    )
            except Exception:
                arm_at_p_pose = False
        print(f"  [{LOG_PREFIX}] 起始臂姿态: "
              f"{'已在 P 姿态 (跳过开始阶段四轴联动)' if arm_at_p_pose else '需四轴联动'}")

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

        # ---- 0.b 开始阶段: 三步并发 (2026-08-10 用户拍板) ----
        #    触发任务点后立即并发执行: ① 四轴联动到 P 姿态 (已在则跳过) ∥
        #    ② lane 前进 0.1m ∥ ③ 开仓 75°。
        #    开仓是纯舵机动作 (不碰臂/底盘), 边采边存整场常开。
        #    ⚠️ 并发期间 composite_run(4 路臂命令) + move_along_lane(底盘轮速)
        #    抢 SerialEngine, composite_run 内部 y 下降偶尔会卡 200-500ms —
        #    用户已知悉并接受 (开始阶段只发生一次, 时间上界可接受)。
        import threading as _th
        _pose_done = [arm_at_p_pose]   # 起始值: 已到位则视为 done

        def _start_pose():
            if dry_run or _pose_done[0]:
                return
            try:
                print(f"  [{LOG_PREFIX}] 开始阶段四轴联动 (arm={pose_p_arm_deg}° "
                      f"x={pose_p_x_mm}mm y={pose_p_y_mm}mm hand={pose_p_hand_deg}°)")
                arm_client.composite_run(
                    arm=pose_p_arm_deg,
                    x_mm=pose_p_x_mm,
                    y_mm=pose_p_y_mm,
                    hand=pose_p_hand_deg,
                    speed=100,
                    timeout=30.0,
                )
                _pose_done[0] = True
                print(f"  [{LOG_PREFIX}] 开始阶段四轴联动完成")
            except Exception as e:
                _pose_done[0] = False
                print(f"  [{LOG_PREFIX}] ⚠️ 开始阶段四轴联动失败 "
                      f"({type(e).__name__}: {str(e)[:80]})")

        def _start_open():
            if dry_run:
                return
            try:
                print(f"  [{LOG_PREFIX}] 打开存储仓 (angle={STORAGE_OPEN_ANGLE_DEG}°)")
                arm_client.set_storage_angle(
                    STORAGE_OPEN_ANGLE_DEG, speed=STORAGE_OPEN_SPEED, timeout=10.0)
            except Exception as e:
                print(f"  [{LOG_PREFIX}] ⚠️ 开仓失败 ({type(e).__name__}: {str(e)[:80]})")

        t_pose = _th.Thread(target=_start_pose, name="task4-start-pose", daemon=True)
        t_open = _th.Thread(target=_start_open, name="task4-start-open", daemon=True)
        t_pose.start()
        t_open.start()
        if dry_run or START_LANE_FORWARD_M <= 0.0:
            print(f"  [{LOG_PREFIX}] {'[DRY-RUN] ' if dry_run else ''}跳过 lane 前进")
        else:
            try:
                print(f"  [{LOG_PREFIX}] 开始阶段 lane 前进 {START_LANE_FORWARD_M:.2f}m "
                      f"@ {START_LANE_FORWARD_VX_MPS:.2f}m/s")
                from main.chassis.controllers import move_along_lane
                move_along_lane(vx=START_LANE_FORWARD_VX_MPS,
                                distance_m=START_LANE_FORWARD_M)
                print(f"  [{LOG_PREFIX}] 开始阶段 lane 前进完成")
            except Exception as e:
                print(f"  [{LOG_PREFIX}] ⚠️ lane 前进失败 "
                      f"({type(e).__name__}: {str(e)[:80]}), 继续")
        t_pose.join(timeout=40.0)
        t_open.join(timeout=15.0)
        arm_at_p_pose = bool(_pose_done[0])
        if arm_at_p_pose:
            print(f"  [{LOG_PREFIX}] 开始阶段结束: 臂已在 P 姿态 (含跳过情形)")

        # ---- 2. 新版主流程 (2026-08-11 用户确认) ----
        #    第一球: creep 找球 → 底盘对齐(4s+3s best-effort) → 臂伺服 → 抓放
        #    后续球: 0.1m 前进∥臂回初始 → 找球(3s, 可抓窗口+最左) → 臂伺服 → 抓放
        #    退出: 左IR>0.75 或 picks≥8
        ball_idx = 0
        release_thread = None

        def _record_pick(idx, color, res) -> int:
            ok = bool(res["ok"])
            history.append({"ball": idx,
                            "action": "picked" if ok else "pick_failed",
                            "color": color, "error": res["error"]})
            if not ok:
                print(f"  [{LOG_PREFIX}] ❌ 第 {idx} 球抓取失败: {res['error']}")
            return 1 if ok else 0

        def _grab(color) -> dict:
            return _servo_and_pick(
                arm_client, http_client, runner, color=color, dry_run=dry_run,
                bin_x=(bin_x_blue_mm if color == COLOR_BLUE else bin_x_yellow_mm),
                release_hand=(bin_hand_blue_deg if color == COLOR_BLUE else bin_hand_yellow_deg),
            )

        # ---- 2.1 第一个球: creep 找球 ----
        #    TODO(第一球蠕动终止待讨论): 占位大距离 + 40s 等待兜底 (无球→扫空收工)。
        ball_idx += 1
        print(f"\n========== [{LOG_PREFIX}] 第 {ball_idx} 球 (首个, creep 找球) ==========")
        if dry_run:
            # dry-run 不启 creep 线程 (会真发底盘速度), 用占位球走通流程
            creep_res = {"balls": [{"color": COLOR_BLUE, "cx_norm": 0.05}],
                         "distance_m": 0.0, "elapsed_s": 0.0}
            print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过 creep, 用占位球")
        else:
            creep_thread = _CreepThread(
                http_client,
                speed_mps=creep_speed_mps,
                max_distance_m=FIRST_CREEP_MAX_M,
                poll_hz=CREEP_POLL_HZ,
            )
            creep_thread.start()
            creep_res = creep_thread.wait_for_ball(
                timeout_s=min(max(1.0, max_seconds - (time.monotonic() - t_start)),
                              CREEP_MAX_SECONDS_S + 10.0))
            creep_thread.stop_and_join()
        total_creep_m += creep_res["distance_m"]
        if creep_res["balls"] is None:
            print(f"  [{LOG_PREFIX}] 🏁 首个 creep 未见球 (前移 {creep_res['distance_m']:.3f}m)")
            # TODO: 第一球蠕动终止条件待讨论, 占位按扫空收工。
            final_reason = "zone_cleared"
        else:
            # 底盘对齐 (4s → 超时加时 3s → 失败也继续, 不阻塞、不放弃球)
            print(f"  [{LOG_PREFIX}] 🎯 底盘对齐 (≤{track_max_seconds:.0f}s, "
                  f"超时加时 {TRACK_EXTEND_SECONDS:.0f}s, 失败也继续)")
            track_res = _track_leftmost_ball(
                max_seconds=track_max_seconds,
                extend_seconds=TRACK_EXTEND_SECONDS,
                dry_run=dry_run,
            )
            print(f"  [{LOG_PREFIX}] 底盘对齐结束: arrived={track_res.arrived} "
                  f"reason={track_res.reason}")
            # 定色: track label 优先, creep 帧兜底
            color = _color_from_track(track_res)
            if color is None and not dry_run:
                best = _pick_best_ball(creep_res["balls"] or [])
                color = best["color"] if best else None
            if dry_run and color is None:
                color = COLOR_BLUE
            if color not in (COLOR_BLUE, COLOR_YELLOW):
                print(f"  [{LOG_PREFIX}] ❌ 首个球无法定色, 收工")
                final_reason = "zone_cleared"
            else:
                print(f"  [{LOG_PREFIX}] ✓ 首个球: {color}")
                res = _grab(color)
                n_picks += _record_pick(ball_idx, color, res)
                if not res["ok"]:
                    n_pick_failures += 1
                release_thread = res.get("release_thread")

        # ---- 2.2 后续球循环 (无抓球数封顶) ----
        #    退出 = 前进≥MIN_SCAN_ADVANCES 且 连续 SCAN_EMPTY_ROUNDS 轮无球 且 左 IR 离区
        scan_advances = 0      # 已前进次数
        consecutive_empty = 0  # 连续找球为空轮数
        while final_reason == "unknown":
            elapsed = time.monotonic() - t_start
            if elapsed >= max_seconds:
                final_reason = "time_budget"
                print(f"  [{LOG_PREFIX}] ⏱ 总时长 {elapsed:.1f}s 达预算, 收尾")
                break
            ball_idx += 1
            print(f"\n========== [{LOG_PREFIX}] 第 {ball_idx} 球 (扫描前进) ==========")
            if release_thread is not None and release_thread.is_alive():
                release_thread.join(timeout=15.0)
            # 前进 0.1m ∥ 臂回初始姿势
            _advance_and_arm_init(
                arm_client, http_client,
                pose_p_x_mm=pose_p_x_mm, pose_p_y_mm=pose_p_y_mm,
                pose_p_arm_deg=pose_p_arm_deg, pose_p_hand_deg=pose_p_hand_deg,
                dry_run=dry_run,
            )
            scan_advances += 1
            # 梯度窗口: 前 SCAN_GRAB_TIER_ADVANCES 次前进用窄窗, 之后放宽到宽窗
            grab_half = (SCAN_GRAB_CX_HALF if scan_advances <= SCAN_GRAB_TIER_ADVANCES
                         else SCAN_GRAB_CX_HALF_LATE)
            # 找球 ≤3s (可抓窗口 + 最左); 窗口内无球 → 累积空轮, 达标才退出
            ball = _look_grabbable_ball(http_client, timeout_s=SCAN_LOOK_S,
                                        grab_half=grab_half, dry_run=dry_run)
            if ball is None:
                consecutive_empty += 1
                print(f"  [{LOG_PREFIX}] 未见可抓球 (窗口内无球), 空轮 {consecutive_empty} 次")
                if (scan_advances >= MIN_SCAN_ADVANCES
                        and consecutive_empty >= SCAN_EMPTY_ROUNDS):
                    final_reason = "zone_cleared"
                    print(f"  [{LOG_PREFIX}] 🏁 已前进 {scan_advances} 次, 连续 "
                          f"{consecutive_empty} 轮无球, 收工")
                    break
                continue
            consecutive_empty = 0
            color = ball["color"]
            print(f"  [{LOG_PREFIX}] ✓ 锁定 {color} 球 "
                  f"(最左, cx={float(ball.get('cx_norm', 0)):.3f})")
            res = _grab(color)
            n_picks += _record_pick(ball_idx, color, res)
            if not res["ok"]:
                n_pick_failures += 1
            release_thread = res.get("release_thread")
            if dry_run and n_picks >= SCAN_MAX_PICKS:
                # dry-run 占位球抓满即停, 防止空转 (实车不封顶)
                final_reason = "completed"
                print(f"  [{LOG_PREFIX}] [DRY-RUN] 抓满 {SCAN_MAX_PICKS} 占位球, 收工")
                break

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
            # 最后一球的放仓线程收尾, 再关仓 (防臂还在动就关舵机)
            if release_thread is not None and release_thread.is_alive():
                release_thread.join(timeout=2.0)
            # task4→task5 正常交接时关仓/P 姿态/arm_feed 都由 orchestrator 的
            # 巡航后台线程负责; 异常/独立运行仍由本 finally 收尾。
            # 2026-08-10: IR 离区退出已删除, final_reason 不可能为 "ir_odom_exit",
            # 所以 handoff_deferred 恒 False → task4 自己收尾 (关仓/回 P/arm_feed)。
            # 若要恢复 task4→task5 并发交接, 需把这里改键到 zone_cleared。
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
    print(f"  reason={final_reason}  picks={n_picks}  "
          f"skips={n_skips}  pick_failures={n_pick_failures}  "
          f"前移={total_creep_m:.3f}m  elapsed={elapsed:.1f}s")

    return {
        "ok": final_reason in (
            "completed", "zone_cleared", "time_budget", "keyboard_interrupt",
            "align_only",
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
    p.add_argument("--debug-recognition", action="store_true",
                   help="fetch_balls 打印每条 detection 过滤原因")
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
        debug_recognition=args.debug_recognition,
        pose_p_x_mm=args.pose_x,
        pose_p_y_mm=args.pose_y,
        pose_p_arm_deg=args.pose_arm,
        pose_p_hand_deg=args.pose_hand,
    )

    print(f"\n[{LOG_PREFIX}] 最终结果: {result}")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
