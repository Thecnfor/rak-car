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
       ④ 放 bin        composite_run (抬 y=-130 ∥ 移 bin) → 降 y=-120/-135 → 放气 → 回识别位姿
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
        GRASP_HOLD_S,
        STORAGE_OPEN_ANGLE_DEG, STORAGE_CLOSE_ANGLE_DEG, STORAGE_OPEN_SPEED,
    )
except ImportError:  # pragma: no cover —— 直接 python target4.py 时无包上下文
    from main.arm.each_task.task4 import (  # type: ignore # noqa: E402
        target1, target2, dipan as _dipan,
    )
    from main.arm.each_task.task4.constants import (  # type: ignore # noqa: E402
        LOG_PREFIX_TASK4, COLOR_BLUE, COLOR_YELLOW,
        GRASP_HOLD_S,
        STORAGE_OPEN_ANGLE_DEG, STORAGE_CLOSE_ANGLE_DEG, STORAGE_OPEN_SPEED,
    )

from main.arm import (  # noqa: E402
    ArmClient, ArmRunner,
)
from main.arm.each_task.common import (  # noqa: E402
    goto_pose_p, POSE_P_X_MM, POSE_P_Y_MM,
)
from main.chassis import track_chassis  # noqa: E402
from main.chassis.loops.visual_track import TrackChassisResult  # noqa: E402


LOG_PREFIX: str = LOG_PREFIX_TASK4 + "/target4"

# 时间戳辅助: 距 task4 启动的秒数, 打在每个动作前定位每步延迟。
_TASK4_T0: Optional[float] = None


def _ts_str() -> str:
    global _TASK4_T0
    if _TASK4_T0 is None:
        _TASK4_T0 = time.monotonic()
    return f"t=+{time.monotonic() - _TASK4_T0:.1f}s"


# ---- 默认参数 ----

DEFAULT_MAX_PICKS: int = 1000
"""最多抓取数 (距离优先模式下设为极大值, 实际不限制)。"""

DEFAULT_MAX_CREEP_M: float = 0.58
"""累计前移距离预算 (m, 开环 速度×时间 记账)。唯一实际生效的终止条件。"""

DEFAULT_MAX_SECONDS: float = 9999.0
"""任务总时长预算 (s) (距离优先模式下设为极大值, 实际不限制)。"""

DEFAULT_CREEP_SPEED_MPS: float = 0.06
"""creep 搜索前移速度 (m/s)。"""

CREEP_POLL_HZ: float = 20.0
"""creep 期间 fetch_balls 轮询频率。"""

CREEP_MAX_SECONDS_S: float = 30.0
"""单次 creep 墙钟兜底上限 (s): 距离/IR/见球任一不满足也强制退出, 防 odom 卡死干等。"""

DEFAULT_TRACK_MAX_SECONDS: float = 6.0
"""单球底盘视觉伺服收敛预算 (s)。"""

DEFAULT_MAX_CONSECUTIVE_PICK_FAILURES: int = 1000
"""连续 pick 失败超过此数 → 退出 (距离优先模式下设为极大值, 实际不限制)。"""

DEFAULT_MAX_CONSECUTIVE_TRACK_FAILURES: int = 2
"""连续 track 失败超过此数 → 退出。
"""

DEFAULT_PICK_TIMEOUT_S: float = 60.0
"""pick_by_vision 总超时 (s)。"""

DEFAULT_TRACK_SOFT_DEADBAND: float = 0.15
"""track_chassis 软死区 (cx_err/cy_err 绝对值 < 此值视为"接近对齐")。
"""

DEFAULT_TRACK_RETRY_SECONDS: float = 1.0
"""软收敛额外 time budget (s). near_arrived 时再给 <1s 用更大 deadband 重试.

避免 6s 整除 -> 死线刚好卡 deadline 时 hard_timeout. 给 1s 缓冲意味着
最坏 7s track 总时长, 但 max_seconds-bounded 防止极端超时.
"""

DEFAULT_TRACK_WIDE_DEADBAND: float = 0.45
"""track_chassis 宽死区 (cx_err/cy_err 绝对值 < 此值视为"近似对齐可以一试").
"""

DEFAULT_RETURN_X_MM: Optional[float] = POSE_P_X_MM
"""放 bin 后 x 回的目标位置 (mm)。默认 = POSE_P_X (P 姿态 x), None = 不回。"""
# ---- P 姿态参数 (可由外部覆盖) ----
TASK4_POSE_P_Y_MM: float = -160.0
TASK4_POSE_P_X_MM: float = -295.0
TASK4_POSE_P_ARM_DEG: float = 90.0
TASK4_POSE_P_HAND_DEG: float = 10.0

# ---- 抓取 / 中转位姿 ----

X_PICK_MM: float = -240.0
"""盲降前横移 x (mm)。"""

Y_PICK_MM: float = -65.0
"""抓球 y (吸盘贴近球面)。"""

Y_TRANSIT_MM: float = -140.0
"""中转 y (放仓位之前的过渡位)。"""

X_TRANSIT_MM: float = -220.0
"""中转 x (车体中线附近, 两次小位移降低 belt-slip 风险)。"""

# ---- 放 bin 参数 ----

Y_PUT_MM: float = -140.0
"""放球 y (再深 10mm 防脱落)。"""

BIN_X_MM = {COLOR_BLUE: 0.0, COLOR_YELLOW: -60.0}
"""蓝 bin x=0, 黄 bin x=-70。"""

BIN_Y_MM = {COLOR_BLUE: -140.0}
"""蓝 bin y=-135; 黄沿用 Y_PUT_MM。"""

BIN_HAND_DEG = {COLOR_BLUE: 10.0}
"""蓝 bin hand=-30°; 黄沿用 P 姿态 hand=10°。"""

# ---- 其他 ----

Y_FINAL_MM: float = -140.0
"""最终 y (识别位姿, 历史值)。"""

BALL_LABELS = ["ball_blue", "ball_yellow"]
"""track_chassis 目标集 (PaddleDet 模型标签)。"""

# task4 任务点由 orchestrator 的左 IR < 0.70m 触发。
# 任务运行期间 IR 回到 > 0.70m, 连续确认后视为离开任务区, 再前进 0.30m 收工。
IR_FAR_THRESHOLD_M: float = 0.70
IR_FAR_CONFIRM_FRAMES: int = 2
POST_IR_LOSS_DISTANCE_M: float = 0.30


class _Task4SearchState:
    """跨每一球保存 task4 的 IR 生命周期和末端 0.3m 记账。"""

    def __init__(self, *, ir_started: bool = True,
                 far_threshold_m: float = IR_FAR_THRESHOLD_M,
                 far_confirm_frames: int = IR_FAR_CONFIRM_FRAMES):
        import threading
        self.ir_started = bool(ir_started)
        self.far_threshold_m = float(far_threshold_m)
        self.far_confirm_frames = max(1, int(far_confirm_frames))
        self.ir_lost = False
        self.far_streak = 0
        self.post_loss_distance_m = 0.0
        self._pending_far_distance_m = 0.0
        self.finished_by_ir_odom = False
        self._lock = threading.Lock()

    def update_ir(self, left_ir, *, distance_m: float = 0.0) -> bool:
        """更新左 IR；连续远读数锁存为 IR 丢失并吸收确认期间位移。"""
        with self._lock:
            if self.ir_lost or not self.ir_started:
                return self.ir_lost
            try:
                far = float(left_ir) > self.far_threshold_m
            except (TypeError, ValueError):
                far = False
            if far:
                self.far_streak += 1
                self._pending_far_distance_m += max(0.0, float(distance_m))
                if self.far_streak >= self.far_confirm_frames:
                    self.ir_lost = True
                    self.post_loss_distance_m += self._pending_far_distance_m
                    self._pending_far_distance_m = 0.0
                    if self.post_loss_distance_m >= POST_IR_LOSS_DISTANCE_M:
                        self.finished_by_ir_odom = True
            else:
                self.far_streak = 0
                self._pending_far_distance_m = 0.0
            return self.ir_lost

    def add_post_loss_distance(self, delta_m: float) -> bool:
        """累计 IR 丢失后的位移，并返回是否达到 0.30m。"""
        with self._lock:
            if not self.ir_lost or self.finished_by_ir_odom:
                return self.finished_by_ir_odom
            self.post_loss_distance_m += max(0.0, float(delta_m))
            if self.post_loss_distance_m >= POST_IR_LOSS_DISTANCE_M:
                self.finished_by_ir_odom = True
            return self.finished_by_ir_odom


# ---- 后台保前移线程 (P 姿态 + creep 并发) ----

class _CreepThread:
    """后台线程保底盘前移 + 主线程摆臂。"""

    def __init__(self, http_client, *, state: Optional[_Task4SearchState] = None,
                 speed_mps: float, max_distance_m: float,
                 poll_hz: float = CREEP_POLL_HZ,
                 max_seconds_s: float = CREEP_MAX_SECONDS_S):
        import threading
        self.http = http_client
        self.state = state or _Task4SearchState()
        self.speed_mps = speed_mps
        self.max_distance_m = max_distance_m
        self.poll_hz = poll_hz
        # 墙钟上限只是单次 worker 的安全兜底；正常 task4 收尾由 IR 丢失+0.3m 决定。
        self.max_seconds_s = float(max_seconds_s)
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="task4-creep")
        self._stop_event = threading.Event()
        self.completion_event = threading.Event()
        self.distance_m = 0.0
        self.elapsed_s = 0.0
        self.balls = None
        self.found_ball = False
        self.finished_by_ir_odom = False
        self.timed_out = False
        self._odo_start_x = None
        self._last_odom_x = None
        self._odom_stall_s = 0.0

    def start(self) -> None:
        # 记录启动时里程计 x，后续闭环累加。
        try:
            odo = self.http.get_odom_state() or {}
            odo_data = odo.get("odom_state") or {}
            self._odo_start_x = odo_data.get("x")
        except Exception:
            self._odo_start_x = None
        self._thread.start()

    def _loop(self) -> None:
        period = 1.0 / max(self.poll_hz, 1.0)
        t0 = time.monotonic()
        try:
            while not self._stop_event.is_set():
                if self.state.finished_by_ir_odom:
                    self.finished_by_ir_odom = True
                    break

                try:
                    self.http.post(
                        "/v1/realtime/chassis-velocity",
                        {"vx": float(self.speed_mps), "vy": 0.0, "wz": 0.0},
                        timeout=1.0,
                    )
                except Exception:
                    pass
                time.sleep(period)

                # 优先使用 odom 的本轮增量；读不到或冻结时保留开环兜底。
                old_distance_m = self.distance_m
                odom_delta_available = False
                if self._odo_start_x is not None:
                    try:
                        odo = self.http.get_odom_state() or {}
                        odo_data = odo.get("odom_state") or {}
                        current_x = odo_data.get("x")
                        if current_x is not None:
                            odom_dist = max(0.0, current_x - self._odo_start_x)
                            if (self._last_odom_x is not None
                                    and abs(current_x - self._last_odom_x) < 1e-9):
                                self._odom_stall_s += period
                            else:
                                self._odom_stall_s = 0.0
                            self._last_odom_x = current_x
                            if self._odom_stall_s >= 1.0:
                                self.distance_m += self.speed_mps * period
                            else:
                                self.distance_m = max(self.distance_m, odom_dist)
                            odom_delta_available = self.distance_m > old_distance_m
                    except Exception:
                        self.distance_m += self.speed_mps * period
                else:
                    self.distance_m += self.speed_mps * period
                movement_delta_m = max(0.0, self.distance_m - old_distance_m)
                self.elapsed_s = time.monotonic() - t0

                try:
                    ir_payload = self.http.get_ir_state() or {}
                    ir_data = ir_payload.get("ir_state") or {}
                    left_ir = ir_data.get("left")
                except Exception:
                    left_ir = None
                was_ir_lost = self.state.ir_lost
                ir_lost = self.state.update_ir(
                    left_ir, distance_m=movement_delta_m,
                )
                if ir_lost:
                    # IR 丢失阶段禁止再拿新球；确认帧已计入的位移不重复累加。
                    if was_ir_lost and not odom_delta_available:
                        movement_delta_m = self.speed_mps * period
                        self.distance_m += movement_delta_m
                    if was_ir_lost:
                        finished = self.state.add_post_loss_distance(movement_delta_m)
                    else:
                        finished = self.state.finished_by_ir_odom
                    if finished:
                        self.finished_by_ir_odom = True
                        print(f"  [{LOG_PREFIX}] IR 丢失后继续前进 "
                              f"{self.state.post_loss_distance_m:.3f}m, 结束搜索")
                        self._stop_event.set()
                        break
                    continue

                if self.elapsed_s >= self.max_seconds_s:
                    self.timed_out = True
                    print(f"  [{LOG_PREFIX}] creep 单次超时 {self.max_seconds_s:.0f}s, 结束搜索")
                    break

                try:
                    balls = target2.fetch_balls(
                        self.http, color_filter=None, debug=False,
                    )
                    if any(b.get("color") in (COLOR_BLUE, COLOR_YELLOW)
                           for b in balls):
                        self.balls = balls
                        self.found_ball = True
                        try:
                            self.http.post(
                                "/v1/realtime/chassis-velocity",
                                {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                                timeout=0.5,
                            )
                        except Exception:
                            pass
                        break
                except Exception:
                    pass
        finally:
            try:
                self.http.post(
                    "/v1/realtime/chassis-velocity",
                    {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                    timeout=1.0,
                )
            except Exception:
                pass
            self.completion_event.set()

    def wait_for_ball(self, timeout_s: float) -> dict:
        """阻塞等见球, 见球或超时返回。"""
        got = self.completion_event.wait(timeout=timeout_s)
        return {
            "balls": self.balls if got and self.found_ball else None,
            "finished_by_ir_odom": bool(self.finished_by_ir_odom),
            "distance_m": self.distance_m,
            "elapsed_s": self.elapsed_s,
        }

    def stop_and_join(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            try:
                self.http.post(
                    "/v1/realtime/chassis-velocity",
                    {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                    timeout=1.0,
                )
            except Exception:
                pass


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


def _track_leftmost_ball(
    *,
    max_seconds: float,
    dry_run: bool,
    soft_deadband: float = DEFAULT_TRACK_SOFT_DEADBAND,
    retry_seconds: float = DEFAULT_TRACK_RETRY_SECONDS,
    wide_deadband: float = DEFAULT_TRACK_WIDE_DEADBAND,
):
    """底盘视觉伺服: 把画面最左 (cx 最小) 的球拉到画面中心。

    走 main.chassis.track_chassis (现场标定的 sign/kp/v_max/slew),
    内部 finally 自动零速。返回 TrackChassisResult
    (arrived / reason / final_frame.label=cx 最小的球 label)。

    2026-08-06 第 7 次迭代: 现场反馈 "没有失败, 然后失败了也不结束".
    之前软成功只覆盖 timeout + final_frame 落入 [soft, 2*soft] 区间
    重试 — 拉得不够, 现场说"应该差不多对齐"时 final_frame 偏 0.30+
    (远处偏不到位的球) 仍被判失败 → 退出。
    改进: 五段式成功判据
      1. 硬停: 3 帧连续 cx_err/cy_err < 0.05 → arrived=True
      2. 软成: timeout, final_frame |cx_err| < soft_deadband (0.15) → 视为 arrived
      3. 软重试: timeout, final_frame |cx_err| ∈ [soft_deadband, 2*soft_deadband]
                  → 额外 1s 重试, 用 2x kp 收口
      4. 宽成: timeout, final_frame |cx_err| ∈ [2*soft_deadband, wide_deadband (0.45)]
                  → 视为 near_arrived_wide, 走 pick, 错误计数不算
      5. 硬失败: 偏 wide_deadband 之外 (cx_err > 0.45 ≈ 画面 1/4 宽) → 失败

    设计意图: "现场肉眼看着差不多对齐" = cx_err < wide_deadband → 走 pick 一次.
    pick 失败再计数. 避免单次 visual 偏一点就硬退.
    """
    # 用户反馈底盘抖动, 回调稳: kp=0.10, v_max=0.08, v_slew=0.02, hold=3。
    res = track_chassis(
        target=BALL_LABELS,
        select_mode="leftmost",
        setpoint_cxcy=(0.0, 0.0),
        kp=0.20,
        v_max=0.12,
        deadband=0.05,
        hold_frames=3,
        v_slew=0.04,
        decouple_xy=False,
        max_seconds=max_seconds,
        dry_run=dry_run,
    )

    # 2026-08-06: 已 arrived / 软成功 / 重试全部覆盖后, 真正的失败
    # (no_target / watchdog / stopped 等) 仍返回原 res. step_target4 自己按 reason 决定.
    if res.arrived or res.reason != "timeout":
        return res

    # 软成功判定: final_frame 在软死区内 → 视为 arrived
    ff = res.final_frame
    if ff is not None and ff.target_found:
        cx_err = ff.cx_err if ff.cx_err is not None else 0.0
        cy_err = ff.cy_err if ff.cy_err is not None else 0.0
        if abs(cx_err) < soft_deadband and abs(cy_err) < soft_deadband:
            print(f"  [{LOG_PREFIX}] �� track 软成功: timeout 但 final_frame |cx_err|="
                  f"{abs(cx_err):.3f} |cy_err|={abs(cy_err):.3f} "
                  f"均在软死区 {soft_deadband:.2f} 内, 视为 arrived")
            # 强行构造 arrived=True 返回: TrackChassisResult 是 dataclass, 替换
            res = TrackChassisResult(
                arrived=True,
                reason="near_arrived_soft",
                final_frame=ff,
                frames=res.frames,
                elapsed_s=res.elapsed_s,
                stop_ok=getattr(res, "stop_ok", True),
                motion_ok=getattr(res, "motion_ok", True),
                enc_delta=getattr(res, "enc_delta", None),
            )
            return res

        # 软重试: final_frame 偏 [soft_deadband, 2*soft_deadband] 区间
        # 再跑一次 retry_seconds 短时 track, 用更大 kp 让它"加把劲"收口
        if retry_seconds > 0 and abs(cx_err) < 2 * soft_deadband and abs(cy_err) < 2 * soft_deadband:
            print(f"  [{LOG_PREFIX}] �� track 软重试: 偏 [soft, 2*soft] 区间, "
                  f"再给 {retry_seconds:.1f}s 用更大 kp 收口")
            retry_res = track_chassis(
                target=BALL_LABELS,
                select_mode="leftmost",
                setpoint_cxcy=(0.0, 0.0),
                kp=0.20,
                v_max=0.12,
                deadband=0.05,
                hold_frames=3,
                v_slew=0.04,
                decouple_xy=False,
                max_seconds=retry_seconds,
                dry_run=dry_run,
            )
            if retry_res.arrived:
                print(f"  [{LOG_PREFIX}] �� track 软重试成功 arrived=True "
                      f"reason={retry_res.reason}")
                return retry_res
            # 软重试失败: 也用 final_frame 软死区判一次
            rff = retry_res.final_frame
            if rff is not None and rff.target_found:
                rcx = rff.cx_err if rff.cx_err is not None else 0.0
                rcy = rff.cy_err if rff.cy_err is not None else 0.0
                if abs(rcx) < soft_deadband and abs(rcy) < soft_deadband:
                    print(f"  [{LOG_PREFIX}] �� track 软重试后 final_frame 落入软死区: "
                          f"|cx_err|={abs(rcx):.3f} |cy_err|={abs(rcy):.3f}")
                    return TrackChassisResult(
                        arrived=True,
                        reason="near_arrived_soft_retry",
                        final_frame=rff,
                        frames=res.frames + retry_res.frames,
                        elapsed_s=res.elapsed_s + retry_res.elapsed_s,
                        stop_ok=getattr(retry_res, "stop_ok",
                                        getattr(res, "stop_ok", True)),
                        motion_ok=getattr(retry_res, "motion_ok",
                                          getattr(res, "motion_ok", True)),
                        enc_delta=getattr(
                            retry_res, "enc_delta",
                            getattr(res, "enc_delta", None)),
                    )
            # 软重试失败: 也检查宽死区
            rff = retry_res.final_frame
            if rff is not None and rff.target_found:
                rcx = rff.cx_err if rff.cx_err is not None else 0.0
                rcy = rff.cy_err if rff.cy_err is not None else 0.0
                if abs(rcx) < wide_deadband and abs(rcy) < wide_deadband:
                    print(f"  [{LOG_PREFIX}] �� track 软重试后落入宽死区 "
                          f"({wide_deadband:.2f}): |cx_err|={abs(rcx):.3f} "
                          f"|cy_err|={abs(rcy):.3f}, 视为 near_arrived_wide")
                    return TrackChassisResult(
                        arrived=True,
                        reason="near_arrived_wide",
                        final_frame=rff,
                        frames=res.frames + retry_res.frames,
                        elapsed_s=res.elapsed_s + retry_res.elapsed_s,
                        stop_ok=getattr(retry_res, "stop_ok",
                                        getattr(res, "stop_ok", True)),
                        motion_ok=getattr(retry_res, "motion_ok",
                                          getattr(res, "motion_ok", True)),
                        enc_delta=getattr(
                            retry_res, "enc_delta",
                            getattr(res, "enc_delta", None)),
                    )
            # 真正失败: 仍返回原 res (arrived=False, reason=timeout)
            print(f"  [{LOG_PREFIX}] ❌ track 软重试也失败: "
                  f"arrived={retry_res.arrived} reason={retry_res.reason}")

        # 宽成: 第一阶段 timeout 但 final_frame 在 [2*soft, wide_deadband] 区间
        # (软重试未触发), 视为"差不多对齐" → 走 pick
        elif abs(cx_err) < wide_deadband and abs(cy_err) < wide_deadband:
            print(f"  [{LOG_PREFIX}] �� track 宽成: 偏 [2*soft, wide] 区间, "
                  f"视为 near_arrived_wide: |cx_err|={abs(cx_err):.3f} "
                  f"|cy_err|={abs(cy_err):.3f} (< 宽死区 {wide_deadband:.2f})")
            return TrackChassisResult(
                arrived=True,
                reason="near_arrived_wide",
                final_frame=ff,
                frames=res.frames,
                elapsed_s=res.elapsed_s,
                stop_ok=getattr(res, "stop_ok", True),
                motion_ok=getattr(res, "motion_ok", True),
                enc_delta=getattr(res, "enc_delta", None),
            )

    return res


# 2026-08-06: track_chassis 在 no_target 场景也常常是"底盘响应但视觉丢了"
# 或"底盘真没动但视野里球仍在" —— 两种情况都是 set_chassis_velocity 没真正
# 生效 (CLAUDE.md 提的 OPEN chassis realtime-velocity no-motion bug).
# 解决: no_target 时主动读一次 odom_encoder 比对命令速度, 若 0.05s 内轮速
# 变化 < 阈值, 判定"底盘没动", 发一次强制 reset-stop + 直发轮速 IK 重启通信.
# 这只针对 no_target (视野里球还在但 lost_frames++), 不动 timeout / arrived.
# 见 _chassis_rearm_if_stuck() 详情.

def _chassis_rearm_if_stuck(http_client, *, settle_s: float = 0.5) -> bool:
    """底盘 stuck 检测 + 重新武装。

    流程:
      1. 读当前 wheel_encoders (fast-path, 单次 < 2ms)
      2. sleep settle_s 一段时间
      3. 再读 wheel_encoders
      4. 如果 4 轮编码器总变化 < 1.0 (≈ 0.5mm 累计, 极保守阈值)
         → 判定"底盘没动", 顺序 call:
            a) POST /v1/control/reset-stop (清 _stop_flag, 急停残留)
            b) POST /v1/realtime/chassis-velocity (vx=0, vy=0, wz=0)
            c) POST /v1/realtime/wheels/speeds (IK 反算 4 轮速, 通过 SerialEngine
               协调心跳, 跟 set_chassis_velocity 不同链路) ½s 内
            d) 再次发 vx=0 vy=0 wz=0
            返回 True (重武装成功)
         否则返回 False (底盘真的在动, 不需要 re-arm).

    这是粗暴的兜底: 不重建 chassis 引用 (那是 runtime 层), 也不重启守护线程
    (那是 force=True 路径). 只清 stop_flag + 重新下发 baseline 速度, 重置
    SerialEngine 的 IK 命令缓存.
    """
    try:
        e1 = http_client.get(f"{http_client.api_prefix}/realtime/wheels/encoders")
    except Exception:
        return False
    if not isinstance(e1, dict):
        return False
    enc1 = e1.get("encoders") or []
    if not isinstance(enc1, list) or len(enc1) < 4:
        return False
    try:
        enc1 = [float(x) for x in enc1]
    except (TypeError, ValueError):
        return False

    import time as _t
    _t.sleep(settle_s)

    try:
        e2 = http_client.get(f"{http_client.api_prefix}/realtime/wheels/encoders")
    except Exception:
        return False
    if not isinstance(e2, dict):
        return False
    enc2 = e2.get("encoders") or []
    if not isinstance(enc2, list) or len(enc2) < 4:
        return False
    try:
        enc2 = [float(x) for x in enc2]
    except (TypeError, ValueError):
        return False

    total_delta = sum(abs(enc2[i] - enc1[i]) for i in range(4))
    if total_delta >= 1.0:
        # 底盘在动, 不需要 re-arm
        return False

    # 底盘 stuck: 重武装
    try:
        http_client.post(f"{http_client.api_prefix}/control/reset-stop", payload={})
    except Exception:
        pass
    attempts = [
        ("realtime/chassis-velocity", {"vx": 0.0, "vy": 0.0, "wz": 0.0}),
        ("realtime/wheels/speeds", {"speeds": [0.0, 0.0, 0.0, 0.0]}),
    ]
    for path, payload in attempts:
        try:
            http_client.post(f"{http_client.api_prefix}/{path}", payload=payload, timeout=1.0)
        except Exception:
            pass
    try:
        http_client.post(
            f"{http_client.api_prefix}/realtime/chassis-velocity",
            {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            timeout=1.0,
        )
    except Exception:
        pass
    return True


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
    pick_timeout_s: float,  # noqa: ARG001 — 保留参数兼容性, 当前同步流程不直接用
    pick_x_mm: float = X_PICK_MM,
    pick_y_mm: float = Y_PICK_MM,
    transit_y_mm: float = Y_TRANSIT_MM,
    transit_x_mm: float = X_TRANSIT_MM,
    put_y_mm: float = Y_PUT_MM,
    bin_x_mm: float = BIN_X_MM[COLOR_BLUE],
    bin_y_mm: float = Y_PUT_MM,
    bin_hand_deg: float = TASK4_POSE_P_HAND_DEG,
) -> dict:
    """底盘对齐后盲降抓球 + 同步放 bin。

    流程 (同步, 6 步, 无 sleep):
      0. composite_run x=pick_x                盲降前横移
      1. composite_run y=Y_PICK                盲降到抓球位
      2. grasp(True)                           真空开
      3. composite_run y=Y_TRANSIT, x=X_TRANSIT  抬到中转位
      4. composite_run x=bin_x                 横移到 bin 上方
      5. composite_run y=bin_y, hand=bin_hand  降到放仓位
      6. grasp(False)                          放气

    Returns:
        {"ok": bool, "error": str|None, "release_thread": None}
    """
    bin_x = BIN_X_MM[color]

    # 0. 盲降前横移到 pick_x (待现场测; 短距, belt-slip 风险低)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [0/6] composite_run(x={pick_x_mm:+.0f})  盲降前横移到 {pick_x_mm}")
    try:
        arm_client.composite_run(x_mm=pick_x_mm, speed=80, timeout=30.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(x={pick_x_mm}) 横移失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 1. 盲降到抓球位
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [1/6] composite_run(y={pick_y_mm:+.0f})  盲降到抓球位")
    try:
        arm_client.composite_run(y_mm=pick_y_mm, speed=80, timeout=10.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(y={pick_y_mm}) 盲降失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 2. grasp + 直接下一动作 (无 sleep, SDK 真空建立自闭环)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [2/6] grasp(True)  真空开 (无 sleep)")
    try:
        runner.grasp(True, timeout=5.0)
    except Exception as e:
        return {"ok": False,
                "error": f"grasp(True) 失败: {type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 3. 抬到中转位 (y=transit_y) ∥ 横移到中转位 x=transit_x (composite_run 并行)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [3/6] composite_run(y={transit_y_mm:+.0f}, x={transit_x_mm:+.0f})  "
          f"抬升+横移到中转位")
    try:
        arm_client.composite_run(y_mm=transit_y_mm, x_mm=transit_x_mm,
                                 speed=80, timeout=30.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(y={transit_y_mm}, x={transit_x_mm}) 中转失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 4. 横移到 bin 上方 (中转 x=transit_x → bin_x)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [4/6] composite_run(x={bin_x:+.0f})  横移到 {color} bin 上方")
    try:
        arm_client.composite_run(x_mm=bin_x, speed=80, timeout=30.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(x={bin_x}) 横移失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 5. 降到放仓位 (中转 y=transit_y → bin y=bin_y_mm, 同时调整 hand=bin_hand_deg)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [5/6] composite_run(y={bin_y_mm:+.0f}, hand={bin_hand_deg:+.0f})  降到放仓位")
    try:
        arm_client.composite_run(y_mm=bin_y_mm, hand=bin_hand_deg,
                                 speed=80, timeout=10.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(y={bin_y_mm}, hand={bin_hand_deg}) 降放仓位失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 6. 放气 (无 sleep, 直接让下一球 goto_pose_p 回 P 姿态)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [6/6] grasp(False)  放气 (无 sleep)")
    try:
        runner.grasp(False, timeout=5.0)
    except Exception as e:
        return {"ok": False,
                "error": f"grasp(False) 失败: {type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    return {"ok": True, "error": None, "release_thread": None}


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
    # ---- 姿态参数 (默认值在模块级常量, 可由外部覆盖) ----
    pose_p_y_mm: float = TASK4_POSE_P_Y_MM,
    pose_p_x_mm: float = TASK4_POSE_P_X_MM,
    pose_p_arm_deg: float = TASK4_POSE_P_ARM_DEG,
    pose_p_hand_deg: float = TASK4_POSE_P_HAND_DEG,
    pick_x_mm: float = X_PICK_MM,
    pick_y_mm: float = Y_PICK_MM,
    transit_y_mm: float = Y_TRANSIT_MM,
    transit_x_mm: float = X_TRANSIT_MM,
    put_y_mm: float = Y_PUT_MM,
    bin_x_blue_mm: float = BIN_X_MM[COLOR_BLUE],
    bin_x_yellow_mm: float = BIN_X_MM[COLOR_YELLOW],
    bin_y_blue_mm: float = BIN_Y_MM.get(COLOR_BLUE, Y_PUT_MM),
    bin_y_yellow_mm: float = Y_PUT_MM,
    bin_hand_blue_deg: float = BIN_HAND_DEG.get(COLOR_BLUE, TASK4_POSE_P_HAND_DEG),
    bin_hand_yellow_deg: float = TASK4_POSE_P_HAND_DEG,
    defer_task5_handoff: bool = False,
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
        debug_recognition: 是否打印每条 detection 的过滤原因。
        defer_task5_handoff: task4 由 orchestrator 调度时，IR+odom 结束后将
            关仓 + task5 Phase 1 姿态交给巡航线程并行执行；独立运行保持 False。

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
    # orchestrator 已经用左 IR 触发 task4；该状态跨每一球 creep worker 保留。
    search_state = _Task4SearchState(ir_started=True)
    t_start = time.monotonic()
    global _TASK4_T0
    _TASK4_T0 = t_start

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
        if not dry_run:
            try:
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
        last_color = None  # 上一球颜色 (用于优化 P 姿态恢复)
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

            # 2.0 P 姿态 + creep 并发 (2026-08-08: 改回并发, 避免 composite_run 期间车子完全静止)
            #    2026-08-09 用户拍板: **只在任务刚触发 (第 1 球) 并发**, 之后每球
            #    顺序执行 (先 P 姿态, 再 creep)。并发时 3 个风险:
            #      a) creep_thread 与 composite_run 同时跑 → 4 路臂命令 + 底盘 vx
            #         抢 SerialEngine, composite_run 内部 y 下降偶尔卡 200-500ms;
            #      b) 见球瞬间 creep 没主动发 0 速, 靠 track_chassis 第一帧覆盖,
            #         50-150ms 窗口内底盘按 creep 速度窜一下 → "track 开始瞬间抖";
            #      c) arm_feed 守护线程 20Hz 轮询 y/x/arm_angle 跟 composite_run
            #         4 路并发抢 share_key, 进一步加长 composite_run 时间.
            #    现场每球 creep 0.000m → 错过球, 这 3 个风险在并发下被放大。
            if release_thread is not None and release_thread.is_alive():
                release_thread.join(timeout=15.0)

            # 首球 (任务刚触发): creep 和 P 姿态 composite_run 并发省时间;
            # 后续球: 顺序 —— 先 P 姿态到位, 再启动 creep, 避免抢串口错过球。
            # IR 状态跨球共享，首球也参与远端确认；任务触发时已是近 IR。
            concurrent_creep = (ball_idx == 1)
            creep_thread = None
            if concurrent_creep:
                creep_thread = _CreepThread(
                    http_client, state=search_state,
                    speed_mps=creep_speed_mps,
                    max_distance_m=remaining_m,
                    poll_hz=CREEP_POLL_HZ,
                    max_seconds_s=CREEP_MAX_SECONDS_S,
                )
                creep_thread.start()

            pose_ok = True
            if not dry_run:
                try:
                    # ---- 動態決定傳哪些軸 (優化延遲):
                    #   - 放 bin 後 y 已在 -130 (P 姿態 y), 不需要再動 y
                    #   - 黃色球 hand 已在 10° (P 姿態 hand), 不需要再動 hand
                    #   - 藍色球 hand 在 -75°, 需要恢復到 10°
                    #   - x 從 bin_x (0 或 -60) 走到 -295, 行程約 235-295mm,
                    #     單獨用 200 速度快移, 其餘軸保持 100 不影響安全。
                    if last_color != COLOR_YELLOW:
                        # 藍色球 / 第一球: 需要恢復 hand, 先快速移 x 再恢復 arm+hand
                        print(f"\n[{_ts_str()}] ========== {LOG_PREFIX}/球{ball_idx} 恢复 P 姿態 "
                              f"(x 快速橫移 → arm/hand) ==========")
                        arm_client.composite_run(
                            x_mm=pose_p_x_mm,
                            speed=200,
                            timeout=30.0,
                        )
                        arm_client.composite_run(
                            arm=pose_p_arm_deg,
                            hand=pose_p_hand_deg,
                            speed=100,
                            timeout=30.0,
                        )
                    else:
                        # 黃色球: hand 已在 10°, 只需恢復 arm + x (x 仍快速)
                        print(f"\n[{_ts_str()}] ========== {LOG_PREFIX}/球{ball_idx} 恢复 P 姿態 "
                              f"(x 快速橫移 → arm) ==========")
                        arm_client.composite_run(
                            x_mm=pose_p_x_mm,
                            speed=200,
                            timeout=30.0,
                        )
                        arm_client.composite_run(
                            arm=pose_p_arm_deg,
                            speed=100,
                            timeout=30.0,
                        )
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

            # 2.2 底盘视觉定位 (最左球 → 畫面中心)
            #    只用底盤對齊 (track_chassis), 跳過臂側 find_target /
            #    move_to_vision_target —— 假設底盤對齊後吸嘴已在球正上方,
            #    直接 composite_run(y_bin高位) → y 下 → grasp 即可。
            #    2026-08-08: track 段也計入里程計預算, 避免過沖。
            odo_before = None
            if not dry_run:
                try:
                    odo_before = http_client.call(
                        "car", "get_odometry", timeout=5.0)
                except Exception:
                    pass
            print(f"  [{LOG_PREFIX}] 🎯 track_chassis(leftmost, "
                  f"≤{track_max_seconds:.0f}s) — 僅底盤對齊, 跳過臂視覺伺服")
            track_res = _track_leftmost_ball(
                max_seconds=track_max_seconds, dry_run=dry_run,
            )
            if not dry_run and odo_before is not None:
                try:
                    odo_after = http_client.call(
                        "car", "get_odometry", timeout=5.0)
                    if (isinstance(odo_before, dict) and isinstance(odo_after, dict)
                            and odo_before.get("status") == "succeeded"
                            and odo_after.get("status") == "succeeded"):
                        x_before = (odo_before.get("result") or {}).get("x", 0)
                        x_after = (odo_after.get("result") or {}).get("x", 0)
                        track_dist = max(0.0, x_after - x_before)
                        if track_dist > 0:
                            total_creep_m += track_dist
                            print(f"  [{LOG_PREFIX}] 📏 track 段前移 "
                                  f"{track_dist:.3f}m (里程計差值), "
                                  f"累計前移 {total_creep_m:.3f}m")
                except Exception:
                    pass
            print(f"  [{LOG_PREFIX}] track 結束: arrived={track_res.arrived} "
                  f"reason={track_res.reason}")
            # 2026-08-09: 下沉后 align 返回 stop_ok / motion_ok —— finally 零速是否
            # 到达轮子 / 期间轮子是否物理位移 (真实编码器反馈)。stop_ok=False =
            # 命令路径断了 (串口/下位机异常) 或车仍在滑行; motion_ok=False = 发了
            # 命令但编码器没动 ("200 但轮不转" 假死)。两者任一 → 先显式停稳,
            # 再决定是否重武装 (旧代码只 trust track_chassis 内部 finally)。
            if not dry_run and (
                not getattr(track_res, "stop_ok", True)
                or not getattr(track_res, "motion_ok", True)
            ):
                print(f"  [{LOG_PREFIX}] ⚠️ track 命令/位移异常 "
                      f"(stop_ok={getattr(track_res, 'stop_ok', True)} "
                      f"motion_ok={getattr(track_res, 'motion_ok', True)}), 显式停稳...")
                try:
                    http_client.post(
                        f"{http_client.api_prefix}/realtime/chassis-velocity",
                        {"vx": 0.0, "vy": 0.0, "wz": 0.0}, timeout=2.0)
                except Exception:
                    pass
                try:
                    http_client.wait_wheels_stopped(settle_s=0.15, timeout_s=1.0)
                except Exception:
                    pass
            # 2026-08-06 软成功 / 宽成 提示. arrived=True 但 reason 是 near_arrived_*
            # 表示 timeout 内没硬停但 final_frame 落入软/宽死区, 视为对齐.
            if track_res.arrived and track_res.reason in (
                "near_arrived_soft", "near_arrived_soft_retry", "near_arrived_wide",
            ):
                ff = track_res.final_frame
                if ff is not None:
                    th = (DEFAULT_TRACK_WIDE_DEADBAND
                          if track_res.reason == "near_arrived_wide"
                          else DEFAULT_TRACK_SOFT_DEADBAND)
                    print(f"  [{LOG_PREFIX}] ✨ 软成功对齐 ({track_res.reason}): "
                          f"label={ff.label} "
                          f"|cx_err|={abs(ff.cx_err or 0):.3f} "
                          f"|cy_err|={abs(ff.cy_err or 0):.3f} "
                          f"(< 阈值 {th:.2f})")

            # 2026-08-06 第 6 次迭代: no_target 现场反馈 "看到之后动都不动".
            #   这是 CLAUDE.md 顶部提的 OPEN chassis realtime-velocity no-motion bug
            #   的一种表现: track_chassis 期间 _set_vel 多次成功 (没抛), 但轮速
            #   实际没下发 (SerialEngine 队列 / _stop_flag / 串口抢锁各种原因).
            #   lost_frames++ 是因为 "球在视野里没动 → cx_err 不收敛 → 选取失败"
            #   而不是 "球真的消失". 救治思路: 在 no_target 退出后, 主动检查
            #   encoder 是否真没动, 若没动 → 一次 reset-stop + 强制 0 速重下发
            #   ("重武装"), 给下一拍 set_chassis_velocity 一次干净的环境.
            # 2026-08-09: 下沉后 reason 集合变了 — 串口掉线但视觉仍活时车不动,
            #   cx_err 不收敛 → 满预算 timeout (不是 no_target); 若命令路径彻底
            #   断了, align 现在会 control_lost 快速退出 + stop_ok=False; 甚至
            #   arrived 但 motion_ok=False (发了命令编码器没动 = 200 但轮不转)。
            # 2026-08-09 用户: "为什么老是底盘重武装".
            #   根因 1: needs_rearm 把 no_target / watchdog 也算进去 —— 这俩是
            #   视觉丢球 (task_feed 没球帧), 底盘响应正常; 重武装救不了视觉,
            #   却让每球白烧 0.5s settle + 3s retry (现场连续失败 1/2 的元凶)。
            #   根因 2: _chassis_rearm_if_stuck 在 track 已 finally 零速后采样
            #   编码器 —— 车已停, 0.5s 内编码器必 < 1.0 → 永远判"底盘无响应"。
            #   重武装只应在"命令路径确实失败"时触发: runtime 已在整个 track
            #   窗口用真实编码器算好 motion_ok / stop_ok / control_lost, 直接信它。
            track_trusted = bool(track_res.arrived) and getattr(
                track_res, "motion_ok", True)
            needs_rearm = (
                not dry_run
                and not track_trusted
                and (
                    track_res.reason == "control_lost"
                    or not getattr(track_res, "stop_ok", True)
                    or not getattr(track_res, "motion_ok", True)
                )
            )
            if needs_rearm:
                # runtime 已判定命令路径假死 (control_lost / stop_ok=False /
                # motion_ok=False) → 重武装: reset-stop + 0 速 + 直发轮速 IK,
                # 重建 SerialEngine 命令缓存, 给下一拍 set_chassis_velocity
                # 一次干净环境。motion_ok/stop_ok 已是整个 track 窗口的判定,
                # 无需再采样 (stopped-sample 必 < 1.0)。
                print(f"  [{LOG_PREFIX}] [REARM] 底盘命令路径异常 "
                      f"(reason={track_res.reason} stop_ok="
                      f"{getattr(track_res, 'stop_ok', True)} motion_ok="
                      f"{getattr(track_res, 'motion_ok', True)}), "
                      f"重武装后重试一次 track")
                _chassis_rearm_if_stuck(http_client, settle_s=0.5)
                # 重试 1 次: 拿 3s 给一次集中机会, 若再失败就硬失败
                retry = track_chassis(
                    target=BALL_LABELS,
                    select_mode="leftmost",
                    setpoint_cxcy=(0.0, 0.0),
                    kp=0.10,
                    v_max=0.08,
                    hold_frames=3,
                    v_slew=0.02,
                    max_seconds=min(3.0, track_max_seconds),
                    dry_run=dry_run,
                )
                if retry.arrived:
                    track_res = retry
                    print(f"  [{LOG_PREFIX}] [REARM] 重试 track 成功: "
                          f"arrived=True reason={retry.reason}")

            # 2026-08-06: track 失败要计入 n_consecutive_failures, 否则
            # 永不触发 max_consecutive_pick_failures 退出 —— 这是 task4
            # "卡住 + 跳过"的隐性根因之一. reason ∈ {arrived} 视为成功,
            # 其他 {timeout, no_target, watchdog, stopped, control_lost} 全算失败.
            track_ok = bool(track_res.arrived)
            if not track_ok and not dry_run:
                n_consecutive_failures += 1
                n_pick_failures += 1
                history.append({
                    "ball": ball_idx, "action": "track_failed",
                    "color": None,
                    "error": f"track reason={track_res.reason}",
                })
                print(f"  [{LOG_PREFIX}] ❌ track 失败 (reason={track_res.reason}); "
                      f"连续失败 {n_consecutive_failures}/{max_consecutive_pick_failures}")
                if n_consecutive_failures >= max_consecutive_pick_failures:
                    final_reason = "pick_error_exceeded"
                    print(f"  [{LOG_PREFIX}] ❌ 连续失败达到 "
                          f"{max_consecutive_pick_failures}, 退出循环")
                    break
                # 兜底: track 失败本轮不再走 pick, 直接下一轮 (creep 继续搜)
                continue

            # 2.3 定颜色:
            #    creep 阶段已经把当前帧 balls 拿到 (creep_res['balls']),
            #    直接复用 — 不再 track 完再 fetch_balls 一次, 省 1 个 HTTP RTT。
            #    优先用 creep 看到的 balls; track label 兜底; 都没有就 skip。
            color = _color_from_track(track_res)
            if color is None and not dry_run:
                best = _pick_best_ball(creep_res["balls"] or [])
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
            print(f"  [{LOG_PREFIX}] ✓ 锁定 {color} 球, 直接进入抓手姿态")

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
                pick_x_mm=pick_x_mm,
                pick_y_mm=pick_y_mm,
                transit_y_mm=transit_y_mm,
                transit_x_mm=transit_x_mm,
                put_y_mm=put_y_mm,
                bin_x_mm=bin_x_blue_mm if color == COLOR_BLUE else bin_x_yellow_mm,
                bin_y_mm=bin_y_blue_mm if color == COLOR_BLUE else bin_y_yellow_mm,
                bin_hand_deg=bin_hand_blue_deg if color == COLOR_BLUE else bin_hand_yellow_deg,
            )
            release_thread = res.get("release_thread")  # 记录放仓线程, 下轮回P前 join
            history.append({"ball": ball_idx,
                            "action": "picked" if res["ok"] else "pick_failed",
                            "color": color, "error": res["error"]})
            if res["ok"]:
                n_picks += 1
                n_consecutive_failures = 0
                last_color = color  # 记录上一球颜色, 供 P 姿态恢复优化
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
