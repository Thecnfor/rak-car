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


LOG_PREFIX: str = LOG_PREFIX_TASK4 + "/target4"


# ---- 默认参数 (2026-08-03 底盘视觉伺服版) ----

DEFAULT_MAX_PICKS: int = 8
"""最多抓取数 (比赛正常 6-8 球, 给 buffer)。"""

DEFAULT_CREEP_SPEED_MPS: float = 0.01
"""creep 搜索前移速度 (m/s)。慢 = 帧覆盖密, 不漏球; 快 = 省时间。
2026-08-03 用户现场反馈 0.03 太慢, ×1.5 → 0.045。
2026-08-06 用户: 再减半 → 0.0225。"""

DEFAULT_MAX_CREEP_M: float = 0.8
"""累计前移距离预算 (m, 开环 速度×时间 记账)。旧版总行程 0.56m + 余量。"""

DEFAULT_MAX_SECONDS: float = 180.0
"""任务总时长预算 (s)。"""

DEFAULT_TRACK_MAX_SECONDS: float = 6.0
"""单球底盘视觉伺服收敛预算 (s)。超时但目标仍在画面 → 照样试抓 (盲降)。
2026-08-04: 12→8, 配合下面 kp/v_max 提速, 正常 3-5s 收敛。
2026-08-05: 8→6, 配合进一步提速档 (kp=0.40/v_max=0.25/hold=2/slew=0.05),
正常 2-3s 收敛, 给 6s 已留 buffer。"""

DEFAULT_MAX_CONSECUTIVE_PICK_FAILURES: int = 1
"""连续 pick 失败超过此数 → 退出 (单个球抓不起不应拖垮全场)。
2026-08-03 现场: 失败一次就退出, 避免 'P 姿态 → creep → track → pick 失败 →
恢复 P 姿态 → creep → track → pick 失败' 的无限循环。"""

DEFAULT_RETURN_X_MM: Optional[float] = POSE_P_X_MM
"""放 bin 后 x 回的目标位置 (mm)。默认 = POSE_P_X (P 姿态 x), None = 不回。
回 P 姿态便于下一球直接走视觉伺服;若只跑单球可传 None。"""

DEFAULT_PICK_TIMEOUT_S: float = 60.0
"""pick_by_vision 总超时 (s)。"""

CREEP_POLL_HZ: float = 10.0
"""creep 期间 fetch_balls 轮询频率。"""

# ---- 后台保前移线程 (P姿态+creep 并发) ----

class _CreepThread:
    """后台线程保底盘前移 + 主线程摆臂。

    设计 (2026-08-03 现场决定):
      - 后台线程持续下发 vx=creep_speed, 同时 10Hz 轮询 fetch_balls
      - 见球 → 自己写 0 速 + 抛 ball_event
      - 主线程 wait_for_ball() 阻塞等 ball_event 或超时
      - 主线程 stop_and_join() 兜底清场
      - finally 里 _set_chassis_vel(0) 保速度一定清零
    """
    def __init__(self, http_client, *, speed_mps: float, max_distance_m: float,
                 poll_hz: float = CREEP_POLL_HZ):
        import threading
        self.http = http_client
        self.speed_mps = speed_mps
        self.max_distance_m = max_distance_m
        self.poll_hz = poll_hz
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="task4-creep")
        self._stop_event = threading.Event()
        self.ball_event = threading.Event()
        self.distance_m = 0.0
        self.elapsed_s = 0.0
        self.balls = None
        self.found_ball = False   # 2026-08-04: 见球退出 → 不零速, 交给 track_chassis
        self._t0 = 0.0

    def start(self) -> None:
        self._t0 = time.monotonic()
        self._thread.start()

    def _loop(self) -> None:
        period = 1.0 / max(self.poll_hz, 1.0)
        t0 = time.monotonic()
        dist = 0.0
        try:
            while not self._stop_event.is_set():
                # 1. 下发前移速度
                try:
                    self.http.post(
                        "/v1/realtime/chassis-velocity",
                        {"vx": float(self.speed_mps), "vy": 0.0, "wz": 0.0},
                        timeout=1.0,
                    )
                except Exception:
                    pass
                time.sleep(period)
                dist += self.speed_mps * period
                self.distance_m = dist
                self.elapsed_s = time.monotonic() - t0
                # 2. 距离预算耗尽就停
                if dist >= self.max_distance_m:
                    break
                # 3. fetch_balls 轮询, 见球 → 标记 + 停
                try:
                    balls = target2.fetch_balls(
                        self.http, color_filter=None, debug=False,
                    )
                    if any(b.get("color") in (COLOR_BLUE, COLOR_YELLOW)
                           for b in balls):
                        self.balls = balls
                        self.found_ball = True
                        self.ball_event.set()
                        break
                except Exception:
                    pass
        finally:
            # 2026-08-04 (用户: 看见球别停一下): 见球退出 → 不零速, 保持 creep
            #   前移交给 track_chassis 无缝接管 (track_chassis finally 自己零速)。
            #   只有未见球 (预算耗尽/被叫停) 才零速 —— 那条路没有接管者。
            if not self.found_ball:
                try:
                    self.http.post(
                        "/v1/realtime/chassis-velocity",
                        {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                        timeout=1.0,
                    )
                except Exception:
                    pass

    def wait_for_ball(self, timeout_s: float) -> dict:
        """阻塞等见球, 见球或超时返回。"""
        got = self.ball_event.wait(timeout=timeout_s)
        return {
            "balls": self.balls if got else None,
            "distance_m": self.distance_m,
            "elapsed_s": self.elapsed_s,
        }

    def stop_and_join(self) -> None:
        # 2026-08-04: 零速交给 _loop 的 finally (见球不零速/未见球零速), 这里不再
        #   冗余二次 POST (省一次 HTTP往返 ≈0.2s)。仅 set+join; 若 join 超时线程
        #   仍活着 (异常未走 finally) 才兜底零速。
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

# ---- P 姿态参数 (可由外部覆盖) ----

TASK4_POSE_P_Y_MM: float = -130.0
TASK4_POSE_P_X_MM: float = -300.0
TASK4_POSE_P_ARM_DEG: float = 90.0
TASK4_POSE_P_HAND_DEG: float = 10.0

# ---- 放 bin 参数 (2026-08-05 用户拍板: 新放球序列) ----

BIN_X_MM = {COLOR_BLUE: 0.0, COLOR_YELLOW: -65.0}
"""蓝 bin x=0, 黄 bin x=-65。"""

X_PICK_MM: float = -248.0
"""盲降前 x (2026-08-06 用户: -280 → -240)。从 P 姿态 x=-300 → -240 (更靠中间)。
⚠️ 待现场测: 横移距离短 = belt-slip 风险低; x=-240 是否能让吸嘴落在球心
上方需要 13_nozzle_align_pose_p.py 标定的 (sx, sy) 配合验证。
如果 (sx, sy) 偏移明显, 球径 4cm 仍允许小偏差抓到, 不用回伺服。"""

Y_PICK_MM: float = -58.0
"""抓球 y (吸盘贴近球面)。"""

Y_TRANSIT_MM: float = -130.0
"""中转 y (2026-08-05 用户拍板 -190 → -130, 不需要那么深的中转位)。
留出 y=-130 是放仓位 y=-110 之前的过渡 — y_gate 上界 -145, 中转位高于
gate 上界, 业务层 OK (开仓舵机在 y ∈ [-205,-145] 才卡; 我们不放仓,
只在这里横移)。"""

X_TRANSIT_MM: float = -150.0
"""中转 x (2026-08-05 用户拍板)。从 P 姿态 x=-300 → 中转 -150 (车体中线
附近), 然后再横移到 bin x=0/-65。两次小位移, belt-slip 风险低。"""

Y_PUT_MM: float = -110.0
"""放球 y (2026-08-06 用户: -100 → -110, 再深 10mm)。"""

Y_FINAL_MM: float = -133.0
"""最终 y (识别位姿, 历史值, 下一阶段 target 识别用)。"""

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
    # 2026-08-06 (用户: 底盘抖动严重, 大幅回调稳):
    #   kp 0.40 → 0.10 (保守增益, 消除振荡)
    #   v_max 0.25 → 0.08 (低速对齐, 稳)
    #   v_slew 0.05 → 0.02 (极平滑加减速)
    #   hold_frames 2 → 3 (连续3帧才判到, 防误判)
    return track_chassis(
        target=BALL_LABELS,
        select_mode="leftmost",
        setpoint_cxcy=(0.0, 0.0),
        kp=0.10,
        v_max=0.08,
        hold_frames=3,
        v_slew=0.02,
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
    pick_timeout_s: float,  # noqa: ARG001 — 保留参数兼容性, 当前同步流程不直接用
    pick_x_mm: float = X_PICK_MM,
    pick_y_mm: float = Y_PICK_MM,
    transit_y_mm: float = Y_TRANSIT_MM,
    transit_x_mm: float = X_TRANSIT_MM,
    put_y_mm: float = Y_PUT_MM,
    bin_x_mm: float = BIN_X_MM[COLOR_BLUE],
) -> dict:
    """底盘对齐后盲降抓球 + 同步放 bin (2026-08-05 用户拍板: 新放球序列)。

    历史迭代:
      - 2026-08-03 (旧): track_chassis → find_target_arm_cross (吸嘴对齐, 3-5s) →
        下探 + grasp + 后台放球
      - 2026-08-05 v1: 砍吸嘴对齐, 同步放球 (5 步)
      - **2026-08-05 v2 (当前)**: 用户拍板, 进一步优化:
        1. 中转位 (y=-130, x=-150) 取代原 y=-190, x=-300 (车体中线过渡,
           两次小位移代替一次大位移, belt-slip 风险更低)
        2. 放仓位 y=-110 (原来 -100, 再深 10mm)
        3. 盲降前 x=-240 横移 (待现场测, 球心可能在这个 x 上方)
        4. **两个 grasp 后不要 sleep, 直接下一动作** (真空建立靠 SDK 自闭环)

    新流程 (同步, 6 步, 无 sleep):
      0. composite_run x=-240                盲降前横移到 -240
      1. composite_run y=Y_PICK              盲降到抓球位
      2. grasp(True)                         真空开 (无 sleep)
      3. composite_run y=Y_TRANSIT, x=X_TRANSIT  抬到中转 (-130, -150)
      4. composite_run x=bin_x               横移到 bin 上方
      5. composite_run y=Y_PUT               降到放仓位 (-110)
      6. grasp(False)                        放气 (无 sleep)

    Returns:
        {"ok": bool, "error": str|None, "release_thread": None}
    """
    bin_x = BIN_X_MM[color]

    # 0. 盲降前横移到 pick_x (待现场测; 短距, belt-slip 风险低)
    print(f"  [{LOG_PREFIX}] [0/6] composite_run(x={pick_x_mm:+.0f})  盲降前横移到 {pick_x_mm}")
    try:
        arm_client.composite_run(x_mm=pick_x_mm, speed=80, timeout=30.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(x={pick_x_mm}) 横移失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 1. 盲降到抓球位
    print(f"  [{LOG_PREFIX}] [1/6] composite_run(y={pick_y_mm:+.0f})  盲降到抓球位")
    try:
        arm_client.composite_run(y_mm=pick_y_mm, speed=80, timeout=10.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(y={pick_y_mm}) 盲降失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 2. grasp + 直接下一动作 (无 sleep, SDK 真空建立自闭环)
    print(f"  [{LOG_PREFIX}] [2/6] grasp(True)  真空开 (无 sleep)")
    try:
        runner.grasp(True, timeout=5.0)
    except Exception as e:
        return {"ok": False,
                "error": f"grasp(True) 失败: {type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 3. 抬到中转位 (y=transit_y) ∥ 横移到中转位 x=transit_x (composite_run 并行)
    print(f"  [{LOG_PREFIX}] [3/6] composite_run(y={transit_y_mm:+.0f}, x={transit_x_mm:+.0f})  "
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
    print(f"  [{LOG_PREFIX}] [4/6] composite_run(x={bin_x:+.0f})  横移到 {color} bin 上方")
    try:
        arm_client.composite_run(x_mm=bin_x, speed=80, timeout=30.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(x={bin_x}) 横移失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 5. 降到放仓位 (中转 y=transit_y → bin y=put_y)
    print(f"  [{LOG_PREFIX}] [5/6] composite_run(y={put_y_mm:+.0f})  降到放仓位")
    try:
        arm_client.composite_run(y_mm=put_y_mm, speed=80, timeout=10.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(y={put_y_mm}) 降放仓位失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 6. 放气 (无 sleep, 直接让下一球 goto_pose_p 回 P 姿态)
    print(f"  [{LOG_PREFIX}] [6/6] grasp(False)  放气 (无 sleep)")
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
        # ---- 0. 打开存储仓 (2026-08-04 用户: task4 开始开仓, 结束关仓) ----
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

        # ---- 1. 准备位姿: 已删 ----
        #    2026-08-04 (用户): 开头那个 target1 准备位姿 (~8s) 是冗余 legacy —
        #    主循环第一球立刻 goto_pose_p 覆盖它。P 姿态才是真正的搜索姿态。
        #    直接从 P 姿态开始 creep, 省 ~8s。

        # ---- 2. 主循环: P姿态+creep 并发 → 见球 → 抓取 → 放仓 → 循环 ----
        #    2026-08-03 现场 (用户拍板):
        #      a) P 姿态 + creep 并发 (背景线程保前移, 主线程摆臂)
        #      b) 见球 → creep 停 → track → pick → 放仓固定两位置
        #      c) 再恢复 P 姿态 + creep 继续, 循环到 8 球
        #      d) 失败一次就退出, 不死循环
        ball_idx = 0
        release_thread = None  # 2026-08-04: 上一球放仓后台线程, 回P前需 join
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

            # 2.0 P 姿态 + creep 并发:
            #    - 后台线程保前移 (chassis-velocity realtime 通道, 见球或超时即停)
            #    - 主线程同步摆臂到 P 姿态 (composite_run)
            #    - 见球 → 主线程通知后台停 → 等后台清理
            creep_thread = _CreepThread(
                http_client, speed_mps=creep_speed_mps,
                max_distance_m=remaining_m,
                poll_hz=CREEP_POLL_HZ,
            )
            creep_thread.start()
            # 2026-08-04: 先等上一球放仓后台线程结束, 再回 P 姿态 ——
            #   放仓线程不回 P, 由这里统一回; 不 join 会与其臂命令竞争。
            #   (chassis 的 creep 已在上面 start, join 期间底盘照常前移不浪费时间)
            if release_thread is not None and release_thread.is_alive():
                release_thread.join(timeout=15.0)
            if not dry_run:
                try:
                    print(f"\n========== {LOG_PREFIX}/球{ball_idx} 恢复 P 姿态 "
                          f"(composite_run 4 轴同步: y={pose_p_y_mm} → x={pose_p_x_mm} "
                          f"arm={pose_p_arm_deg}°/hand={pose_p_hand_deg}°) ==========")
                    arm_client.composite_run(
                        arm=pose_p_arm_deg,
                        x_mm=pose_p_x_mm,
                        y_mm=pose_p_y_mm,
                        hand=pose_p_hand_deg,
                        speed=60,
                        timeout=30.0,
                    )
                    print(f"========== {LOG_PREFIX}/球{ball_idx} 恢复 P 姿态完成 ==========\n")
                except Exception as e:
                    print(f"  [{LOG_PREFIX}] ⚠️ 恢复 P 姿态失败: "
                          f"{type(e).__name__}: {str(e)[:120]}, 继续")
            else:
                print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过 P 姿态")

            # 2.1 等后台见球 / 见 ball 见 fetch_balls 触发停 + 累计前移记账
            creep_res = creep_thread.wait_for_ball(
                timeout_s=max(1.0, max_seconds - elapsed))
            creep_thread.stop_and_join()
            total_creep_m += creep_res["distance_m"]
            print(f"  [{LOG_PREFIX}] creep 结束: 前移 {creep_res['distance_m']:.3f}m "
                  f"/ {creep_res['elapsed_s']:.1f}s → "
                  f"{'见球' if creep_res['balls'] is not None else '未见球'}")
            if creep_res["balls"] is None:
                final_reason = "zone_cleared"
                print(f"  [{LOG_PREFIX}] 🏁 前移预算内未见球, 视作采区走完")
                break

            # 2.2 底盘视觉定位 (最左球 → 画面中心)
            #    2026-08-03 现场: 'P 姿态 + 底盘对齐之后直接抓手姿态 + grasp'
            #    只用底盘对齐 (track_chassis), 跳过臂侧 find_target /
            #    move_to_vision_target —— 假设底盘对齐后吸嘴已在球正上方,
            #    直接 composite_run(y_bin高位) → y 下 → grasp 即可。
            print(f"  [{LOG_PREFIX}] 🎯 track_chassis(leftmost, "
                  f"≤{track_max_seconds:.0f}s) — 仅底盘对齐, 跳过臂视觉伺服")
            track_res = _track_leftmost_ball(
                max_seconds=track_max_seconds, dry_run=dry_run,
            )
            print(f"  [{LOG_PREFIX}] track 结束: arrived={track_res.arrived} "
                  f"reason={track_res.reason}")

            # 2.3 定颜色 (并发策略 2026-08-05):
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
                # 2026-08-03 现场: pick 失败后下一轮又 goto_pose_p → creep → track → pick,
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
            # 2026-08-04: 最后一球的放仓线程收尾, 再关仓 (防臂还在动就关舵机)
            if release_thread is not None and release_thread.is_alive():
                release_thread.join(timeout=15.0)
            # ---- 关闭存储仓 (2026-08-04 用户: task4 结束关仓) ----
            try:
                print(f"  [{LOG_PREFIX}] 关闭存储仓 (angle={STORAGE_CLOSE_ANGLE_DEG}°)")
                arm_client.set_storage_angle(
                    STORAGE_CLOSE_ANGLE_DEG, speed=STORAGE_OPEN_SPEED, timeout=10.0)
            except Exception as e:
                print(f"  [{LOG_PREFIX}] ⚠️ 关仓失败 ({type(e).__name__}: {str(e)[:80]})")

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
