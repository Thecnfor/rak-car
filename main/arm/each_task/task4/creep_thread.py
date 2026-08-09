"""task4 / target4 —— 后台保前移线程 + 跨球 IR 生命周期 + 底盘速度 helper。

从 target4.py 拆出 (2026-08-10 拆分): 单一职责 = "边前移边扫球" 的并发/状态逻辑。
- ``_Task4SearchState``  跨每一球保存 IR 生命周期和末端 0.3m 记账 (纯逻辑, 锁保护)。
- ``_CreepThread``       后台线程保底盘前移 + 主线程摆臂; 见球/IR 离开/超时即停。
- ``_set_chassis_vel``   下一次 chassis 速度 (realtime 门, 与 track_chassis 同通道)。
"""
from __future__ import annotations

import sys
import time
from typing import Optional

from . import target2  # noqa: E402
from .constants import (  # noqa: E402
    COLOR_BLUE, COLOR_YELLOW,
    CREEP_POLL_HZ, CREEP_MAX_SECONDS_S,
    IR_FAR_THRESHOLD_M, IR_FAR_CONFIRM_FRAMES, POST_IR_LOSS_DISTANCE_M,
    LOG_PREFIX_TARGET4 as LOG_PREFIX,
)


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
        # orchestrator 已用左 IR < 0.7m 触发任务，进入 task4 即视为已进入任务区。
        self.ir_seen_near = True
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
            # None/非法值表示传感器暂时没有新样本，不能当作“远离”。
            # 否则 task4 刚暂停 orchestrator 的 ir_feed 后会连续收到 ---，
            # 两帧就被误判离区并提前走完 0.3m。
            try:
                ir_value = float(left_ir)
            except (TypeError, ValueError):
                self.far_streak = 0
                self._pending_far_distance_m = 0.0
                return False
            near = ir_value <= self.far_threshold_m
            if near:
                self.ir_seen_near = True
                self.far_streak = 0
                self._pending_far_distance_m = 0.0
                return False
            if not self.ir_seen_near:
                return False
            # 只有 task4 触发后实际读到远 IR，才进入离开确认。
            far = ir_value > self.far_threshold_m
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
        self._ir_rearm_at = None

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
                    # 离区确认帧仍检查一次视觉：球和 IR 同帧命中时，不能吞掉已识别目标。
                    # 本轮若有球则先停车交给抓取流程；抓取完成后下一轮再按
                    # IR 丢失后的 0.3m 规则收尾。
                    try:
                        balls = target2.fetch_balls(
                            self.http, color_filter=None,
                            score_min=0.35, aspect_tol=1.0,
                            area_min=0.03, area_max=0.90,
                            debug=True,
                        )
                    except Exception as e:
                        balls = []
                        print(f"  [{LOG_PREFIX}] 离区确认帧视觉读取异常: "
                              f"{type(e).__name__}: {str(e)[:100]}", file=sys.stderr)
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
                    # 搜索阶段优先保证“看到球就停”，避免运动模糊/距离变化
                    # 让过严的静态框阈值把真球过滤掉。
                    balls = target2.fetch_balls(
                        self.http, color_filter=None,
                        score_min=0.35,
                        aspect_tol=1.0,
                        area_min=0.03,
                        area_max=0.90,
                        debug=True,
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
                except Exception as e:
                    print(f"  [{LOG_PREFIX}] fetch_balls 异常: "
                          f"{type(e).__name__}: {str(e)[:100]}", file=sys.stderr)
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
