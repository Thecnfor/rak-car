"""main/arm/api/motion.py — 单/双轴位置移动 mixin.

依赖 SafetyMixin (由聚合类统一 mixin). 本 mixin 不显式继承 SafetyMixin, 避免 MRO 菱形冲突.
内部通过 self._check_safe / self._check_y_protected 调用, 由 Python 运行时解析到聚合类实例.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional


def _mm_to_m(v_mm: float) -> float:
    return float(v_mm) / 1000.0


logger = logging.getLogger(__name__)


class MotionMixin:
    """set_pose / move_xy / move_x / move_y"""

    def set_pose(self, x_mm: Optional[float], y_mm: Optional[float],
                 timeout: float = 30.0) -> dict:
        """一次设置 x/y (None 表示不动). side/hand 已删 2026-07-16."""
        x_m = _mm_to_m(x_mm) if x_mm is not None else None
        y_m = _mm_to_m(y_mm) if y_mm is not None else None
        self._check_y_protected("set_pose")
        self._check_safe(y_mm=y_mm)
        return self._call_arm("set_arm_pose", timeout=timeout, x=x_m, y=y_m)

    def move_xy(self, x_mm: float, y_mm: float,
                v_max_mms: float = 40.0, a_max_mms2: float = 100.0,
                timeout: Optional[float] = None) -> dict:
        """双轴同步移动 (x_mm, y_mm)."""
        self._check_y_protected("move_xy")
        self._check_safe(y_mm=y_mm)
        state = self.get_state()
        plan = self.traj.plan_xy(
            x0=state.x_mm, y0=state.y_mm,
            x1=x_mm, y1=y_mm,
            v_max=v_max_mms, a_max=a_max_mms2,
        )
        if timeout is None:
            timeout = max(5.0, plan.T * 2.0 + 1.0)
        return self._call_arm(
            "goto_position", timeout=timeout,
            x=_mm_to_m(x_mm), y=_mm_to_m(y_mm),
        )

    def move_y(self, y_mm: float, v_max_mms: float = 80.0,
               timeout: float = 20.0) -> dict:
        """单轴 y 移动 (走 y 步进电机, 不动舵机).

        2026-08-07 优化: step_loss 校验改用 ``_read_y_mm_realtime`` fast-path
        (1 HTTP GET → arm_feed 缓存), 替代 ``get_state()`` (3 HTTP calls).
        若 realtime 不可用则静默跳过校验 (原 try/except 行为).
        """
        self._check_safe(y_mm=y_mm)
        job = self._call_arm("move_y_position", timeout=timeout,
                             target=_mm_to_m(y_mm))
        from ..state import ArmOrigin
        origin = self.origin or ArmOrigin()
        try:
            y_mm_rt = self._read_y_mm_realtime()  # fast-path: 1 HTTP GET
            near_bottom = abs(y_mm) <= 0.1 * origin.soft_y_max_mm
            if near_bottom and y_mm_rt is not None:
                # 补充读一次 arm_state 拿 y_limit (同一次 HTTP response)
                try:
                    st = self.http.get_arm_state()
                    st_data = st.get("arm_state", {}) if isinstance(st, dict) else {}
                except Exception:
                    st_data = {}
                y_limit = bool(st_data.get("y_limit", False))
                if not y_limit:
                    print(
                        f"[move_y] 警告: 目标 y={y_mm:.1f}mm 接近触底(0mm), "
                        f"但车端 y_limit 仍为 False (磁感应未触发).",
                        flush=True,
                    )
            if y_mm_rt is not None:
                self._check_step_loss("y", target_mm=y_mm, actual_mm=y_mm_rt,
                                      threshold_mm=origin.step_loss_y_mm)
        except Exception as e:
            print(f"[move_y] 状态校验读取失败: {e}", flush=True)
        return job

    def move_x(self, x_mm: float, v_max_mms: float = 40.0,
               out_time: float = 15.0, timeout: float = 30.0) -> dict:
        """单轴 x 移动 (编码器闭环).

        2026-08-07 优化: step_loss 校验改用 ``_read_x_mm_realtime`` fast-path
        (1 HTTP GET → arm_feed 缓存), 替代 ``get_state()`` (3 HTTP calls).
        若 realtime 不可用则静默跳过校验.
        """
        self._check_y_protected("move_x")
        job = self._call_arm("move_x_position", timeout=timeout,
                             target=_mm_to_m(x_mm), out_time=out_time,
                             v_max_mms=v_max_mms)
        from ..state import ArmOrigin
        origin = self.origin or ArmOrigin()
        try:
            x_mm_rt = self._read_x_mm_realtime()  # fast-path: 1 HTTP GET
            if x_mm_rt is not None:
                self._check_step_loss("x", target_mm=x_mm, actual_mm=x_mm_rt,
                                      threshold_mm=origin.step_loss_x_mm)
        except Exception as e:
            print(f"[move_x] 状态校验读取失败: {e}", flush=True)
        return job

    # ---- x_speed safety watchdog（belt-slip 兜底）----
    #
    # 背景：x 轴是 motor_280 + 编码器 + 同步带，belt-slip 下电机在转但车不动。
    # 纯开环 x_speed 不知道车动没动，堵转时空转烧带子/电机。
    #
    # 兜底策略：每次开环 x_speed 时起一个 daemon 线程，周期（默认 100ms）读
    # realtime x_mm，若 max_stale_s 秒内 x 变化 < 0.5mm，自动调 x_speed(0) 停机。
    # 见 ARM_API.md §10 + memory [[x-speed-safety-watchdog]]。
    #
    # latest-wins：再次调用 x_speed_with_safety 会取消前一个 watchdog + 设新速度；
    # 显式 stop_x_speed_safety() 立即停。watchdog 不依赖 _call_arm 同步返回，
    # 完全可以跟其他动作并发。
    #
    # 2026-08-01: 从 b1806da WIP commit 的 monolith api.py 迁回 (mixin 拆分 c9fbc99
    # 时漏掉了,导致 common.move_x_with_split kick 路径 AttributeError)。

    def x_speed_with_safety(
        self,
        velocity: float,
        max_stale_s: float = 2.0,
        poll_interval_s: float = 0.1,
        move_threshold_mm: float = 0.5,
        timeout: float = 10.0,
    ) -> dict:
        """开环 x_speed + 后台 watchdog 兜底 belt-slip 堵转。

        Args:
            velocity: 目标速度（m/s，正值向 x 增大方向，负值反向）。
                     与车端 arm.x_speed(velocity) 同单位（m/s）。
            max_stale_s: watchdog 容忍"无位移"最长时间（秒）。超此值自动 x_speed(0)。
            poll_interval_s: watchdog 轮询间隔（秒）。
            move_threshold_mm: 判定"x 有动"的最小位移（mm），避免编码器噪声误判。
            timeout: car action HTTP 超时（秒）。

        Returns:
            ``/v1/execute`` 异步返回的 job dict（sync=False 不等完成）。

        注意：
          - 调用后 motor 立即按 velocity 转；调用方负责在合适时机调
            ``stop_x_speed_safety()`` 或再调一次 ``x_speed_with_safety(0)``。
          - latest-wins：再调一次会自动取消前一个 watchdog + 设新速度。
        """
        v_ms = float(velocity)
        # 确保 _x_safety_lock 已初始化 (聚合类 __init__ 里初始化,但 mixin 内兜底)
        if not hasattr(self, "_x_safety_lock"):
            self._init_x_safety_state()
        with self._x_safety_lock:
            # 1) 取消前一个 watchdog（保留取消设置，但下面要立刻建新的）
            self._cancel_x_safety_locked()

            # 2) 起新 watchdog
            start_x = self._read_x_mm_realtime()  # 起点（realtime 真值）
            stop_event = threading.Event()
            self._x_safety_stop_event = stop_event
            self._x_safety_start_x_mm = start_x
            self._x_safety_velocity_ms = v_ms

            def _watchdog() -> None:
                last_x = start_x
                last_change_t = time.time()
                # 在内部循环里访问 self，daemon 线程随 client 生命周期共存
                while not stop_event.wait(poll_interval_s):
                    cur = self._read_x_mm_realtime()
                    if cur is None:
                        # 读不到就继续等下一轮（realtime 偶发不可用）
                        continue
                    if abs(cur - last_x) > move_threshold_mm:
                        last_x = cur
                        last_change_t = time.time()
                    elif (time.time() - last_change_t) > max_stale_s:
                        # 卡住超时 → 强停 + 退出
                        try:
                            self._call_arm(
                                "x_speed", timeout=5.0, sync=False, velocity=0.0
                            )
                            logger.warning(
                                "x_speed_with_safety: x_mm %.1fmm 超 %ss 未变，"
                                "已自动 x_speed(0)（belt-slip 兜底）",
                                last_x, max_stale_s,
                            )
                        except Exception as exc:  # pragma: no cover
                            logger.warning(
                                "x_speed_with_safety: 自动停机失败: %s", exc
                            )
                        return

            t = threading.Thread(
                target=_watchdog, daemon=True, name="x-safety-watchdog"
            )
            self._x_safety_thread = t
            t.start()

        # 3) 下发开环速度（异步，不等完成）
        return self._call_arm(
            "x_speed", timeout=timeout, sync=False, velocity=v_ms
        )

    def stop_x_speed_safety(self) -> dict:
        """停 watchdog + 立即 x_speed(0)。

        行为：
          - 取消在跑的 watchdog 线程（latest-wins 的"前一个"被取消语义）。
          - 下发一次 x_speed(0) 异步停止电机。

        Returns:
            ``/v1/execute`` 异步返回的 x_speed(0) job dict。

        注意：即使当前没有 safety session（is_x_safety_active()=False），
        调本方法也安全 —— watchdog 取消 no-op + x_speed(0) 必下。
        """
        if not hasattr(self, "_x_safety_lock"):
            self._init_x_safety_state()
        with self._x_safety_lock:
            self._cancel_x_safety_locked()
        # 立即下发停车（async，不等完成）
        return self._call_arm(
            "x_speed", timeout=5.0, sync=False, velocity=0.0
        )

    def is_x_safety_active(self) -> bool:
        """watchdog 线程是否在跑。

        Returns:
            True = 上一次 ``x_speed_with_safety()`` 起的 watchdog 还在监控中；
            False = 没在跑（或已被 ``stop_x_speed_safety()`` / 新的
            ``x_speed_with_safety()`` 取消）。
        """
        if not hasattr(self, "_x_safety_lock"):
            return False
        with self._x_safety_lock:
            t = self._x_safety_thread
            return t is not None and t.is_alive()

    def _cancel_x_safety_locked(self) -> None:
        """取消 watchdog（调用方必须持 ``_x_safety_lock``）。"""
        if self._x_safety_stop_event is not None:
            self._x_safety_stop_event.set()
        self._x_safety_thread = None
        self._x_safety_stop_event = None
        self._x_safety_start_x_mm = None
        self._x_safety_velocity_ms = 0.0

    def _init_x_safety_state(self) -> None:
        """初始化 x_speed safety watchdog 状态 (聚合类 __init__ 已调, 这里兜底)。"""
        self._x_safety_lock = threading.Lock()
        self._x_safety_stop_event: Optional[threading.Event] = None
        self._x_safety_thread: Optional[threading.Thread] = None
        self._x_safety_start_x_mm: Optional[float] = None
        self._x_safety_velocity_ms: float = 0.0
