"""main/arm/api/state_io.py — 状态读取 / 急停 / ping mixin."""
from __future__ import annotations

from typing import Optional, Tuple

from ..state import ArmOrigin, ArmState


def _m_to_mm(v_m) -> float:
    return float(v_m) * 1000.0


class StateIOMixin:
    """get_state / get_pose_mm / get_x_mm / get_y_mm / emergency_stop / ping."""

    def _read_raw_state(self) -> dict:
        try:
            y_job = self._call_arm("y_get_position", timeout=10.0)
            y_val = y_job.get("result") if isinstance(y_job, dict) else None
        except Exception:
            y_val = None
        try:
            x_job = self._call_arm("x_get_position", timeout=10.0)
            x_val = x_job.get("result") if isinstance(x_job, dict) else None
        except Exception:
            x_val = None
        return {"raw_x_m": float(x_val) if x_val is not None else 0.0,
                "raw_y_m": float(y_val) if y_val is not None else 0.0}

    def get_state(self) -> ArmState:
        """读当前机械臂完整状态。

        2026-08-07 优化：优先走 arm_feed 缓存 fast-path（1 HTTP GET），
        仅当缓存不可用时退回原路径（``_read_raw_state`` 2 HTTP calls + ``get_arm_state`` 1 HTTP call）。
        ``y_origin_valid`` 在 fast-path 下依赖 ``get_arm_state`` 返回的 ``y_limit`` 字段，
        若该字段不可用则置 ``False``。
        """
        # ---- fast-path: 优先读 arm_feed 缓存 (1 HTTP GET) ----
        x_mm_rt = self._read_x_mm_realtime()
        y_mm_rt = self._read_y_mm_realtime()
        rt_err = self.last_realtime_error()

        origin = self.origin or ArmOrigin()

        if x_mm_rt is not None and y_mm_rt is not None and rt_err is None:
            # 双轴缓存都就绪：拿 side/hand/arm_angle/y_origin_valid
            try:
                st = self.http.get_arm_state()
                st_data = st.get("arm_state") if isinstance(st, dict) else {}
            except Exception:
                st_data = {}
            side = str(st_data.get("side", "MID"))
            hand = str(st_data.get("hand_angle", "UP"))
            arm_angle = st_data.get("arm_angle")
            hand_angle = st_data.get("hand_angle")
            y_origin_valid = bool(st_data.get("y_limit", False))
            return ArmState(
                x_mm=x_mm_rt, y_mm=y_mm_rt,
                side=side, hand=hand, grasping=False,
                y_origin_valid=y_origin_valid, x_origin_valid=False,
                soft_y_max_mm=origin.soft_y_max_mm,
                soft_x_min_mm=None, soft_x_max_mm=None,
                raw_x_m=x_mm_rt / 1000.0, raw_y_m=y_mm_rt / 1000.0,
                arm_angle=arm_angle, hand_angle=hand_angle,
            )

        # ---- fallback: 原路径 (3 HTTP calls) ----
        raw = self._read_raw_state()
        st_job = self._call_car("get_arm_state", timeout=10.0, sync=True)
        st_data = st_job.get("result") if isinstance(st_job, dict) else {}
        if not isinstance(st_data, dict):
            st_data = {}
        side = str(st_data.get("side", "MID"))
        hand = str(st_data.get("hand_angle", "UP"))
        return ArmState(
            x_mm=_m_to_mm(raw["raw_x_m"]),
            y_mm=_m_to_mm(raw["raw_y_m"]),
            side=side, hand=hand, grasping=False,
            y_origin_valid=bool(st_data.get("y_limit", False)),
            x_origin_valid=False,
            soft_y_max_mm=origin.soft_y_max_mm,
            soft_x_min_mm=None, soft_x_max_mm=None,
            raw_x_m=raw["raw_x_m"], raw_y_m=raw["raw_y_m"],
            arm_angle=st_data.get("arm_angle"),
            hand_angle=st_data.get("hand_angle"),
        )

    def get_pose_mm(self) -> Tuple[float, float, str, str]:
        st = self.get_state()
        return st.x_mm, st.y_mm, st.side, st.hand

    def get_x_mm(self) -> float:
        return self.get_state().x_mm

    def get_y_mm(self) -> float:
        return self.get_state().y_mm

    def emergency_stop(self) -> dict:
        return self.http.emergency_stop()

    def ping(self, timeout: float = 5.0) -> bool:
        try:
            self.http.get_health()
            return True
        except Exception:
            return False

    # ---- arm_feed realtime 接口（替代坏掉的 x_get_position / y_get_position）----

    def _read_x_mm_realtime(self) -> Optional[float]:
        """从 runtime `arm_feed` 守护线程缓存读 x 位置 (mm)。

        为什么不用 ``get_state()``/``get_x_mm()``: SDK `x_get_position`
        返回的 ``x_pose_start`` 在 ``reset_x`` 撞墙后被锁死, 业务坐标不再
        反映真实位置; arm_feed 守护线程通过 `/v1/realtime/wheels/encoders`
        拿原始编码器, 业务层自行减 ``x_origin_m``, 是当前唯一可信源。

        Returns:
            float: x 业务坐标 (mm), 单位米换算后的整毫米值; None 表示
                调用失败 (网络断 / arm_feed 未启 / runtime 未初始化)。
                失败原因存到 ``self._last_rt_err``, 业务层可调
                ``last_realtime_error()`` 取出来。

        Side effects:
            设置 ``self._last_rt_err`` (成功置 None, 失败置错误字符串)。
        """
        try:
            st = self.http.get_arm_state()
        except Exception as e:
            self._last_rt_err = f"{type(e).__name__}: {str(e)[:120]}"
            return None

# runtime `/v1/realtime/arm/state` 返回 {"ok": bool, "arm_state": dict, ...}
        # (见 runtime/api/routers/realtime.py:62 — 注意不是 "result",这是 2026-08-01
        # 发现的 latent bug: 旧版按 "result" 取永远 None,导致 split 模式总报
        # "realtime x_mm 读不到"。)
        result = st.get("arm_state") if isinstance(st, dict) else None
        if not isinstance(result, dict):
            self._last_rt_err = (
                f"get_arm_state 返回无 arm_state (top-level keys="
                f"{list(st.keys()) if isinstance(st, dict) else 'N/A'})"
            )
            return None

        # runtime 正在 init (controller reboot 后) → arm_state 全 None / active=False
        if not result.get("active"):
            self._last_rt_err = (
                f"arm_feed 未启 (active=False, mode={result.get('mode')!r}) — "
                f"runtime 可能在 reinit,或 MyCar() 未构造完成"
            )
            return None

        x_mm = result.get("x_mm")
        if x_mm is None:
            # arm_feed 可能刚启动,x_mm 还未上报
            self._last_rt_err = "arm_feed active 但 x_mm 仍 None (feed 刚启动?)"
            return None

        self._last_rt_err = None
        return float(x_mm)

    def _read_y_mm_realtime(self) -> Optional[float]:
        """从 runtime `arm_feed` 守护线程缓存读 y 位置 (mm)。

        与 ``_read_x_mm_realtime`` 同构：走 `/v1/realtime/arm/state` fast-path
        (不进 job_queue, 不抢 SerialEngine), 拿 arm_feed 20Hz 缓存的 y_mm。

        2026-08-07 新增：原 ``get_state()`` 调 ``y_get_position`` 走 arm_queue
        (和 composite_run 抢 worker), 本方法在 step_loss 校验等高频只读场景
        替代 ``get_state()``, 省 2 HTTP calls (y_get_position + x_get_position)。

        Returns:
            float: y 业务坐标 (mm); None 表示调用失败 (同 _read_x_mm_realtime)。
        """
        try:
            st = self.http.get_arm_state()
        except Exception as e:
            self._last_rt_err = f"{type(e).__name__}: {str(e)[:120]}"
            return None

        result = st.get("arm_state") if isinstance(st, dict) else None
        if not isinstance(result, dict):
            self._last_rt_err = (
                f"get_arm_state 返回无 arm_state (keys="
                f"{list(st.keys()) if isinstance(st, dict) else 'N/A'})"
            )
            return None

        if not result.get("active"):
            self._last_rt_err = (
                f"arm_feed 未启 (active=False, mode={result.get('mode')!r})"
            )
            return None

        y_mm = result.get("y_mm")
        if y_mm is None:
            self._last_rt_err = "arm_feed active 但 y_mm 仍 None (feed 刚启动?)"
            return None

        self._last_rt_err = None
        return float(y_mm)

    def _read_arm_angle_realtime(self) -> Optional[int]:
        """从 runtime `arm_feed` 缓存读大臂角度 (deg, int)。

        2026-08-07 新增：set_hand_angle 安全判断只用到 arm_angle，
        不必调完整的 ``get_state()`` (3 HTTP calls)。
        """
        try:
            st = self.http.get_arm_state()
        except Exception as e:
            self._last_rt_err = f"{type(e).__name__}: {str(e)[:120]}"
            return None

        result = st.get("arm_state") if isinstance(st, dict) else None
        if not isinstance(result, dict) or not result.get("active"):
            self._last_rt_err = "arm_feed 不可用"
            return None

        val = result.get("arm_angle")
        if val is None:
            self._last_rt_err = "arm_feed active 但 arm_angle 为 None"
            return None

        self._last_rt_err = None
        return int(val)

    def wait_for_arm_state_active(
        self,
        timeout_s: float = 10.0,
        poll_interval_s: float = 0.2,
    ) -> bool:
        """等 runtime `arm_feed` 守护线程 active=True **且** x_mm 有真值。

        适用 startup race (2026-08-01 暴露):
          - runtime `_create_car_locked` 触发 arm_feed 起线程要几百 ms
          - 业务脚本 `ArmClient.connect()` 后立即 `move_x_with_split`
            会撞上 arm_feed 还没上报第一帧,旧版直接 raise
          - 现在业务层在第一轮读 x 前调一下本函数,容忍 startup 抖动

        Args:
            timeout_s: 总超时 (秒)。arm_feed 真的死了 / 控制器没接上,
                超时返回 False,业务层自己 raise。
            poll_interval_s: 轮询间隔 (秒)。

        Returns:
            bool: True = 等到 active + x_mm; False = 超时仍未就绪。
                超时时 ``self._last_rt_err`` 保留最后一次失败原因,
                业务层可 ``last_realtime_error()`` 取出来。

        Side effects:
            覆盖 ``self._last_rt_err`` (最后一次调用的状态)。
        """
        import time as _time

        deadline = _time.monotonic() + float(timeout_s)
        last_log_ts = 0.0
        while True:
            x = self._read_x_mm_realtime()
            if x is not None:
                return True
            now = _time.monotonic()
            if now >= deadline:
                return False
            # 每 1s 打一次日志,避免刷屏 (e.g. 5s 等 25 次只打 5 行)
            if now - last_log_ts >= 1.0:
                err = self._last_rt_err or "未知"
                print(f"  [wait_arm_feed] {timeout_s - (deadline - now):.1f}s 内仍不可用: {err}")
                last_log_ts = now
            _time.sleep(poll_interval_s)

    def last_realtime_error(self) -> Optional[str]:
        """读上次 ``_read_x_mm_realtime`` 失败的错误上下文。

        区分三类失败:
          - 网络断 (``ConnectionError`` / ``Timeout``)
          - runtime 没启 arm_feed ("arm_feed result 缺 x_mm")
          - runtime 没初始化 ("get_arm_state 返回无 result")

        业务层在 move_x 之后做"二次校验"时, 看到这个非 None 就该:
          1. 怀疑 x 真值, 走退化模式 (退到上一步的位置)
          2. 上报, 排查 runtime / network
        """
        return getattr(self, "_last_rt_err", None)
