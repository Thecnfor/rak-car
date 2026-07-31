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
        raw = self._read_raw_state()
        st_job = self._call_car("get_arm_state", timeout=10.0, sync=True)
        st_data = st_job.get("result") if isinstance(st_job, dict) else {}
        if not isinstance(st_data, dict):
            st_data = {}
        side = str(st_data.get("side", "MID"))
        hand = str(st_data.get("hand_angle", "UP"))
        origin = self.origin or ArmOrigin()
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

    # ---- arm_feed realtime 接口（替代坏掉的 x_get_position）----

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

        # RuntimeApiClient.get_arm_state() 返回 {"ok": bool, "result": dict, ...}
        result = st.get("result") if isinstance(st, dict) else None
        if not isinstance(result, dict):
            self._last_rt_err = "get_arm_state 返回无 result"
            return None

        x_mm = result.get("x_mm")
        if x_mm is None:
            # arm_feed 可能只填了 y_m (init 期常见), x_mm 还未上报
            self._last_rt_err = "arm_feed result 缺 x_mm (feed 刚启动?)"
            return None

        self._last_rt_err = None
        return float(x_mm)

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
