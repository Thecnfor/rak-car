"""main/chassis/controllers/calibration.py
误差标定层：把 lane 模型的**裸输出**标成控制律需要的物理量（默认米/弧度）。

为什么需要这一层：
  lane_feed 把模型输出 ``result[0]`` 原样当 ``error_y``、``result[1]`` 当
  ``error_angle``（见 runtime/services/my_car/feeds.py）。但 lane 模型是一个
  128×128 自训练 CNN，输出的是"偏移距离 + 角度"的**模型尺度**——未必是米/弧度。
  而控制律（state.py 注释、Stanley 的"视觉误差直接当横向修正（米）"、TUI 的
  ``ey*100 cm``）全部假设 error_y 单位是米。若模型输出量级是 1e-3 / 归一化 /
  像素，则 ``vy = +kp_y * error_y ≈ 0``，横移通道失效 → 车只剩 ω 在修 →
  "出弯后修正慢、只能靠长直线修回来"。

本类在"原始误差"与"控制律"之间加一层可调标定：
    d_e'  = (error_y - offset_y) * scale_y
    d_a'  = (error_angle - offset_angle) * scale_angle
可选加指数平滑（EMA），滤掉模型逐帧抖动（对应"误差滤波"思路）。

**默认严格 no-op**：scale=1.0 / offset=0.0 / ema_alpha=None 时，
``calibrate(state)`` 返回原 state，行为与现在完全一致。配置后先跑
``python3 -m main.chassis.cli.diag_lane_error`` 实测模型输出分布，再定 scale。
"""
from __future__ import annotations

from dataclasses import replace
from typing import Optional, Tuple

from ..state import LaneState


class ErrorCalibrator:
    """error_y / error_angle 的标定 + 去抖。

    用法：
        cal = ErrorCalibrator(scale_y=1000.0, ema_alpha=0.30)  # d_e 疑似缩 1e-3
        state = cal.calibrate(state)    # 返回新 LaneState，其余字段透传
        cal.reset()                     # 换段 / pause-resume 时清 EMA
    """

    def __init__(
        self,
        *,
        scale_y: float = 1.0,
        offset_y: float = 0.0,
        scale_angle: float = 1.0,
        offset_angle: float = 0.0,
        ema_alpha: Optional[float] = None,
    ) -> None:
        self.scale_y = float(scale_y)
        self.offset_y = float(offset_y)
        self.scale_angle = float(scale_angle)
        self.offset_angle = float(offset_angle)
        # ema_alpha ∈ (0, 1]；None = 不做平滑。越接近 1 越信任新帧。
        if ema_alpha is not None:
            ema_alpha = float(ema_alpha)
            if not (0.0 < ema_alpha <= 1.0):
                raise ValueError("ema_alpha 必须在 (0, 1] 或 None")
        self.ema_alpha = ema_alpha
        # EMA 状态：None = 还没见过帧，首帧直接播种（不做平滑）
        self._ey_ema: Optional[float] = None
        self._ea_ema: Optional[float] = None

    @property
    def is_noop(self) -> bool:
        """严格 no-op 判断：scale=1/offset=0 且无 EMA 时，step 原样返回。"""
        return (
            self.scale_y == 1.0
            and self.offset_y == 0.0
            and self.scale_angle == 1.0
            and self.offset_angle == 0.0
            and self.ema_alpha is None
        )

    def reset(self) -> None:
        """清 EMA 状态。pause/resume、换场地时调用，避免旧段数据污染新段首帧。"""
        self._ey_ema = None
        self._ea_ema = None

    # ── 核心：单轴标定 + 平滑 ────────────────────────────────
    def _axis(self, raw: float, scale: float, offset: float,
              ema_state: Optional[float]) -> Tuple[float, Optional[float]]:
        """一个轴：affine 标定后过 EMA。返回 (标定值, 更新后的 EMA 状态)。"""
        val = (raw - offset) * scale
        if self.ema_alpha is None:
            return val, ema_state
        if ema_state is None:
            return val, val  # 首帧播种
        nxt = self.ema_alpha * val + (1.0 - self.ema_alpha) * ema_state
        return nxt, nxt

    def step(self, ey: Optional[float], ea: Optional[float]
             ) -> Tuple[Optional[float], Optional[float]]:
        """逐帧标定 (error_y, error_angle)。None 输入 → None 输出（保持丢线语义）。"""
        if ey is None:
            self._ey_ema = None  # 丢线：不更新 EMA，恢复时不跳
            ey_out = None
        else:
            ey_out, self._ey_ema = self._axis(
                float(ey), self.scale_y, self.offset_y, self._ey_ema
            )
        if ea is None:
            self._ea_ema = None
            ea_out = None
        else:
            ea_out, self._ea_ema = self._axis(
                float(ea), self.scale_angle, self.offset_angle, self._ea_ema
            )
        return ey_out, ea_out

    def calibrate(self, state: LaneState) -> LaneState:
        """把原始 LaneState 标定成控制律视角的 LaneState。

        - 没有误差帧（has_error=False）→ 原样返回，保留丢线语义。
        - no-op 配置 → 原样返回（省一次 replace）。
        - 否则只替换 error_y / error_angle，其余字段透传。
        """
        if not state.has_error or self.is_noop:
            return state
        ey, ea = self.step(state.error_y, state.error_angle)
        return replace(state, error_y=ey, error_angle=ea)
