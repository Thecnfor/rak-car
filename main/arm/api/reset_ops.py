"""main/arm/api/reset_ops.py — 复位动作 mixin (reset_y / reset_x / reset_all / reset_origin)."""
from __future__ import annotations

import time

from ..state import ArmOrigin


class ResetOpsMixin:
    """复位入口; 不依赖 SafetyMixin (复位本就允许在保护区内)."""

    def reset_y(self, timeout: float = 30.0) -> dict:
        return self._call_arm("reset_y", timeout=timeout)

    def reset_x(self, direction: str = "right",
                reset_velocity_mms: float = 30.0,
                probe_time: float = 0.3,
                timeout: float = 30.0) -> dict:
        """撞墙定 x 原点 (ARM_API §9.1).

        Args:
            direction: "right" (target 增大方向) 或 "left"。
            reset_velocity_mms: 撞墙速度 (mm/s),业务层限速。
            probe_time: 反向探针时间 (秒),默认 0.3 跟 arm_base.py 默认对齐;
                设为 0 在"车刚好在 selected 方向的墙上"场景下会立即误判 stall
                → calibrate 失败,留 0.3 让反向探针先验证 motor 工作更稳。
                (2026-08-01 之前 wrapper 不透传 probe_time,业务层必须绕过
                wrapper 直调 _call_arm;现在 wrapper 支持,escape hatch 可删。)
            timeout: HTTP 同步超时 (秒)。
        """
        if direction not in ("right", "left"):
            raise ValueError("direction 必须是 'right' 或 'left'")
        return self._call_arm(
            "reset_x", timeout=timeout,
            direction=direction,
            reset_velocity=reset_velocity_mms / 1000.0,
            probe_time=probe_time,
        )

    def reset_all(self, arm_angle: float = 90, hand_angle: float = -90,
                  x_direction: str = "right",
                  reset_x_velocity_mms: float = 30.0,
                  timeout: float = 120.0) -> dict:
        return self._call_arm(
            "reset_all", timeout=timeout,
            arm_angle=arm_angle, hand_angle=hand_angle,
            x_direction=x_direction,
            reset_x_velocity=reset_x_velocity_mms / 1000.0,
        )

    def reset_origin(self, x_wall: str = "left", timeout: float = 60.0) -> dict:
        if x_wall not in ("left", "right"):
            raise ValueError("x_wall 必须是 'left' 或 'right'")
        job = self._call_arm("reset_position", timeout=timeout)
        st = self._read_raw_state()
        new_origin = ArmOrigin(
            y_origin_m=st["raw_y_m"], x_origin_m=0.0,
            x_wall=x_wall,
            soft_y_max_m=self.origin.soft_y_max_m if self.origin else 0.20,
            calibrated_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )
        self.save_origin(new_origin)
        return job
