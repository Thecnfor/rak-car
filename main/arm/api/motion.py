"""main/arm/api/motion.py — 单/双轴位置移动 mixin.

依赖 SafetyMixin (由聚合类统一 mixin). 本 mixin 不显式继承 SafetyMixin, 避免 MRO 菱形冲突.
内部通过 self._check_safe / self._check_y_protected 调用, 由 Python 运行时解析到聚合类实例.
"""
from __future__ import annotations

from typing import Optional


def _mm_to_m(v_mm: float) -> float:
    return float(v_mm) / 1000.0


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
        """单轴 y 移动 (走 y 步进电机, 不动舵机)."""
        self._check_safe(y_mm=y_mm)
        job = self._call_arm("move_y_position", timeout=timeout,
                             target=_mm_to_m(y_mm))
        from ..state import ArmOrigin
        origin = self.origin or ArmOrigin()
        try:
            state = self.get_state()
            near_bottom = abs(y_mm) <= 0.1 * origin.soft_y_max_mm
            if near_bottom and not state.y_origin_valid:
                print(
                    f"[move_y] 警告: 目标 y={y_mm:.1f}mm 接近触底(0mm), "
                    f"但车端 y_limit 仍为 False (磁感应未触发).",
                    flush=True,
                )
            self._check_step_loss("y", target_mm=y_mm, actual_mm=state.y_mm,
                                  threshold_mm=origin.step_loss_y_mm)
        except Exception as e:
            print(f"[move_y] 状态校验读取失败: {e}", flush=True)
        return job

    def move_x(self, x_mm: float, v_max_mms: float = 40.0,
               out_time: float = 15.0, timeout: float = 30.0) -> dict:
        """单轴 x 移动 (编码器闭环)."""
        self._check_y_protected("move_x")
        job = self._call_arm("move_x_position", timeout=timeout,
                             target=_mm_to_m(x_mm), out_time=out_time)
        from ..state import ArmOrigin
        origin = self.origin or ArmOrigin()
        try:
            state = self.get_state()
            self._check_step_loss("x", target_mm=x_mm, actual_mm=state.x_mm,
                                  threshold_mm=origin.step_loss_x_mm)
        except Exception as e:
            print(f"[move_x] 状态校验读取失败: {e}", flush=True)
        return job
