"""main/arm/state.py
机械臂位姿状态 dataclass。

业务层只读这个，不再分别调 arm.x_get_position / arm.y_get_position 多次。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional


# 合法枚举
SIDES = ("LEFT", "MID", "RIGHT")
HANDS = ("UP", "MID", "DOWN")


@dataclass
class ArmOrigin:
    """业务层坐标系软限位 + 标注（与 arm_cfg.yaml:pos_cfg 是两套）。"""

    y_origin_m: float = 0.0           # y 触底时的原始 motor_y.get_dis() 值
    x_origin_m: float = 0.0           # x 撞墙时的原始 motor_x.get_dis() 值
    x_wall: str = "left"              # 上次撞的是哪一侧
    soft_y_max_m: float = 0.18        # 业务软上限（m）
    soft_x_min_m: float = 0.005
    soft_x_max_m: float = 0.30
    calibrated_at: str = ""           # ISO 8601

    @property
    def soft_y_max_mm(self) -> float:
        return self.soft_y_max_m * 1000.0

    @property
    def soft_x_min_mm(self) -> float:
        return self.soft_x_min_m * 1000.0

    @property
    def soft_x_max_mm(self) -> float:
        return self.soft_x_max_m * 1000.0


@dataclass
class ArmState:
    """业务层看到的位姿状态（单位：mm + 枚举）。"""

    # 业务位姿（相对原点）
    x_mm: float = 0.0
    y_mm: float = 0.0
    side: str = "MID"
    hand: str = "UP"
    grasping: bool = False

    # 坐标系可信度
    y_origin_valid: bool = False
    x_origin_valid: bool = False

    # 软限位（从 ArmOrigin 拷过来）
    soft_y_max_mm: float = 180.0
    soft_x_min_mm: float = 5.0
    soft_x_max_mm: float = 300.0

    # 原始坐标（车端读数，调试用；单位 m）
    raw_x_m: float = 0.0
    raw_y_m: float = 0.0

    # 角状态（舵机反馈；如果可读；读不到时为 None）
    arm_angle: Optional[int] = None
    hand_angle: Optional[int] = None

    # 时间戳
    fetched_at: float = field(default_factory=time.time)

    # ---- 校验 ----

    def in_safe_box(self, x_mm: float, y_mm: float) -> bool:
        """给定 (x, y) 是否在业务软限位内（含）。"""
        return (
            self.soft_x_min_mm <= x_mm <= self.soft_x_max_mm
            and 0.0 <= y_mm <= self.soft_y_max_mm
        )

    def is_ready(self) -> bool:
        """是否所有坐标系都可信 + 都在安全区内。"""
        if not (self.y_origin_valid and self.x_origin_valid):
            return False
        return self.in_safe_box(self.x_mm, self.y_mm)

    def describe(self) -> str:
        return (
            f"ArmState(x={self.x_mm:.1f}mm, y={self.y_mm:.1f}mm, "
            f"side={self.side}, hand={self.hand}, grasp={self.grasping}, "
            f"y_valid={self.y_origin_valid}, x_valid={self.x_origin_valid}, "
            f"safe=[{self.soft_x_min_mm:.0f}..{self.soft_x_max_mm:.0f} x "
            f"0..{self.soft_y_max_mm:.0f}]mm)"
        )
