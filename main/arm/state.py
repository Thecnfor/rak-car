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

# 存储仓（二选一档位）枚举。
#
# 角度范围：ServoPwm wrapper 是 0~180 度（mc602 没 clamp，但下层 8-bit
# 通道收到 0~180 之外的 angle 会被协议层回中 / 回弹）。
# 实测：
#   - LEFT  -42° -> wrapper 后协议值 = -42+90 = 48，合法，不回弹
#   - RIGHT 90°  -> wrapper 后协议值 = 90+90  = 180，临界值，合法
# 历史事故：RIGHT 曾用 165°，协议值 255 超 0~180，mc602 会回弹。
# 因此 RIGHT 改用 90° 附近的值（=协议值 180，刚好是上边界）。
#
# 注意：car_wrap_2026.servo_1_angle_list[1] 也保持同步是 90，否则 set_storage
# 传 True 会再被这里"换算前"的 -42/90 算成 out-of-range。
# 默认 LEFT 用 -42° 保持和初始化（注释掉前）一致；如果后续发现 mc602 也不喜欢
# -42，统一改成 0~180 区间。
STORAGE_SIDES = ("LEFT", "RIGHT")
STORAGE_DEFAULT_LEFT_ANGLE = -42
STORAGE_DEFAULT_RIGHT_ANGLE = 90


@dataclass
class ArmOrigin:
    """业务层坐标系软限位 + 标注（与 arm_cfg.yaml:pos_cfg 是两套）。"""

    y_origin_m: float = 0.0           # y 触底时的原始 motor_y.get_dis() 值
    x_origin_m: float = 0.0           # x 撞墙时的原始 motor_x.get_dis() 值
    x_wall: str = "left"              # 上次撞的是哪一侧
    soft_y_max_m: float = 0.18        # 业务软上限（m）
    soft_x_min_m: float = 0.005
    soft_x_max_m: float = 0.30
    # 丢步/位置偏差阈值（mm）：move_x / move_y 完成后对比 actual vs target，超此值 warn。
    # y 是步进电机，堵转/失步较常见，默认 2mm（≈1 step）；x 是编码器闭环，默认 5mm。
    step_loss_y_mm: float = 2.0
    step_loss_x_mm: float = 5.0
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

    # 存储仓档位（独立 PWM 舵机，二选一）
    storage_side: str = "LEFT"  # "LEFT" / "RIGHT"，None 表示未知

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
        """给定 (x, y) 是否在业务软限位内（含）。

        y 业务坐标：触底=0，向下取正、向上取负；区间 [-soft_y_max_mm, 0]。
        """
        return (
            self.soft_x_min_mm <= x_mm <= self.soft_x_max_mm
            and -self.soft_y_max_mm <= y_mm <= 0.0
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
            f"-{self.soft_y_max_mm:.0f}..0]mm)"
        )
