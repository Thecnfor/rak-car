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
# 角度常量**严格对齐**官方 `baidu_smartcar_2026/car_wrap_2026.py:389`
#   servo_1_angle_list = [-42, 165]。
# **不要修改这两个常量** —— 改了就和官方车体物理位置对不上。
#
# 物理细节（仅供参考，不要据此"修正"角度）：
#   - LEFT  = -42° → ServoPwm 协议值 = -42+90 = 48，0~180 合法
#   - RIGHT = 165° → ServoPwm 协议值 = 165+90 = 255，**超 0~180**
#   - mc602 协议层对 0~180 之外的协议值是"不识别"而非"回弹"，
#     实际舵机行为由 mc602 固件决定（现场观察稳定，官方车这么用就行）。
#   - mc601 会自动 clamp 到 0~180，所以换 mc601 时也不需要改。
#
# **业务层只允许 LEFT/RIGHT 二选一**（set_storage() 不接受任意 angle）。
# 想传任意 angle 是反模式，会绕过官方标定 —— 不允许。
STORAGE_SIDES = ("LEFT", "RIGHT")
STORAGE_DEFAULT_LEFT_ANGLE = -42
STORAGE_DEFAULT_RIGHT_ANGLE = 165


@dataclass
class ArmOrigin:
    """业务层坐标系原点 + 软限位 + 标注（与 arm_cfg.yaml:pos_cfg 是两套）。

    注：x 轴软限位已取消（2026-07-16 用户原话："灵活使用就好，一般不会超"）。
    soft_x_min_m / soft_x_max_m 字段保留为 None，仅为兼容旧 arm_origin.yaml 读盘。
    """

    y_origin_m: float = 0.0           # y 触底时的原始 motor_y.get_dis() 值
    x_origin_m: float = 0.0           # x 当前原点（reset_x 已删除，无撞墙校准，固定为 0）
    x_wall: str = "left"              # 上次撞的是哪一侧（reset_x 已删除，仅历史标注）
    soft_y_max_m: float = 0.20        # 业务软上限（m）,实测行程可达 -200mm 还有富余
    soft_x_min_m: Optional[float] = None   # 软限位已取消，固定 None
    soft_x_max_m: Optional[float] = None   # 软限位已取消，固定 None
    # 丢步/位置偏差阈值（mm）：move_x / move_y 完成后对比 actual vs target，超此值 warn。
    # y 是步进电机，堵转/失步较常见，默认 2mm（≈1 step）；x 是编码器闭环，默认 5mm。
    step_loss_y_mm: float = 2.0
    step_loss_x_mm: float = 5.0
    calibrated_at: str = ""           # ISO 8601

    @property
    def soft_y_max_mm(self) -> float:
        return self.soft_y_max_m * 1000.0

    @property
    def soft_x_min_mm(self) -> Optional[float]:
        """x 轴软限位已取消；保留返回 None 以兼容旧代码读取路径。"""
        return None

    @property
    def soft_x_max_mm(self) -> Optional[float]:
        """x 轴软限位已取消；保留返回 None 以兼容旧代码读取路径。"""
        return None


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
    x_origin_valid: bool = False  # reset_x 已删除，固定 False

    # 软限位（从 ArmOrigin 拷过来）。
    # x 轴软限位已取消，固定 None；y 轴保留 [-soft_y_max_mm, 0] mm。
    soft_y_max_mm: float = 200.0
    soft_x_min_mm: Optional[float] = None
    soft_x_max_mm: Optional[float] = None

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

        x 轴软限位已取消（永远返回 True）；y 业务坐标：触底=0，向下取正、
        向上取负；区间 [-soft_y_max_mm, 0]。
        """
        return -self.soft_y_max_mm <= y_mm <= 0.0

    def is_ready(self) -> bool:
        """是否所有坐标系都可信 + y 在安全区内。x 轴软限位已取消，无校验。"""
        if not self.y_origin_valid:
            return False
        return self.in_safe_box(self.x_mm, self.y_mm)

    def describe(self) -> str:
        return (
            f"ArmState(x={self.x_mm:.1f}mm, y={self.y_mm:.1f}mm, "
            f"side={self.side}, hand={self.hand}, grasp={self.grasping}, "
            f"y_valid={self.y_origin_valid}, x_valid={self.x_origin_valid}, "
            f"safe=[x:无, -{self.soft_y_max_mm:.0f}..0]mm)"
        )
