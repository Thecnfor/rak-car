"""main/arm 子包：机械臂业务层。

外部 import 只允许指向 main.*，不接触 runtime / smartcar。
"""
from .api import ArmClient, ArmSafetyError
from .state import (
    ArmState,
    ArmOrigin,
    SIDES,
    HANDS,
    STORAGE_SIDES,
    STORAGE_DEFAULT_LEFT_ANGLE,
    STORAGE_DEFAULT_RIGHT_ANGLE,
)
from .origin import OriginCalibrator, run_calibrator
from .trajectory import (
    TrajectoryGenerator,
    TrajectoryPlan,
    TrajectorySample,
)
from .loops.runner import ArmRunner
# 2026-07-31 视觉伺服封装（VISION_SERVO_DESIGN.md）：
from .labels import (
    Label, LabelInfo, LABELS, LABEL_GROUPS,
    get_label_info, is_in_group,
)
from .vision import (
    ArmVisionClient,
    Detection, BBoxNorm, BBoxPixels,
    TargetSelector, SelectionStrategy,
    ServoTrace, ServoResult,
)

__all__ = [
    # ── 机械臂客户端 / 执行器 ──
    "ArmClient",         # 机械臂 API 聚合客户端（Safety + Motion + Setters + Composite + 状态），业务层主入口
    "ArmSafetyError",    # 安全门违例抛出的异常（角度硬限 / y 保护区 / 不满足抓取前置）
    "ArmRunner",         # 机械臂业务编排器：把 ArmClient + S 曲线 dry-run + 超时/丢步核对包成同步调用

    # ── 状态 / 位姿常量 ──
    "ArmState",          # 机械臂实时状态（x/y 米、arm/hand 角度、y_origin_valid 等）
    "ArmOrigin",         # 零点标定数据（原点、软限位、吸嘴偏移、丢步阈值）
    "SIDES",             # 侧向枚举: ("LEFT", "MID", "RIGHT")
    "HANDS",             # 手爪姿态枚举: ("UP", "MID", "DOWN")
    "STORAGE_SIDES",     # 存储仓档位枚举: ("LEFT", "RIGHT")
    "STORAGE_DEFAULT_LEFT_ANGLE",   # 存储仓左档默认舵机角度
    "STORAGE_DEFAULT_RIGHT_ANGLE",  # 存储仓右档默认舵机角度

    # ── 零点标定 ──
    "OriginCalibrator",  # 交互式零点标定器（x/y 撞墙 + 角度标定）
    "run_calibrator",    # 一键跑零点标定，返回 ArmOrigin

    # ── XY 轨迹规划 ──
    "TrajectoryGenerator",  # XY 双轴 S 曲线轨迹规划器（plan_xy 生成平滑路径）
    "TrajectoryPlan",       # 一条规划的轨迹（总时长 T / 峰值速度 peak_vx/peak_vy）
    "TrajectorySample",     # 轨迹上的单个采样点（t/x/y/v 等）

    # ── 视觉标签 ──
    "Label",           # 目标类别枚举（cylinder/ball/water_tower 等）
    "LabelInfo",       # 单个标签元数据（名称 / 中文名 / 是否抓取物）
    "LABELS",          # 全部标签元数据的元组
    "LABEL_GROUPS",    # 标签分组（如按功能用途分组的 dict）
    "get_label_info",  # 按名称查 LabelInfo
    "is_in_group",     # 判断某标签是否属于指定分组

    # ── 视觉伺服 ──
    "ArmVisionClient",    # 视觉伺服客户端（ServoLoop + RealtimeLoop + VelocityLoop 聚合）
    "Detection",          # 单次检测结果（label + 归一化/像素 bbox + 置信度）
    "BBoxNorm",           # 归一化边界框（0~1）
    "BBoxPixels",         # 像素边界框
    "TargetSelector",     # 目标选择器（label + 策略 + track_id，决定多目标时选哪个）
    "SelectionStrategy",  # 目标选择策略枚举（最高分 / 锁定首个 / 最近中心 …）
    "ServoTrace",         # 伺服过程逐帧轨迹（每帧误差 / 命中 / 丢失）
    "ServoResult",        # 伺服结果（是否收敛 + 完整 trace）
]