"""task7 业务常量 —— 2 单元 × 6 户 = 12 个住户 × 机械臂每个零件的参数。

来源 / 现状:
  - 用户 2026-07-31 澄清结构: 2 个单元(1单元 / 2单元),**每单元 6 个住户**,
    一共 12 户。**每个单元里 6 个分支**对应 6 个住户的 arm 零件参数。
  - **当前所有数值都是根据直觉拍的占位值**, 现场 0 标定。**不要直接跑**,
    跑前需要按 (单元 → 6 个配送点位置 + 平板高度 + 障碍) 现场调。
  - 原 b1_detect_house.py:18-21 只硬编码 2 户 (1单元 张三, 2单元 李四),
    本文件统一接管 12 户。b1/b2 后续应改 from .constants import RESIDENT_PROFILES。

机械臂零件清单 (ArmClient 暴露的全部可控零件, 5 个):
  1. y       : 步进电机 vertical (mm, 触底=0, 向上取负)
  2. x       : 编码器 horizontal (mm, 相对原点)
  3. arm_angle: 大臂总线舵机 (°, 硬限 [+90, -150])
  4. hand_angle: 手爪 PWM 舵机 (°, 硬限 [-90, 0])
  5. grasp   : 吸真空 bool (grasp(True)=吸, grasp(False)=放)

约定:
  - 触底 y=0, 业务 y 范围 [-200, 0]; 配送点 pre_y 通常 [-180, -150], place_y
    通常 [-100, -60] (放上平板)。**place_y 必须 > -30** 避开 y 保护区。
  - arm_angle 0° = MID (水平), +90° = 复位位, 负值 = 收起方向
  - hand_angle 0° = DOWN, -90° = UP, -37° = MID
  - x 相对原点 (reset_position 后), 配送点 ±200mm 内, 6 户按 60mm 等距
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


# ---------- 单住户机械臂参数 ----------

@dataclass
class ResidentArmProfile:
    """1 个住户的机械臂全套零件参数 (5 零件 + 3 流程)。

    字段分两类:
      - **强相关** (跑前必现场调): place_x_mm, pre_y_mm, place_y_mm,
        arm_angle_deg, hand_angle_deg
      - **弱相关** (默认值即可, 跑前看一眼): v_max_mms, settle_s,
        retract_y_mm, grasp_release
    """
    # ---- 身份 ----
    door: str                       # 门牌号 e.g. "101" / "206"
    unit: str                       # "1" / "2" (不带"单元"后缀, 便于拼 key)

    # ---- 机械臂 5 个零件 ----
    place_x_mm: float               # 1. x: 配送点 x 位置 (mm, 相对原点)
    pre_y_mm: float                 # 2. y: 接近 y (高处, 通常 -150~-180)
    place_y_mm: float               # 2. y: 放置 y (低处, 贴平板, 通常 -100~-60)
    arm_angle_deg: float            # 3. arm_angle: 大臂角度 (°, [+90, -150])
    hand_angle_deg: float           # 4. hand_angle: 手爪角度 (°, [-90, 0])
    grasp_release: bool             # 5. grasp: 放置时是否关闭真空释放

    # ---- 配送流程参数 (弱相关) ----
    v_max_mms: float = 40.0         # arm 移动速度上限 (mm/s)
    settle_s: float = 0.5           # 放置后稳定时间 (s, grasp=False 后等一下)
    retract_y_mm: float = -150.0    # 放完抬起 y (mm, 通常比 place_y 高 50~80)

    @property
    def key(self) -> str:
        """标准 key, e.g. "1单元_101"."""
        return f"{self.unit}单元_{self.door}"

    @property
    def full_name(self) -> str:
        """完整显示名, e.g. "1单元-101户"."""
        return f"{self.unit}单元-{self.door}户"


# ---------- x 等距布点 (6 户, 60mm 间距, ±150mm) ----------
# 占位: 实际现场量 6 个住户的 x 位置, 覆盖到 [-150, +150] 这个 60mm 间隔
# 也行, 或换成 [-180, +180] 的 72mm 间隔 (总计 360mm 跨距)。
# 如果实际 6 个点不均, 把每个 place_x_mm 单独改即可。

_X_STANDARD: list[float] = [-150.0, -90.0, -30.0, 30.0, 90.0, 150.0]


def _build_unit_profiles(unit: str, x_list: list[float]) -> dict[str, ResidentArmProfile]:
    """按 x_list 顺序生成 1 个单元的 6 户 profile。

    占位策略: 6 户里**故意做差异化**让"6 个分支"有实际意义:
      - 第 1 户: 标准位姿
      - 第 2 户: 标准位姿
      - 第 3 户: **手爪前倾** (适配前有遮挡的平板)
      - 第 4 户: **平板较高** (place_y 抬高)
      - 第 5 户: **大臂微收** (防右侧障碍)
      - 第 6 户: **慢速 + 稳定** (易倒货物)

    真实参数需现场调, 这里只是给"6 个分支"占个有差异的形状, 方便跑前识别哪个对应哪个。
    """
    door_starts = {"1": 101, "2": 201}[unit]
    doors = [str(door_starts + i) for i in range(6)]  # 101-106 / 201-206

    profiles: dict[str, ResidentArmProfile] = {}
    for i, (door, x) in enumerate(zip(doors, x_list)):
        if i == 0:
            # 标准
            prof = ResidentArmProfile(
                door=door, unit=unit,
                place_x_mm=x, pre_y_mm=-180.0, place_y_mm=-100.0,
                arm_angle_deg=0.0, hand_angle_deg=0.0,
                grasp_release=True,
            )
        elif i == 1:
            # 标准
            prof = ResidentArmProfile(
                door=door, unit=unit,
                place_x_mm=x, pre_y_mm=-180.0, place_y_mm=-100.0,
                arm_angle_deg=0.0, hand_angle_deg=0.0,
                grasp_release=True,
            )
        elif i == 2:
            # 手爪前倾, 适配前方遮挡
            prof = ResidentArmProfile(
                door=door, unit=unit,
                place_x_mm=x, pre_y_mm=-180.0, place_y_mm=-100.0,
                arm_angle_deg=0.0, hand_angle_deg=-30.0,
                grasp_release=True,
            )
        elif i == 3:
            # 平板较高, place_y 抬高
            prof = ResidentArmProfile(
                door=door, unit=unit,
                place_x_mm=x, pre_y_mm=-180.0, place_y_mm=-70.0,
                arm_angle_deg=0.0, hand_angle_deg=0.0,
                grasp_release=True,
            )
        elif i == 4:
            # 大臂微收, 防右侧障碍
            prof = ResidentArmProfile(
                door=door, unit=unit,
                place_x_mm=x, pre_y_mm=-180.0, place_y_mm=-110.0,
                arm_angle_deg=-30.0, hand_angle_deg=0.0,
                grasp_release=True,
                v_max_mms=30.0,
            )
        else:  # i == 5
            # 慢速 + 长稳定, 易倒货物
            prof = ResidentArmProfile(
                door=door, unit=unit,
                place_x_mm=x, pre_y_mm=-180.0, place_y_mm=-120.0,
                arm_angle_deg=0.0, hand_angle_deg=0.0,
                grasp_release=True,
                v_max_mms=20.0, settle_s=1.0,
            )
        profiles[prof.key] = prof
    return profiles


# ---------- 12 个住户 (2 单元 × 6 户) ----------

RESIDENT_PROFILES_1: dict[str, ResidentArmProfile] = _build_unit_profiles("1", _X_STANDARD)
"""1 单元 6 户: 1单元_101 ~ 1单元_106"""

RESIDENT_PROFILES_2: dict[str, ResidentArmProfile] = _build_unit_profiles("2", _X_STANDARD)
"""2 单元 6 户: 2单元_201 ~ 2单元_206"""

RESIDENT_PROFILES: dict[str, ResidentArmProfile] = {**RESIDENT_PROFILES_1, **RESIDENT_PROFILES_2}
"""12 户全集: key 形如 "1单元_101" / "2单元_206"."""

assert len(RESIDENT_PROFILES) == 12, f"应为 12 户, 实际 {len(RESIDENT_PROFILES)}"


# ---------- 便捷查询 ----------

def get_profile(unit: Optional[str] = None, door: Optional[str] = None,
                key: Optional[str] = None) -> ResidentArmProfile:
    """查住户 profile。3 种调用方式 (优先 key):

        get_profile(key="1单元_101")
        get_profile(unit="1", door="101")
        get_profile(unit="1")  → 返回该单元第 1 户
    """
    if key is not None:
        if key not in RESIDENT_PROFILES:
            raise KeyError(f"未找到 key={key!r}。已注册: {list(RESIDENT_PROFILES.keys())}")
        return RESIDENT_PROFILES[key]
    if unit is not None and door is not None:
        return get_profile(key=f"{unit}单元_{door}")
    if unit is not None:
        # 单元的第 1 户
        unit_profiles = RESIDENT_PROFILES_1 if unit == "1" else RESIDENT_PROFILES_2
        return next(iter(unit_profiles.values()))
    raise ValueError("必须给 key 或 (unit, door) 之一")


def list_residents(unit: Optional[str] = None) -> list[str]:
    """列住户 key。unit=None 列 12 户全集; unit="1"/"2" 列该单元 6 户。"""
    if unit is None:
        return list(RESIDENT_PROFILES.keys())
    if unit == "1":
        return list(RESIDENT_PROFILES_1.keys())
    if unit == "2":
        return list(RESIDENT_PROFILES_2.keys())
    raise ValueError(f"unit 必须是 '1' 或 '2', 收到: {unit!r}")


# ---------- 通用参数 (弱相关) ----------

PRE_Y_PROTECTED: float = -30.0
"""y 保护区上界 (api.py:460 一致, 不允许大臂/手爪动)。place_y 必须 > 此值。"""

MOVE_X_V_MAX_MMS_DEFAULT: float = 40.0
"""业务层默认 x 移动速度 (task4 constants.py:59 同款)。"""

GRASP_HOLD_S: float = 0.5
"""grasp(True) 保持时间 (放货前不需要抓取, 这里指放前的最后一次微抬确认)。"""

LOG_PREFIX_TASK7: str = "[task7]"


# ---------- 自检 ----------
if __name__ == "__main__":
    print(f"=== task7 constants: 12 户 (2 单元 × 6 户) ===")
    for k, p in RESIDENT_PROFILES.items():
        print(f"  {k:18s}  x={p.place_x_mm:+6.0f}  pre_y={p.pre_y_mm:.0f}  "
              f"place_y={p.place_y_mm:.0f}  arm={p.arm_angle_deg:+5.0f}°  "
              f"hand={p.hand_angle_deg:+5.0f}°  v={p.v_max_mms:.0f}  settle={p.settle_s:.1f}")
    print(f"\n单元 1: {list_residents('1')}")
    print(f"单元 2: {list_residents('2')}")
    print(f"\n示例: get_profile('1', '103') = {get_profile('1', '103').full_name}, "
          f"x={get_profile('1', '103').place_x_mm}mm")
