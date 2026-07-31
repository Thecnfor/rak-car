"""main/arm/tasks/pick_left.py
左侧抓取（车体左侧取物）。

2026-07-31 PR#13：改走 runner.pick（= composite_pick），单次 HTTP 内部并发，
不再分三步串行发 set_arm_angle / move_xy / grasp。

业务映射（重要，别记错）:
  - 车体左侧 = 大臂负方向（-90°），因为 +93°（旧 LEFT 协议值）在物理结构上撞车。
  - 车体右侧 = 大臂正方向（+93°，需先出 y 保护区，见 pick_right.py）。
  - LEFT/MID/RIGHT 业务枚举硬限 [+90, -150]°，**禁止** SIDES="LEFT" 直传
    （SIDES["LEFT"]=+93° 越界）。本模块只走 set_arm_angle 数字接口。
"""
from typing import Optional

from ..api import ArmClient
from ..loops.runner import ArmRunner


# 业务硬限 [+90, -150]°：+90 是复位位，-150 是结构极限（2026-07-27 重定义）
_ARM_ANGLE_FOR_LEFT_PICK = -90.0   # ≈ 旧 LEFT=+93° 的反向


def pick_left(x_mm: float, y_mm: float, client: Optional[ArmClient] = None) -> dict:
    """左侧抓取：走 runner.pick (= composite_pick),大臂角度 = -90°。

    业务前置：当前 y 必须 < -30mm(出保护区)。
    """
    client = client or ArmClient.connect()
    runner = ArmRunner(client)
    return runner.pick(arm_angle=_ARM_ANGLE_FOR_LEFT_PICK, x_mm=x_mm, y_mm=y_mm)


if __name__ == "__main__":
    import sys
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else -40.0
    print(pick_left(x, y))