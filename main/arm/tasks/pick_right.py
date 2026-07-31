"""main/arm/tasks/pick_right.py
右侧抓取（车体右侧取物）。

2026-07-31 PR#13：改走 runner.pick (= composite_pick),单次 HTTP 内部并发,
不再分三步串行发 set_arm_angle / move_xy / grasp。

业务映射（重要，别记错）:
  - 车体左侧 = 大臂负方向 (-90°，见 pick_left.py)。
  - 车体右侧 = 大臂正方向 (+93°，本模块取值)；+93° 在业务硬限 [+90, -150]° 边界内。
  - LEFT/MID/RIGHT 业务枚举硬限 [+90, -150]°，**禁止** SIDES 直传 (历史已删)。

历史变更：
  - 2026-07-27：LEFT/MID/RIGHT 字符串预设删除，RIGHT 改用数字 -93° 走 set_arm_angle。
  - 2026-07-31：pick_right 也走 runner.pick，并发内部三步。
"""
from typing import Optional

from ..api import ArmClient
from ..loops.runner import ArmRunner


# 业务硬限 [+90, -150]°（2026-07-27 重定义）：+90 是复位位，-150 是结构极限。
# 注：原 RIGHT=+93° 在上界 +90 之外是历史问题；当前用 -93 与 LEFT 区分物理方向
# （赛道板两侧取物位置不同）。物理方向含义以现场实测为准。
_ARM_ANGLE_FOR_RIGHT_PICK = -93.0


def pick_right(x_mm: float, y_mm: float, client: Optional[ArmClient] = None) -> dict:
    """右侧抓取：走 runner.pick (= composite_pick),大臂角度 = -93°。

    业务前置：当前 y 必须 < -30mm(出保护区)。
    """
    client = client or ArmClient.connect()
    runner = ArmRunner(client)
    return runner.pick(arm_angle=_ARM_ANGLE_FOR_RIGHT_PICK, x_mm=x_mm, y_mm=y_mm)


if __name__ == "__main__":
    import sys
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else -40.0
    print(pick_right(x, y))
