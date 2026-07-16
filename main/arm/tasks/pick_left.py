"""main/arm/tasks/pick_left.py
左侧抓取：set_arm_angle(-90) -> move_xy -> grasp(True)

注：set_side("LEFT") 已禁用（LEFT=+93° 撞车），改用 set_arm_angle(-90)。
"左侧" 是车体布局语义（车体左侧取物），不再用 SIDES 枚举。
"""
from typing import Optional

from ..api import ArmClient
from ..loops.runner import ArmRunner


# 业务硬限：0 是最大，-150 是物理下界
_ARM_ANGLE_FOR_LEFT_PICK = -90.0   # ≈ 旧 LEFT=93° 的反向


def pick_left(x_mm: float, y_mm: float, client: Optional[ArmClient] = None) -> dict:
    """左侧抓取：业务 set_arm_angle(-90) + move_xy + grasp(True)。

    注：底层走 set_arm_angle（数字角度），绕过 SIDES="LEFT" 业务硬限检查。
    """
    client = client or ArmClient.connect()
    runner = ArmRunner(client)
    runner.set_arm_angle(_ARM_ANGLE_FOR_LEFT_PICK)
    runner.move_xy(x_mm=x_mm, y_mm=y_mm)
    return runner.grasp(True)


if __name__ == "__main__":
    import sys
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else -40.0
    print(pick_left(x, y))