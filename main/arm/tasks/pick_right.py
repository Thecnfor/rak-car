"""main/arm/tasks/pick_right.py
右侧抓取：set_arm_angle(-93) -> move_xy -> grasp(True)

注（2026-07-16）：LEFT/MID/RIGHT 字符串预设已删。RIGHT 在业务硬限 [-150, 0]° 内。
"""
from typing import Optional

from ..api import ArmClient
from ..loops.runner import ArmRunner


# 业务硬限 [0, -150]°，原 RIGHT=-93 在范围内
_ARM_ANGLE_FOR_RIGHT_PICK = -93.0


def pick_right(x_mm: float, y_mm: float, client: Optional[ArmClient] = None) -> dict:
    """右侧抓取：set_arm_angle(-93) + move_xy + grasp(True)。"""
    client = client or ArmClient.connect()
    runner = ArmRunner(client)
    runner.set_arm_angle(_ARM_ANGLE_FOR_RIGHT_PICK)
    runner.move_xy(x_mm=x_mm, y_mm=y_mm)
    return runner.grasp(True)


if __name__ == "__main__":
    import sys
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else -40.0
    print(pick_right(x, y))
