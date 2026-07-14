"""main/arm/tasks/pick_right.py
右侧抓取：set_side(RIGHT) -> move_xy -> grasp(True)
"""
from typing import Optional

from ..api import ArmClient
from ..loops.runner import ArmRunner


def pick_right(x_mm: float, y_mm: float, client: Optional[ArmClient] = None) -> dict:
    client = client or ArmClient.connect()
    runner = ArmRunner(client)
    return runner.pick("RIGHT", x_mm=x_mm, y_mm=y_mm)


if __name__ == "__main__":
    import sys
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
    print(pick_right(x, y))
