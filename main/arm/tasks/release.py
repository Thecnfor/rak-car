"""main/arm/tasks/release.py
释放：set_hand(DOWN) -> move_xy -> grasp(False)
"""
from typing import Optional

from ..api import ArmClient
from ..loops.runner import ArmRunner


def release(
    drop_x_mm: float = 0.0,
    drop_y_mm: float = 30.0,
    client: Optional[ArmClient] = None,
) -> dict:
    client = client or ArmClient.connect()
    runner = ArmRunner(client)
    return runner.release(drop_x_mm=drop_x_mm, drop_y_mm=drop_y_mm)


if __name__ == "__main__":
    import sys
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    print(release(x, y))
