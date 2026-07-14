"""main/arm/tasks/go_home.py
回到原点 + 安全姿态：y=0, x=0, hand=UP, side=MID。
"""
from typing import Optional

from ..api import ArmClient
from ..loops.runner import ArmRunner


def go_home(client: Optional[ArmClient] = None) -> dict:
    client = client or ArmClient.connect()
    runner = ArmRunner(client)
    return runner.go_home()


if __name__ == "__main__":
    print(go_home())
