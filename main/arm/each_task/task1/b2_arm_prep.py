#!/usr/bin/python3
"""task1 / step b2 —— 机械臂准备:手爪 + 大臂方向

动作:
  - set_hand("DOWN")  手爪放下,吸盘朝下准备吸取
  - set_side("MID")   大臂默认居中(如果种子在视野左/右再调 LEFT/RIGHT)

⚠️ 真实硬件,手爪/大臂会真的转
依赖:B1 已返回 seed 信息
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b2_arm_prep(client: ArmClient, runner: ArmRunner,
                    side: str = "MID", hand: str = "DOWN") -> dict:
    """B2:arm 准备。默认 DOWN+MID;如果种子偏一侧,传 side='LEFT'/'RIGHT'。"""
    print("=== [B2] 机械臂准备 ===")
    print(f"  [arm] set_side({side})  大臂指向 {side}")
    runner.set_side(side, timeout=10)
    print(f"  [arm] set_hand({hand})  手爪 {'放下(吸盘朝下)' if hand == 'DOWN' else '抬起'}")
    runner.set_hand(hand, timeout=10)
    print("=== [B2] 完成 ===\n")
    return {"side": side, "hand": hand}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b2_arm_prep(client, runner)


if __name__ == "__main__":
    main()
