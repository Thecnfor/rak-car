#!/usr/bin/python3
"""task4 / grasp —— 真空泵冒烟测试 (吸气 3 秒 + 放气)

⚠️ 本脚本**不**做位姿 setup, 调用前必须先保证:
  - y 出 [0, -80] 保护区 (任意 ≤ -80 mm 即可)
  - arm/hand 角度任意 (不影响 grasp)
  - x 任意

典型用法 (在 test_yellow/test_blue 摆好位姿后单独跑):
  1. python test_yellow.py    # 摆位姿到 x=-65 (或其他位置)
  2. python grasp.py          # 在当前位置抽气 3 秒
  3. 验证: 听泵声 / 看指示灯 / 测吸力 (要测吸力需先在地上放块小纸片对准吸盘)

输出:
  - {"ok": True, "pump": "tested_3s_on_off"}
"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


GRASP_HOLD_S: float = 5  # 吸气保持时长


def step_grasp(client: ArmClient, runner: ArmRunner,
               hold_s: float = GRASP_HOLD_S) -> dict:
    """吸气 hold_s 秒 + 放气.

    Args:
        client: ArmClient 实例.
        runner: ArmRunner 实例.
        hold_s: 吸气保持秒数 (默认 3.0).

    Returns:
        {"ok": True, "pump": "tested_<hold_s>s_on_off"}
    """
    print("=== [grasp] 真空泵冒烟测试 ===")
    print(f"  [pump] grasp(True)   吸气 (hold {hold_s:.1f}s)")
    runner.grasp(True, timeout=10.0)
    print(f"  [pump] sleep({hold_s:.1f})    保持")
    time.sleep(hold_s)
    print("  [pump] grasp(False)  放气")
    runner.grasp(False, timeout=10.0)
    print(f"=== [grasp] 完成 (抽气 {hold_s:.1f}s + 放气) ===\n")
    return {
        "ok": True,
        "pump": f"tested_{hold_s:.1f}s_on_off",
        "hold_s": hold_s,
    }


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_grasp(client, runner)


if __name__ == "__main__":
    main()