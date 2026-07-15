#!/usr/bin/python3
"""test_arm_servo.py
机械臂 大臂(bus 舵机,port=2)测试。

测试项:set_side("LEFT" / "MID" / "RIGHT") 后读回 arm_angle
预期角度(arm_cfg.yaml:hand_cfg.hand.angle_list):
  LEFT  =  93
  MID   =   0
  RIGHT = -93

⚠️ 真实硬件,大臂会真的转动。先确认机械臂周围无障碍物。

运行:
  export RAK_CAR_SERVER_ORIGIN=http://192.168.3.60
  python3 main/arm/test/test_arm_servo.py
"""
import os
import sys

# 把项目根目录(rak-car/)加到 sys.path,这样才能 import main.*
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.test._runtime_guard import preflight, postflight  # noqa: E402


# 期望角度表(与 arm_cfg.yaml:hand_cfg.hand.angle_list 对齐)
EXPECTED_ANGLE = {
    "LEFT": 93,
    "MID": 0,
    "RIGHT": -93,
}

# 测试序列:从当前方向出发 -> 走完全部 3 个目标 -> 回到 MID
SEQUENCE = ["LEFT", "MID", "RIGHT", "MID"]


def main() -> None:
    client = ArmClient.connect()
    # 跳过 client.ping() —— api.py:340 的 ping() 有 bug

    # ---- runtime 就绪检查 ----
    if not preflight(client):
        sys.exit(1)
    print()

    runner = ArmRunner(client)

    st = client.get_state()
    print("=== 初始状态 ===")
    print(f"  side={st.side}  arm_angle={st.arm_angle}")
    print()

    print("=== 大臂 bus 舵机测试 ===")
    fails = 0
    for side in SEQUENCE:
        runner.set_side(side, timeout=10)
        cur = client.get_state()
        expect = EXPECTED_ANGLE[side]
        side_ok = (cur.side == side)
        angle_ok = (cur.arm_angle == expect)
        flag = "OK  " if (side_ok and angle_ok) else "FAIL"
        print(
            f"  [{flag}] cmd={side:<6}  actual side={cur.side:<6}  "
            f"arm_angle={cur.arm_angle:>4}  (expected {expect:>3})  "
            f"side_match={side_ok}  angle_match={angle_ok}"
        )
        if not (side_ok and angle_ok):
            fails += 1

    print()
    total = len(SEQUENCE)
    postflight(client, "after")
    print(f"{'PASS' if fails == 0 else 'FAIL'}: {total - fails}/{total} ok")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
