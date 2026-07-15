#!/usr/bin/python3
"""test_servo_pump.py
机械臂 舵机(大臂 + 手爪) + 吸气泵 测试。

测试项:
  1) 大臂 bus 舵机   (port=2)   arm.set_arm_angle  : LEFT/MID/RIGHT
  2) 手爪 PWM 舵机   (port=2)   arm.set_hand_angle  : UP/MID/DOWN
  3) 吸气泵 + 阀      (2/3)     arm.grasp           : True/False

⚠️ 真实硬件:
   - 大臂/手爪会真的转动,先确认机械臂周围无障碍物
   - 吸气泵会真的通电/断电,空吸盘看不出来,听电机声 + 看电流

运行:
  export RAK_CAR_SERVER_ORIGIN=http://192.168.3.60
  python3 main/arm/test/test_servo_pump.py
"""
import os
import sys

# 把项目根目录(rak-car/)加到 sys.path,这样才能 import main.*
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.test._runtime_guard import preflight, postflight  # noqa: E402


def main() -> None:
    client = ArmClient.connect()
    # 跳过 client.ping() —— api.py:340 的 ping() 有 bug

    # ---- runtime 就绪检查 (支持 ArmClient,自动 unwrap .http) ----
    if not preflight(client):
        sys.exit(1)
    print()

    runner = ArmRunner(client)

    st = client.get_state()
    print("=== 初始状态 ===")
    print(f"  x={st.x_mm:.1f}mm  y={st.y_mm:.1f}mm  side={st.side}  hand={st.hand}")
    print(f"  arm_angle={st.arm_angle}  hand_angle={st.hand_angle}")
    print()

    # ---- 1) 大臂 bus 舵机 ----
    # 大臂舵机一次 ~93° 旋转 + 串口 round-trip,常 >10s。timeout=10 太短,
    # 之前 TimeoutError: HTTPConnectionPool Read timed out. 必须放宽到 30s
    # 跟 api.py:reset_x/y 默认 30s 对齐。
    print("=== 1) 大臂 bus 舵机 (set_side) ===")
    for side in ["LEFT", "MID", "RIGHT", "MID"]:
        runner.set_side(side, timeout=30)
        cur = client.get_state()
        match = "OK" if cur.side == side else "MISMATCH"
        print(f"  cmd={side:<6}  actual side={cur.side:<6}  arm_angle={cur.arm_angle}  [{match}]")
    print()

    # ---- 2) 手爪 PWM 舵机 ----
    print("=== 2) 手爪 PWM 舵机 (set_hand) ===")
    for hand in ["UP", "MID", "DOWN", "UP"]:
        runner.set_hand(hand, timeout=20)
        cur = client.get_state()
        match = "OK" if cur.hand == hand else "MISMATCH"
        print(f"  cmd={hand:<5}  actual hand={cur.hand:<5}  hand_angle={cur.hand_angle}  [{match}]")
    print()

    # ---- 3) 吸气泵 + 阀 ----
    print("=== 3) 吸气泵 (grasp) ===")
    for on in [True, False, True, False]:
        runner.grasp(on, timeout=20)
        # 车端没暴露 grasping 反馈,只能验证调用成功 + 听声音/看电流
        label = "ON (吸)" if on else "OFF (放)"
        print(f"  cmd=grasp({on})  -> {label}  [调用成功,无反馈字段]")
    print()

    print("=== done ===")

    postflight(client, "after")


if __name__ == "__main__":
    main()
