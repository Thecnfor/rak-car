#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""test_y_up.py
机械臂 y 轴向上 最简测试（业务层 API）。

骨架（按 ARM_API.md）:
  preflight -> reset_y()               # 触底 -> y=0
           -> 沿 y 轴跑多目标点 (向上爬升再回到底)
           -> 每点读 state.y_mm,核对误差
           -> postflight -> 总结

判定:
  - 每点 |y_actual - y_target| < tol 算 PASS
  - API 返回 succeeded 不代表真动 —— 用 get_state().y_mm 做物理判定
  - 只验证 y 轴,不验证 x 轴是否还在工作

约束:
  - 走 main.arm.ArmClient 业务封装(ARM_API.md:30),不绕开调 raw action
  - 单位 mm,move_y(150.0) 业务层自动 /1000 下发
  - y 区间 [-180, 0]mm(语义翻转后),目标必须在区间内
  - main.arm.test._runtime_guard.preflight/postflight 统一 3 类异常处理
  - ⚠️ ArmClient.reset_y() 实际调 arm.reset_position —— 同时复位 x;
    因此测试开始时 x 也会被撞墙归位。这是预期的,不是 bug。

运行:
  PowerShell:
    $env:RAK_CAR_SERVER_ORIGIN = "http://192.168.3.60"
    python main\\arm\\test\\test_y_up.py
"""
import os
import sys

# 让 main.* 可被 import
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.runtime_guard._runtime_guard import preflight, postflight  # noqa: E402


# y 轴目标序列 mm:从上限向下探再回到顶(语义已翻转)
# y 区间 [-180, 0],-150 安全(距下界 30mm 留余量)
TARGETS_MM = [-120.0, -80.0, -150.0, -80.0, 0.0]
TOL_MM = 5.0   # 车端 PID 闭环典型 <2mm;放宽到 5mm 给老舵机余量


def main() -> int:
    client = ArmClient.connect()

    # ---- runtime 就绪检查(localhost / Connection / 未初始化 三类分别处理)----
    if not preflight(client):
        return 1
    print()

    runner = ArmRunner(client)

    # ---- 1) reset_y:从触底开始(y=0)----
    # ⚠️ 实际调 arm.reset_position —— y 触底 + x 堵转 一起
    print("=== 1) client.reset_y()  [arm.reset_position: y 触底 + x 堵转] ===")
    try:
        r = client.reset_y()
        reset_ok = (r.get("status") == "succeeded")
    except Exception as e:
        reset_ok = False
        print(f"  [FAIL] exception: {type(e).__name__}: {str(e)[:120]}")
    else:
        flag = "OK  " if reset_ok else "FAIL"
        print(f"  [{flag}] status={r.get('status')}  err={r.get('error')}")
    print()

    if not reset_ok:
        return 1

    fails = 0

    # ---- 2) 沿 y 轴跑多目标点 ----
    print(f"=== 2) runner.move_y() 沿 y 轴跑 {len(TARGETS_MM)} 个目标点 ===")
    for ty in TARGETS_MM:
        try:
            job = runner.move_y(y_mm=ty)
        except Exception as e:
            print(f"  [FAIL] cmd y={ty:6.1f}mm  exception: {type(e).__name__}: {str(e)[:80]}")
            fails += 1
            continue

        st = client.get_state()
        err = st.y_mm - ty
        ok = (job.get("status") == "succeeded") and (abs(err) < TOL_MM)
        flag = "OK  " if ok else "FAIL"
        print(f"  [{flag}] cmd y={ty:6.1f}mm  actual y={st.y_mm:6.1f}mm  "
              f"err={err:+5.1f}mm  (tol +/-{TOL_MM:.0f}mm)")
        if not ok:
            fails += 1

    print()

    # ---- 跑后 health ----
    postflight(client, "after")
    print()

    # ---- 总结 ----
    total = len(TARGETS_MM) + 1   # +1 = reset_y
    passed = total - fails
    label = "PASS" if fails == 0 else "FAIL"
    print("=== 总结 ===")
    print(f"  目标序列: {' -> '.join(f'{t:.0f}' for t in [0.0] + TARGETS_MM)} mm (含 reset_y 起点)")
    print(f"  容差:     +/-{TOL_MM:.0f}mm")
    print(f"  结果:     {passed}/{total} ok")
    print()
    print(label)
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())