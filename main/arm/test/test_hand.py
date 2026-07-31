#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""test_hand.py
手爪 PWM 舵机简单测试。

骨架:
  preflight -> 大臂角预检 (arm 必须停在 MID=+90°/复位位,否则 set_hand_angle 非 UP 会
             抛 ValueError 拒动 — 见 main/arm/api.py:464 展开区互斥规则)
          -> 预提 y 轴到 -150mm (move_y_position,UP 方向,避免手爪下放时擦地/撞车)
          -> 三个档位 (UP → MID → DOWN → MID → UP) 来回走 N 轮
          -> 每档位 car.get_arm_state 读回 hand_angle 与命令值比对
          -> 收尾:回到 UP 安全姿态
          -> postflight

硬件:
  手爪 PWM 舵机  ServoPwm port=2   (arm_cfg.yaml:hand_cfg.port)

判定标准 (任一 FAIL 整体 FAIL):
  1) API 层:每次 set_hand_angle HTTP 调用 status=succeeded,不抛异常
  2) 反馈层:每次 car.get_arm_state 读回的 hand_angle 与命令值偏差 ≤ ±5°
  3) 预检  :大臂在 MID(+90°/复位位)才能跑全部三档;非 MID 时直接 ABORT,提示先走 reset_all

业务硬限(看 ARM_API.md §1.1 set_hand_angle):
  angle ∈ [-90, 0]°  (UP=-90°, DOWN=0°)

⚠️ 真实硬件,舵机真的会转。先确认手爪周围没东西再跑。

运行:
  PowerShell:
    $env:RAK_CAR_SERVER_ORIGIN = "http://192.168.3.60"
    python main\\arm\\test\\test_hand.py
"""
import os
import sys
import time

# 让 main.* 可被 import
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.api_client import RuntimeApiClient  # noqa: E402
from main.arm.test._runtime_guard import preflight, postflight  # noqa: E402


# ---- 测试参数(可调) ----
N_CYCLES = 3                # 来回循环轮数
ANGLE_UP = -90              # 收起 (init 安全姿态)
ANGLE_MID = -45             # 半开
ANGLE_DOWN = 0              # 放下 (吸盘贴底,实际任务位)
SPEED = 80                  # PWM duty (car_wrap_2026 默认)
ANGLE_TOLERANCE_DEG = 5     # 读回与命令的容许偏差
ARM_REQUIRED_DEG = 90       # 大臂必须停在 MID / 复位位 (+90°, 2026-07-27 后),否则 set_hand_angle 非 UP 拒动

SET_TIMEOUT_S = 12.0        # set_hand_angle 的 HTTP 超时
READ_TIMEOUT_S = 8.0        # get_arm_state 的 HTTP 超时
SETTLE_S = 0.6              # 到位后等 0.6s 再读,避 PWM 还没收敛

INITIAL_LIFT_Y_MM = -150    # 主循环前先把 y 抬到 -150mm (UP方向) — 业务坐标系 y<0=向上
LIFT_TIMEOUT_S = 18.0       # 抬升 move_y_position 的 HTTP 超时 (实际 5-12s)
LIFT_SETTLE_S = 1.0         # 抬升后等待磁感/PWM 收敛再读回

ANGLE_NAME = {-90: "UP", -45: "MID", 0: "DOWN"}
SEQ = [ANGLE_UP, ANGLE_MID, ANGLE_DOWN, ANGLE_MID, ANGLE_UP]  # 一轮序列


# ---- helpers ----
def call_set(c: RuntimeApiClient, angle: int, timeout: float) -> tuple:
    """set_hand_angle 同步调用。kwargs 显式给 angle + speed,sync=True 必填
    (runtime v2 默认 async 会立刻返回 status=running,见 memory execute-sync-default)。
    """
    try:
        r = c.execute_arm_action(
            "set_hand_angle",
            angle=angle, speed=SPEED,
            timeout=timeout, sync=True,
        )
        return r.get("status"), r.get("error")
    except Exception as e:
        return "exception", str(e)[:120]


def call_read(c: RuntimeApiClient, timeout: float):
    """car.get_arm_state 同步读舵机反馈。返回 (hand_angle, status_msg, error_msg)。"""
    try:
        r = c.execute_car_action("get_arm_state", timeout=timeout, sync=True)
        if r.get("status") != "succeeded":
            return None, r.get("status"), r.get("error")
        result = r.get("result") or {}
        return result.get("hand_angle"), "ok", None
    except Exception as e:
        return None, "exception", str(e)[:120]


def arm_precheck(c: RuntimeApiClient) -> bool:
    """大臂预检:arm_angle 必须 = ARM_REQUIRED_DEG,否则 set_hand_angle 非 UP 会
    被 api.py:464 的"展开区互斥规则"挡住。
    """
    a, s, e = call_read(c, READ_TIMEOUT_S)
    if a is None:
        print(f"[FAIL] get_arm_state 读不回来: status={s}  err={e}")
        return False
    # arm_angle 字段在 result 里,需要重读拿全 — 上面的 call_read 只取了 hand_angle
    try:
        r = c.execute_car_action("get_arm_state", timeout=READ_TIMEOUT_S, sync=True)
        result = (r.get("result") or {})
        arm = result.get("arm_angle")
    except Exception as e:
        print(f"[FAIL] get_arm_state (取 arm_angle) 异常: {str(e)[:120]}")
        return False
    if arm != ARM_REQUIRED_DEG:
        print(f"[ABORT] 大臂必须在 MID({ARM_REQUIRED_DEG}°) 才能跑三档测试")
        print(f"        当前 arm_angle = {arm}°")
        print(f"        先走 ArmClient.reset_all() 或 examples/01_calibrate_origin.py")
        return False
    print(f"  [OK  ] arm_angle = {arm}° (MID), 可跑手爪三档")
    return True


def call_lift_y(c: RuntimeApiClient, y_mm: float, timeout: float) -> tuple:
    """move_y_position 同步调用。y_mm 用业务坐标 (mm,负=UP/远离触底),内部转 m。

    ⚠️ sign 约定 (2026-07-17 翻负): y_mm=-150 → SDK 端 target=-0.150 m → 物理向上。
    业务坐标 vs SDK 完全一致: y<0=向上,y=0=触底。
    """
    target_m = y_mm / 1000.0
    try:
        r = c.execute_arm_action(
            "move_y_position",
            target=target_m,
            timeout=timeout, sync=True,
        )
        return r.get("status"), r.get("error")
    except Exception as e:
        return "exception", str(e)[:120]


# ---- main ----
def main() -> int:
    c = RuntimeApiClient()

    # ---- runtime 就绪检查 ----
    if not preflight(c):
        return 1
    print()

    # ---- 大臂预检 ----
    print("=== 大臂预检 ===")
    if not arm_precheck(c):
        return 1
    print()

    # ---- 预提 y 轴到 INITIAL_LIFT_Y_MM ----
    print("=== 预提 y 轴 ===")
    target_y_mm = INITIAL_LIFT_Y_MM
    target_y_m = target_y_mm / 1000.0
    direction = "UP" if target_y_mm < 0 else "DOWN"
    print(f"    目标: y = {target_y_mm} mm ({direction}方向)")
    status, err = call_lift_y(c, target_y_mm, LIFT_TIMEOUT_S)
    ok_lift = (status == "succeeded")
    flag = "OK  " if ok_lift else "FAIL"
    print(f"  [{flag}] move_y_position(target={target_y_m:.3f} m)  "
          f"status={status}  err={err}")
    if not ok_lift:
        print(f"[ABORT] 预提失败,主循环不跑 (可能 y 轴卡死 / 限位 / 编码器异常)")
        return 1
    time.sleep(LIFT_SETTLE_S)
    # 读回当前 y 确认落位
    try:
        r = c.execute_car_action("get_arm_state", timeout=READ_TIMEOUT_S, sync=True)
        y_now_m = (r.get("result") or {}).get("y")
    except Exception as e:
        print(f"  [WARN] 读回 y 失败: {str(e)[:80]}")
        y_now_m = None
    if y_now_m is not None:
        y_now_mm = y_now_m * 1000.0
        y_diff_mm = y_now_mm - target_y_mm
        lift_within = (abs(y_diff_mm) <= 5.0)
        lflag = "OK  " if lift_within else "WARN"
        print(f"  [{lflag}] 当前 y = {y_now_mm:.1f} mm  "
              f"Δ={y_diff_mm:+.1f} mm (目标 {target_y_mm} mm, 容差 ±5 mm)")
        if not lift_within:
            print(f"  [WARN] y 落位偏差超过容差,主循环会因手爪高度异常影响判定,可手动 reset_y 后重跑")
    else:
        print(f"  [WARN] y 读不回来,跳过落位验证")
    print()

    fails = 0
    total_set_ok = 0
    total_read_ok = 0
    total_within = 0

    # ---- 主循环 ----
    seq_str = " -> ".join(f"{a:>4}°/{ANGLE_NAME.get(a, '?')}" for a in SEQ)
    print(f"=== {N_CYCLES} 轮 三档位来回走 ===")
    print(f"    序列: {seq_str}")
    print(f"    容差 ±{ANGLE_TOLERANCE_DEG}°, speed={SPEED}")
    print()
    for i in range(1, N_CYCLES + 1):
        print(f"--- 轮 #{i} ---")
        for a in SEQ:
            # ---- set ----
            status, err = call_set(c, a, SET_TIMEOUT_S)
            ok_set = (status == "succeeded")
            if ok_set:
                total_set_ok += 1
            else:
                fails += 1
            flag = "OK  " if ok_set else "FAIL"
            print(f"  [{flag}] set_hand_angle({a:>4}°/{ANGLE_NAME.get(a, '?'):<4})  "
                  f"speed={SPEED}  status={status}  err={err}")
            time.sleep(SETTLE_S)

            # ---- readback ----
            hand, rs, re_ = call_read(c, READ_TIMEOUT_S)
            if hand is None:
                print(f"  [FAIL] get_arm_state 读不回来  status={rs}  err={re_}")
                fails += 1
                continue
            total_read_ok += 1
            diff = hand - a
            within = (abs(diff) <= ANGLE_TOLERANCE_DEG)
            if within:
                total_within += 1
            else:
                fails += 1
            rflag = "OK  " if within else "WARN"
            print(f"  [{rflag}] readback hand_angle={hand:>5}°  "
                  f"Δ={diff:+d}° (命令 {a}°, 容差 ±{ANGLE_TOLERANCE_DEG}°)")
        print()

    # ---- 收尾:UP 安全位 ----
    print("=== 收尾:回到 UP 安全位 ===")
    status, err = call_set(c, ANGLE_UP, SET_TIMEOUT_S)
    flag = "OK  " if status == "succeeded" else "FAIL"
    print(f"  [{flag}] set_hand_angle({ANGLE_UP}°/{ANGLE_NAME[ANGLE_UP]})  status={status}  err={err}")
    print()

    # ---- 跑后 health ----
    print("=== 跑后 health ===")
    postflight(c, "after")
    print()

    # ---- 总结 ----
    total_moves = N_CYCLES * len(SEQ)
    print("=== 总结 ===")
    print(f"  SET    调用  : {total_set_ok}/{total_moves} succeeded")
    print(f"  READBACK     : {total_read_ok}/{total_moves} 成功读回 hand_angle")
    print(f"    ├─ 容差内  : {total_within}")
    print(f"    └─ 容差外  : {total_read_ok - total_within}")
    print()
    if fails == 0:
        print("结果: PASS  (舵机响应正常,反馈与命令一致)")
        rc = 0
    else:
        print(f"结果: FAIL  ({fails} 项不通过)")
        rc = 1
    print()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
