#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""test_arm_servo.py
大臂总线舵机 (ServoBus port=3) 简单测试。

骨架:
  preflight -> 手爪角预检 (hand 必须停在 UP=-90°,否则大臂往展开区走手爪
             会被互斥规则挡 — 见 main/arm/api.py:464 展开区互斥)
          -> 预提 y 轴到 -150mm (UP 方向,给大臂展开留出垂直空间,避免 y 保护区)
          -> 大臂三档位来回走 N 轮 (MID → HALF → FAR → HALF → MID)
          -> 每档位 car.get_arm_state 读回 arm_angle 与命令值比对
          -> 收尾:回到 MID / 复位位 (=+90°) 安全姿态
          -> postflight

硬件:
  大臂总线舵机  ServoBus port=3   (arm_cfg.yaml:arm_cfg.port)

判定标准 (任一 FAIL 整体 FAIL):
  1) API 层:每次 set_arm_angle HTTP 调用 status=succeeded,不抛异常
  2) 反馈层:每次 car.get_arm_state 读回的 arm_angle 与命令值偏差 ≤ ±5°
  3) 预检  :手爪在 UP=-90° 才能跑;非 UP 时直接 ABORT,提示先走 reset_all

业务硬限(看 ARM_API.md §1.1 set_arm_angle, 2026-07-27 重定义):
  angle ∈ [+90, -150]°  —— **只往负方向展开 (+90 是复位位)**,正方向 +30° 已禁 (LEFT=+93° 会撞车)
  +90° = MID / 复位位 (init)
  -60° = 半展开
  -120°= 接近全展 (留 30° 余量,防协议触发)

⚠️ 真实硬件,舵机真的会转。确认大臂周围没东西再跑。

运行:
  PowerShell:
    $env:RAK_CAR_SERVER_ORIGIN = "http://192.168.3.60"
    python main\\arm\\test\\test_arm_servo.py
"""
import os
import sys
import time

# 让 main.* 可被 import
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.api_client import RuntimeApiClient  # noqa: E402
from main.arm.runtime_guard._runtime_guard import preflight, postflight  # noqa: E402


# ---- 测试参数(可调) ----
N_CYCLES = 3                # 来回循环轮数
ANGLE_MID = 90              # MID / 复位位 (init 安全姿态, 2026-07-27 后)
ANGLE_HALF = -60            # 半展开
ANGLE_FAR = -120            # 接近全展 (留 30° 余量)
SPEED = 80                  # 总线舵机速度
ANGLE_TOLERANCE_DEG = 5     # 读回与命令的容许偏差
HAND_REQUIRED_DEG = -90     # 手爪必须停在 UP=-90°,否则大臂展开被互斥规则挡

SET_TIMEOUT_S = 14.0        # set_arm_angle 的 HTTP 超时 (大臂总线动作可能慢)
READ_TIMEOUT_S = 8.0        # get_arm_state 的 HTTP 超时
SETTLE_S = 0.8              # 到位后等 0.8s 再读,避总线还在追

INITIAL_LIFT_Y_MM = -150    # 预提 y=-150mm,业务坐标系 y<0=向上
LIFT_TIMEOUT_S = 18.0       # 抬升 move_y_position 的 HTTP 超时
LIFT_SETTLE_S = 1.0         # 抬升后等待磁感/PWM 收敛再读回

ANGLE_NAME = {90: "MID", -60: "HALF", -120: "FAR"}
SEQ = [ANGLE_MID, ANGLE_HALF, ANGLE_FAR, ANGLE_HALF, ANGLE_MID]


# ---- helpers ----
def call_set_arm(c: RuntimeApiClient, angle: int, timeout: float) -> tuple:
    """set_arm_angle 同步调用。kwargs 显式给 angle + speed, sync=True 必填
    (runtime v2 默认 async 会立刻返回 status=running,见 memory execute-sync-default)。
    """
    try:
        r = c.execute_arm_action(
            "set_arm_angle",
            angle=angle, speed=SPEED,
            timeout=timeout, sync=True,
        )
        return r.get("status"), r.get("error")
    except Exception as e:
        return "exception", str(e)[:120]


def call_read(c: RuntimeApiClient, timeout: float):
    """car.get_arm_state 同步读舵机反馈。返回 (arm_angle, status_msg, error_msg)。"""
    try:
        r = c.execute_car_action("get_arm_state", timeout=timeout, sync=True)
        if r.get("status") != "succeeded":
            return None, r.get("status"), r.get("error")
        result = r.get("result") or {}
        return result.get("arm_angle"), "ok", None
    except Exception as e:
        return None, "exception", str(e)[:120]


def call_lift_y(c: RuntimeApiClient, y_mm: float, timeout: float) -> tuple:
    """move_y_position 同步调用。y_mm 业务坐标(负=UP),内部 mm→m。"""
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


def hand_precheck(c: RuntimeApiClient) -> bool:
    """手爪预检:hand_angle 必须 = HAND_REQUIRED_DEG(=-90°/UP)。

    大臂往展开区走时,非 UP 的手爪会被 api.py:464 的展开区互斥规则挡住 ——
    不是遥操问题,是业务层软锁。先保证手爪在 UP,主循环才不会第一个 set_arm_angle
    就报 ValueError。
    """
    try:
        r = c.execute_car_action("get_arm_state", timeout=READ_TIMEOUT_S, sync=True)
        hand = (r.get("result") or {}).get("hand_angle")
    except Exception as e:
        print(f"[FAIL] get_arm_state 异常: {str(e)[:120]}")
        return False
    if hand is None:
        print(f"[FAIL] 手爪角读不回来 (hand_angle=None)")
        return False
    if hand != HAND_REQUIRED_DEG:
        print(f"[ABORT] 手爪必须在 UP({HAND_REQUIRED_DEG}°) 才能跑大臂展开测试")
        print(f"        当前 hand_angle = {hand}°")
        print(f"        先手 set_hand_angle(-90°) 或 ArmClient.reset_all()")
        return False
    print(f"  [OK  ] hand_angle = {hand}° (UP), 可跑大臂三档")
    return True


# ---- main ----
def main() -> int:
    c = RuntimeApiClient()

    # ---- runtime 就绪检查 ----
    if not preflight(c):
        return 1
    print()

    # ---- 手爪预检 ----
    print("=== 手爪预检 ===")
    if not hand_precheck(c):
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
            print(f"  [WARN] y 落位偏差超过容差,可手动 reset_y 后重跑")
    else:
        print(f"  [WARN] y 读不回来,跳过落位验证")
    print()

    fails = 0
    total_set_ok = 0
    total_read_ok = 0
    total_within = 0

    # ---- 主循环 ----
    seq_str = " -> ".join(f"{a:>4}°/{ANGLE_NAME.get(a, '?')}" for a in SEQ)
    print(f"=== {N_CYCLES} 轮 大臂三档位来回走 ===")
    print(f"    序列: {seq_str}")
    print(f"    容差 ±{ANGLE_TOLERANCE_DEG}°, speed={SPEED}")
    print()
    for i in range(1, N_CYCLES + 1):
        print(f"--- 轮 #{i} ---")
        for a in SEQ:
            # ---- set ----
            status, err = call_set_arm(c, a, SET_TIMEOUT_S)
            ok_set = (status == "succeeded")
            if ok_set:
                total_set_ok += 1
            else:
                fails += 1
            flag = "OK  " if ok_set else "FAIL"
            print(f"  [{flag}] set_arm_angle({a:>5}°/{ANGLE_NAME.get(a, '?'):<4})  "
                  f"speed={SPEED}  status={status}  err={err}")
            time.sleep(SETTLE_S)

            # ---- readback ----
            arm, rs, re_ = call_read(c, READ_TIMEOUT_S)
            if arm is None:
                print(f"  [FAIL] get_arm_state 读不回来  status={rs}  err={re_}")
                fails += 1
                continue
            total_read_ok += 1
            diff = arm - a
            within = (abs(diff) <= ANGLE_TOLERANCE_DEG)
            if within:
                total_within += 1
            else:
                fails += 1
            rflag = "OK  " if within else "WARN"
            print(f"  [{rflag}] readback arm_angle={arm:>5}°  "
                  f"Δ={diff:+d}° (命令 {a}°, 容差 ±{ANGLE_TOLERANCE_DEG}°)")
        print()

    # ---- 收尾:MID 安全位 ----
    print("=== 收尾:回到 MID 安全位 ===")
    status, err = call_set_arm(c, ANGLE_MID, SET_TIMEOUT_S)
    flag = "OK  " if status == "succeeded" else "FAIL"
    print(f"  [{flag}] set_arm_angle({ANGLE_MID}°/{ANGLE_NAME[ANGLE_MID]})  status={status}  err={err}")
    print()

    # ---- 跑后 health ----
    print("=== 跑后 health ===")
    postflight(c, "after")
    print()

    # ---- 总结 ----
    total_moves = N_CYCLES * len(SEQ)
    print("=== 总结 ===")
    print(f"  SET    调用  : {total_set_ok}/{total_moves} succeeded")
    print(f"  READBACK     : {total_read_ok}/{total_moves} 成功读回 arm_angle")
    print(f"    ├─ 容差内  : {total_within}")
    print(f"    └─ 容差外  : {total_read_ok - total_within}")
    print()
    if fails == 0:
        print("结果: PASS  (大臂舵机响应正常,反馈与命令一致)")
        rc = 0
    else:
        print(f"结果: FAIL  ({fails} 项不通过)")
        rc = 1
    print()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
