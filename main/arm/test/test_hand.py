#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""test_hand.py
机械臂 末端舵机 (手爪 PWM servo, port=2 mode=180) 最简单测试。

骨架:
  health check -> set_hand_angle(UP/-90 → MID/-37 → DOWN/0 → MID/-37) 顺序
              -> 每站读 get_arm_state.result.hand_angle,核对期望值
              -> 跑后 health
              -> 总结 (API + 角度判定)

期望角度 (arm_cfg.yaml:hand_cfg.hand2.angle_list):
  UP   = -90
  MID  = -37
  DOWN =   0

约束:
  - 直接走 RuntimeApiClient.execute_arm_action(**kwargs), 避开
    api.py 的 timeout 占位冲突 (跟 grasp bug 同源)。
  - 默认 SERVER_ORIGIN 已是 http://192.168.3.60 (main/settings.py:6),
    PowerShell 直接跑就行,不用先 export。
  - ⚠️ 真硬件:手爪会真的转动,先确认周围无障碍物。

运行:
  PowerShell:
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


# 测试点序列:(方向名,期望角度,角度容差)
# 走 UP -> MID -> DOWN -> MID 是常见起手 + 收尾。
SEQUENCE = [
    ("UP",   -90),
    ("MID",  -37),
    ("DOWN",   0),
    ("MID",  -37),
]
ANGLE_TOL = 2  # 闭环舵机精度大概 ±1°,放宽到 2°
HAND_TIMEOUT_S = 12.0
READ_TIMEOUT_S = 8.0


def call_arm(c: RuntimeApiClient, name: str, timeout: float, **kwargs) -> dict:
    """单次 arm.* 调用,kwargs 透传。"""
    try:
        r = c.execute_arm_action(name, timeout=timeout, **kwargs)
        return {
            "ok": r.get("status") == "succeeded",
            "status": r.get("status"),
            "error": r.get("error"),
            "raw": r,
        }
    except Exception as e:
        return {"ok": False, "status": "exception", "error": str(e)[:120], "raw": None}


def call_car(c: RuntimeApiClient, name: str, timeout: float, **kwargs) -> dict:
    """单次 car.* 调用(给 car 上的 get_arm_state 用)。"""
    try:
        r = c.execute_car_action(name, timeout=timeout, **kwargs)
        return {
            "ok": r.get("status") == "succeeded",
            "status": r.get("status"),
            "error": r.get("error"),
            "raw": r,
        }
    except Exception as e:
        return {"ok": False, "status": "exception", "error": str(e)[:120], "raw": None}


def read_hand_angle(c: RuntimeApiClient) -> tuple:
    """读 car.get_arm_state(不是 arm action!),返回 (success, hand_angle, err)。"""
    r = call_car(c, "get_arm_state", timeout=READ_TIMEOUT_S)
    if not r["ok"]:
        return False, None, r["error"] or r["status"]
    data = r["raw"].get("result") if isinstance(r["raw"], dict) else None
    if not isinstance(data, dict):
        return False, None, "no result dict"
    return True, data.get("hand_angle"), None


def show_health(c: RuntimeApiClient, label: str) -> None:
    try:
        s = c.get_health().get("state", {})
        print(f"  [{label}] initialized={s.get('initialized')}  "
              f"initializing={s.get('initializing')}  "
              f"last_error={s.get('last_error')}")
    except Exception as e:
        print(f"  [{label}] health FAIL: {str(e)[:80]}")


def main() -> int:
    c = RuntimeApiClient()

    # ---- runtime 就绪检查(localhost/Connection/未初始化 三类分别处理)----
    if not preflight(c):
        return 1
    print()

    fails_api = 0
    fails_angle = 0
    total = 0

    # ---- 循环 ----
    print(f"=== 末端舵机顺序 {len(SEQUENCE)} 站 ===")
    print(f"    容差: ±{ANGLE_TOL}°")
    print()

    for i, (hand_name, expect_angle) in enumerate(SEQUENCE, 1):
        t0 = time.time()
        print(f"--- 站 #{i}: {hand_name} -> 期望 {expect_angle:+d}° ---")

        # 1) 写
        r = call_arm(c, "set_hand_angle", timeout=HAND_TIMEOUT_S,
                     angle=expect_angle, speed=80)
        total += 1
        flag = "OK  " if r["ok"] else "FAIL"
        print(f"  [{flag}] set_hand_angle(angle={expect_angle:+d}, speed=80)  "
              f"status={r['status']}  err={r['error']}")
        if not r["ok"]:
            fails_api += 1

        # 2) 读(给舵机 200ms 时间到位)
        time.sleep(0.2)
        ok, hand_angle, err = read_hand_angle(c)
        if not ok:
            print(f"  [FAIL] read get_arm_state  err={err}")
            fails_api += 1
        else:
            actual = hand_angle
            match = isinstance(actual, int) and abs(actual - expect_angle) <= ANGLE_TOL
            angle_flag = "OK  " if match else "FAIL"
            print(f"  [{angle_flag}] read hand_angle={actual!r}  (expected {expect_angle:+d} "
                  f"+/-{ANGLE_TOL}°)")
            if not match:
                fails_angle += 1

        dt = time.time() - t0
        print(f"  cycle_dt = {dt:.2f}s")
        print()

    # ---- 跑后 health ----
    print("=== 跑后 health ===")
    postflight(c, "after")
    print()

    # ---- 总结 ----
    print("=== 总结 ===")
    print(f"  API  写: {total - fails_api}/{total} succeeded")
    print(f"  角度判定: {len(SEQUENCE) - fails_angle}/{len(SEQUENCE)} in tol")
    print()

    if fails_api == 0 and fails_angle == 0:
        print("PASS")
        return 0
    else:
        print(f"FAIL  (api={fails_api}, angle={fails_angle})")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
