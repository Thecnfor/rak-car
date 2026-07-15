#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""test_y_negative.py
机械臂 y 轴向负方向移动(向下, toward y=0 触底)最简单测试。

骨架:
  health check -> arm.reset_y (向下触底,y_pose_start 被设)
              -> 读 state.y_mm (~0)
              -> arm.move_y(target=80.0) 抬到中位(向上)
              -> 读 state.y_mm (~80)
              -> arm.move_y(target=0.0)  向负方向移动(向下)
              -> 读 state.y_mm (~0)
              -> 跑后 health
              -> 总结 (API + 物理判定)

判定:
  - API 层:每步 succeeded,无异常 = PASS
  - 物理层:向下后 y_mm ≈ 0(限位 sensor 触底)
  - 这只是验证"能向负方向(下)移动",不验证"B3 的 x 是否还在工作"

约束:
  - 直接走 RuntimeApiClient.execute_arm_action(**kwargs) + execute_car_action(),
    避开 api.py:234 grab bug + api.py:291 hand 字段读取错误这两处旧坑。
  - arm.move_y_position / arm.reset_y 都是 arm action;
    y 移动到位后用 car.get_arm_state 读业务坐标 y_mm。
  - 默认 SERVER_ORIGIN 已在 main/settings.py:6。
  - ⚠️ 真硬件:arm 真会下到触底 —— 触底瞬间 y_limit_sensor(port=6)
    会被读到 True (arm_base.py 用此 break 出 reset_y 循环)。

运行:
  PowerShell:
    python main\\arm\\test\\test_y_negative.py
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


# 测试点序列:y 中位先 (向上) → y=0 (向下,负方向)
Y_HOVER_MM = 80.0
Y_BOTTOM_MM = 0.0
Y_TOL_MM = 2.0   # 车端 PID 闭环典型 <1mm;放宽到 2mm 给老舵机
RESET_TIMEOUT_S = 30.0
MOVE_TIMEOUT_S = 20.0
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
    """单次 car.* 调用(给 car.get_arm_state 用)。"""
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


def read_y_mm(c: RuntimeApiClient) -> tuple:
    """读 car.get_arm_state(),返回 (success, y_mm, err)。"""
    r = call_car(c, "get_arm_state", timeout=READ_TIMEOUT_S)
    if not r["ok"]:
        return False, None, r["error"] or r["status"]
    data = r["raw"].get("result") if isinstance(r["raw"], dict) else None
    if not isinstance(data, dict):
        return False, None, "no result dict"
    raw_y_m = data.get("y")
    try:
        y_mm = float(raw_y_m) * 1000.0 if raw_y_m is not None else None
    except (TypeError, ValueError):
        y_mm = None
    return True, y_mm, None


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

    # ---- runtime 就绪检查 (localhost / Connection / 未初始化 三类分别处理) ----
    if not preflight(c):
        return 1
    print()

    fails_api = 0
    fails_phys = 0
    total_api = 0
    ok, y_after_reset, err = read_y_mm(c)
    print(f"--- 初始 y --- ")
    print(f"  raw_y_m = {y_after_reset!r} mm" if not ok else f"  read FAIL: {err}")
    print()

    # ---- 1) reset_y:确保 y 复位到底 (y_pose_start 被设为当前 raw_y_m) ----
    print(f"=== 1) arm.reset_y ===")
    r = call_arm(c, "reset_y", timeout=RESET_TIMEOUT_S)
    total_api += 1
    flag = "OK  " if r["ok"] else "FAIL"
    print(f"  [{flag}] status={r['status']}  err={r['error']}")
    if not r["ok"]:
        fails_api += 1
    time.sleep(0.2)

    ok, y_after_reset, err = read_y_mm(c)
    if ok and y_after_reset is not None:
        flag = "OK  " if abs(y_after_reset - 0.0) < Y_TOL_MM else "WARN"
        print(f"  raw_y_m = {y_after_reset:.2f} mm  (expect ~0)")
        if abs(y_after_reset - 0.0) >= Y_TOL_MM:
            fails_phys += 1
    else:
        print(f"  [FAIL] read y_mm after reset: {err}")
        fails_api += 1
    print()

    # ---- 2) move_y(80):抬到中位 (向上) ----
    print(f"=== 2) arm.move_y_position(target={Y_HOVER_MM/1000:.3f}m = {Y_HOVER_MM:.1f}mm) [向上] ===")
    r = call_arm(c, "move_y_position", timeout=MOVE_TIMEOUT_S, target=Y_HOVER_MM / 1000.0)
    total_api += 1
    flag = "OK  " if r["ok"] else "FAIL"
    print(f"  [{flag}] status={r['status']}  err={r['error']}")
    if not r["ok"]:
        fails_api += 1
    time.sleep(0.2)

    ok, y_after_up, err = read_y_mm(c)
    if ok and y_after_up is not None:
        flag = "OK  " if abs(y_after_up - Y_HOVER_MM) < Y_TOL_MM else "FAIL"
        print(f"  raw_y_m = {y_after_up:.2f} mm  (expect ~{Y_HOVER_MM:.1f}, tol +/-{Y_TOL_MM:.0f})")
        if not (abs(y_after_up - Y_HOVER_MM) < Y_TOL_MM):
            fails_phys += 1
    else:
        print(f"  [FAIL] read y_mm after move_y(up): {err}")
        fails_api += 1
    print()

    # ---- 3) move_y(0):★ 向负方向(下)移动 ★ ----
    print(f"=== 3) arm.move_y_position(target=0.0m = 触底)  [向下,负方向] ===")
    r = call_arm(c, "move_y_position", timeout=MOVE_TIMEOUT_S, target=0.0)
    total_api += 1
    flag = "OK  " if r["ok"] else "FAIL"
    print(f"  [{flag}] status={r['status']}  err={r['error']}")
    if not r["ok"]:
        fails_api += 1
    time.sleep(0.2)

    ok, y_after_down, err = read_y_mm(c)
    if ok and y_after_down is not None:
        flag = "OK  " if abs(y_after_down - 0.0) < Y_TOL_MM else "FAIL"
        print(f"  raw_y_m = {y_after_down:.2f} mm  (expect ~0, tol +/-{Y_TOL_MM:.0f})")
        if not (abs(y_after_down - 0.0) < Y_TOL_MM):
            fails_phys += 1
    else:
        print(f"  [FAIL] read y_mm after move_y(down): {err}")
        fails_api += 1
    print()

    # ---- 跑后 health ----
    print("=== 跑后 health ===")
    postflight(c, "after")
    print()

    # ---- 总结 ----
    print("=== 总结 ===")
    print(f"  API   写: {total_api - fails_api}/{total_api} succeeded")
    print(f"  物理判定: y 能向下回到底 = 测试目的")
    if fails_phys == 0:
        print(f"          测出来: 向上 ~{y_after_up:.1f}mm → 向下 ~{y_after_down:.1f}mm (负方向 OK)")
    print()

    if fails_api == 0 and fails_phys == 0:
        print("PASS")
        return 0
    else:
        print(f"FAIL  (api={fails_api}, phys={fails_phys})")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
