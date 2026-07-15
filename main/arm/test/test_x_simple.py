#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""test_x_simple.py
机械臂 x 轴最简单轴测试。

骨架:
  health check -> reset_x -> move_x(safe_min)
              -> move_x(target_mm)
              -> 读 state   (业务坐标)
              -> 读 port=6 encoder (硬件层)
              -> PASS / FAIL

注意:
  - API 返回 succeeded 不代表电机真动 —— _call_arm 仅验证方法返回,
    车端 move_x_position 的 PID 闭环卡死时也会返回 None/succeeded。
    本测试同时打印 raw_x_m 和 port=6 encoder,两个都用上才能定真伪。
  - 软限位 soft_x_min=5mm, soft_x_max=300mm,目标必须在区间内,
    不能 move_x(0) —— 等价于 move_x(origin.soft_x_min_mm)。
  - 不依赖 ArmClient.ping() 那个有 bug 的方法;用 RuntimeApiClient 直跑。

运行:
  PowerShell:
    $env:RAK_CAR_SERVER_ORIGIN = "http://192.168.3.60"
    python main\arm\test\test_x_simple.py
  bash:
    RAK_CAR_SERVER_ORIGIN=http://192.168.3.60 python3 main/arm/test/test_x_simple.py
"""
import os
import sys

# 让 main.* 可被 import
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.api_client import RuntimeApiClient  # noqa: E402
from main.arm.test._runtime_guard import preflight, postflight  # noqa: E402


# 软区间 [5, 300] mm —— target 在中间,远离墙
TARGET_X_MM = 120.0
TOL_MM = 5.0   # 车端 PID 闭环典型 <2mm;放宽到 5mm 给老舵机余量
X_MOTOR_PORT = 6  # arm_cfg.yaml: horiz_cfg.motor.id


def http_json(method: str, base: str, path: str, payload=None, timeout: float = 5.0):
    """简单 HTTP 封装,直接打 REST 端点(绕开 execute 的层层重试)。"""
    import requests  # 局部 import,免得测试文件顶头就 requests NotFound
    url = f"{base}{path}"
    if method == "GET":
        r = requests.get(url, timeout=timeout)
    elif method == "POST":
        r = requests.post(url, json=payload, timeout=timeout)
    else:
        raise ValueError(method)
    r.raise_for_status()
    return r.json() if r.content else {}


def call_arm(c: RuntimeApiClient, name: str, timeout: float = 20.0, **kwargs) -> dict:
    """单次 arm.* 调用,带超时 + 异常捕获。kwargs 走透传,避开 api.py:grab 的 timeout 冲突。"""
    try:
        r = c.execute_arm_action(name, timeout=timeout, **kwargs)
        return {"ok": r.get("status") == "succeeded",
                "status": r.get("status"),
                "error": r.get("error"),
                "result": r.get("result")}
    except Exception as e:
        return {"ok": False, "status": "exception", "error": str(e)[:120], "result": None}


def show_health(c: RuntimeApiClient, label: str) -> None:  # legacy, kept for compat
    """旧版,新版用 main.arm.test._runtime_guard.postflight。"""
    try:
        s = c.get_health().get("state", {})
        print(f"  [{label}] initialized={s.get('initialized')}  "
              f"initializing={s.get('initializing')}  "
              f"last_error={s.get('last_error')}")
    except Exception as e:
        print(f"  [{label}] health FAIL: {str(e)[:80]}")


def main() -> int:
    c = RuntimeApiClient()
    base = c.api_base  # 例如 http://192.168.3.231:5050

    # ---- runtime 就绪检查(localhost/Connection/未初始化 三类分别处理)----
    if not preflight(c):
        return 1
    print()

    fails = 0

    # ---- 1) reset_x ----
    print(f"=== 1) arm.reset_x ===")
    r = call_arm(c, "reset_x", timeout=30.0)
    flag = "OK  " if r["ok"] else "FAIL"
    print(f"  [{flag}] {r['status']}  err={r['error']}")
    if not r["ok"]:
        fails += 1
    print()

    # 重新拉 arm_state
    pre = call_arm(c, "get_arm_state", timeout=10.0)
    pre_data = pre.get("result") if pre["ok"] else {}
    pre_x_m = float(pre_data.get("x", 0.0)) if isinstance(pre_data, dict) else 0.0
    pre_y_m = float(pre_data.get("y", 0.0)) if isinstance(pre_data, dict) else 0.0
    print(f"  before: raw_x_m={pre_x_m:.4f}  raw_y_m={pre_y_m:.4f}")
    print()

    # ---- 2) move_x(TARGET) ----
    print(f"=== 2) arm.move_x_position(target={TARGET_X_MM/1000:.3f}m = {TARGET_X_MM:.1f}mm) ===")
    r = call_arm(c, "move_x_position", timeout=20.0, target=TARGET_X_MM / 1000.0)
    flag = "OK  " if r["ok"] else "FAIL"
    print(f"  [{flag}] {r['status']}  err={r['error']}")
    if not r["ok"]:
        fails += 1
    print()

    # ---- 3) 读 arm_state(业务坐标,车端给)----
    print(f"=== 3) 读业务坐标 / 硬件编码器 ===")
    post = call_arm(c, "get_arm_state", timeout=10.0)
    post_data = post.get("result") if post["ok"] else {}
    post_x_m = float(post_data.get("x", 0.0)) if isinstance(post_data, dict) else 0.0
    post_y_m = float(post_data.get("y", 0.0)) if isinstance(post_data, dict) else 0.0
    bus_x_mm = post_x_m * 1000.0
    bus_x_err = bus_x_mm - TARGET_X_MM
    bus_y_mm = post_y_m * 1000.0
    bus_y_drift = bus_y_mm - (pre_y_m * 1000.0)

    print(f"  business: x={bus_x_mm:.1f}mm  y={bus_y_mm:.1f}mm")
    print(f"  err_x    ={bus_x_err:+.1f}mm  (tol +/-{TOL_MM:.0f}mm)")
    print(f"  y_drift  ={bus_y_drift:+.1f}mm")

    # 硬件层:直接读 port=6 编码器
    enc = http_json("GET", base, f"/v1/realtime/encoder?port={X_MOTOR_PORT}&reverse=1", timeout=5.0)
    enc_val = enc.get("encoder", "?")
    print(f"  hardware: port={X_MOTOR_PORT} encoder={enc_val}")

    # 判定:业务坐标误差 < tol 就算走通
    api_ok = abs(bus_x_err) < TOL_MM
    flag = "OK  " if api_ok else "FAIL"
    print(f"  [{flag}] 业务坐标判定")
    if not api_ok:
        fails += 1
    print()

    # ---- 跑后 health ----
    print("=== 跑后 health ===")
    postflight(c, "after")
    print()

    total = 3
    label = "PASS" if fails == 0 else "FAIL"
    print(f"{label}: {total - fails}/{total} ok")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
