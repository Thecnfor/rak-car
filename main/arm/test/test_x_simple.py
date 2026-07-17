#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""test_x_simple.py
x 轴端到端测试 —— 慢速版。5 个独立 case 覆盖 ARM_API.md §7.2/§9 的关键不变量。

为什么慢速:
  move_x_position(target, out_time=6.0) 没有 velocity 参数,实际速度全由车端 PID 决定
  (output_limits ±0.4 m/s = ±400 mm/s),50mm 跑完只要 ~0.13s,根本看不清。
  本版改走 x_speed() 开环慢速 + 编码器到位即停,默认 25 mm/s —— 比 reset_x 撞墙的
  50 mm/s 还慢一倍,方便观察 + 留余量给编码器观测。
  想跑快:命令行 --velocity 80 或 150。

骨架 (向左走 —— 负方向,远离右墙):
  preflight (runtime + initialized)
  CASE 1: arm.reset_x                       —— 撞墙 + 模型判据 + 物理清零 (ref_encoder → 0)
  CASE 2: x_speed 慢速 → -50mm              —— reset_x 之后能不能动 (v3 死循环修复回归)
  CASE 3: x_speed 慢速 → -150mm             —— 中位定位精度 (左中位)
  CASE 4: x_speed 慢速 → +100mm (回 -50)    —— 反向运动 / 双向都通
  CASE 5: x_speed 慢速 → -250mm (到 -300)   —— 左侧软限位边界,验顶段减速带无过冲
  全程 y 漂移统计 + postflight + 总分

⚠️ 坐标约定 (实测你这边的硬件方向):
  reset_x 撞右墙 → x=0 在右墙位置
  正方向 (+x) = 向物理右走 (会再撞回右墙,无意义)
  负方向 (-x) = 向物理左走 (远离右墙,有效行程 -320 ~ 0 mm)
  软限位 threshold=[-0.32, 0.32],CASE 5 目标 -300mm 距左墙 20mm

判定:
  - API 层:每步 succeeded,无异常 = PASS
  - 物理层:post x_mm 与 target 误差 < TOL_X_MM
  - ref_encoder 清零:CASE 1 后 x_mm 必须 ≈ 0
  - y 不漂:起点 vs 终点 |Δy| < Y_DRIFT_TOL_MM
  - 速度合规:实际耗时应在 [理论 * 0.7, 理论 * 1.6] 区间
    (开环 25 mm/s 跑 50mm ≈ 2.0s,太快 = 没在跑/编码器没追上,太慢 = 卡阻)

运行:
  PowerShell:
    $env:RAK_CAR_SERVER_ORIGIN = "http://192.168.6.231"
    python main\arm\test\test_x_simple.py
    python main\arm\test\test_x_simple.py --velocity 50    # 跑快一点
    python main\arm\test\test_x_simple.py --velocity 10    # 更慢 (调试用)
"""
import argparse
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.api_client import RuntimeApiClient  # noqa: E402
from main.arm.test._runtime_guard import preflight, postflight  # noqa: E402


# === 测试参数 (左方向:全部负值) ===
TARGET_NEAR_MM = -50.0       # CASE 2 绝对位置 (左 50mm)
TARGET_MID_MM = -150.0       # CASE 3 中位 (左 150mm)
TARGET_NEAR_RIGHT_MM = -50.0 # CASE 4 绝对位置 (向左走完后再回到这里)
TARGET_BOUNDARY_MM = -300.0  # CASE 5 左侧软限位边界 (距左墙 -320 留 20mm)

TOL_X_MM = 5.0
ENCODER_ARRIVE_MM = 2.0
Y_DRIFT_TOL_MM = 5.0

RESET_TIMEOUT_S = 30.0
XSPEED_FIRE_TIMEOUT_S = 5.0
XSPEED_POLL_HZ = 20.0
READ_TIMEOUT_S = 8.0
MOVE_DEADLINE_S = 15.0

X_MOTOR_PORT = 6

DEFAULT_VELOCITY_MM_S = 25.0
MIN_SAFE_VELOCITY_MM_S = 10.0


def call_arm(c, name, timeout, **kwargs):
    # sync=True: 阻塞到 succeeded/failed。execute_arm_action 默认 sync=False,
    # 不传 sync 只会立即拿到 status=queued (job 已入队但还没跑),不是动作结果。
    try:
        r = c.execute_arm_action(name, timeout=timeout, sync=True, **kwargs)
        return {"ok": r.get("status") == "succeeded", "status": r.get("status"),
                "error": r.get("error"), "raw": r}
    except Exception as e:
        return {"ok": False, "status": "exception", "error": str(e)[:120], "raw": None}


def call_car(c, name, timeout, **kwargs):
    # sync=True 同 call_arm
    try:
        r = c.execute_car_action(name, timeout=timeout, sync=True, **kwargs)
        return {"ok": r.get("status") == "succeeded", "status": r.get("status"),
                "error": r.get("error"), "raw": r}
    except Exception as e:
        return {"ok": False, "status": "exception", "error": str(e)[:120], "raw": None}


def read_arm_xy(c):
    r = call_car(c, "get_arm_state", timeout=READ_TIMEOUT_S)
    if not r["ok"]:
        return False, None, None, r["error"] or r["status"]
    data = r["raw"].get("result") if isinstance(r["raw"], dict) else None
    if not isinstance(data, dict):
        return False, None, None, "no result dict"
    raw_x = data.get("x"); raw_y = data.get("y")
    try:
        x_mm = float(raw_x) * 1000.0 if raw_x is not None else None
        y_mm = float(raw_y) * 1000.0 if raw_y is not None else None
    except (TypeError, ValueError):
        return False, None, None, "non-numeric x/y"
    return True, x_mm, y_mm, None


def read_x_encoder(c):
    try:
        import requests
        url = f"{c.api_base}/v1/realtime/encoder?port={X_MOTOR_PORT}&reverse=1"
        r = requests.get(url, timeout=5.0)
        r.raise_for_status()
        j = r.json() if r.content else {}
        return True, j.get("encoder"), None
    except Exception as e:
        return False, None, f"{type(e).__name__}: {str(e)[:80]}"


def show_xy_and_enc(c, label):
    ok, x_mm, y_mm, err = read_arm_xy(c)
    ok2, enc, err2 = read_x_encoder(c)
    if ok:
        print(f"  [{label}] x={x_mm:+.1f}mm  y={y_mm:+.1f}mm" if y_mm is not None else f"  [{label}] x={x_mm:+.1f}mm  y=?")
    else:
        print(f"  [{label}] read FAIL: {err}")
    if ok2:
        print(f"           port={X_MOTOR_PORT} encoder={enc}")
    else:
        print(f"           port={X_MOTOR_PORT} encoder FAIL: {err2}")
    return x_mm, y_mm, enc


def move_x_slow(c, target_mm, velocity_mm_s, timeout_s=MOVE_DEADLINE_S):
    """开环 x_speed 慢速移到 target_mm,编码器到位即停。

    与 move_x_position(PID) 的区别:
      - 显式速度,适合测试观察
      - 无 PID 修位置,完全靠编码器
      - 不触发 §9.7 的 calibrate 副作用

    返回 dict: ok, took_s, x_start, x_end, x_speed_ok
    """
    ok, x_start, y_start, err = read_arm_xy(c)
    if not ok or x_start is None:
        return {"ok": False, "error": f"起点读失败:{err}"}

    delta_mm = target_mm - x_start
    if abs(delta_mm) < ENCODER_ARRIVE_MM:
        return {"ok": True, "took_s": 0.0, "x_start": x_start, "x_end": x_start,
                "x_speed_ok": True, "note": "已在目标附近"}

    direction = 1 if delta_mm > 0 else -1
    velocity_m_s = direction * (velocity_mm_s / 1000.0)
    theoretical_s = abs(delta_mm) / velocity_mm_s
    deadline = time.monotonic() + timeout_s

    # 开火 x_speed
    r = call_arm(c, "x_speed", timeout=XSPEED_FIRE_TIMEOUT_S, velocity=velocity_m_s)
    if not r["ok"]:
        return {"ok": False, "error": f"x_speed({velocity_m_s}) FAIL: {r['error']}",
                "x_start": x_start, "x_speed_ok": False}

    # 编码器到位轮询
    poll_interval = 1.0 / XSPEED_POLL_HZ
    t0 = time.monotonic()
    x_now = x_start
    while time.monotonic() < deadline:
        time.sleep(poll_interval)
        ok, x_now, _, _ = read_arm_xy(c)
        if ok and x_now is not None and abs(x_now - target_mm) < ENCODER_ARRIVE_MM:
            break
    took_s = time.monotonic() - t0

    # 必停 —— 绝不留速度残留
    call_arm(c, "x_speed", timeout=XSPEED_FIRE_TIMEOUT_S, velocity=0.0)

    return {"ok": True, "took_s": took_s, "x_start": x_start, "x_end": x_now,
            "x_speed_ok": True, "theoretical_s": theoretical_s}


def run_case_reset_x(c, fails_api, fails_phys):
    print("=== CASE 1 reset_x ===")
    t0 = time.monotonic()
    r = call_arm(c, "reset_x", timeout=RESET_TIMEOUT_S)
    took_s = time.monotonic() - t0
    flag = "OK  " if r["ok"] else "FAIL"
    print(f"  [{flag}] status={r['status']}  err={r['error']}  took={took_s:.2f}s")
    if not r["ok"]:
        fails_api.append("CASE 1")
    time.sleep(0.15)
    x_mm, y_mm, _ = show_xy_and_enc(c, "post")

    # 强判:reset 后 x 必须 ≈ 0 (ref_encoder 物理清零的标志)
    if x_mm is not None:
        ok0 = abs(x_mm) < TOL_X_MM
        f = "OK  " if ok0 else "FAIL"
        print(f"  [{f}] reset 后 x 应 ≈ 0mm 实测 {x_mm:+.1f}mm")
        if not ok0:
            fails_phys.append("CASE 1-ref_encoder")
    print()
    return {"post_x_mm": x_mm, "post_y_mm": y_mm}


def run_case_move(c, name, target_mm, velocity_mm_s, fails_api, fails_phys):
    print(f"=== {name}: 慢速 → {target_mm:+.0f}mm @ {velocity_mm_s:.0f}mm/s ===")
    res = move_x_slow(c, target_mm, velocity_mm_s)

    if not res.get("ok"):
        print(f"  [FAIL] {res.get('error', 'unknown')}")
        fails_api.append(name)
        print()
        return res

    if res.get("note"):
        print(f"  [SKIP] {res['note']}")
        print()
        return res

    took = res["took_s"]
    theoretical = res["theoretical_s"]
    speed_ratio = took / theoretical
    print(f"  x: {res['x_start']:+.1f}mm → {res['x_end']:+.1f}mm  took={took:.2f}s  theory={theoretical:.2f}s  ratio={speed_ratio:.2f}")
    show_xy_and_enc(c, "post")

    # 速度合规判据:ratio ∈ [0.7, 1.6]
    speed_ok = 0.7 <= speed_ratio <= 1.6
    f = "OK  " if speed_ok else "WARN"
    print(f"  [{f}] 速度合规 (ratio={speed_ratio:.2f} ∈ [0.7, 1.6])")
    if not speed_ok:
        fails_phys.append(f"{name}-speed")

    # 位置精度判据
    err = res["x_end"] - target_mm
    pos_ok = abs(err) < TOL_X_MM
    f = "OK  " if pos_ok else "FAIL"
    print(f"  [{f}] x_err={err:+.1f}mm  (expect {target_mm:+.1f} ±{TOL_X_MM:.0f}mm)")
    if not pos_ok:
        fails_phys.append(f"{name}-x_err")
    print()
    return res


def main():
    parser = argparse.ArgumentParser(description="x 轴慢速端到端测试")
    parser.add_argument("--velocity", type=float, default=DEFAULT_VELOCITY_MM_S,
                        help=f"开环速度 mm/s (默认 {DEFAULT_VELOCITY_MM_S},安全下限 {MIN_SAFE_VELOCITY_MM_S})")
    args = parser.parse_args()

    velocity = args.velocity
    if velocity < MIN_SAFE_VELOCITY_MM_S:
        print(f"[WARN] velocity={velocity}mm/s 低于安全下限 {MIN_SAFE_VELOCITY_MM_S}mm/s,电机可能过不了静摩擦")
        print(f"       自动夹到 {MIN_SAFE_VELOCITY_MM_S}mm/s")
        velocity = MIN_SAFE_VELOCITY_MM_S

    print(f"=== 配置:velocity={velocity}mm/s  (reset_x 撞墙速度 50 mm/s) ===\n")

    c = RuntimeApiClient()
    if not preflight(c):
        return 1
    print()

    fails_api, fails_phys = [], []

    # 起点
    print("=== 0) 起点 ===")
    ok, x_start, y_start, err = read_arm_xy(c)
    if ok and y_start is not None:
        print(f"  start: x={x_start:+.1f}mm  y={y_start:+.1f}mm")
    else:
        print(f"  start read FAIL: {err}")
    print()

    # CASE 1
    print("-" * 60)
    r1 = run_case_reset_x(c, fails_api, fails_phys)

    # CASE 2: reset_x 后向左 -50 (v3 死循环修复回归)
    print("-" * 60)
    r2 = run_case_move(c, "CASE 2 move_-50 (向左)", TARGET_NEAR_MM, velocity, fails_api, fails_phys)

    # CASE 3: 向左到 -150 (左中位)
    print("-" * 60)
    r3 = run_case_move(c, "CASE 3 move_-150 (向左)", TARGET_MID_MM, velocity, fails_api, fails_phys)

    # CASE 4: 向右回 +100 (-150 → -50),验证双向都通
    print("-" * 60)
    r4 = run_case_move(c, "CASE 4 move_+100 (向右回)", TARGET_NEAR_RIGHT_MM, velocity, fails_api, fails_phys)

    # CASE 5: 向左 -250 (-50 → -300,左侧软限位边界)
    print("-" * 60)
    r5 = run_case_move(c, "CASE 5 move_-250_to_left_boundary", TARGET_BOUNDARY_MM, velocity, fails_api, fails_phys)

    # 跑后 health
    print("=== 跑后 health ===")
    postflight(c, "after")
    print()

    # 终点
    print("=== 终点 ===")
    ok, x_end, y_end, err = read_arm_xy(c)
    if ok and y_end is not None:
        print(f"  end: x={x_end:+.1f}mm  y={y_end:+.1f}mm")
    else:
        print(f"  end read FAIL: {err}")

    # y 漂移
    print("\n=== y 漂移统计 (x 动时 y 不应动) ===")
    if y_start is not None and y_end is not None:
        y_drift = y_end - y_start
        y_drift_ok = abs(y_drift) < Y_DRIFT_TOL_MM
        f = "OK  " if y_drift_ok else "FAIL"
        print(f"  [{f}] |Δy| = {abs(y_drift):.1f}mm  (tol <{Y_DRIFT_TOL_MM:.0f}mm)")
        if not y_drift_ok:
            fails_phys.append("y_drift")
    else:
        print("  [SKIP] 起点 / 终点 y 读不到")
    print()

    # 总结
    print("=" * 60)
    print("=== 总结 ===")
    n_total = 5
    print(f"  velocity    : {velocity}mm/s (开环慢速)")
    print(f"  API  : {n_total - len(fails_api)}/{n_total} succeeded  fails={fails_api or '无'}")
    print(f"  物理 : {len(fails_phys)} 项失败            fails={fails_phys or '无'}")
    if not fails_api and not fails_phys:
        print("\nPASS")
        return 0
    print(f"\nFAIL  (api={len(fails_api)}, phys={len(fails_phys)})")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())