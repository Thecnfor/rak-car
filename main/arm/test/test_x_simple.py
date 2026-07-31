#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""test_x_simple.py
机械臂 x 轴 最简测试（业务层 API）。

骨架（按 ARM_API.md）:
  preflight -> 读 x 基准(realtime) -> 沿 x 轴跑几个相对目标点
           -> 每点用 realtime 读真值核对 -> postflight -> 总结

判定（ARM_API.md §11 §7.2.1）:
  - ⚠️ x 读数只信 realtime(_read_x_mm_realtime): x_get_position 走 calibrate 框架读数飘,禁用
  - ⚠️ API 返回 succeeded 不代表真到位: 同步带打滑(belt slip)单次有效行程仅 24-46mm
  - 每点 |x_actual - x_target| < tol 算 PASS; 明显欠行程时额外打 [SLIP] 提示
  - 只验证 x 轴,不动 y / 大臂 / 手爪

约束:
  - 走 main.arm.ArmClient 业务封装,不绕开调 raw action
  - 单位 mm; move_x(x_mm, v_max_mms=30) 业务层限速 30mm/s(SDK 端临时收紧 PID 限幅)
  - x 轴无软限位(2026-07-16 取消),物理墙 ≈ ±119.5mm; 目标用相对小位移避免撞墙
  - _runtime_guard.preflight/postflight 统一 3 类异常处理

运行:
  PowerShell:
    $env:RAK_CAR_SERVER_ORIGIN = "http://192.168.6.231"
    python main\\arm\\test\\test_x_simple.py
"""
import os
import sys

# 让 main.* 可被 import
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient  # noqa: E402
from main.arm.test._runtime_guard import preflight, postflight  # noqa: E402


# 相对基准的目标偏移 mm: 先向正(远端)走再回到起点
# 单次控制在 40mm 内(考虑 belt slip 24-46mm 有效行程 + 不撞 ±119.5mm 物理墙)
OFFSETS_MM = [-0.0, -90,-120, 0.0]
TOL_MM = 5.0        # realtime 抖动 <1mm; 放宽到 5mm 给 PID 闭环余量
SLIP_RATIO = 0.6    # 有效行程 < 期望 60% 视为疑似打滑
V_MAX_MMS = 30.0    # 业务限速（2026-07-22 限速透传 bug 修复后定档 30）


def _read_x(client) -> float:
    """走 realtime 真值路径(ARM_API.md §11); 读不到直接判失败。"""
    x = client._read_x_mm_realtime()
    if x is None:
        raise RuntimeError("realtime x_mm 读不到 (arm_feed 未启 / realtime 不可用)")
    return x


def main() -> int:
    client = ArmClient.connect()

    # ---- runtime 就绪检查 ----
    if not preflight(client):
        return 1
    print()

    # ---- 1) 读 x 基准(realtime) ----
    print("=== 1) 读 x 基准 (realtime /v1/realtime/arm/state) ===")
    try:
        x0 = _read_x(client)
    except Exception as e:
        print(f"  [FAIL] {type(e).__name__}: {str(e)[:120]}")
        return 1
    print(f"  [OK  ] 基准 x0 = {x0:+.1f} mm")
    print()

    fails = 0
    targets = [x0 + off for off in OFFSETS_MM]

    # ---- 2) 沿 x 轴跑相对目标点 ----
    print(f"=== 2) client.move_x() 跑 {len(targets)} 个目标点 (v_max={V_MAX_MMS:.0f}mm/s) ===")
    x_prev = x0
    for tx, off in zip(targets, OFFSETS_MM):
        want = tx - x_prev   # 本段期望位移
        try:
            job = client.move_x(x_mm=tx, v_max_mms=V_MAX_MMS)
        except Exception as e:
            print(f"  [FAIL] cmd x={tx:+7.1f}mm  exception: {type(e).__name__}: {str(e)[:80]}")
            fails += 1
            continue

        try:
            x_now = _read_x(client)
        except Exception as e:
            print(f"  [FAIL] cmd x={tx:+7.1f}mm  读 realtime 失败: {str(e)[:80]}")
            fails += 1
            continue

        err = x_now - tx
        got = x_now - x_prev   # 本段实际位移
        ok = (job.get("status") == "succeeded") and (abs(err) < TOL_MM)
        flag = "OK  " if ok else "FAIL"
        print(f"  [{flag}] cmd x={tx:+7.1f}mm  actual x={x_now:+7.1f}mm  "
              f"err={err:+5.1f}mm  (tol +/-{TOL_MM:.0f}mm)")

        # 打滑检测: 期望走一段但实际欠行程明显
        if abs(want) > 1.0 and abs(got) < abs(want) * SLIP_RATIO:
            print(f"         [SLIP] 期望位移 {want:+.1f}mm 实际仅 {got:+.1f}mm —— 疑似同步带打滑 (§7.2.1)")

        if not ok:
            fails += 1
        x_prev = x_now

    print()

    # ---- 跑后 health ----
    postflight(client, "after")
    print()

    # ---- 总结 ----
    total = len(targets)
    passed = total - fails
    label = "PASS" if fails == 0 else "FAIL"
    print("=== 总结 ===")
    print(f"  基准 x0:  {x0:+.1f} mm")
    print(f"  目标序列: {' -> '.join(f'{t:+.0f}' for t in targets)} mm")
    print(f"  容差:     +/-{TOL_MM:.0f}mm  (读数走 realtime)")
    print(f"  结果:     {passed}/{total} ok")
    print()
    print(label)
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
