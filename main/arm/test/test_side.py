#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""test_side.py
机械臂 大臂(bus 舵机)测试 —— **用真实编码器反馈**判定。

骨架（按 ARM_API.md 业务层 API 下发 + runtime realtime 端点回读）:
  preflight -> 循环 set_side(MID→LEFT→MID→RIGHT→MID)
           -> 每站用 realtime_bus_servo_read(1) 读 raw angle,核对期望角度(±2°)
           -> postflight -> 总结

期望角度 (与 arm_cfg.yaml:hand_cfg.arm.angle_list 对齐):
  LEFT  =  +93°
  MID   =    0°
  RIGHT =  -93°

判定:
  - 物理角度: |realtime_bus_servo_read(1) - expect| <= ANGLE_TOL
  - 业务 API: set_side job.status == succeeded

⚠️ 关于 car.get_arm_state().arm_angle:
  该字段是命令回显 (smartcar/whalesbot/vehicle/arm/arm_base.py:472
  返回 _arm_angle_last),不是真实编码器读数 —— 物理判定必须走
  /v1/realtime/bus-servo/angle?port=BUS_SERVO_PORT。

约束:
  - 走 main.arm.ArmClient + ArmRunner 业务封装下发命令 (ARM_API.md:30)
  - 物理读走 main.api_client.RuntimeApiClient.realtime_bus_servo_read
  - 大臂会真的转动 —— 先确认机械臂周围无障碍物
  - main.arm.test._runtime_guard.preflight/postflight 统一 3 类异常处理
  - 默认 SERVER_ORIGIN 已是 http://192.168.3.60(main/settings.py:6)

运行:
  PowerShell:
    python main\\arm\\test\\test_side.py
"""
import os
import sys

# 让 main.* 可被 import
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.test._runtime_guard import preflight, postflight  # noqa: E402


# 期望角度表(与 arm_cfg.yaml:hand_cfg.arm.angle_list 对齐)
EXPECTED_ANGLE = {
    "LEFT": 93,
    "MID": 0,
    "RIGHT": -93,
}

# 测试序列:MID 起手 → 走完全部 3 方向 → 回到 MID
SEQUENCE = ["LEFT", "MID", "RIGHT", "MID"]

# bus 舵机 ID(arm_base.py:318 ServoBus(hand["port"]),quick_start.py:64 验证 port=1)
BUS_SERVO_PORT = 1

ANGLE_TOL = 2     # bus 舵机闭环精度 ±1°,放宽到 2°
SIDE_TIMEOUT_S = 12.0


def read_raw_angle(client) -> int | None:
    """走 realtime 端点读真实编码器角度(避免 arm.angle 的命令回显假象)。

    Returns:
        int: 实测角度(°);失败时返回 None。
    """
    try:
        r = client.http.realtime_bus_servo_read(BUS_SERVO_PORT)
        if isinstance(r, dict) and r.get("ok") and "angle" in r:
            return int(r["angle"])
        return None
    except Exception:
        return None


def main() -> int:
    client = ArmClient.connect()

    # ---- runtime 就绪检查(localhost / Connection / 未初始化 三类分别处理)----
    if not preflight(client):
        return 1
    print()

    runner = ArmRunner(client)

    # ---- 初始 raw 读数(看舵机当前位置)----
    raw0 = read_raw_angle(client)
    print("=== 初始 raw 角度 ===")
    print(f"  bus_servo(port={BUS_SERVO_PORT}) angle = {raw0}°")
    print()

    fails = 0

    # ---- 大臂 bus 舵机循环 ----
    print(f"=== 大臂 bus 舵机:跑 {len(SEQUENCE)} 站 (期望角度 ±{ANGLE_TOL}°) ===")
    print(f"    物理判定: realtime_bus_servo_read(port={BUS_SERVO_PORT}) [真编码器]")
    print()

    for side in SEQUENCE:
        # 1) 下发命令(业务层 API)
        try:
            job = runner.set_side(side, timeout=SIDE_TIMEOUT_S)
            api_ok = (job.get("status") == "succeeded")
        except Exception as e:
            print(f"  [FAIL] cmd={side:<6}  exception: {type(e).__name__}: {str(e)[:80]}")
            fails += 1
            continue

        # 2) 给舵机一点时间到位(set_angle 协议本身不等完成)
        import time as _t
        _t.sleep(0.4)

        # 3) 读真角度(不是 car.get_arm_state().arm_angle!)
        raw = read_raw_angle(client)
        expect = EXPECTED_ANGLE[side]

        if raw is None:
            print(f"  [FAIL] cmd={side:<6}  expect={expect:+4d}°  raw=None (读不出)")
            fails += 1
            continue

        angle_ok = abs(raw - expect) <= ANGLE_TOL
        ok = api_ok and angle_ok
        flag = "OK  " if ok else "FAIL"
        print(
            f"  [{flag}] cmd={side:<6}  expect={expect:+4d}°  "
            f"raw={raw:+4d}°  err={raw - expect:+3d}°  "
            f"api_ok={api_ok}  raw_match={angle_ok}"
        )
        if not ok:
            fails += 1

    print()

    # ---- 跑后 health ----
    postflight(client, "after")
    print()

    # ---- 总结 ----
    total = len(SEQUENCE)
    label = "PASS" if fails == 0 else "FAIL"
    print("=== 总结 ===")
    print(f"  序列: {' -> '.join(['MID'] + SEQUENCE)}")
    print(f"  期望: LEFT=+93°, MID=0°, RIGHT=-93°")
    print(f"  容差: ±{ANGLE_TOL}°")
    print(f"  数据源: realtime_bus_servo_read(port={BUS_SERVO_PORT}) — 真实编码器")
    print(f"  结果: {total - fails}/{total} ok")
    print()
    print(label)
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())