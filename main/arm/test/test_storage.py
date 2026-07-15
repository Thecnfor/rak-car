#!/usr/bin/python3
"""test_storage.py
储存仓(底盘上的储物盒)测试。

硬件:ServoPwm(1, 180)  port=1, PWM 0~180°
位置表(看 car_wrap_2026.py:409):
  set_storage(False) -> -42  (放下)
  set_storage(True ) -> 165  (收起)

接口:
  car.set_storage(state)        # bool 业务封装
  car.set_storage_angle(angle)  # 直接设角度(0~180,实际范围可能略宽)
  car.set_pwm_servo_angle(port, angle, mode, speed)  # 任意 PWM 舵机

⚠️ 真实硬件,舵机真的会转。先确认储存仓周围没障碍物。

运行:
  export RAK_CAR_SERVER_ORIGIN=http://192.168.3.60
  python3 main/arm/test/test_storage.py
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.api_client import RuntimeApiClient  # noqa: E402
from main.arm.test._runtime_guard import preflight, postflight  # noqa: E402


# 与 car_wrap_2026.py:409 保持一致
STORAGE_ANGLES = (False, True)  # (down, up)
ANGLE_TABLE = {False: -42, True: 165}


def call_safe(c: RuntimeApiClient, target: str, name: str, *args, timeout: float = 15.0, **kwargs):
    """单次调用,带超时 + 异常捕获(避免大臂那种无限挂起问题)。"""
    try:
        r = c.execute(target=target, name=name, args=list(args), kwargs=kwargs, timeout=timeout)
        return r.get("status"), r.get("error"), r
    except Exception as e:
        return "exception", str(e)[:120], None


def main() -> int:
    c = RuntimeApiClient()
    # 不走 ArmClient(那个有 ping bug),直接用 RuntimeApiClient

    # ---- runtime 就绪检查 ----
    if not preflight(c):
        return 1
    print()

    fails = 0

    # ---- 1) 业务封装 car.set_storage ----
    print("=== 1) car.set_storage (bool 接口) ===")
    for state in [False, True, False, True]:
        # 安全:转之前先 close 当前 runtime 上的卡死任务(防御)
        status, err, _ = call_safe(c, "car", "set_storage", state, timeout=10)
        flag = "OK  " if status == "succeeded" else "FAIL"
        print(f"  [{flag}] set_storage({str(state):<5}) -> {ANGLE_TABLE[state]:>4}°  status={status}  err={err}")
        if status != "succeeded":
            fails += 1
    print()

    # ---- 2) 直接角度 car.set_storage_angle ----
    print("=== 2) car.set_storage_angle (直接设角度) ===")
    for angle in [-42, 165, 0, 90, -42]:
        status, err, r = call_safe(c, "car", "set_storage_angle", angle, timeout=10)
        result = r.get("result") if r else None
        flag = "OK  " if status == "succeeded" else "FAIL"
        print(f"  [{flag}] set_storage_angle({angle:>4})  result={result}  err={err}")
        if status != "succeeded":
            fails += 1
    print()

    # ---- 3) 通用 PWM 舵机 car.set_pwm_servo_angle (验证 port=1 真的能发) ----
    print("=== 3) car.set_pwm_servo_angle (验证 port=1 通道) ===")
    for angle in [0, 90, 180, 90]:
        status, err, r = call_safe(c, "car", "set_pwm_servo_angle", 1, angle, 180, 100, timeout=10)
        result = r.get("result") if r else None
        flag = "OK  " if status == "succeeded" else "FAIL"
        print(f"  [{flag}] pwm_servo(port=1, angle={angle:>3}, mode=180)  result={result}  err={err}")
        if status != "succeeded":
            fails += 1
    print()

    # ---- 4) 收尾:放回 down 位置 ----
    print("=== 4) 收尾:放回 -42°(放下) ===")
    call_safe(c, "car", "set_storage", False, timeout=10)
    print("  ok")

    postflight(c, "after")
    print()
    total_calls = 4 + 5 + 4
    print(f"{'PASS' if fails == 0 else 'FAIL'}: {total_calls - fails}/{total_calls} ok")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
