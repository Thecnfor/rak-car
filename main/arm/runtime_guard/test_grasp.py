#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""test_grasp.py
吸气泵"能不能吸"的最简单测试。

骨架:
  health check -> N 轮 (grasp(ON) → 睡 S1 → 用户摸吸盘 → grasp(OFF) → 睡 S2)
              -> 跑后 health
              -> 总结 (API 全 OK + 用户物理判断)

判定标准 (2 个,任一 FAIL 整个 FAIL):
  1) API 层:每轮 grasp(ON)/(OFF) 都返回 status=succeeded,不抛异常
  2) 物理层:每轮 ON 期间用户摸吸盘口感受到真空吸力
     (听泵声:嗡 → 嗡 → 嗡 持续;摸:手指轻贴吸盘口会被吸住)

约束:
  - 直接走 RuntimeApiClient.execute_arm_action(**kwargs),绕开
    api.py:234 arm.grasp() 的 multiple values for timeout 那个 bug。
  - PowerShell 设 RAK_CAR_SERVER_ORIGIN=http://192.168.3.60 后再跑,
    否则 localhost 卡死不会超时退出。

硬件:
  真空泵  PoutD  port=2  (arm_cfg.yaml:hand_cfg.grap.port_pump)
  阀      PoutD  port=3  (arm_cfg.yaml:hand_cfg.grap.port_valve)

运行:
  PowerShell:
    $env:RAK_CAR_SERVER_ORIGIN = "http://192.168.3.60"
    python main\\arm\\test\\test_grasp.py
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


# 测试参数(可调)
N_CYCLES = 3          # 开-关循环轮数
ON_HOLD_S = 3.0       # 每轮 ON 持续时间,够用户摸
OFF_PAUSE_S = 1.5     # 每轮 OFF 间隔,给真空释放/声音静下来
PUMP_TIMEOUT_S = 8.0  # grasp(ON) 的 HTTP 超时
RELEASE_TIMEOUT_S = 5.0  # grasp(OFF) 的 HTTP 超时


def call_grasp(c: RuntimeApiClient, on: bool, timeout: float) -> dict:
    """单次 grasp 调用,带超时 + 异常捕获。kwargs=value 避开 api.py:234 bug。"""
    try:
        r = c.execute_arm_action("grasp", timeout=timeout, value=bool(on))
        return {
            "ok": r.get("status") == "succeeded",
            "status": r.get("status"),
            "error": r.get("error"),
        }
    except Exception as e:
        return {
            "ok": False,
            "status": "exception",
            "error": str(e)[:120],
        }


def show_health(c: RuntimeApiClient, label: str) -> None:  # legacy,kept for compat
    try:
        s = c.get_health().get("state", {})
        print(f"  [{label}] initialized={s.get('initialized')}  "
              f"initializing={s.get('initializing')}  "
              f"last_error={s.get('last_error')}")
    except Exception as e:
        print(f"  [{label}] health FAIL: {str(e)[:80]}")


def main() -> int:
    c = RuntimeApiClient()

    # ---- runtime 就绪检查 ----
    if not preflight(c):
        return 1
    print()

    api_fails = 0  # API 失败计数
    total_calls = 0

    # ---- 循环 ----
    print(f"=== {N_CYCLES} 轮 (ON → 用户摸 → OFF) ===")
    print(f"    每轮 ON = {ON_HOLD_S}s,OFF 间隔 = {OFF_PAUSE_S}s")
    print()
    for i in range(1, N_CYCLES + 1):
        t0 = time.time()
        print(f"--- 轮 #{i} ---")

        # ON
        r = call_grasp(c, True, PUMP_TIMEOUT_S)
        total_calls += 1
        flag = "OK  " if r["ok"] else "FAIL"
        print(f"  [{flag}] #{i}.1 grasp(ON )  status={r['status']}  err={r['error']}")
        if not r["ok"]:
            api_fails += 1

        # 用户物理验证窗口:呼吸 3 秒,够摸 + 听
        print(f"  >>> 现在摸吸盘口,/ 听泵声  ({ON_HOLD_S}s) <<<")
        time.sleep(ON_HOLD_S)

        # OFF
        r = call_grasp(c, False, RELEASE_TIMEOUT_S)
        total_calls += 1
        flag = "OK  " if r["ok"] else "FAIL"
        print(f"  [{flag}] #{i}.2 grasp(OFF)  status={r['status']}  err={r['error']}")
        if not r["ok"]:
            api_fails += 1

        # 释放
        print(f"  (释放 {OFF_PAUSE_S}s 让真空掉完)")
        time.sleep(OFF_PAUSE_S)

        dt = time.time() - t0
        print(f"  cycle_dt = {dt:.2f}s")
        print()

    # ---- 跑后 health ----
    print("=== 跑后 health ===")
    postflight(c, "after")
    print()

    # ---- 总结 ----
    api_pass = (api_fails == 0)
    print("=== 总结 ===")
    print(f"  API  调用: {total_calls - api_fails}/{total_calls} succeeded")
    print(f"  物理判定: 你刚才在每轮 ON 期间摸吸盘是否感到吸力?")
    print(f"           * 感到持续 '嗡' 声 + 手指被吸住 -> 物理 OK")
    print(f"           * 没声音 / 没吸力 / 一碰就掉 -> 硬件 (泵/阀/管路) 故障")
    print()

    if api_pass:
        print("API 层: PASS")
    else:
        print(f"API 层: FAIL ({api_fails} 次调用失败)")
    print()
    print("物理层: 请根据现场手感/人耳判断 -> 在脚本之外的笔记里记 PASS/FAIL")
    print()
    label = "API_PASS" if api_pass else "API_FAIL"
    print(f"脚本退出码 = {0 if api_pass else 1}  ({label})")
    return 0 if api_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
