#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""test_side_diag.py
大臂 bus 舵机最小诊断 —— 不走业务层,直接打 realtime 端点验证舵机是否能物理转动。

⚠️ 第一次跑发现的问题:
  - car.get_arm_state().arm_angle 是命令回显(arm_base.py:472),物理判定必须绕开
  - quick_start.py:64 用 port=1,但 arm_cfg.yaml 里 hand.port 可能是别的值
    → 猜的 port 不一定对,需要扫

本脚本做三件事:
  1) SCAN: 扫 port 1..10,只读不写,找出哪些 port 真有舵机响应
  2) PROBE: 在扫到的 port 上,set_angle + read_angle,看 raw 是否真的变
  3) DIAG: 输出可能的根因(找不到活 port / 找到了但不动 / 找到了且正常)

约束:
  - SCAN 阶段只读不发,安全
  - PROBE 阶段只动一个已确认的 port,不会乱打到别的舵机
  - 默认 SERVER_ORIGIN 已是 http://192.168.3.60

运行:
  PowerShell:
    python main\\arm\\test\\test_side_diag.py
"""
import os
import sys

# 让 main.* 可被 import
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.api_client import RuntimeApiClient  # noqa: E402
from main.arm.test._runtime_guard import preflight, postflight  # noqa: E402


# SCAN 范围
SCAN_PORTS = list(range(1, 11))   # 1..10
# 默认 port(给 quick_start.py 参考,扫描成功后会覆盖)
DEFAULT_PORT = 1
# 探测时的目标(协议层下发 raw 角度,不走 angle_list)
PROBE_TARGETS = [0, 93, 0, -93, 0]


def read_angle(client: RuntimeApiClient, port: int) -> dict:
    """读真角度,返回 {"ok": bool, "angle": int|None, "err": str|None, "body": str|None}。

    HTTPError 时把 response body 抓出来 —— FastAPI 默认 500 处理器会把
    异常 detail 放在 body 的 JSON 里,默认 RuntimeApiClient 不暴露。
    """
    import requests as _req

    # 走 raw HTTP,捕获完整响应
    base = client.api_base
    url = f"{base}/v1/realtime/bus-servo/angle?port={int(port)}"
    try:
        resp = _req.get(url, timeout=5.0)
    except _req.exceptions.ConnectionError as e:
        return {"ok": False, "angle": None,
                "err": f"ConnectionError: {str(e)[:120]}", "body": None}
    except _req.exceptions.Timeout as e:
        return {"ok": False, "angle": None,
                "err": f"Timeout: {str(e)[:120]}", "body": None}
    except Exception as e:
        return {"ok": False, "angle": None,
                "err": f"{type(e).__name__}: {str(e)[:120]}", "body": None}

    # 抓 body(可能为空,但尽量拿)
    body_text = resp.text[:500] if resp.text else ""
    try:
        body_json = resp.json()
        body_text = str(body_json)[:500]
    except Exception:
        pass

    if resp.status_code == 200:
        try:
            data = resp.json()
            if data.get("ok") and "angle" in data:
                return {"ok": True, "angle": int(data["angle"]), "err": None, "body": body_text}
            return {"ok": False, "angle": None,
                    "err": f"unexpected response: {body_text}", "body": body_text}
        except Exception as e:
            return {"ok": False, "angle": None,
                    "err": f"json parse fail: {e}", "body": body_text}

    # 非 200:把状态码 + body 都返回
    return {
        "ok": False,
        "angle": None,
        "err": f"HTTP {resp.status_code}",
        "body": body_text,
    }


def set_angle(client: RuntimeApiClient, port: int, angle: int) -> dict:
    """协议层下发,返回原始响应或异常字符串。"""
    try:
        r = client.realtime_bus_servo_angle(port, angle, speed=80)
        return {"ok": True, "response": r}
    except Exception as e:
        return {"ok": False, "err": f"{type(e).__name__}: {str(e)[:200]}"}


def scan_ports(client: RuntimeApiClient) -> list:
    """扫 1..10,返回 [{port, ok, angle, err, body}]。只读,安全。"""
    print(f"=== SCAN 阶段:扫 port={SCAN_PORTS[0]}..{SCAN_PORTS[-1]} (只读) ===")
    results = []
    for port in SCAN_PORTS:
        r = read_angle(client, port)
        results.append({"port": port, **r})
        if r["ok"]:
            print(f"  port={port:>2}: [LIVE] angle={r['angle']:+4d}°")
        else:
            err_short = r["err"][:60] if r["err"] else "?"
            body_short = (r.get("body") or "")[:120].replace("\n", " ")
            print(f"  port={port:>2}: [----] {err_short}")
            if body_short and body_short != err_short:
                print(f"        body: {body_short}")
    print()
    return results


def probe_port(client: RuntimeApiClient, port: int) -> int:
    """在已确认活的 port 上 set + read,看 raw 是否真的变。返回失败站数。"""
    import time as _t

    print(f"=== PROBE 阶段:port={port} 探测 {len(PROBE_TARGETS)} 站 ===")
    raw0 = read_angle(client, port)["angle"]
    print(f"  初始 raw: {raw0:+d}°")
    print()

    fails = 0
    last_raw = raw0
    for i, target in enumerate(PROBE_TARGETS, 1):
        print(f"--- 站 #{i}: set_angle({target:+d}°) ---")
        r = set_angle(client, port, target)
        if not r["ok"]:
            print(f"  [FAIL] set_angle 失败: {r['err']}")
            fails += 1
            continue

        # 给舵机到位的时间(±93° / 80 速度大约 1.2s,留 1.5s 兜底)
        _t.sleep(1.5)

        read_r = read_angle(client, port)
        if not read_r["ok"]:
            print(f"  [FAIL] read_angle 失败: {read_r['err']}")
            fails += 1
            continue

        raw = read_r["angle"]
        delta_from_last = raw - last_raw
        delta_from_target = raw - target
        moved = abs(delta_from_last) > 5
        hit = abs(delta_from_target) <= 5

        flag = "OK  " if (moved or i == 1) and hit else "WARN" if moved else "FAIL"
        print(f"  cmd={target:+4d}°  raw={raw:+4d}°  "
              f"from_last={delta_from_last:+4d}°  from_target={delta_from_target:+3d}°  "
              f"moved={moved}  hit_target={hit}")
        if not hit:
            fails += 1
        last_raw = raw
        print()

    return fails


def main() -> int:
    client = RuntimeApiClient()

    # ---- runtime 就绪 ----
    if not preflight(client):
        return 1
    print()

    # ---- SCAN ----
    scan_results = scan_ports(client)
    live_ports = [r["port"] for r in scan_results if r["ok"]]

    if not live_ports:
        print("=" * 60)
        print("❌ SCAN 阶段:没有任何 port 读到角度")
        print()
        # 抓第一个失败的 err/body 详细展示
        first_fail = next((r for r in scan_results if not r["ok"]), None)
        if first_fail:
            print(f"  详细错误(port={first_fail['port']}):")
            print(f"    status: {first_fail['err']}")
            if first_fail.get("body"):
                print(f"    body  : {first_fail['body']}")
            print()
        print("可能根因:")
        print("  1) 总线舵机供电断了 —— 检查舵机壳体 LED / 电源线")
        print("  2) MC602 串口线松了 / 接触不良")
        print("  3) 舵机不在 1..10 范围(不太可能,默认就是 1)")
        print("  4) runtime 的 _get_realtime_instance 缓存了坏实例 —— pm2 restart rak-car-api")
        print("  5) runtime 端 act_mode / send_get 抛非 RuntimeError 异常 (HTTP 500)")
        print("     → 看 runtime 真实异常栈: pm2 logs rak-car-api --lines 200")
        print()
        print("对照上面 SCAN 输出,看看每个 port 的 err 是什么:")
        print("  - 'HTTP 500' + body 有 detail → 服务端异常被转 409 又转 500 的奇怪路径")
        print("  - 'HTTP 500' + body 为空或 'Internal Server Error' → 真异常,看 pm2 logs")
        print("  - 'Connection refused' → runtime 没起来")
        print("  - 'Timeout' → 服务在但 hang")
        return 1

    print(f"找到 {len(live_ports)} 个活 port: {live_ports}")
    print()

    # ---- PROBE:对第一个活的 port 跑探测 ----
    # 优先用默认 port(如果它在 live_ports 里),否则用第一个活的
    if DEFAULT_PORT in live_ports:
        probe_target = DEFAULT_PORT
    else:
        probe_target = live_ports[0]
        print(f"⚠️  默认 port={DEFAULT_PORT} 不在活 port 列表里")
        print(f"   改用 probe_target=port={probe_target} (第一个活的)")

    print(f"=== 准备对 port={probe_target} 跑 PROBE ===")
    if probe_target != DEFAULT_PORT:
        print(f"⚠️  注意:这意味着 arm_cfg.yaml 里 hand.port ≠ {DEFAULT_PORT}")
        print(f"   跑通后请同步把 test_side.py 的 BUS_SERVO_PORT 改成 {probe_target}")
    print()

    fails = probe_port(client, probe_target)

    # ---- 跑后 health ----
    postflight(client, "after")
    print()

    # ---- 总结 ----
    print("=== 总结 ===")
    print(f"  活 port 列表: {live_ports}")
    print(f"  本次 probe port: {probe_target}")
    print(f"  PROBE 失败站数: {fails}/{len(PROBE_TARGETS)}")
    print()

    if fails == 0:
        print("✅ PROBE 全过 —— bus 舵机活着且角度对得上")
        if probe_target != DEFAULT_PORT:
            print()
            print(f"📌 行动项:把 test_side.py 里的 BUS_SERVO_PORT 从 {DEFAULT_PORT} 改成 {probe_target}")
            print(f"   然后重跑 test_side.py 就能用真角度判定了")
        return 0
    else:
        print(f"❌ PROBE 有 {fails} 站不到位")
        print()
        print("可能根因:")
        print("  1) 舵机零位漂移 —— 重启后 raw 总在某个固定值,但跟命令差很远")
        print("  2) 舵机堵转 —— raw 动一点点但到不了目标")
        print("  3) 速度太慢 —— 给 1.5s 不够,改 set_angle 后 sleep 3s 再读")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())