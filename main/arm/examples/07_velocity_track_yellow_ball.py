"""main/arm/examples/07_velocity_track_yellow_ball.py

velocity-mode 实时追踪 (IBVS 速度模式) —— 绕开 arm_queue。

背景 (2026-08-01):
  find_target_track 每帧发 HTTP goto_position 进 arm_queue, 位置闭环 ~500ms/次,
  视觉 ~8Hz, 命令积压 100+, 用户观察"停下来还在乱跑"。治本: 不走位置队列,
  用实时速度命令 (x_speed / y_speed) 直发 —— 像底盘 set_chassis_velocity 一样。

本脚本:
  1. composite_run 到安全位 (arm=0, x=0, y=-130, hand=UP)
  2. stop_arm_feed (释放串口, 减少竞争)
  3. WS subscribe_task_detection 订阅检测
  4. 每帧: 匹配 selector → 误差 dx_norm/dy_norm → 速度命令
         x_vel = -dx_norm * gain  (单位 m/s)
         y_vel = -dy_norm * gain
     检测丢失 / |err| < deadzone → 速度 0 (停)
  5. 通过 POST /v1/realtime/arm-velocity 直发 (免 queue)
  6. timeout 后: 速度 0 → start_arm_feed → 复位

安全:
  - y 有磁感安全门 + 末段/顶段减速 (arm_base.y_speed 内置), 不会撞磁感
  - x 无软限位, 但 gain 限速 + 检测丢失即停; 跑偏时 Ctrl-C 立即发 0
  - 结束时 (含异常) try/finally 强制 x_vel=0 y_vel=0

用法:
    export RAK_CAR_API_BASE=http://192.168.5.230:5050
    /usr/bin/python3 main/arm/examples/07_velocity_track_yellow_ball.py
    /usr/bin/python3 main/arm/examples/07_velocity_track_yellow_ball.py --gain 0.08 --timeout 20
    /usr/bin/python3 main/arm/examples/07_velocity_track_yellow_ball.py --label cylinder_3
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.api_client import RuntimeApiClient
from main.ws_client import RuntimeWsClient

# ---------- 常量 ----------

ARM_ANGLE_DEFAULT = 0.0     # 大臂 0 (竖直 / MID)
HAND_DEFAULT = -90.0        # 手爪 UP
Y_START_DEFAULT = -130.0    # mm
X_START_DEFAULT = 0.0       # mm
GAIN_DEFAULT = 0.05         # m/s per norm unit (dx=0.5 → 25mm/s)
DEADZONE_NORM = 0.02        # |err| 小于此 → 停 (防抖)
HZ_DEFAULT = 20.0           # WS 订阅频率
REALTIME_URL = "/v1/realtime/arm-velocity"


# ---------- 工具 ----------

def _now_utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _section(title: str) -> None:
    print(f"\n=== {title} ===", flush=True)


def _post_velocity(http: RuntimeApiClient, x_vel=None, y_vel=None) -> dict:
    """POST /v1/realtime/arm-velocity 直发 (免 queue)."""
    payload = {}
    if x_vel is not None:
        payload["x_vel"] = float(x_vel)
    if y_vel is not None:
        payload["y_vel"] = float(y_vel)
    # 用底层 requests, 不走 execute_arm_action (那是 queue 路径)
    import requests
    url = http.build_url(REALTIME_URL)  # api_base + "/v1/realtime/arm-velocity"
    r = requests.post(url, json=payload, timeout=2.0)
    r.raise_for_status()
    return r.json()


def _stop_arm(http: RuntimeApiClient) -> None:
    """急停: 速度 0 (免 queue 直发)."""
    try:
        _post_velocity(http, x_vel=0.0, y_vel=0.0)
    except Exception as e:
        print(f"  ⚠️ 急停失败: {type(e).__name__}: {e}", flush=True)


def _set_arm_feed(http: RuntimeApiClient, *, stop: bool) -> None:
    try:
        if stop:
            r = http.execute("car", "stop_arm_feed", kwargs={"force": True},
                             sync=True, timeout=8.0)
            print(f"  stop_arm_feed(force) -> {r.get('job', {}).get('result')}", flush=True)
        else:
            r = http.execute("car", "start_arm_feed", args=[20.0],
                             sync=True, timeout=8.0)
            print(f"  start_arm_feed(20Hz) -> {r.get('job', {}).get('result')}", flush=True)
    except Exception as e:
        print(f"  ⚠️  _set_arm_feed(stop={stop}) failed: {type(e).__name__}: {e}", flush=True)


# ---------- 主循环 ----------

def _pick_best(dets, label: str):
    """从检测里挑目标: 匹配 label, 取最高 score."""
    best = None
    for d in dets:
        if d.get("label") != label:
            continue
        if best is None or (d.get("score") or 0) > (best.get("score") or 0):
            best = d
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--gain", type=float, default=GAIN_DEFAULT,
                    help=f"速度增益 m/s per norm (默认 {GAIN_DEFAULT})")
    ap.add_argument("--deadzone", type=float, default=DEADZONE_NORM)
    ap.add_argument("--label", default="ball_yellow")
    ap.add_argument("--x-start", type=float, default=X_START_DEFAULT)
    ap.add_argument("--y-start", type=float, default=Y_START_DEFAULT)
    ap.add_argument("--hz", type=float, default=HZ_DEFAULT)
    ap.add_argument("--negate-x", action="store_true", help="翻转 x 速度符号")
    ap.add_argument("--negate-y", action="store_true", help="翻转 y 速度符号")
    ap.add_argument("--no-reset", action="store_true")
    args = ap.parse_args()

    sign_x = -1.0 if not args.negate_x else 1.0
    sign_y = -1.0 if not args.negate_y else 1.0

    http = RuntimeApiClient()
    print(f"server: {os.environ.get('RAK_CAR_API_BASE', 'default')}", flush=True)
    print(f"timeout={args.timeout}s gain={args.gain} label={args.label} "
          f"sign=({'+' if sign_x<0 else '-'}, {'+' if sign_y<0 else '-'})", flush=True)

    # 0. 起始安全位
    _section(f"composite_run -> arm={ARM_ANGLE_DEFAULT} x={args.x_start} y={args.y_start} hand={HAND_DEFAULT}")
    try:
        http.execute("arm", "composite_run",
                     kwargs={"arm": ARM_ANGLE_DEFAULT, "x": args.x_start/1000.0,
                             "y": args.y_start/1000.0, "hand": HAND_DEFAULT},
                     sync=True, timeout=30.0)
        print("  ok", flush=True)
    except Exception as e:
        print(f"🔴 起始位失败: {type(e).__name__}: {e}", flush=True)
        return 1

    # 0.5 让位 arm_feed
    _section("让位: stop_arm_feed")
    _set_arm_feed(http, stop=True)

    # 1. 订阅检测 + 速度伺服
    _section(f"velocity track ({args.label}) | timeout={args.timeout}s hz={args.hz}")
    ws = RuntimeWsClient()
    trace: list[dict] = []
    stop_event = threading.Event()
    t0 = time.time()

    def _on_push(state: dict) -> None:
        now = time.time()
        elapsed = now - t0
        if elapsed > args.timeout or stop_event.is_set():
            _stop_arm(http)
            return
        dets = state.get("detections", []) if isinstance(state, dict) else []
        pick = _pick_best(dets, args.label)
        if pick is None:
            # 检测丢失 → 停 (不累积, 不发)
            _stop_arm(http)
            trace.append({"t": round(elapsed, 3), "label": None, "score": 0.0,
                          "dx": 0.0, "dy": 0.0, "x_vel": 0.0, "y_vel": 0.0, "miss": True})
            return
        bn = pick.get("bbox_norm", {})
        dx = float(bn.get("x_center", 0.0))
        dy = float(bn.get("y_center", 0.0))
        score = float(pick.get("score", 0.0))
        # 死区: 小误差不追
        if abs(dx) < args.deadzone:
            x_vel = 0.0
        else:
            x_vel = sign_x * dx * args.gain
        if abs(dy) < args.deadzone:
            y_vel = 0.0
        else:
            y_vel = sign_y * dy * args.gain
        # 限速 ±0.15 m/s (安全)
        x_vel = max(-0.15, min(0.15, x_vel))
        y_vel = max(-0.15, min(0.15, y_vel))
        try:
            _post_velocity(http, x_vel=x_vel, y_vel=y_vel)
        except Exception as e:
            trace.append({"t": round(elapsed, 3), "label": pick.get("label"),
                          "score": round(score, 3), "dx": round(dx, 3), "dy": round(dy, 3),
                          "x_vel": x_vel, "y_vel": y_vel, "http_err": str(e)[:40]})
            return
        trace.append({"t": round(elapsed, 3), "label": pick.get("label"),
                      "score": round(score, 3), "dx": round(dx, 3), "dy": round(dy, 3),
                      "x_vel": round(x_vel, 4), "y_vel": round(y_vel, 4), "miss": False})

    ws.subscribe_task_detection(_on_push, hz=args.hz)
    try:
        # 等待 timeout (on_push 超时后自己会 _stop_arm)
        deadline = t0 + args.timeout
        while time.time() < deadline:
            time.sleep(0.2)
    finally:
        _stop_arm(http)
        print(f"  stop (x_vel=0 y_vel=0) at t={time.time()-t0:.2f}s", flush=True)

    # 2. 汇总
    n = len(trace)
    n_miss = sum(1 for t in trace if t.get("miss"))
    n_hit = n - n_miss
    if n_hit:
        max_vel = max(max(abs(t["x_vel"]), abs(t["y_vel"])) for t in trace if not t.get("miss"))
        avg_abs_vel = sum(abs(t["x_vel"]) + abs(t["y_vel"]) for t in trace if not t.get("miss")) / n_hit
    else:
        max_vel = avg_abs_vel = 0.0
    if n > 1:
        dts = [(trace[i+1]["t"] - trace[i]["t"]) for i in range(n-1)]
        dt_mean = sum(dts) / len(dts) * 1000
        dt_max = max(dts) * 1000
    else:
        dt_mean = dt_max = 0.0

    _section("汇总")
    print(f"  帧数      : {n} (hit={n_hit} miss={n_miss})", flush=True)
    print(f"  帧间隔    : mean={dt_mean:.0f}ms max={dt_max:.0f}ms", flush=True)
    print(f"  |速度|max : {max_vel*1000:.0f} mm/s", flush=True)
    print(f"  |速度|avg : {avg_abs_vel*1000:.0f} mm/s", flush=True)
    print(f"  耗时      : {time.time()-t0:.1f}s", flush=True)

    out_path = f"vel_trace_{_now_utc()}.jsonl"
    with open(out_path, "w", encoding="utf-8") as f:
        for t in trace:
            f.write(json.dumps(t, ensure_ascii=False) + "\n")
    print(f"  trace -> {out_path}", flush=True)

    # 3. 恢复
    _section("恢复: start_arm_feed + 复位")
    try:
        _set_arm_feed(http, stop=False)
        if not args.no_reset:
            http.execute("arm", "composite_run",
                         kwargs={"arm": ARM_ANGLE_DEFAULT, "x": args.x_start/1000.0,
                                 "y": args.y_start/1000.0, "hand": HAND_DEFAULT},
                         sync=True, timeout=30.0)
            print("  复位完成", flush=True)
    except Exception as e:
        print(f"  ⚠️ 恢复/复位失败: {type(e).__name__}: {e}", flush=True)

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⌘ Ctrl-C, 急停...", flush=True)
        try:
            _stop_arm(RuntimeApiClient())
            print("急停成功", flush=True)
        except Exception:
            pass
        sys.exit(130)
