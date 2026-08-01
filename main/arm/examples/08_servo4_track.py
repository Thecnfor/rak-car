"""main/arm/examples/08_servo4_track.py

4-DOF 视觉伺服实时追踪 — xy 十字 + 大臂(yaw) + 手抓(pitch), 末端摄像头对准目标。

机械结构 (用户约定, 2026-08-01):
  - xy 十字滑台 (垂直于大地): x 速度, y 速度           ← velocity 控制
  - 大臂电机: -90°(朝 x 左) ~ +90°(朝 x 右)            ← 角度控制 (水平转向)
  - 手抓电机: -90°(看正面/水平) ~ 0°(朝下)             ← 角度控制 (垂直转向)
  - 末端摄像头绑定在手上 → 相机朝向 = 大臂 yaw + 手抓 pitch

闭环策略 (相机对准模式):
  每帧检测目标 → 误差 dx(水平) / dy(垂直):
    x_vel     = -dx * gain_x              十字水平跟
    y_vel     = -dy * gain_y              十字垂直跟
    arm_target += -dx * gain_arm         大臂水平转向 (相机左右对准目标)
    hand_target += -dy * gain_hand       手抓垂直转向 (相机上下对准目标)
  全部走 POST /v1/realtime/arm-velocity 一个端点 (免 arm_queue, 实时)。

角度软限位 (runtime clamp + 客户端 clamp): arm ∈ [-90,+90], hand ∈ [-90,0]。
检测丢失 → 全轴停 (x_vel=0 y_vel=0, 角度不动), 不累积。

用法:
    export RAK_CAR_API_BASE=http://192.168.5.230:5050
    /usr/bin/python3 main/arm/examples/08_servo4_track.py --label ball_yellow
    /usr/bin/python3 main/arm/examples/08_servo4_track.py --label animal --gain-arm 2.0 --gain-hand 1.0
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

X_START_DEFAULT = 0.0
Y_START_DEFAULT = -130.0
ARM_START_DEFAULT = 0.0        # 大臂初始 0 (垂直中间)
HAND_START_DEFAULT = -90.0     # 手抓初始 -90 (看正面, 相机水平朝前)
GAIN_X_DEFAULT = 0.05          # m/s per norm (十字 x)
GAIN_Y_DEFAULT = 0.05          # m/s per norm (十字 y)
GAIN_ARM_DEFAULT = 2.0         # deg per norm (大臂, dx=0.3 → 转 0.6°)
GAIN_HAND_DEFAULT = 2.0        # deg per norm (手抓)
DEADZONE_NORM = 0.02
ARM_MIN, ARM_MAX = -90.0, 90.0
HAND_MIN, HAND_MAX = -90.0, 0.0
REALTIME_URL = "/v1/realtime/arm-velocity"


# ---------- 工具 ----------

def _now_utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _section(title: str) -> None:
    print(f"\n=== {title} ===", flush=True)


def _post_4dof(http, *, x_vel=None, y_vel=None, arm=None, hand=None) -> dict:
    import requests
    payload = {}
    if x_vel is not None:
        payload["x_vel"] = float(x_vel)
    if y_vel is not None:
        payload["y_vel"] = float(y_vel)
    if arm is not None:
        payload["arm_angle"] = float(arm)
    if hand is not None:
        payload["hand_angle"] = float(hand)
    url = http.build_url(REALTIME_URL)
    r = requests.post(url, json=payload, timeout=2.0)
    r.raise_for_status()
    return r.json()


def _stop_all(http) -> None:
    """急停: xy 速度 0 (角度不动)."""
    try:
        _post_4dof(http, x_vel=0.0, y_vel=0.0)
    except Exception as e:
        print(f"  ⚠️ 急停失败: {type(e).__name__}: {e}", flush=True)


def _composite_start(http, *, x_mm, y_mm, arm_angle, hand_angle) -> None:
    """起始 composite_run 到安全位 (走 queue, 一次性)."""
    http.execute("arm", "composite_run",
                 kwargs={"arm": arm_angle, "x": x_mm / 1000.0,
                         "y": y_mm / 1000.0, "hand": hand_angle},
                 sync=True, timeout=30.0)


def _set_arm_feed(http, *, stop: bool) -> None:
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


def _pick_best(dets, label: str):
    best = None
    for d in dets:
        if d.get("label") != label:
            continue
        if best is None or (d.get("score") or 0) > (best.get("score") or 0):
            best = d
    return best


# ---------- main ----------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--label", default="ball_yellow")
    ap.add_argument("--gain-x", type=float, default=GAIN_X_DEFAULT)
    ap.add_argument("--gain-y", type=float, default=GAIN_Y_DEFAULT)
    ap.add_argument("--gain-arm", type=float, default=GAIN_ARM_DEFAULT)
    ap.add_argument("--gain-hand", type=float, default=GAIN_HAND_DEFAULT)
    ap.add_argument("--deadzone", type=float, default=DEADZONE_NORM)
    ap.add_argument("--x-start", type=float, default=X_START_DEFAULT)
    ap.add_argument("--y-start", type=float, default=Y_START_DEFAULT)
    ap.add_argument("--arm-start", type=float, default=ARM_START_DEFAULT)
    ap.add_argument("--hand-start", type=float, default=HAND_START_DEFAULT)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--no-reset", action="store_true")
    args = ap.parse_args()

    http = RuntimeApiClient()
    print(f"server: {os.environ.get('RAK_CAR_API_BASE', 'default')}", flush=True)
    print(f"timeout={args.timeout}s label={args.label} "
          f"gains=(x:{args.gain_x}, y:{args.gain_y}, arm:{args.gain_arm}, hand:{args.gain_hand})", flush=True)

    # 0. 起始安全位
    _section(f"composite_run -> arm={args.arm_start} x={args.x_start} y={args.y_start} hand={args.hand_start}")
    try:
        _composite_start(http, x_mm=args.x_start, y_mm=args.y_start,
                         arm_angle=args.arm_start, hand_angle=args.hand_start)
        print("  ok", flush=True)
    except Exception as e:
        print(f"🔴 起始位失败: {type(e).__name__}: {e}", flush=True)
        return 1

    # 0.5 让位 arm_feed
    _section("让位: stop_arm_feed")
    _set_arm_feed(http, stop=True)

    # 1. 4-DOF 追踪
    _section(f"4-DOF track ({args.label}) | timeout={args.timeout}s")
    ws = RuntimeWsClient()
    trace: list[dict] = []
    t0 = time.time()
    # 客户端维护角度目标 (增量式, 从初始角度开始)
    arm_target = args.arm_start
    hand_target = args.hand_start

    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _on_push(state: dict) -> None:
        nonlocal arm_target, hand_target
        now = time.time()
        elapsed = now - t0
        if elapsed > args.timeout:
            _stop_all(http)
            return
        dets = state.get("detections", []) if isinstance(state, dict) else []
        pick = _pick_best(dets, args.label)
        if pick is None:
            # 检测丢失 → xy 停, 角度不动 (保持当前姿态, 不累积)
            _stop_all(http)
            trace.append({"t": round(elapsed, 3), "label": None, "dx": 0.0, "dy": 0.0,
                          "arm": round(arm_target, 1), "hand": round(hand_target, 1),
                          "x_vel": 0.0, "y_vel": 0.0, "miss": True})
            return
        bn = pick.get("bbox_norm", {})
        dx = float(bn.get("x_center", 0.0))
        dy = float(bn.get("y_center", 0.0))
        score = float(pick.get("score", 0.0))

        # 死区: 小误差不追 (防抖)
        if abs(dx) < args.deadzone:
            x_vel = 0.0
            d_arm = 0.0
        else:
            x_vel = -dx * args.gain_x
            # 2026-08-01 方向修正: 目标在相机左 (dx<0) → 大臂应向左转 (arm_target 减小)。
            # 原 d_arm=-dx*gain 会让大臂朝反方向转 → 越追越偏 → 球出画面 → 漏检链。
            d_arm = +dx * args.gain_arm
        if abs(dy) < args.deadzone:
            y_vel = 0.0
            d_hand = 0.0
        else:
            # 2026-08-01 方向修正 (y 轴): 目标在画面上方 (dy<0) → 机械臂应向上移。
            # 实测 y_vel 正方向 = 向下 (球在上 dy<0 时原 -dy 给出正 y_vel → 往下走 = 反)。
            # 改 +dy: dy<0 → y_vel<0 → 向上 ✓。
            y_vel = +dy * args.gain_y
            # 手抓垂直转向 (同大臂): 目标在画面上方 (dy<0) → 手抓应向上抬
            # (hand 减小向 -90)。原 d_hand=-dy*gain 让相机越看越低 → 球跑出画面顶部
            # → miss 92% 的根因之一。trace 验证: dy<0 时 hand -89→-55 持续下转 = 反了。
            d_hand = +dy * args.gain_hand

        # 限速 ±0.15 m/s
        x_vel = max(-0.15, min(0.15, x_vel))
        y_vel = max(-0.15, min(0.15, y_vel))
        # 角度增量 + clamp
        arm_target = _clamp(arm_target + d_arm, ARM_MIN, ARM_MAX)
        hand_target = _clamp(hand_target + d_hand, HAND_MIN, HAND_MAX)

        try:
            _post_4dof(http, x_vel=x_vel, y_vel=y_vel,
                       arm=arm_target, hand=hand_target)
        except Exception as e:
            trace.append({"t": round(elapsed, 3), "label": pick.get("label"),
                          "dx": round(dx, 3), "dy": round(dy, 3),
                          "arm": round(arm_target, 1), "hand": round(hand_target, 1),
                          "x_vel": x_vel, "y_vel": y_vel, "http_err": str(e)[:40]})
            return
        trace.append({"t": round(elapsed, 3), "label": pick.get("label"),
                      "dx": round(dx, 3), "dy": round(dy, 3),
                      "arm": round(arm_target, 1), "hand": round(hand_target, 1),
                      "x_vel": round(x_vel, 4), "y_vel": round(y_vel, 4),
                      "d_arm": round(d_arm, 2), "d_hand": round(d_hand, 2), "miss": False})

    ws.subscribe_task_detection(_on_push, hz=args.hz)
    try:
        deadline = t0 + args.timeout
        while time.time() < deadline:
            time.sleep(0.2)
    finally:
        _stop_all(http)
        print(f"  stop (xy=0) at t={time.time()-t0:.2f}s", flush=True)

    # 2. 汇总
    n = len(trace)
    n_miss = sum(1 for t in trace if t.get("miss"))
    n_hit = n - n_miss
    if n_hit:
        max_vel = max(max(abs(t["x_vel"]), abs(t["y_vel"])) for t in trace if not t.get("miss"))
    else:
        max_vel = 0.0
    if n > 1:
        dts = [(trace[i+1]["t"] - trace[i]["t"]) for i in range(n-1)]
        dt_mean = sum(dts) / len(dts) * 1000
        dt_max = max(dts) * 1000
    else:
        dt_mean = dt_max = 0.0

    _section("汇总")
    print(f"  帧数      : {n} (hit={n_hit} miss={n_miss})", flush=True)
    print(f"  帧间隔    : mean={dt_mean:.0f}ms max={dt_max:.0f}ms", flush=True)
    print(f"  |xy速度|max: {max_vel*1000:.0f} mm/s", flush=True)
    print(f"  大臂终角   : {arm_target:+.1f}° (范围 [{-90}, {90}])", flush=True)
    print(f"  手抓终角   : {hand_target:+.1f}° (范围 [{-90}, {0}])", flush=True)

    out_path = f"servo4_trace_{_now_utc()}.jsonl"
    with open(out_path, "w", encoding="utf-8") as f:
        for t in trace:
            f.write(json.dumps(t, ensure_ascii=False) + "\n")
    print(f"  trace -> {out_path}", flush=True)

    # 3. 恢复
    _section("恢复: start_arm_feed + 复位")
    try:
        _set_arm_feed(http, stop=False)
        if not args.no_reset:
            _composite_start(http, x_mm=args.x_start, y_mm=args.y_start,
                             arm_angle=args.arm_start, hand_angle=args.hand_start)
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
            _stop_all(RuntimeApiClient())
            print("急停成功", flush=True)
        except Exception:
            pass
        sys.exit(130)
