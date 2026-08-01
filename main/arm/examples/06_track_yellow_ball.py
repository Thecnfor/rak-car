"""main/arm/examples/06_track_yellow_ball.py

真机测试: 大臂 0 + xy 十字滑台**实时追踪黄色果实** (Label.BALL_YELLOW).

起始位姿 (大臂 "伸到 0"):
    arm_angle=0 (MID / 竖直)
    x=0, y=-130mm (软限位内安全位)
    hand=-90 (UP, 不下扎)

追踪策略:
    ServoLoop.find_target_track (WS push, 不收敛, timeout 后返回)

输出:
    - 实时:每个迭代打印一行 [t=...] [iter=N] bbox(dx,dy) + step(dx,dy mm)
    - 结束:打印汇总 (总时长 / 迭代数 / 命中 vs 漏检 / xy 最大偏移 / 步距分布)
    - 落盘:./track_trace_<UTC ts>.jsonl (每帧一行 ServoTrace,可用 jq 查看)

用法:
    export RAK_CAR_API_BASE=http://192.168.5.230:5050
    /usr/bin/python3 main/arm/examples/06_track_yellow_ball.py

    # 自定义:
    /usr/bin/python3 main/arm/examples/06_track_yellow_ball.py \
        --timeout 60 --y-start -130 --x-start 0 --hz 30
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

# 让 `python3 main/arm/examples/06_*.py` 不依赖 PYTHONPATH=. 也能 import main 包
# (仓库根没有 pyproject.toml / setup.py, 因此走 sys.path 注入)
_ROOT = Path(__file__).resolve().parents[3]  # main/arm/examples/06 -> 3 级 = 项目根
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.arm import ArmClient, ArmRunner, TargetSelector, Label


# ---------- 路径参数 ----------

Y_START_DEFAULT = -130.0    # mm, 安全位 (-130, 在 [-200, 0] 内)
X_START_DEFAULT = 0.0       # mm
ARM_ANGLE_DEFAULT = 0.0     # 大臂 0 (= 竖直 / MID)
HAND_DEFAULT = -90.0        # 手爪 UP
TIMEOUT_DEFAULT = 60.0      # s, 持续追踪多久
HZ_DEFAULT = 30.0           # WS push 频率, 与 task_push_hz 匹配
TARGET_REAL_HEIGHT_M = 0.06 # 黄色果实真实高度, 用于 depth-aware gain (来自 labels.py)


# ---------- 工具 ----------

def _now_utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _section(title: str) -> None:
    print(f"\n=== {title} ===", flush=True)


def _verify_runtime(client: ArmClient) -> None:
    if not client.ping():
        print("🔴 runtime not reachable, check RAK_CAR_API_BASE / pm2", flush=True)
        sys.exit(1)


def _move_safe(client: ArmClient, *, x_mm: float, y_mm: float) -> None:
    """起始 composite_run: 大臂 0 + xy 安全位 + 手爪 UP. arm_feed 30Hz polling 会让 arm_queue 排队,需要 retry + 较长 timeout."""
    _section(f"composite_run -> arm={ARM_ANGLE_DEFAULT} x={x_mm} y={y_mm} hand={HAND_DEFAULT}")
    last_err: Exception | None = None
    for attempt in range(3):
        t0 = time.time()
        try:
            job = client.composite_run(
                arm=ARM_ANGLE_DEFAULT, x_mm=x_mm, y_mm=y_mm, hand=HAND_DEFAULT, timeout=30.0
            )
            elapsed = time.time() - t0
            ok = job.get("result", {}).get("ok")
            steps = job.get("result", {}).get("steps")
            print(f"  attempt={attempt+1} ok={ok} steps={steps} elapsed={elapsed:.2f}s", flush=True)
            if ok:
                return
            raise RuntimeError(f"composite_run result.ok=False: {job.get('result')}")
        except Exception as e:
            elapsed = time.time() - t0
            print(f"  ⚠️  attempt={attempt+1} failed ({elapsed:.1f}s): {type(e).__name__}: {e}", flush=True)
            last_err = e
            time.sleep(2.0 * (attempt + 1))
    raise RuntimeError(f"composite_run 3 次均失败: {last_err}")


def _set_arm_feed(client: ArmClient, *, stop: bool) -> None:
    """让位机制: track 前停 arm_feed 释放 arm_queue, track 后恢复。

    2026-08-01: arm_feed 20Hz 持续 poll arm_queue, 视觉伺服的 goto_position
    排队在 poll 之后, 每帧要等几百 ms (实测 0.5 iter/s)。停掉后 ~1.9 iter/s。
    注意: stop/start_arm_feed 在 CAR_ACTIONS (target=car), 不是 ARM_ACTIONS,
    所以走 http.execute("car", ...) 而不是 execute_arm_action(target=arm)。
    """
    try:
        if stop:
            r = client.http.execute("car", "stop_arm_feed", kwargs={"force": True},
                                    sync=True, timeout=8.0)
            print(f"  stop_arm_feed(force) -> {r.get('job', {}).get('result')}", flush=True)
        else:
            r = client.http.execute("car", "start_arm_feed", args=[20.0],
                                    sync=True, timeout=8.0)
            print(f"  start_arm_feed(20Hz) -> {r.get('job', {}).get('result')}", flush=True)
    except Exception as e:
        print(f"  ⚠️  _set_arm_feed(stop={stop}) failed: {type(e).__name__}: {e}", flush=True)


def _sign(x: float) -> int:
    if x > 0: return 1
    if x < 0: return -1
    return 0


def _summarize_trace(trace) -> dict:
    """从 trace 计算状态分布 (命中/漏检/落点/xy 步距/方向分布/帧间隔)."""
    n_total = len(trace)
    n_hit = sum(1 for t in trace if not t.is_miss)
    n_miss = n_total - n_hit
    if n_hit == 0:
        return {
            "iters": n_total, "hits": n_hit, "misses": n_miss,
            "dx_norm_max": None, "dy_norm_max": None,
            "dx_mm_max": None, "dy_mm_max": None,
            "dx_mm_sum_abs": 0.0,
            "x_pos_steps": 0, "x_neg_steps": 0, "x_zero_steps": 0,
            "y_pos_steps": 0, "y_neg_steps": 0, "y_zero_steps": 0,
            "frame_dt_mean_ms": None, "frame_dt_std_ms": None,
            "max_gap_ms": None,
        }

    dx_norm_max = max(abs(t.dx_norm) for t in trace if not t.is_miss)
    dy_norm_max = max(abs(t.dy_norm) for t in trace if not t.is_miss)
    # 每帧步距 = 当前位置 - 上一位置
    step_x = []
    step_y = []
    last_x, last_y = trace[0].x_mm, trace[0].y_mm
    for t in trace[1:]:
        step_x.append(t.x_mm - last_x)
        step_y.append(t.y_mm - last_y)
        last_x, last_y = t.x_mm, t.y_mm

    x_pos = sum(1 for s in step_x if s > 0.05)   # >0.05mm 算正向
    x_neg = sum(1 for s in step_x if s < -0.05)
    x_zero = sum(1 for s in step_x if abs(s) <= 0.05)
    y_pos = sum(1 for s in step_y if s > 0.05)
    y_neg = sum(1 for s in step_y if s < -0.05)
    y_zero = sum(1 for s in step_y if abs(s) <= 0.05)

    if step_x:
        dx_mm_max = max(abs(s) for s in step_x)
        dy_mm_max = max(abs(s) for s in step_y)
        dx_mm_sum_abs = sum(abs(s) for s in step_x)
    else:
        dx_mm_max = dy_mm_max = 0.0
        dx_mm_sum_abs = 0.0

    # 帧间隔 (诊断"不连续" —— 大 gap 表示 WS push 卡顿)
    if len(trace) > 1:
        dts = [(trace[i+1].t_s - trace[i].t_s) for i in range(len(trace)-1)]
        dts_ms = [d * 1000.0 for d in dts]
        dt_mean = sum(dts_ms) / len(dts_ms)
        dt_var = sum((d - dt_mean) ** 2 for d in dts_ms) / len(dts_ms)
        dt_std = dt_var ** 0.5
        dt_max = max(dts_ms)
    else:
        dt_mean = dt_std = dt_max = None

    return {
        "iters": n_total,
        "hits": n_hit,
        "misses": n_miss,
        "dx_norm_max": round(dx_norm_max, 4),
        "dy_norm_max": round(dy_norm_max, 4),
        "dx_mm_max": round(dx_mm_max, 2),
        "dy_mm_max": round(dy_mm_max, 2),
        "dx_mm_sum_abs": round(dx_mm_sum_abs, 1),
        "x_pos_steps": x_pos, "x_neg_steps": x_neg, "x_zero_steps": x_zero,
        "y_pos_steps": y_pos, "y_neg_steps": y_neg, "y_zero_steps": y_zero,
        "frame_dt_mean_ms": round(dt_mean, 1) if dt_mean is not None else None,
        "frame_dt_std_ms": round(dt_std, 1) if dt_std is not None else None,
        "max_gap_ms": round(dt_max, 1) if dt_max is not None else None,
    }


def _print_summary(summary: dict, *, elapsed_s: float, label: str) -> None:
    print(f"\n", flush=True)
    print(f"========== 追踪汇总 ({label}) ==========", flush=True)
    print(f"  总时长     : {elapsed_s:.2f}s", flush=True)
    print(f"  迭代数     : {summary['iters']}", flush=True)
    hits = summary["hits"]
    misses = summary["misses"]
    print(f"  命中帧     : {hits}", flush=True)
    print(f"  漏检帧     : {misses}", flush=True)
    if summary["iters"]:
        rate = hits / summary["iters"] * 100
        print(f"  命中率     : {rate:.1f}%", flush=True)
    if summary["dx_norm_max"] is not None:
        print(f"  dx_norm max: {summary['dx_norm_max']:.4f}  (峰值 bbox 中心偏离)", flush=True)
        print(f"  dy_norm max: {summary['dy_norm_max']:.4f}", flush=True)
    if summary["dx_mm_max"] is not None:
        print(f"  单帧 |dx| 最大: {summary['dx_mm_max']:.2f} mm", flush=True)
        print(f"  单帧 |dy| 最大: {summary['dy_mm_max']:.2f} mm", flush=True)
        print(f"  |dx| 累计  : {summary['dx_mm_sum_abs']:.1f} mm (xy 有没有真在追 = 看这里)", flush=True)

    # ---- 方向分布 (诊断 单向漂移 / "x 只往 + 走") ----
    xp = summary['x_pos_steps']; xn = summary['x_neg_steps']; xz = summary['x_zero_steps']
    yp = summary['y_pos_steps']; yn = summary['y_neg_steps']; yz = summary['y_zero_steps']
    total_step_x = xp + xn + xz
    total_step_y = yp + yn + yz
    if total_step_x > 0:
        print(f"  x 方向分布  : + {xp} / - {xn} / 0 {xz}  ({(xp/total_step_x*100):.0f}% +, {(xn/total_step_x*100):.0f}% -)", flush=True)
    if total_step_y > 0:
        print(f"  y 方向分布  : + {yp} / - {yn} / 0 {yz}  ({(yp/total_step_y*100):.0f}% +, {(yn/total_step_y*100):.0f}% -)", flush=True)
    if xp and xn == 0 and total_step_x > 10:
        print(f"  ⚠️  x 单向:全部 +, 试 --negate 翻号", flush=True)
    if yp and yn == 0 and total_step_y > 10:
        print(f"  ⚠️  y 单向:全部 +, 试 --negate 翻号", flush=True)
    elif yn and yp == 0 and total_step_y > 10:
        print(f"  ⚠️  y 单向:全部 -, 试 --negate 翻号", flush=True)

    # ---- 帧间隔 (诊断 "不连续") ----
    if summary["frame_dt_mean_ms"] is not None:
        mean_ms = summary["frame_dt_mean_ms"]
        std_ms = summary["frame_dt_std_ms"]
        max_ms = summary["max_gap_ms"]
        target_ms = 1000.0 / args.hz if args.hz > 0 else 33.3
        print(f"  帧间隔均 {mean_ms:.1f}ms ± {std_ms:.1f}ms, max gap {max_ms:.1f}ms (目标 {target_ms:.1f}ms @ hz={args.hz})", flush=True)
        if max_ms > 5 * target_ms:
            print(f"  ⚠️  大 gap >5x 目标,WS push 卡顿或 arm_queue 阻塞", flush=True)
        elif std_ms > 0.5 * target_ms:
            print(f"  ⚠️  std 太大 (抖动 > 50% 均值),推流不连续", flush=True)
    print(f"========================================\n", flush=True)


def _dump_trace_jsonl(trace, path: str) -> None:
    """把 trace 落盘为 jsonl, 每行一条 ServoTrace."""
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# ServoTrace dump, n={len(trace)}\n")
        for t in trace:
            row = {
                "t_s": round(t.t_s, 4),
                "iteration": t.iteration,
                "dx_norm": round(t.dx_norm, 4),
                "dy_norm": round(t.dy_norm, 4),
                "x_mm": round(t.x_mm, 2),
                "y_mm": round(t.y_mm, 2),
                "score": round(t.score, 3),
                "selected_track_id": t.selected_track_id,
                "is_miss": t.is_miss,
            }
            f.write(json.dumps(row, ensure_ascii=False) + "\n")
    print(f"  trace -> {path}", flush=True)


# ---------- main ----------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--timeout", type=float, default=TIMEOUT_DEFAULT,
                    help=f"持续追踪时长 s (默认 {TIMEOUT_DEFAULT})")
    ap.add_argument("--y-start", type=float, default=Y_START_DEFAULT,
                    help=f"起始 y mm (默认 {Y_START_DEFAULT})")
    ap.add_argument("--x-start", type=float, default=X_START_DEFAULT,
                    help=f"起始 x mm (默认 {X_START_DEFAULT})")
    ap.add_argument("--hz", type=float, default=HZ_DEFAULT,
                    help=f"WS push 频率 (默认 {HZ_DEFAULT})")
    ap.add_argument("--label", default=Label.BALL_YELLOW,
                    help=f"追踪目标 label (默认 {Label.BALL_YELLOW.value})")
    ap.add_argument("--no-reset", action="store_true",
                    help="跑完不复位")
    ap.add_argument("--negate", action="store_true",
                    help="翻转 mm_per_norm 符号 (相机装反/镜像时使用)")
    args = ap.parse_args()

    # 方向约定 —— 默认正向; --negate 时翻号 (适配不同相机 mount)
    mm_per_norm_eff = -30.0 if args.negate else 30.0
    sign_label = "NEGATED" if args.negate else "default"

    print(f"server: {os.environ.get('RAK_CAR_API_BASE', 'default')}", flush=True)
    print(f"timeout={args.timeout}s x_start={args.x_start} y_start={args.y_start} hz={args.hz} label={args.label} sign={sign_label}", flush=True)

    client = ArmClient.connect()
    _verify_runtime(client)

    # 0. 大臂伸到 0 + xy 起始位
    try:
        _move_safe(client, x_mm=args.x_start, y_mm=args.y_start)
    except (RuntimeError, ValueError) as e:
        print(f"🔴 起始安全位下发失败: {e}", flush=True)
        return 1

    # 0.5 让位: 停 arm_feed 释放 arm_queue (track 前)
    _section("让位: stop_arm_feed (释放 arm_queue)")
    _set_arm_feed(client, stop=True)

    # 1. 持续追踪
    _section(f"track_vision_target({args.label}) | timeout={args.timeout}s hz={args.hz}")
    runner = ArmRunner(client)
    selector = TargetSelector.for_label(args.label)

    # 关键: 用 _make_vision_with_move() 让 find_target_track 自动跑软限位网
    vision = client._make_vision_with_move()
    t0 = time.time()
    try:
        result = vision.find_target_track(
            selector,
            x_mm=args.x_start, y_mm=args.y_start,
            hz=args.hz,
            mm_per_norm=mm_per_norm_eff,
            timeout=args.timeout,
            target_real_height_m=TARGET_REAL_HEIGHT_M,
            focal_length_px=600.0,
            kp=1.0, ki=0.05, kd=0.2,
            settle_tol_norm=0.05,
        )
        elapsed = time.time() - t0
    except RuntimeError as e:
        elapsed = time.time() - t0
        print(f"🔴 追踪异常: {e}", flush=True)
        if not args.no_reset:
            _move_safe(client, x_mm=args.x_start, y_mm=args.y_start)
        _set_arm_feed(client, stop=False)  # 恢复 arm_feed
        return 2

    # 2. 汇总 + 落盘
    summary = _summarize_trace(result.trace)
    _print_summary(summary, elapsed_s=elapsed, label=str(args.label))

    out_path = f"track_trace_{_now_utc()}.jsonl"
    _dump_trace_jsonl(result.trace, out_path)

    # 3. 复位 (除非 --no-reset)
    if not args.no_reset:
        _section("复位: composite_run -> 起始位")
        try:
            _move_safe(client, x_mm=args.x_start, y_mm=args.y_start)
        except Exception as e:
            print(f"⚠️ 复位失败 (非致命): {e}", flush=True)

    # 4. 恢复 arm_feed (track 后)
    _section("恢复: start_arm_feed (20Hz)")
    _set_arm_feed(client, stop=False)

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⌨️ Ctrl-C, 退出前尝试复位...", flush=True)
        try:
            client = ArmClient.connect()
            client.composite_run(
                arm=ARM_ANGLE_DEFAULT, x_mm=X_START_DEFAULT,
                y_mm=Y_START_DEFAULT, hand=HAND_DEFAULT, timeout=10.0
            )
            print("复位成功", flush=True)
        except Exception:
            pass
        sys.exit(130)
