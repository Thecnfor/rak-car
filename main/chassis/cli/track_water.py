"""main/chassis/cli/track_water.py
真机 water 追踪：两步式(先 dry 看视觉 + 方向检查,再真发车)。

直接执行:
    RAK_CAR_SERVER_ORIGIN=http://192.168.6.231:5050 \
    /usr/bin/python3 -m main.chassis.cli.track_water
"""
from __future__ import annotations

import sys
import time
from typing import List, Tuple

from main.chassis import track_chassis, TrackChassisResult, track_trace, TrackFrame


# ------------ 1. 先 dry_run 5 秒, 纯视觉看 ------------


def run_dry_check(seconds: float = 5.0) -> Tuple[bool, List[str]]:
    """dry_run=True 只看视觉,不下发轮速。返回 (看到过water, 关键信息行)。"""
    print("=" * 64)
    print("[STEP 1] 视觉 dry-run %.1fs (不发车, 只看 camera 是否看得到 water)" % seconds)
    print("=" * 64)
    # on_tick 每 2 帧打一行 + 做摘要
    summary: List[str] = []
    first_found: list = []
    in_band_ever = [0]
    max_err = [0.0, 0.0]  # cx_abs, cy_abs
    label_count: dict = {}

    def _on_tick(frm: TrackFrame, wheels_v):
        vx, vy = wheels_v
        if frm.target_found:
            if not first_found:
                first_found.append(time.monotonic())
                summary.append(
                    "  ✓ 首次看到 water: label=%s  cx=%.3f cy=%.3f score=%.2f"
                    % (frm.label, frm.cx or 0, frm.cy or 0, frm.score or 0)
                )
            lb = frm.label or "<unknown>"
            label_count[lb] = label_count.get(lb, 0) + 1
            if frm.cx is not None:
                e = abs(frm.cx)
                if e > max_err[0]:
                    max_err[0] = e
            if frm.cy is not None:
                e = abs(frm.cy)
                if e > max_err[1]:
                    max_err[1] = e
            if frm.cx_err is not None and frm.cy_err is not None:
                if abs(frm.cx_err) < 0.08 and abs(frm.cy_err) < 0.08:
                    in_band_ever[0] += 1

    result = track_chassis(
        "water",
        on_tick=_on_tick,
        max_seconds=seconds,
        dry_run=True,
        kp=0.50, v_max=0.25, deadband=0.08, hold_frames=3,
        hz=20,
        watchdog_ms=3000.0,
        max_lost_frames=60,  # dry-run 阶段别因为 0.3s 丢帧就退,给 camera 一点时间
    )
    # 打印统计
    print()
    print("--- 视觉检查统计 ---")
    print("  总帧数 :", result.frames)
    if first_found:
        print("  首帧见 water 延迟: %.2fs" % (first_found[0] - (time.monotonic() - result.elapsed_s)))
    if label_count:
        print("  各 label 命中次数:", label_count)
    print("  |cx| 最大偏离 :", "%.3f" % max_err[0])
    print("  |cy| 最大偏离 :", "%.3f" % max_err[1])
    print("  曾在带内次数  :", in_band_ever[0])
    print("  result.arrived:", result.arrived, " reason:", result.reason)
    return bool(first_found), summary


def confirm_yes(prompt: str, default_yes: bool = False) -> bool:
    try:
        ans = input(prompt + (" [Y/n] " if default_yes else " [y/N] ")).strip().lower()
    except EOFError:
        return default_yes
    if ans in ("y", "yes"):
        return True
    if ans in ("n", "no"):
        return False
    return default_yes


# ------------ 2. 真发车追踪 10 秒 ------------


def run_real(seconds: float = 10.0) -> TrackChassisResult:
    print("=" * 64)
    print("[STEP 2] 真机 water 追踪 %.1fs (会真发车, v_max=0.25 m/s)" % seconds)
    print("  请确保车周围 1m 内无障碍物 + 摄像头朝 water")
    print("=" * 64)
    # 3 秒倒计时
    for i in (3, 2, 1):
        print("  %d ..." % i)
        time.sleep(1.0)
    print("  GO!")

    def _tick(frm, wheels_v):
        track_trace(every_n=1)(frm, wheels_v)

    result = track_chassis(
        "water",
        on_tick=_tick,
        max_seconds=seconds,
        dry_run=False,
        kp=0.50, v_max=0.25, deadband=0.08, hold_frames=3,
        hz=20,
        watchdog_ms=1500.0,
        max_lost_frames=10,  # 10 帧(≈0.5s)没 water 就停,别瞎走
    )
    print()
    print("--- 真机追踪结果 ---")
    print("  arrived  :", result.arrived)
    print("  reason   :", result.reason)
    print("  frames   :", result.frames)
    print("  elapsed  : %.2fs" % result.elapsed_s)
    if result.final_frame and result.final_frame.target_found:
        print("  final cx : %.3f  cy : %.3f" % (
            result.final_frame.cx or 0, result.final_frame.cy or 0
        ))
        print("  final cx_err: %+.3f  cy_err: %+.3f" % (
            result.final_frame.cx_err or 0, result.final_frame.cy_err or 0
        ))
        print("  label    :", result.final_frame.label,
              "  score: %.2f" % (result.final_frame.score or 0))
    else:
        print("  final    : 未在画面内看到 water")
    return result


def main():
    # Step 1: dry-run 检查
    found, summary = run_dry_check(5.0)
    for s in summary:
        print(s)
    if not found:
        print()
        print("  ✗ 5 秒内没看到 water。先检查:")
        print("   1. RAK_CAR_SERVER_ORIGIN 是否指向 runtime (默认 http://192.168.6.231:5050)")
        print("   2. 车/摄像头 是否朝 water?")
        print("   3. runtime 侧 task_detect 后端有没有起?")
        if not confirm_yes("  还是要继续真发? (大概率会原地停 10s)", default_yes=False):
            print("已退出。")
            sys.exit(1)
    else:
        # 方向判断
        if not confirm_yes(
            "  视觉 OK。方向对吗? water 在画面右边应该车向左移,在画面下边应该车后退。",
            default_yes=True,
        ):
            print("  请调整 setpoint_cxcy 或把摄像头方向对正 water,然后重跑。")
            sys.exit(2)

    # Step 2: 真跑 10 秒
    run_real(10.0)


if __name__ == "__main__":
    main()
