#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/arm/examples/15_creep_then_align_only.py

一次性脚本: 复刻 target4 的 creep + 见球 + track_chassis (底盘对齐) 阶段,
但**不**调 _pick_and_store (不抓球)。方便现场手动标定: 看底盘把球拉到画面
中心要多久、精度如何、对齐完之后你再决定下一步。

要求:
  - 已在 P 姿态 (跑过 14_goto_pose_p.py)
  - runtime 服务在 (Jetson 上 pm2 拉起)
  - 摄像头 / task_feed / arm_feed 都在

用法:
  export RAK_CAR_API_BASE=http://192.168.5.230:5050
  /usr/bin/python3 main/arm/examples/15_creep_then_align_only.py
  /usr/bin/python3 main/arm/examples/15_creep_then_align_only.py --creep-speed 0.03 --max-creep-m 0.4
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.api_client import RuntimeApiClient  # noqa: E402
from main.arm.each_task.task4.target2 import (  # noqa: E402
    fetch_balls,
)
from main.arm.each_task.task4.constants import (  # noqa: E402
    COLOR_BLUE, COLOR_YELLOW,
)
from main.arm.each_task.task4.target4 import (  # noqa: E402
    _CreepThread, _track_leftmost_ball,
    CREEP_POLL_HZ, BALL_LABELS,
)
from main.chassis import track_chassis  # noqa: E402


LOG_PREFIX = "[15_creep_then_align_only]"


def _set_chassis_vel(http, vx: float, vy: float = 0.0) -> None:
    try:
        http.post(
            "/v1/realtime/chassis-velocity",
            {"vx": float(vx), "vy": float(vy), "wz": 0.0},
            timeout=1.5,
        )
    except Exception as e:
        print(f"  {LOG_PREFIX} ⚠️ chassis 速度下发失败 "
              f"({type(e).__name__}: {str(e)[:60]})")


def main() -> int:
    p = argparse.ArgumentParser(
        description="creep + 见球 + 底盘对齐 (不 pick); 现场标定对齐效果用",
    )
    p.add_argument("--creep-speed", type=float, default=0.045,
                   help="creep 前移速度 (m/s, 默认 0.045)")
    p.add_argument("--max-creep-m", type=float, default=0.8,
                   help="creep 累计前移预算 (m, 默认 0.8)")
    p.add_argument("--max-seconds", type=float, default=60.0,
                   help="creep 总时间上限 (s, 默认 60)")
    p.add_argument("--track-max-seconds", type=float, default=6.0,
                   help="单球底盘伺服预算 (s, 默认 6)")
    args = p.parse_args()

    http = RuntimeApiClient()
    print(f"{LOG_PREFIX} 起步: 已假设在 P 姿态 (跑过 14_goto_pose_p.py)")
    print(f"  creep: {args.creep_speed} m/s × ≤{args.max_creep_m} m")
    print(f"  track 预算: {args.track_max_seconds} s (提速档 kp=0.40/v_max=0.25)")

    # ---- 1. CreepThread: 前移 + 见球即停 ----
    creep_thread = _CreepThread(
        http,
        speed_mps=args.creep_speed,
        max_distance_m=args.max_creep_m,
        poll_hz=CREEP_POLL_HZ,
    )
    creep_thread.start()
    print(f"\n{LOG_PREFIX} creep 启动 (后台 vx={args.creep_speed} m/s)")

    # 主线程空转, 等见球事件或超时
    t0 = time.monotonic()
    seen = False
    while time.monotonic() - t0 < args.max_seconds:
        if creep_thread.ball_event.wait(timeout=0.1):
            seen = True
            break
        if not creep_thread._thread.is_alive():
            break
    creep_thread.stop_and_join()
    total_creep_m = creep_thread.distance_m

    if not seen:
        print(f"\n{LOG_PREFIX} ❌ {args.max_seconds}s 内未见球, 收尾 (前移 {total_creep_m:.3f}m)")
        return 2

    print(f"\n{LOG_PREFIX} ✓ 见球 — 累计前移 {total_creep_m:.3f}m, 进入底盘对齐")
    balls_seen = creep_thread.balls or []
    print(f"  {LOG_PREFIX} balls_seen = {len(balls_seen)} 球")
    for i, b in enumerate(balls_seen):
        if isinstance(b, dict):
            print(f"    [{i}] color={b.get('color')} "
                  f"cx={b.get('cx_norm', 0):+.3f} cy={b.get('cy_norm', 0):+.3f} "
                  f"score={b.get('score', 0):.3f}")

    # ---- 2. 底盘对齐 (track_chassis 左球 → 画面中心) ----
    print(f"\n{LOG_PREFIX} 🎯 track_chassis(leftmost, ≤{args.track_max_seconds}s) — 提速档")
    t_track0 = time.monotonic()
    track_res = _track_leftmost_ball(
        max_seconds=args.track_max_seconds, dry_run=False,
    )
    track_elapsed = time.monotonic() - t_track0

    print(f"\n{LOG_PREFIX} ✅ 对齐结束")
    print(f"  arrived={track_res.arrived}  reason={track_res.reason}  "
          f"frames={track_res.frames}  elapsed={track_elapsed:.2f}s")
    ff = track_res.final_frame
    if ff is not None and getattr(ff, "target_found", False):
        print(f"  final_frame: label={getattr(ff, 'label', None)} "
              f"cx={getattr(ff, 'cx', 0):+.4f} cy={getattr(ff, 'cy', 0):+.4f} "
              f"score={getattr(ff, 'score', 0):.3f}")
        print(f"  cx_err={getattr(ff, 'cx_err', 0):+.4f}  "
              f"cy_err={getattr(ff, 'cy_err', 0):+.4f}  ← 接近 0 = 对齐精度")
    else:
        print(f"  ⚠️ final_frame 无目标 (对齐失败 / 已丢失)")

    print(f"\n{LOG_PREFIX} 🎯 此时可在画面里观察球是否落在画面中心。")
    print(f"{LOG_PREFIX} 不调 _pick_and_store (不抓球)。脚本结束, 臂保持在 P 姿态。")

    # 兜底底盘速度清零 (track_chassis 内部 finally 已经零速, 双保险)
    _set_chassis_vel(http, 0.0)
    return 0 if track_res.arrived else 1


if __name__ == "__main__":
    sys.exit(main())