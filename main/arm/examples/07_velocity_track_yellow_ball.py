"""main/arm/examples/07_velocity_track_yellow_ball.py

velocity-mode 实时追踪 (IBVS 速度模式) —— 绕开 arm_queue。

背景 (2026-08-01):
  find_target_track 每帧发 HTTP goto_position 进 arm_queue, 位置闭环 ~500ms/次,
  视觉 ~8Hz, 命令积压 100+, 用户观察"停下来还在乱跑"。治本: 不走位置队列,
  用实时速度命令 (x_speed / y_speed) 直发 —— 像底盘 set_chassis_velocity 一样。

本脚本是 thin wrapper: 真正逻辑在
  ArmRunner.track_velocity → ArmVisionClient.find_target_velocity
(VelocityLoop, main/arm/vision/velocity.py)。
它只动 xy 十字; 大臂/手爪增量联调用示例 08。

安全:
  - y 有磁感安全门 + 末段/顶段减速 (arm_base.y_speed 内置)
  - x 无软限位, 但 gain 限速 + 检测丢失即停; 跑偏时 Ctrl-C 立即发 0
  - 结束 (含异常) 必然 x_vel=0 y_vel=0 (封装内 finally 保证)

用法:
    export RAK_CAR_API_BASE=http://192.168.5.230:5050
    /usr/bin/python3 main/arm/examples/07_velocity_track_yellow_ball.py
    /usr/bin/python3 main/arm/examples/07_velocity_track_yellow_ball.py --gain 0.08 --timeout 20
    /usr/bin/python3 main/arm/examples/07_velocity_track_yellow_ball.py --label h_jin_zhen_gu
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.arm import ArmClient, ArmRunner

GAIN_DEFAULT = 0.05
DEADZONE_NORM = 0.02


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--gain", type=float, default=GAIN_DEFAULT)
    ap.add_argument("--deadzone", type=float, default=DEADZONE_NORM)
    ap.add_argument("--label", default="ball_yellow")
    ap.add_argument("--x-start", type=float, default=0.0)
    ap.add_argument("--y-start", type=float, default=-130.0)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--negate-x", action="store_true", help="翻转 x 速度符号")
    ap.add_argument("--negate-y", action="store_true", help="翻转 y 速度符号")
    ap.add_argument("--no-reset", action="store_true")
    args = ap.parse_args()

    runner = ArmRunner(ArmClient.connect())
    print(f"track_velocity({args.label}) timeout={args.timeout}s gain={args.gain} "
          f"sign=({'-' if not args.negate_x else '+'}, {'+' if not args.negate_y else '-'})",
          flush=True)

    result = runner.track_velocity(
        args.label, timeout=args.timeout, hz=args.hz,
        gain=args.gain, deadzone=args.deadzone,
        x_start=args.x_start, y_start=args.y_start,
        sign_x=1.0 if args.negate_x else -1.0,
        sign_y=-1.0 if args.negate_y else 1.0,
        no_reset=args.no_reset,
    )
    print(result.summary(), flush=True)

    out_path = f"vel_trace_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.jsonl"
    with open(out_path, "w", encoding="utf-8") as f:
        for t in result.trace:
            f.write(json.dumps(dataclasses.asdict(t), ensure_ascii=False) + "\n")
    print(f"trace -> {out_path}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⌘ Ctrl-C, 急停由封装内 finally 保证 (x_vel=0 y_vel=0)", flush=True)
        sys.exit(130)
