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
    y_vel     = +dy * gain_y              十字垂直跟   (2026-08-02 方向修正)
    arm_target += +dx * gain_arm         大臂水平转向
    hand_target += +dy * gain_hand       手抓垂直转向
  全部走 POST /v1/realtime/arm-velocity 一个端点 (免 arm_queue, 实时)。

角度软限位: arm ∈ [-90,+90], hand ∈ [-90,0]。检测丢失 → 全轴停 (xy 停, 角度不动)。

本脚本是 thin wrapper: 真正逻辑在
  ArmRunner.track_4dof → ArmVisionClient.find_target_4dof
(VelocityLoop, main/arm/vision/velocity.py)。

用法:
    export RAK_CAR_API_BASE=http://192.168.5.230:5050
    /usr/bin/python3 main/arm/examples/08_servo4_track.py --label h_tu_dou
    /usr/bin/python3 main/arm/examples/08_servo4_track.py --label animal --gain-arm 2.0 --gain-hand 1.0
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

GAIN_X_DEFAULT = 0.05
GAIN_Y_DEFAULT = 0.05
GAIN_ARM_DEFAULT = 2.0
GAIN_HAND_DEFAULT = 2.0
DEADZONE_NORM = 0.02


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--label", default="ball_yellow")
    ap.add_argument("--gain-x", type=float, default=GAIN_X_DEFAULT)
    ap.add_argument("--gain-y", type=float, default=GAIN_Y_DEFAULT)
    ap.add_argument("--gain-arm", type=float, default=GAIN_ARM_DEFAULT)
    ap.add_argument("--gain-hand", type=float, default=GAIN_HAND_DEFAULT)
    ap.add_argument("--deadzone", type=float, default=DEADZONE_NORM)
    ap.add_argument("--x-start", type=float, default=0.0)
    ap.add_argument("--y-start", type=float, default=-130.0)
    ap.add_argument("--arm-start", type=float, default=0.0)
    ap.add_argument("--hand-start", type=float, default=-90.0)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--no-reset", action="store_true")
    args = ap.parse_args()

    runner = ArmRunner(ArmClient.connect())
    print(f"track_4dof({args.label}) timeout={args.timeout}s "
          f"gains=(x:{args.gain_x}, y:{args.gain_y}, arm:{args.gain_arm}, hand:{args.gain_hand})",
          flush=True)

    result = runner.track_4dof(
        args.label, timeout=args.timeout, hz=args.hz,
        gain_x=args.gain_x, gain_y=args.gain_y,
        gain_arm=args.gain_arm, gain_hand=args.gain_hand,
        deadzone=args.deadzone,
        x_start=args.x_start, y_start=args.y_start,
        arm_start=args.arm_start, hand_start=args.hand_start,
        no_reset=args.no_reset,
    )
    print(result.summary(), flush=True)

    out_path = f"servo4_trace_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.jsonl"
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
