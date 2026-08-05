#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/arm/examples/13_nozzle_align_pose_p.py

task4 P 姿态下的吸嘴-相机偏移标定。

背景:
  10_nozzle_align.py 在基准位 arm=-90°, x=0, y=-100 标定 nozzle_offset,
  但 task4 在 P 姿态 arm=+90°, x=-300, y=-120 抓球 — 大臂转了 180°,
  相机视角镜像, 吸嘴在画面里的位置也翻了。沿用 10 的标定 → task4
  find_target_arm_cross 把球拉到"画面中心"而不是"吸嘴正下方", 抓球偏。

基准位 (用户约定 2026-08-05):
  x=-300, y=-120, 大臂=+90°, 手抓=+10°  ← P 姿态 (Pose-P)

用法:
  export RAK_CAR_API_BASE=http://192.168.5.230:5050
  /usr/bin/python3 main/arm/examples/13_nozzle_align_pose_p.py             # 只测坐标
  /usr/bin/python3 main/arm/examples/13_nozzle_align_pose_p.py --save       # 写回 arm_origin.yaml
  /usr/bin/python3 main/arm/examples/13_nozzle_align_pose_p.py --servo-test --save
"""
from __future__ import annotations

import argparse
import datetime
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.arm.api import ArmClient
from main.arm.vision import ArmVisionClient, TargetSelector

# P 姿态 (Pose-P, task4 单一真相源, 与 main/arm/each_task/common.py 同步)
REF_X_MM = -300.0
REF_Y_MM = -120.0
REF_ARM = 90.0
REF_HAND = 10.0


def _verdict(ok: bool, text: str) -> None:
    tag = "[PASS]" if ok else "[FAIL]"
    print(f"  {tag} {text}", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--label", default=None,
                    help="限定目标 label; 默认所有 ball_* / 第一个检测")
    ap.add_argument("--samples", type=int, default=15,
                    help="采样帧数 (P 姿态标定建议多采, 滤掉 jitter)")
    ap.add_argument("--interval", type=float, default=0.3,
                    help="采样间隔 (s)")
    ap.add_argument("--save", action="store_true",
                    help="把测量值写入 arm_origin.yaml (nozzle_offset_map)")
    ap.add_argument("--global-save", action="store_true",
                    help="写入全局 nozzle_offset_x/y_norm 而非 per-label map")
    ap.add_argument("--servo-test", action="store_true",
                    help="⚠️ 会动臂: 用标定 setpoint 短时伺服验证")
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="servo-test 超时")
    args = ap.parse_args()

    client = ArmClient.connect()
    print(f"server: {client.http.api_base}", flush=True)

    # ---- [1] 摆 P 姿态 ----
    print(f"\n=== [1] composite_run 到 P 姿态 "
          f"arm={REF_ARM} x={REF_X_MM} y={REF_Y_MM} hand={REF_HAND} ===",
          flush=True)
    st = client.get_state()
    print(f"  move 前: x={st.x_mm:.1f} y={st.y_mm:.1f} "
          f"arm={st.arm_angle} hand={st.hand_angle}", flush=True)
    r = client.composite_run(
        arm=REF_ARM, x_mm=REF_X_MM, y_mm=REF_Y_MM, hand=REF_HAND,
        timeout=30.0)
    print(f"  steps: {r.get('job', {}).get('result', {}).get('steps')}",
          flush=True)
    st = client.get_state()
    print(f"  move 后: x={st.x_mm:.1f} y={st.y_mm:.1f} "
          f"arm={st.arm_angle} hand={st.hand_angle}", flush=True)
    if abs(st.x_mm - REF_X_MM) > 5 or abs(st.y_mm - REF_Y_MM) > 5:
        print("  ⚠️ P 姿态偏差较大, 检查机械 / 复位后重试。", flush=True)

    # ---- [2] 用户手动把球放到吸嘴正下方 ----
    print(f"\n=== [2] ⚠️ 现在请手动把 ball 放到吸嘴正下方 ===", flush=True)
    print(f"  球就位后按 Enter 继续采样 (15 帧 ~ {15 * 0.3:.1f}s):")
    try:
        input("  > ")
    except (EOFError, KeyboardInterrupt):
        print("  取消。", flush=True)
        return 130

    # ---- [3] 采样目标坐标 ----
    print(f"\n=== [3] 采样目标 bbox 中心 ({args.samples} 帧) ===", flush=True)
    vision = ArmVisionClient(client.http)
    samples = []
    sel = (TargetSelector.for_label(args.label)
           if args.label else TargetSelector())
    for i in range(args.samples):
        dets = vision.get_state_filtered(sel)
        if dets:
            pick = max(dets, key=lambda d: d.score)
            samples.append((pick.label, pick.score,
                            pick.bbox_norm.x_center, pick.bbox_norm.y_center,
                            pick.bbox_norm.width, pick.bbox_norm.height))
            print(f"  t{i}: {pick.label}[{pick.score:.2f}] "
                  f"cx={pick.bbox_norm.x_center:+.4f} "
                  f"cy={pick.bbox_norm.y_center:+.4f}", flush=True)
        else:
            print(f"  t{i}: 无检测 (miss)", flush=True)
        time.sleep(args.interval)

    if not samples:
        _verdict(False, "全程无检测 — 目标不在相机视野? "
                        "检查 cam2 画面 / 对焦 / label。")
        return 1

    xs = [s[2] for s in samples]
    ys = [s[3] for s in samples]
    sx, sy = sum(xs) / len(xs), sum(ys) / len(ys)
    jx, jy = max(xs) - min(xs), max(ys) - min(ys)
    labels_seen = {s[0] for s in samples}
    print(f"\n  setpoint_x_norm = {sx:+.4f}  setpoint_y_norm = {sy:+.4f}")
    print(f"  x 抖动 ±{jx/2:.4f}  y 抖动 ±{jy/2:.4f}  label={labels_seen}")
    _verdict(jx <= 0.03 and jy <= 0.03,
             f"采样稳定 (抖动 ≤0.03); label 集合 {labels_seen}")
    _verdict(len(labels_seen) == 1, "目标唯一")

    # ---- [4] 写回 origin ----
    if args.save:
        origin = client.origin or client._load_origin_or_default()
        if args.global_save:
            origin.nozzle_offset_x_norm = sx
            origin.nozzle_offset_y_norm = sy
            origin.calibrated_at = datetime.datetime.now().isoformat()
            client.save_origin(origin)
            print(f"\n  ✅ 已写入全局: nozzle_offset_x_norm={sx:.4f} "
                  f"nozzle_offset_y_norm={sy:.4f}", flush=True)
        else:
            # per-label map (2026-08-02 起 task4 ball_yellow / ball_blue 分开标定)
            for label in labels_seen:
                origin.nozzle_offset_map[label] = (sx, sy)
            # 同步全局兜底 (未知 label 走这里)
            origin.nozzle_offset_x_norm = sx
            origin.nozzle_offset_y_norm = sy
            origin.calibrated_at = datetime.datetime.now().isoformat()
            client.save_origin(origin)
            print(f"\n  ✅ 已写入 nozzle_offset_map[{labels_seen}]="
                  f"({sx:.4f}, {sy:.4f})", flush=True)
            print(f"     同步全局 nozzle_offset=({sx:.4f}, {sy:.4f})",
                  flush=True)

    # ---- [5] 伺服验证 ----
    if args.servo_test:
        print("\n=== [5] find_target_pid 用 setpoint 验证 "
              "(球已在吸嘴下, 应近零移动) ===", flush=True)
        print("  ⚠️ 臂会动! 若方向/增益异常按 Ctrl-C 立即停。", flush=True)
        pick_sel = (TargetSelector.for_label(next(iter(labels_seen)))
                    if labels_seen else TargetSelector())
        finder = client._make_vision_with_move()
        result = finder.find_target(
            pick_sel,
            x_mm=REF_X_MM, y_mm=REF_Y_MM,
            setpoint_x_norm=sx, setpoint_y_norm=sy,
            mm_per_norm=30.0,
            kp=1.0, ki=0.0, kd=0.0,
            settle_tol_norm=0.05, settle_stable_frames=1,
            min_step_mm=1.0, timeout=args.timeout, max_iter=200,
        )
        print(f"  converged={result.converged} iterations={result.iterations} "
              f"elapsed={result.elapsed_s:.2f}s "
              f"x_mm={result.x_mm:.1f} y_mm={result.y_mm:.1f}", flush=True)
        drift = max(abs(result.x_mm - REF_X_MM),
                    abs(result.y_mm - REF_Y_MM))
        _verdict(drift <= 15.0,
                 f"最终位置漂移 {drift:.1f}mm "
                 f"(≤15mm 即 setpoint 收敛正确)")
        if result.trace and len(result.trace) > 1:
            print("  前 3 步轨迹:", flush=True)
            for t in result.trace[:3]:
                print(f"    iter={t.iteration} dx_norm={t.dx_norm:+.3f} "
                      f"x_mm={t.x_mm:+8.1f} y_mm={t.y_mm:+8.1f}",
                      flush=True)

    print("\n完成。", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[中断]", flush=True)
        sys.exit(130)