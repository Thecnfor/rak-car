"""main/arm/examples/10_nozzle_align.py

吸嘴-相机偏移标定 + 对准验证。

背景 (2026-08-01):
  吸嘴和末端摄像头刚性绑定，相对位置不变（吸嘴在相机画面正中心上方几厘米）。
  视觉伺服 find_target_* 默认把目标对准画面正中心 (0,0)，但吸嘴对准目标时
  目标应出现在"吸嘴正下方"那个点 —— 即 (nozzle_offset_x_norm, nozzle_offset_y_norm)。
  本脚本在**基准位**把目标放到吸嘴正下方，采样其 bbox 中心坐标作为 setpoint。

基准位 (用户约定, 2026-08-01, 之后都以这个为基准):
  x=0, y=-100(mm), 大臂=-90°, 手抓=0°

流程:
  1. composite_run 到基准位
  2. 采样目标 bbox 中心 → 打印 setpoint (均值 + 抖动)
  3. --save   → 写入 arm_origin.yaml（nozzle_offset_x_norm / y_norm），之后
               pick_by_vision / track_vision_target 默认自动读这个偏移
  4. --servo-test → 短时 find_target_pid 验证：目标已在吸嘴正下方时，用刚标定的
               setpoint 伺服应近零移动 / 立即收敛（同时暴露方向符号问题）

用法:
  export RAK_CAR_API_BASE=http://192.168.5.230:5050
  /usr/bin/python3 main/arm/examples/10_nozzle_align.py                 # 只测坐标
  /usr/bin/python3 main/arm/examples/10_nozzle_align.py --save           # 写回 yaml
  /usr/bin/python3 main/arm/examples/10_nozzle_align.py --servo-test --save
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.arm.api import ArmClient
from main.arm.vision import ArmVisionClient, TargetSelector

# 基准位常量 (用户约定)
REF_X_MM = 0.0
REF_Y_MM = -100.0
REF_ARM = -90.0
REF_HAND = 0.0


def _verdict(ok: bool, text: str) -> None:
    tag = "[PASS]" if ok else "[FAIL]"
    print(f"  {tag} {text}", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--label", default=None, help="限定目标 label；默认取第一个检测")
    ap.add_argument("--samples", type=int, default=10, help="采样帧数")
    ap.add_argument("--interval", type=float, default=0.3, help="采样间隔 (s)")
    ap.add_argument("--save", action="store_true", help="把测量值写入 arm_origin.yaml")
    ap.add_argument("--servo-test", action="store_true",
                    help="⚠️ 会动机械臂: 用标定 setpoint 短时伺服验证")
    ap.add_argument("--timeout", type=float, default=5.0, help="servo-test 超时")
    args = ap.parse_args()

    client = ArmClient.connect()
    print(f"server: {client.http.api_base}", flush=True)

    # ---- [1] 基准位 ----
    print(f"\n=== [1] composite_run 到基准位 arm={REF_ARM} x={REF_X_MM} y={REF_Y_MM} hand={REF_HAND} ===", flush=True)
    st = client.get_state()
    print(f"  move 前: x={st.x_mm:.1f} y={st.y_mm:.1f} arm={st.arm_angle} hand={st.hand_angle}", flush=True)
    r = client.composite_run(arm=REF_ARM, x_mm=REF_X_MM, y_mm=REF_Y_MM, hand=REF_HAND, timeout=30.0)
    print(f"  steps: {r.get('job', {}).get('result', {}).get('steps')}", flush=True)
    st = client.get_state()
    print(f"  move 后: x={st.x_mm:.1f} y={st.y_mm:.1f} arm={st.arm_angle} hand={st.hand_angle}", flush=True)
    if abs(st.x_mm - REF_X_MM) > 5 or abs(st.y_mm - REF_Y_MM) > 5:
        print("  ⚠️ 基准位偏差较大，检查机械 / 复位后重试。", flush=True)

    # ---- [2] 采样目标坐标 ----
    print(f"\n=== [2] 采样目标 bbox 中心 ({args.samples} 帧) ===", flush=True)
    vision = ArmVisionClient(client.http)
    samples = []
    sel = TargetSelector.for_label(args.label) if args.label else TargetSelector()
    for i in range(args.samples):
        dets = vision.get_state_filtered(sel)
        if dets:
            # 取 score 最高的（场景应只有一个目标在吸嘴正下方）
            pick = max(dets, key=lambda d: d.score)
            samples.append((pick.label, pick.score,
                            pick.bbox_norm.x_center, pick.bbox_norm.y_center,
                            pick.bbox_norm.width, pick.bbox_norm.height))
            print(f"  t{i}: {pick.label}[{pick.score:.2f}] cx={pick.bbox_norm.x_center:+.4f} "
                  f"cy={pick.bbox_norm.y_center:+.4f}", flush=True)
        else:
            print(f"  t{i}: 无检测 (miss)", flush=True)
        time.sleep(args.interval)

    if not samples:
        _verdict(False, "全程无检测 — 目标不在相机视野? 检查 cam2 画面 / 对焦 / label。")
        return 1
    xs = [s[2] for s in samples]
    ys = [s[3] for s in samples]
    sx, sy = sum(xs) / len(xs), sum(ys) / len(ys)
    jx, jy = max(xs) - min(xs), max(ys) - min(ys)
    labels = {s[0] for s in samples}
    print(f"\n  setpoint_x_norm = {sx:+.4f}  setpoint_y_norm = {sy:+.4f}")
    print(f"  x 抖动 ±{jx/2:.4f}  y 抖动 ±{jy/2:.4f}  label={labels}")
    _verdict(jx <= 0.02 and jy <= 0.02, f"采样稳定 (抖动 ≤0.02)；label 集合 {labels}")
    _verdict(len(labels) == 1, "目标唯一")

    # ---- [3] 写回 origin ----
    if args.save:
        origin = client.origin or client._load_origin_or_default()
        origin.nozzle_offset_x_norm = sx
        origin.nozzle_offset_y_norm = sy
        import datetime
        origin.calibrated_at = datetime.datetime.now().isoformat()
        client.save_origin(origin)
        print(f"\n  ✅ 已写入 arm_origin.yaml: nozzle_offset=({sx:.4f}, {sy:.4f})", flush=True)

    # ---- [4] 伺服验证 ----
    if args.servo_test:
        print("\n=== [4] find_target_pid 用 setpoint 验证 (目标已在吸嘴下, 应近零移动) ===", flush=True)
        print("  ⚠️ 机械臂会动! 若方向/增益异常按 Ctrl-C 立即停。", flush=True)
        pick_sel = TargetSelector.for_label(next(iter(labels))) if labels else TargetSelector()
        # 走 _make_vision_with_move() 安全 wrap (每个 move 带 _check_y_protected + _check_safe);
        # find_target 传 setpoint → 自动路由到 PID + mm_per_norm 桥.
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
              f"elapsed={result.elapsed_s:.2f}s x_mm={result.x_mm:.1f} y_mm={result.y_mm:.1f}", flush=True)
        # 目标在吸嘴下, 伺服应几乎不动: |最终位置-基准位| 应小
        drift = max(abs(result.x_mm - REF_X_MM), abs(result.y_mm - REF_Y_MM))
        _verdict(drift <= 15.0,
                 f"最终位置漂移 {drift:.1f}mm (≤15mm 即 setpoint 收敛正确；"
                 f">15mm 说明 setpoint 或方向符号有问题)")
        if result.trace and len(result.trace) > 1:
            print("  前 3 步轨迹:", flush=True)
            for t in result.trace[:3]:
                print(f"    iter={t.iteration} dx_norm={t.dx_norm:+.3f} "
                      f"x_mm={t.x_mm:+8.1f} y_mm={t.y_mm:+8.1f}", flush=True)

    print("\n完成。", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⌘ 中断", flush=True)
        sys.exit(130)
