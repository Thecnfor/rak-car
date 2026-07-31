"""真机视觉伺服 smoke 测试（main/arm/examples/05_visual_servo_smoke.py）

TP1: cache 读取（task_feed 30Hz）
TP2: 同步 snap（含 bbox_pixels）
TP3: 视觉伺服到指定 label（动臂微调）
TP4: composite_run 4 路真并行

用法：
    export RAK_CAR_API_BASE=http://192.168.5.230:5050
    python3 main/arm/examples/05_visual_servo_smoke.py
"""
from __future__ import annotations

import os
import sys
import time

from main.arm import ArmClient, ArmRunner, TargetSelector, Label


def _section(title: str):
    print(f"\n=== {title} ===")


def main() -> int:
    print(f"server: {os.environ.get('RAK_CAR_API_BASE', 'default')}")
    client = ArmClient.connect()
    if not client.ping():
        print("ERROR: runtime not reachable")
        return 1

    # TP1: cache read
    _section("TP1: cache read (task_feed 30Hz)")
    dets = client.vision.get_state()
    print(f"get_state returned {len(dets)} detections")
    for d in dets[:5]:
        print(f"  {d}")

    # TP2: snap
    _section("TP2: snap (POST /v1/vision/task)")
    dets = client.vision.snap(timeout=10)
    print(f"snap returned {len(dets)} detections")
    for d in dets[:5]:
        print(f"  {d}")
        if d.bbox_pixels:
            print(f"    bbox_pixels: x1={d.bbox_pixels.x1} y1={d.bbox_pixels.y1} "
                  f"w={d.bbox_pixels.width} h={d.bbox_pixels.height}")

    # TP4: composite_run 4 路真并行
    _section("TP4: composite_run 4 路真并行")
    print("目标: arm=0, x=0, y=-150mm, hand=UP(-90) [安全位]")
    t0 = time.time()
    result = client.composite_run(
        arm=0.0, x_mm=0.0, y_mm=-150.0, hand=-90.0, timeout=15.0
    )
    elapsed = time.time() - t0
    print(f"result: ok={result.get('result', {}).get('ok')}, "
          f"steps={result.get('result', {}).get('steps')}, "
          f"elapsed={elapsed:.2f}s")

    # TP3: 视觉伺服
    _section("TP3: 视觉伺服（visual servo）")
    runner = ArmRunner(client)
    print("目标 label: cylinder_1（如果不在视野内会 RuntimeError —— 正确的 fail-fast）")
    try:
        t0 = time.time()
        result = runner.move_to_vision_target(
            TargetSelector.for_label(Label.CYLINDER_1, strategy="highest_score"),
            x_mm=0.0, y_mm=-150.0, arm_angle=0.0, hand=-90.0,
            mm_per_norm=20.0, settle_tol_norm=0.08, timeout=8.0,
        )
        elapsed = time.time() - t0
        print(f"converged={result.converged} iters={result.iterations} "
              f"conf={result.confidence:.2f} elapsed={elapsed:.2f}s")
        print(f"final: x={result.x_mm:.1f}mm y={result.y_mm:.1f}mm")
        print(f"trace_len={len(result.trace)}")
    except RuntimeError as e:
        print(f"Vision servo 失败: {e}")
        print("（正常情况：如果目标不在视野，5 帧未命中 raise）")

    # 复位
    _section("复位到 (0, -150, arm=90, hand=UP)")
    client.composite_run(arm=90.0, x_mm=0.0, y_mm=-150.0, hand=-90.0, timeout=15.0)
    print("复位完成")

    return 0


if __name__ == "__main__":
    sys.exit(main())