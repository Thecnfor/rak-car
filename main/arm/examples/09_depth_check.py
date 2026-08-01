"""main/arm/examples/09_depth_check.py

2026-08-01 快速真机验证 — depth-aware 增益复活检查.

修复内容: task_feed 缓存带 frame_shape → main/arm 端 _parse_cache 由 bbox_norm
自算 bbox_pixels → find_target_pid 的 depth-aware 分支真正生效 (原为死代码).

用法 (默认只读, 不动机械臂):
    /usr/bin/python3 main/arm/examples/09_depth_check.py
    /usr/bin/python3 main/arm/examples/09_depth_check.py --height-m 0.06
    /usr/bin/python3 main/arm/examples/09_depth_check.py --snap      # 对比 runtime 同步 bbox_pixels
    /usr/bin/python3 main/arm/examples/09_depth_check.py --servo-test --label ball_yellow --height-m 0.06

判定标准:
  [PASS] task_state.frame_shape 非 None          → runtime 已重启, 修复生效
  [PASS] get_state() 的 bbox_pixels 非 None      → client 端自算正确
  [PASS] compute_depth 输出与目标实际距离同量级
  [FAIL] frame_shape None                        → 旧进程, 需 `pm2 restart rak-car-api`
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.api_client import RuntimeApiClient
from main.arm.vision import ArmVisionClient, TargetSelector


def _verdict(ok: bool, text: str) -> None:
    tag = "[PASS]" if ok else "[FAIL]"
    print(f"  {tag} {text}", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--height-m", type=float, default=0.06,
                    help="目标真实高度 (m), 用于 compute_depth 反算距离")
    ap.add_argument("--focal", type=float, default=600.0)
    ap.add_argument("--snap", action="store_true", help="额外走一次同步 snap() 对比 bbox_pixels")
    ap.add_argument("--servo-test", action="store_true",
                    help="⚠️ 会移动机械臂: 短时 find_target_pid 验证 depth 增益 (y=-100mm 保护区外)")
    ap.add_argument("--label", default="ball_yellow")
    ap.add_argument("--timeout", type=float, default=6.0)
    args = ap.parse_args()

    http = RuntimeApiClient()
    print(f"server: {http.api_base}", flush=True)

    # ---- [1] task_state 是否带 frame_shape ----
    print("\n=== [1] task_feed cache: frame_shape ===", flush=True)
    raw = http.get_vision_task_cache()
    state = raw.get("task_state") or {}
    frame_shape = state.get("frame_shape")
    dets_raw = state.get("detections") or []
    print(f"  active={state.get('active')} mode={state.get('mode')} "
          f"frame_shape={frame_shape} detections={state.get('count')}", flush=True)
    _verdict(frame_shape is not None,
             "frame_shape 已带上" if frame_shape is not None
             else "frame_shape=None — runtime 未重启 (旧进程), 需 `pm2 restart rak-car-api`")
    if frame_shape is None:
        print("  继续用 client 侧验证降级路径 (bbox_pixels 应为 None, 深度走 fallback).", flush=True)

    # ---- [2] client 端自算 bbox_pixels ----
    print("\n=== [2] ArmVisionClient.get_state() → bbox_pixels ===", flush=True)
    client = ArmVisionClient(http)
    dets = client.get_state()
    print(f"  {len(dets)} 个检测 (frame_shape={frame_shape})", flush=True)
    if not dets:
        _verdict(False, "无检测 — 相机没照到目标? (检查 cam2 是否上电/对焦)")
        print("  可通过 --snap 或检查 /stream/ 看画面确认.", flush=True)
    else:
        any_px = False
        for d in dets:
            bp = d.bbox_pixels
            depth = client.compute_depth(bp, args.height_m, args.focal) if bp else None
            px_txt = f"bbox_pixels(h={bp.height}px)" if bp else "bbox_pixels=None"
            depth_txt = f"depth≈{depth:.2f}m" if depth else "depth=fallback(0.30m)"
            print(f"  {d.label}#{d.track_id} score={d.score:.2f} "
                  f"norm=({d.bbox_norm.x_center:+.3f},{d.bbox_norm.y_center:+.3f}) "
                  f"{px_txt} {depth_txt}", flush=True)
            any_px = any_px or (bp is not None)
        _verdict(any_px,
                 "bbox_pixels 已自算填充 → depth-aware 可用" if any_px
                 else "bbox_pixels=None — frame_shape 缺失, depth 仍 fallback")

    # ---- [3] snap() 同步对比 (runtime 官方 bbox_pixels vs client 自算) ----
    if args.snap:
        print("\n=== [3] snap() 同步对比 (runtime 官方 vs client 自算) ===", flush=True)
        sync = http.request_vision_task(timeout=15.0)
        print(f"  POST /v1/vision/task → {len(sync.get('detections') or [])} 个, "
              f"frame_shape={sync.get('frame_shape')}", flush=True)
        for d in sync.get("detections") or []:
            bp = d.get("bbox_pixels") or {}
            print(f"  {d.get('label')} runtime px: w={bp.get('width')} h={bp.get('height')} "
                  f"(x1={bp.get('x1')} y1={bp.get('y1')})", flush=True)

    # ---- [4] 伺服 depth 增益验证 (会动机械臂!) ----
    if args.servo_test:
        print("\n=== [4] find_target_pid depth 增益 (机械臂会动!) ===", flush=True)
        sel = TargetSelector.for_label(args.label)
        result = client.find_target_pid(
            sel, x_mm=0.0, y_mm=-100.0,
            target_real_height_m=args.height_m,
            focal_length_px=args.focal,
            ref_depth_m=0.30,
            mm_per_norm_base=30.0,
            kp=1.0, ki=0.0, kd=0.0,
            settle_tol_norm=0.03,
            settle_stable_frames=99,   # 不收敛停, 只看前几步步长
            min_step_mm=0.5,
            timeout=args.timeout, max_iter=200,
        )
        print(f"  converged={result.converged} iterations={result.iterations} "
              f"elapsed={result.elapsed_s:.2f}s", flush=True)
        if result.trace:
            print("  前 5 步 (x_mm 步长):", flush=True)
            for t in result.trace[:5]:
                # bbox 越近 → 步长越大, 可对比高度/深度输出
                print(f"    iter={t.iteration} dx_norm={t.dx_norm:+.3f} "
                      f"x_mm={t.x_mm:+8.1f} y_mm={t.y_mm:+8.1f}", flush=True)
            step = abs(result.trace[0].x_mm - 0.0) if result.trace else 0.0
            print(f"  首步 |Δx| = {step:.1f}mm", flush=True)
            if frame_shape is not None and args.height_m > 0:
                _verdict(True, "depth-aware 路径已激活 (frame_shape 存在, bbox_pixels 参与增益)")
            else:
                print("  [INFO] 无 frame_shape → 走固定增益 30mm/单位, 属预期降级.", flush=True)

    print("\n完成。", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⌘ 中断", flush=True)
        sys.exit(130)
