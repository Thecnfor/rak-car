"""main/arm/examples/11_grasp.py

完整抓取流程真机测试（可重复跑）。

流程 (用户约定 2026-08-01):
  1. composite_run 到基准位 (arm=-90, x=0, y=-100, hand=0)
  2. find_target 视觉伺服, 把目标对准吸嘴 setpoint (arm_origin.yaml 注入);
     lock_first=True 锁定首个检测目标 (多目标场景防来回跳)
  3. move_y 下降 lower_mm (开环, 中途丢目标没关系)
  4. grasp(True) 吸气 + hold
  5. lift_back 抬回原 y

用法:
  export RAK_CAR_API_BASE=http://192.168.5.230:5050
  /usr/bin/python3 main/arm/examples/11_grasp.py                          # 抓 cylinder_1
  /usr/bin/python3 main/arm/examples/11_grasp.py --label cylinder_3 --lower 80
  /usr/bin/python3 main/arm/examples/11_grasp.py --list                   # 只列当前检测, 不动机械臂

注意:
  - 机械臂会动 + 吸气; 跑之前确认目标在吸嘴可达范围内。
  - 别人在真机上测的时候别跑 (会和外部命令打架)。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.arm.api import ArmClient
from main.arm.loops.runner import ArmRunner
from main.arm.vision import TargetSelector, SelectionStrategy

REF_X_MM = 0.0
REF_Y_MM = -100.0
REF_ARM = -90.0
REF_HAND = 0.0


def _list_detections(client: ArmClient) -> None:
    dets = client.vision.get_state()
    if not dets:
        print("  [无检测]", flush=True)
        return
    for d in dets:
        print(f"  {d.label:<14} score={d.score:.2f} det_id={d.track_id} "
              f"cx={d.bbox_norm.x_center:+.3f} cy={d.bbox_norm.y_center:+.3f}", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--label", default="cylinder_1", help="抓取目标 label")
    ap.add_argument("--x", type=float, default=REF_X_MM, help="粗定位 x (mm)")
    ap.add_argument("--y", type=float, default=REF_Y_MM, help="粗定位 y (mm)")
    ap.add_argument("--arm", type=float, default=REF_ARM, help="大臂角度")
    ap.add_argument("--hand", type=float, default=REF_HAND, help="手抓角度")
    ap.add_argument("--grasp-y", type=float, default=0.0,
                    help="抓取位 y (mm), 协议默认降到 0 才吸")
    ap.add_argument("--timeout", type=float, default=10.0, help="伺服超时 (s)")
    ap.add_argument("--hold", type=float, default=0.6, help="吸气保持 (s)")
    ap.add_argument("--no-lift", action="store_true", help="抓完不抬回")
    ap.add_argument("--no-lock", action="store_true", help="不锁定首个目标 (默认锁定)")
    ap.add_argument("--reposition", action="store_true",
                    help="先 composite_run 到基准位再抓 (默认 False: 用当前位姿直接抓)")
    ap.add_argument("--no-align", action="store_true",
                    help="跳过视觉伺服, 直接用当前位姿下降抓 (目标已大致对准时)")
    ap.add_argument("--list", action="store_true", help="只列当前检测, 不动机械臂")
    args = ap.parse_args()

    client = ArmClient.connect()
    runner = ArmRunner(client)

    if args.list:
        print(f"server: {client.http.api_base}", flush=True)
        _list_detections(client)
        return 0

    sel = TargetSelector.for_label(
        args.label,
        strategy=SelectionStrategy.HIGHEST_SCORE.value,  # runner 会按 lock_first 升级
    )
    print(f"server: {client.http.api_base}  label={args.label}  "
          f"setpoint=({client.origin.nozzle_offset_x_norm:+.4f}, "
          f"{client.origin.nozzle_offset_y_norm:+.4f})", flush=True)
    print(f"粗定位: arm={args.arm} x={args.x} y={args.y} hand={args.hand}  "
          f"抓取位 y={args.grasp_y}mm", flush=True)
    print("起始姿态:", client.get_state().describe(), flush=True)

    result = runner.pick_by_vision_lower(
        sel,
        x_mm=args.x, y_mm=args.y,
        arm_angle=args.arm, hand=args.hand,
        grasp_y_mm=args.grasp_y,
        settle_tol_norm=0.05, timeout=args.timeout,
        hold_s=args.hold,
        lift_back=not args.no_lift,
        lock_first=not args.no_lock,
        reposition=args.reposition,
        align=not args.no_align,
    )

    sv = result.get("servo")
    print("\n=== 抓取结果 ===", flush=True)
    print(f"ok    : {result.get('ok')}", flush=True)
    print(f"reason: {result.get('reason')}", flush=True)
    print(f"steps : {result.get('steps')}", flush=True)
    if sv is not None:
        print(f"servo : converged={sv.converged} iter={sv.iterations} "
              f"elapsed={sv.elapsed_s:.2f}s settle={sv.settle_stable}", flush=True)
    print(f"y     : {result.get('y_before')} -> 下降{result.get('y_lower')}", flush=True)
    print("最终姿态:", client.get_state().describe(), flush=True)
    print(f"\n{'✅ 抓取成功' if result.get('ok') else '❌ 抓取未成功'}", flush=True)
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⌘ 中断, 急停...", flush=True)
        try:
            ArmClient.connect().grasp(False)
        except Exception:
            pass
        sys.exit(130)
