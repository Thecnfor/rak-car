"""main/arm/examples/12_vision_pick_water.py

任务二"cam2 视觉抓水立方"单块真机测试 —— 参数都从 task_config.yml 的
water_tower_task.pick_vision 读, 现场只改配置文件 + 跑本脚本, 不重复传参.

流程 (与 main/task/task2_water_tower.py::_pick_cube 视觉路径完全一致):
  1. composite_run: 大臂 → +95° + X → --x + Y → servo_y_mm (并发)
  2. 手爪转 0° (末端朝下, cam2 能看吸嘴正下方的水立方)
  3. 轮询 arm_state 确认大臂物理到位
  4. runner.track_velocity_pick: cam2 识别水立方 → 视觉定位到吸嘴正下方
     (velocity 模式, 免 arm_queue) → move_y 到吸附高度 → 吸附 → 抬回
  5. 补 move_y 到运输高度 (y_lift_mm)

用法:
  export RAK_CAR_API_BASE=http://192.168.5.230:5050
  /usr/bin/python3 main/arm/examples/12_vision_pick_water.py                # 默认第一个方块列
  /usr/bin/python3 main/arm/examples/12_vision_pick_water.py --x -155       # 指定 X (同组第 1 块)
  /usr/bin/python3 main/arm/examples/12_vision_pick_water.py --label water  # 覆盖检测标签
  /usr/bin/python3 main/arm/examples/12_vision_pick_water.py --sign-x +1    # 实车方向反了取反
  /usr/bin/python3 main/arm/examples/12_vision_pick_water.py --dry-run      # 只摆姿态, 不伺服不抓

注意:
  - 机械臂会动 + 吸气; 跑之前确认目标在吸嘴可达范围内。
  - 别人在真机上测的时候别跑 (会和外部命令打架)。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.api_client import RuntimeApiClient
from main.arm.api import ArmClient
from main.arm.loops.runner import ArmRunner
from main.task._config import load_task_config


def _wait_arm_angle(arm_client: ArmClient, target_deg: float,
                    tolerance: float = 3.0, timeout: float = 10.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            cur = arm_client.get_state().arm_angle
        except Exception:
            time.sleep(0.15)
            continue
        if cur is not None and abs(cur - target_deg) <= tolerance:
            print(f"  大臂物理到位: {cur:.1f}° (目标 {target_deg:.0f}° ± {tolerance:.0f}°)", flush=True)
            return
        time.sleep(0.15)
    raise RuntimeError(f"大臂角度在 {timeout:.0f}s 内未到达 {target_deg:.0f}°")


def main() -> int:
    cfg = load_task_config("water_tower_task")
    pick = cfg["pick_pose"]
    vision = cfg.get("pick_vision") or {}
    if not vision.get("enabled"):
        print("pick_vision.enabled=false, 请先在 task_config.yml 打开", flush=True)
        return 2

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--x", type=float, default=None, help="方块 X (mm), 默认 first_cube_x_mm")
    ap.add_argument("--label", default=None, help="覆盖 pick_vision.label")
    ap.add_argument("--sign-arm", type=int, default=None, help="覆盖 sign_arm")
    ap.add_argument("--sign-x", type=int, default=None, help="覆盖 sign_x")
    ap.add_argument("--gain-arm", type=float, default=None, help="覆盖 gain_arm")
    ap.add_argument("--gain-x", type=float, default=None, help="覆盖 gain_x")
    ap.add_argument("--deadzone", type=float, default=None, help="覆盖 deadzone")
    ap.add_argument("--timeout", type=float, default=None, help="覆盖 timeout (伺服超时)")
    ap.add_argument("--dry-run", action="store_true", help="只摆姿态 + 手爪 0°, 不伺服不抓")
    args = ap.parse_args()

    cube_x_mm = args.x if args.x is not None else float(cfg["first_cube_x_mm"])
    label = args.label or vision.get("label", "water")
    servo_y = float(vision.get("servo_y_mm", pick["y_transition_mm"]))

    client = ArmClient.connect()
    if not client.ping():
        print("机械臂 runtime 未在线, 请检查 arm_feed 守护进程", flush=True)
        return 1
    runner = ArmRunner(client)
    print(f"server: {client.http.api_base}  label={label}  x={cube_x_mm}mm", flush=True)
    print(f"起始姿态:", client.get_state().describe(), flush=True)

    # 1) 摆 S 姿态 (大臂 +95°, X → 方块列, Y → servo_y_mm, 不含手爪)
    print(f"[1] composite_run: arm={pick['arm_angle_deg']} x={cube_x_mm} y={servo_y}", flush=True)
    runner.client.composite_run(
        arm=float(pick["arm_angle_deg"]),
        x_mm=cube_x_mm,
        y_mm=servo_y,
    )
    # 2) 手爪转 0°
    print(f"[2] 手爪转 0°", flush=True)
    client.set_hand_angle(float(pick["hand_angle_deg"]), speed=80, timeout=10.0)
    # 3) 等大臂物理到位
    print(f"[3] 等大臂 {pick['arm_angle_deg']}° 物理到位", flush=True)
    _wait_arm_angle(client, float(pick["arm_angle_deg"]))

    if args.dry_run:
        print("[dry-run] 姿态就位, 不伺服不抓", flush=True)
        return 0

    # 4) cam2 视觉伺服抓取 (track_velocity_pick)
    print(f"[4] track_velocity_pick({label}) "
          f"sign_arm={args.sign_arm if args.sign_arm is not None else vision.get('sign_arm')} "
          f"sign_x={args.sign_x if args.sign_x is not None else vision.get('sign_x')}",
          flush=True)
    result = runner.track_velocity_pick(
        label,
        x_start=cube_x_mm,
        y_start=servo_y,
        arm_start=float(pick["arm_angle_deg"]),
        hand_start=float(pick["hand_angle_deg"]),
        grasp_y_mm=float(vision.get("grasp_y_mm", pick["y_descend_mm"])),
        timeout=args.timeout if args.timeout is not None else float(vision.get("timeout", 15.0)),
        hz=float(vision.get("hz", 20.0)),
        gain_arm=args.gain_arm if args.gain_arm is not None else float(vision.get("gain_arm", 0.4)),
        gain_x=args.gain_x if args.gain_x is not None else float(vision.get("gain_x", 0.08)),
        deadzone=args.deadzone if args.deadzone is not None else float(vision.get("deadzone", 0.03)),
        max_vel=float(vision.get("max_vel", 0.20)),
        sign_arm=float(args.sign_arm if args.sign_arm is not None else vision.get("sign_arm", 1.0)),
        sign_x=float(args.sign_x if args.sign_x is not None else vision.get("sign_x", -1.0)),
        arm_min=vision.get("arm_min"),
        arm_max=vision.get("arm_max"),
        settle_hits=int(vision.get("settle_hits", 3)),
        hold_s=float(vision.get("hold_s", 0.3)),
        lift_back=True,
        skip_pose_align=True,
    )

    # 5) 补 move_y 到运输高度
    if result.get("ok"):
        lift_y = float(pick.get("y_lift_mm", -150.0))
        if abs(lift_y - servo_y) > 1.0:
            print(f"[5] move_y({lift_y}mm) 到运输高度", flush=True)
            runner.move_y(lift_y)

    print("\n=== 抓取结果 ===", flush=True)
    print(f"ok      : {result.get('ok')}", flush=True)
    print(f"reason  : {result.get('reason')}", flush=True)
    print(f"settled : {result.get('settled')}  trace_hits={result.get('trace_hits')}", flush=True)
    print(f"end_arm : {result.get('end_arm')}  end_hand={result.get('end_hand')}", flush=True)
    print(f"steps   : {result.get('steps')}", flush=True)
    print("最终姿态:", client.get_state().describe(), flush=True)
    print(f"\n{'✅ 抓取成功' if result.get('ok') else '❌ 抓取未成功'}", flush=True)
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⌘ 中断, 急停...", flush=True)
        sys.exit(130)
