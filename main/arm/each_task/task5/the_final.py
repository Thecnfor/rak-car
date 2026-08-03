"""task5 / the_final —— **task5 收官脚本** (2026-07-30 重写, 4 步严格按用户指定)。

业务流程:
  1. 调用 ``target.py`` 识别高仓颜色 → 记为 color A
     (A ∈ {blue, yellow, unknown};  unknown → 退出 3)
  2. **同色球进高仓**:
     - A = blue   → ``last_blue_to_high.main()``
     - A = yellow → ``last_yellow_to_high.main()``
  3. **底盘到 LOW 仓取位** (2026-08-03 起两档):
     - ``--align-area`` 传了 → 视觉闭环对仓 (main.chassis.make_align_runner,
       按 bbox 面积前后微调; 需现场标定 ref_area), 失败回退开环
     - 默认 → 开环后撤 165mm: ``dipan._run(client, dist_mm=-165.0)``
  4. **反色球进 LOW 仓**:
     - A = blue   → ``last_yellow_to_low.main()``   (高仓是蓝 → LOW 仓放黄)
     - A = yellow → ``last_blue_to_low.main()``     (高仓是黄 → LOW 仓放蓝)

⚠️ **color A 是唯一的真相源**: 阶段 1 识别一次, 阶段 2 / 4 都用它分流。
   不在阶段 4 重新识别 — 中间已经移车 (high + 后撤), 视野完全变了,
   重新识别会拿到 LOW 仓自己的标签, 跟高仓无关。

⚠️ **本文件自包含** (task5/__init__.py 提到的"曾被外部清空"防御):
  - 只 import 6 个 task5 兄弟模块, 不重复实现 move_for / HSV / _last_loop 骨架。

跑法:
    python main/arm/each_task/task5/the_final.py                # 端到端
    python main/arm/each_task/task5/the_final.py --color blue   # 手动指定色
    python main/arm/each_task/task5/the_final.py --balls 2      # 强制 2 轮
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402

import main.arm.each_task.task5.target as target_module                       # noqa: E402
import main.arm.each_task.task5.dipan as dipan_module                         # noqa: E402
import main.arm.each_task.task5.last_blue_to_high as last_blue_high_module    # noqa: E402
import main.arm.each_task.task5.last_yellow_to_high as last_yellow_high_module  # noqa: E402
import main.arm.each_task.task5.last_blue_to_low as last_blue_low_module      # noqa: E402
import main.arm.each_task.task5.last_yellow_to_low as last_yellow_low_module  # noqa: E402


# ---------- 常量 ----------

LOG_PREFIX: str = "[task5/the_final]"

BACK_DIST_MM: float = -165.0
"""底盘后撤距离 (mm), 用户 2026-07-30 指定。"""

EXIT_OK = 0
EXIT_BAD_COLOR = 3           # 阶段 1 识别失败
EXIT_DISPATCH_FAIL = 5       # 阶段 2 / 4 调 last_* 失败


# ---------- 4 阶段 ----------

def step1_detect(client: ArmClient, runner: ArmRunner, args) -> str:
    """阶段 1: 调 target.run() 识别高仓颜色, 返回 color A。"""
    print(f"\n========== {LOG_PREFIX} [1/4] 识别高仓色标 (target.run) ==========")
    print(f"  cam={args.cam}  roi={args.roi}  timeout={args.color_timeout}s")
    result = target_module.run(
        client, runner,
        detect_color=True,
        cam=args.cam,
        roi=args.roi,
        color_timeout=args.color_timeout,
    )
    color_info = result.get("color_info") if isinstance(result, dict) else None
    if not isinstance(color_info, dict):
        print(f"  {LOG_PREFIX} [FAIL] target.run() 没返回 color_info")
        return "unknown"
    color = str(color_info.get("color", "unknown")).lower()
    print(f"  → color A = {color!r}  "
          f"(blue_ratio={color_info.get('blue_ratio', 0.0):.3f}, "
          f"yellow_ratio={color_info.get('yellow_ratio', 0.0):.3f})")
    return color


def step2_high(client: ArmClient, runner: ArmRunner, color_a: str, args) -> int:
    """阶段 2: 同色球进高仓。blue → last_blue_to_high; yellow → last_yellow_to_high."""
    print(f"\n========== {LOG_PREFIX} [2/4] 同色球进高仓 ==========")
    last_argv = _build_last_argv(args)
    if color_a == "blue":
        print(f"  A=blue → last_blue_to_high (argv={last_argv})")
        return _invoke(last_blue_high_module, last_argv, "last_blue_to_high")
    if color_a == "yellow":
        print(f"  A=yellow → last_yellow_to_high (argv={last_argv})")
        return _invoke(last_yellow_high_module, last_argv, "last_yellow_to_high")
    print(f"  [SKIP] A={color_a!r} 不是 blue/yellow, 跳过 high")
    return EXIT_BAD_COLOR


def step3_back(client: ArmClient, dist_mm: float = BACK_DIST_MM,
               align_area: float = None, align_label: str = None,
               align_max_s: float = 15.0) -> dict:
    """阶段 3: 底盘到 LOW 仓取位。

    两种模式 (2026-08-03):
      1. align_area 传了 → **视觉闭环对仓**: make_align_runner(ref_area=align_area)
         按目标 bbox 面积前后微调 (main.chassis 新方法, 只动 vx 不横移不旋转)。
         到位 → 完成; 失败 (no_target / watchdog / 超时) → 回退开环后撤。
      2. align_area=None (默认) → 开环后撤 dist_mm mm (dipan._run, 旧行为)。

    ⚠️ align_area 需**现场标定**: 手动把车摆到理想放料位, 读一帧
       GET /v1/realtime/vision/task 里目标 bbox 的 width*height 填进来。
       align_label=None 时取画面面积最大目标。
    """
    if align_area is not None:
        print(f"\n========== {LOG_PREFIX} [3/4] 视觉闭环对仓 "
              f"(ref_area={align_area}, label={align_label}, ≤{align_max_s}s) ==========")
        try:
            from main.chassis import make_align_runner  # 局部 import, 不开环时零依赖
            runner = make_align_runner(ref_area=align_area, label=align_label)
            result = runner.run(max_seconds=align_max_s)
            print(f"  {LOG_PREFIX} 视觉对仓结果: arrived={result.arrived} "
                  f"reason={result.reason} frames={result.frames} "
                  f"elapsed={result.elapsed_s:.1f}s")
            if result.arrived:
                return {"mode": "vision_align", "arrived": True,
                        "reason": result.reason}
            print(f"  [WARN] 视觉对仓未到位 (reason={result.reason}), 回退开环后撤")
        except Exception as e:
            print(f"  [WARN] 视觉对仓异常: {type(e).__name__}: {e}, 回退开环后撤")

    print(f"\n========== {LOG_PREFIX} [3/4] 底盘开环后撤 {abs(dist_mm):.0f}mm ==========")
    print(f"  走 dipan._run(client, dist_mm={dist_mm}) "
          f"(max_velocity=0.10 m/s, timeout=20.0s)")
    job = dipan_module._run(
        client, dist_mm=dist_mm, max_velocity_ms=0.10, timeout=20.0,
    )
    print(f"  {LOG_PREFIX} 后撤完成")
    return {"mode": "open_loop", "job": job}


def step4_low(client: ArmClient, runner: ArmRunner, color_a: str, args) -> int:
    """阶段 4: 反色球进 LOW 仓。blue → last_yellow_to_low; yellow → last_blue_to_low."""
    print(f"\n========== {LOG_PREFIX} [4/4] 反色球进 LOW 仓 ==========")
    last_argv = _build_last_argv(args)
    if color_a == "blue":
        print(f"  A=blue → LOW 仓放黄球: last_yellow_to_low (argv={last_argv})")
        return _invoke(last_yellow_low_module, last_argv, "last_yellow_to_low")
    if color_a == "yellow":
        print(f"  A=yellow → LOW 仓放蓝球: last_blue_to_low (argv={last_argv})")
        return _invoke(last_blue_low_module, last_argv, "last_blue_to_low")
    print(f"  [SKIP] A={color_a!r} 不是 blue/yellow, 跳过 LOW")
    return EXIT_BAD_COLOR


# ---------- helpers ----------

def _invoke(module, argv: list, label: str) -> int:
    """调 last_*_to_{high,low}.main(argv); 异常 → EXIT_DISPATCH_FAIL。"""
    try:
        rc = module.main(argv)
    except Exception as e:
        print(f"  {LOG_PREFIX} [FAIL] {label} 抛异常: {type(e).__name__}: {e}")
        return EXIT_DISPATCH_FAIL
    return int(rc) if rc is not None else EXIT_OK


def _build_last_argv(args) -> list:
    """the_final Namespace → last_*_to_{high,low} 接受的 argv。

    ⚠️ --no-detect 语义冲突: the_final 是「跳过 target 识别」, last_* 是「跳过球
    识别」。**不**透传 --no-detect 给 last_*, 让它照常做球识别 (更稳)。
    """
    argv = []
    if args.balls is not None and args.balls >= 0:
        argv += ["--balls", str(args.balls)]
    if args.hold is not None:
        argv += ["--hold", str(args.hold)]
    if args.no_prep:
        argv += ["--no-prep"]
    if args.detect_timeout is not None:
        argv += ["--detect-timeout", str(args.detect_timeout)]
    if args.score_min is not None:
        argv += ["--score-min", str(args.score_min)]
    if args.area_min is not None:
        argv += ["--area-min", str(args.area_min)]
    if args.area_max is not None:
        argv += ["--area-max", str(args.area_max)]
    if args.aspect_tol is not None:
        argv += ["--aspect-tol", str(args.aspect_tol)]
    # 2026-08-03: 视觉闭环取球参数透传给 last_* (再透传 test_run_fn / pick_and_place)
    if getattr(args, "vision", False):
        argv += ["--vision"]
        if getattr(args, "grasp_y", None) is not None:
            argv += ["--grasp-y", str(args.grasp_y)]
        if not getattr(args, "vision_fallback", True):
            argv += ["--no-vision-fallback"]
        if getattr(args, "sign_arm", 1.0) != 1.0:
            argv += ["--sign-arm", str(args.sign_arm)]
        if getattr(args, "sign_x", -1.0) != -1.0:
            argv += ["--sign-x", str(args.sign_x)]
        if getattr(args, "vision_timeout", 20.0) != 20.0:
            argv += ["--vision-timeout", str(args.vision_timeout)]
    return argv


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task5 the_final: target识别 → 同色high → 后撤165mm → 反色low",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # ---- 阶段 1 色标识别 ----
    p.add_argument("--color", choices=("blue", "yellow"), default=None,
                   help="手动指定高仓色 (跳过阶段 1 target 识别)")
    p.add_argument("--cam", default=target_module.DEFAULT_CAM,
                   help=f"色标识别相机 (默认 {target_module.DEFAULT_CAM}=side)")
    p.add_argument("--roi", type=int, nargs=4, default=None,
                   metavar=("X", "Y", "W", "H"),
                   help=f"色标 ROI, 默认 target.DEFAULT_ROI ({target_module.DEFAULT_ROI})")
    p.add_argument("--color-timeout", type=float,
                   default=target_module.JPEG_FETCH_TIMEOUT_S,
                   dest="color_timeout",
                   help="抓 JPEG HTTP 超时 (秒)")
    # ---- 透传给 last_* ----
    p.add_argument("--balls", type=int, default=-1,
                   help="强制执行轮数 (透传给 last_*), -1=用检测球数")
    p.add_argument("--hold", type=float, default=None,
                   help="每轮吸气保持秒数 (透传给 last_*)")
    p.add_argument("--no-prep", action="store_true", dest="no_prep",
                   help="跳过 last_* 检测前的摆臂 (透传)")
    p.add_argument("--detect-timeout", type=float, default=None,
                   dest="detect_timeout",
                   help="last_* 球识别轮询总时长 (秒)")
    p.add_argument("--score-min", type=float, default=None,
                   dest="score_min", help="last_* 识别最低置信度")
    p.add_argument("--area-min", type=float, default=None,
                   dest="area_min", help="last_* 识别最小归一化面积")
    p.add_argument("--area-max", type=float, default=None,
                   dest="area_max", help="last_* 识别最大归一化面积")
    p.add_argument("--aspect-tol", type=float, default=None,
                   dest="aspect_tol",
                   help="last_* 识别宽高比容差 |aspect-1|≤tol")
    # ---- 2026-08-03 新增: 视觉闭环取球 (透传 last_* → test_run_fn) ----
    p.add_argument("--vision", action="store_true",
                   help="启用视觉闭环取球 (track_velocity_pick); "
                        "⚠️ sign 是 task1 姿态标定值, task5 位姿首跑先确认方向")
    p.add_argument("--grasp-y", type=float, default=None, dest="grasp_y",
                   help="视觉模式吸气 y (mm); 不传用 pick_and_place 默认 (-70)")
    p.add_argument("--no-vision-fallback", dest="vision_fallback", action="store_false",
                   help="视觉失败不回退开环盲吸 (默认回退)")
    p.add_argument("--sign-arm", type=float, default=1.0, dest="sign_arm",
                   help="视觉伺服大臂轴符号 (±1, 现场标定)")
    p.add_argument("--sign-x", type=float, default=-1.0, dest="sign_x",
                   help="视觉伺服 x 轴符号 (±1, 现场标定)")
    p.add_argument("--vision-timeout", type=float, default=20.0,
                   dest="vision_timeout", help="视觉伺服总超时 (秒)")
    # ---- 2026-08-03 新增: 阶段 3 视觉闭环对仓 (make_align_runner) ----
    p.add_argument("--align-area", type=float, default=None, dest="align_area",
                   help="阶段 3 视觉对仓参考面积 (现场标定); 传了则优先视觉闭环, "
                        "失败回退开环后撤; 不传走开环后撤 (旧行为)")
    p.add_argument("--align-label", default=None, dest="align_label",
                   help="视觉对仓目标 label; 不传取画面面积最大目标")
    p.add_argument("--align-max-s", type=float, default=15.0, dest="align_max_s",
                   help="视觉对仓最长时长 (秒)")
    p.set_defaults(vision_fallback=True)
    return p


def main(argv=None) -> int:
    t_total = time.perf_counter()
    args = build_parser().parse_args(argv)
    print(f"========== {LOG_PREFIX} run ==========")
    print(f"  --color={args.color}  --balls={args.balls}")

    client = ArmClient.connect()
    runner = ArmRunner(client)

    # 阶段 1: 识别高仓色 → color A
    if args.color is not None:
        color_a = args.color
        print(f"\n  {LOG_PREFIX} [1/4] 跳过 target 识别, 手动 --color={color_a}")
    else:
        color_a = step1_detect(client, runner, args)

    if color_a not in ("blue", "yellow"):
        print(f"  {LOG_PREFIX} [FAIL] color A={color_a!r} 不是 blue/yellow, 终止")
        return EXIT_BAD_COLOR

    # 阶段 2: 同色球进高仓
    rc2 = step2_high(client, runner, color_a, args)
    if rc2 not in (EXIT_OK, EXIT_BAD_COLOR):
        print(f"  {LOG_PREFIX} 阶段 2 失败 (rc={rc2}), 流水线终止")
        return rc2

    # 阶段 3: 底盘后撤 165mm
    step3_back(client, BACK_DIST_MM)

    # 阶段 4: 反色球进 LOW 仓
    rc4 = step4_low(client, runner, color_a, args)
    if rc4 not in (EXIT_OK, EXIT_BAD_COLOR):
        print(f"  {LOG_PREFIX} 阶段 4 失败 (rc={rc4})")
        return rc4

    elapsed = time.perf_counter() - t_total
    print(f"\n========== {LOG_PREFIX} 完成 (A={color_a}, "
          f"{elapsed:.3f}s) ==========\n")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())