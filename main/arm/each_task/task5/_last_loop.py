"""task5 / _last_loop —— last_*_to_{high,low} 系列脚本的共享骨架 (2026-07-29 抽出)。

设计动机:
  - last_yellow_to_high / last_blue_to_high / last_yellow_to_low / last_blue_to_low
    四个脚本的**结构**完全一致 (prep_pose → 检测 → 循环调 test{1..4}_run),
    仅**接线**不同 (prep 用哪个 target_*, 循环调哪个 testN_run, 颜色过滤哪个)。
  - 把结构抽到这个文件, 四个 wrapper 只配参数。这样逻辑改一处全跟。
  - 文件名带 `_` 前缀 (私有), 不会被 `_last_loop.py` 自身或 user 当成 runnable。

⚠️ **不要直接 import 这个文件** —— 它的接口是给四个 wrapper 用的; 用户应该跑
   `last_yellow_to_high.py` / `last_blue_to_high.py` / `last_yellow_to_low.py` /
   `last_blue_to_low.py` 之一。
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Callable

from main.arm import ArmClient, ArmRunner  # noqa: E402


def _prep_pose(client: ArmClient, runner: ArmRunner,
               target_module, log_prefix: str,
               y_mm: float = None, x_mm: float = None,
               arm_deg: float = None, hand_deg: float = None) -> None:
    """检测前摆臂: 仿照目标模块 (target_* 或 get_*) 的前 4 步。

    默认从 target_module.{TARGET1_X_MM/Y_MM/ARM_DEG/HAND_DEG} 拿参数 (观察位姿);
    显式传 y_mm/x_mm/arm_deg/hand_deg 时用传入值 (用于 last_blue_* 改用 get_blue
    参数)。

    通过 import 整个 target_module 拿 _move_x_with_split (belt-slip 安全 move_x)。

    Args:
        target_module: 已 import 的目标模块 (提供 _move_x_with_split 和默认位姿参数)。
        log_prefix: 自己脚本的 LOG_PREFIX (用于打印)。
        y_mm / x_mm / arm_deg / hand_deg: 显式覆盖默认 (None = 用 target_module 默认)。
    """
    if y_mm is None: y_mm = target_module.TARGET1_Y_MM
    if x_mm is None: x_mm = target_module.TARGET1_X_MM
    if arm_deg is None: arm_deg = target_module.TARGET1_ARM_DEG
    if hand_deg is None: hand_deg = target_module.TARGET1_HAND_DEG
    print(f"  {log_prefix} [prep] 摆臂到检测位姿 (仿 {target_module.__name__.split('.')[-1]} 前 4 步)")
    # 1. y → y_mm (抬出保护区)
    print(f"  [prep 1/4] move_y({y_mm}mm)  抬出保护区 [0,-30]")
    runner.move_y(y_mm, timeout=30.0)
    # 2. x → x_mm (belt-slip 安全)
    print(f"  [prep 2/4] move_x({x_mm}mm)  belt-slip 安全")
    x_info = target_module._move_x_with_split(client, x_mm)
    print(f"             x_info={x_info}")
    # 3. 大臂 → arm_deg (业务硬限上界 / 复位位 / init 例外位)
    print(f"  [prep 3/4] set_arm_angle({arm_deg:.1f}°)")
    client.set_arm_angle(arm_deg, speed=80, timeout=10.0)
    # 4. 手爪 → hand_deg (DOWN, 底层直调, 跟兄弟脚本一致)
    print(f"  [prep 4/4] 手爪 → {hand_deg:.1f}° (DOWN, 底层直调)")
    client._call_arm(
        "set_hand_angle", timeout=10.0, sync=True,
        angle=hand_deg, speed=80,
    )
    print(f"  {log_prefix} [prep] 完成\n")


def run_last_loop(
    client: ArmClient,
    runner: ArmRunner,
    *,
    log_prefix: str,
    target_module,                # e.g. target_yellow / target_blue (提供 _move_x_with_split + 默认位姿参数)
    test_run_fn: Callable,        # e.g. test1_run / test2_run / test3_run / test4_run
    test_log_prefix: str,         # e.g. "[task5/test1_...]"
    color_label: str,             # e.g. "yellow" / "blue"
    balls: int = -1,
    detect: bool = True,
    prep_pose: bool = True,
    prep_y_mm: float = None,
    prep_x_mm: float = None,
    prep_arm_deg: float = None,
    prep_hand_deg: float = None,
    hold_s: float = 5.0,
    detect_timeout_s: float = 3.0,
    score_min: float = 0.60,
    area_min: float = 0.10,
    area_max: float = 0.24,
    aspect_tol: float = 0.8,
    vision: bool = False,
    grasp_y_mm: float = None,
    vision_fallback: bool = True,
    sign_arm: float = 1.0,
    sign_x: float = -1.0,
    vision_timeout: float = 20.0,
) -> dict:
    """prep_pose → 检测 → 循环调 test_run_fn 的统一骨架。

    由四个 last_*_to_{high,low}.py 共享。每个 wrapper 只传自己的接线 (target_module,
    test_run_fn, color_label) + 可选的 prep_pose 参数覆盖。

    prep_pose 参数默认从 target_module.TARGET1_* 拿 (观察位姿)。
    显式传 prep_y_mm/prep_x_mm/prep_arm_deg/prep_hand_deg 可覆盖 (如 last_blue_*
    改用 get_blue 的 GET_BLUE_* 参数)。

    Returns:
        {"ok": True, "detected_balls": int, "rounds_run": int, "rounds_results": list}
    """
    print(f"\n========== {log_prefix} run ==========")

    # ---- 阶段 1: 检测 (prep_pose 在检测前) ----
    detected: list = []
    if detect:
        if prep_pose:
            print(f"  [1/?] prep_pose: 摆臂到检测位姿")
            _prep_pose(client, runner, target_module, log_prefix,
                       y_mm=prep_y_mm, x_mm=prep_x_mm,
                       arm_deg=prep_arm_deg, hand_deg=prep_hand_deg)
        else:
            print(f"  [1/?] prep_pose  已跳过 (--no-prep), 用当前位姿直接检测")
        print(f"  [1/?] 球类识别 (≤{detect_timeout_s}s, score≥{score_min} "
              f"area∈[{area_min},{area_max}] |aspect-1|≤{aspect_tol}, color={color_label})")
        detected = target_module.detect_balls(
            client,
            color_filter=target_module.DETECT_COLOR_FILTER,
            timeout_s=detect_timeout_s,
            score_min=score_min,
            area_min=area_min, area_max=area_max,
            aspect_tol=aspect_tol,
        )
    else:
        print(f"  [1/?] 球类识别  已跳过 (--no-detect), 必须用 --balls 指定轮数")

    n_detected = len(detected)

    # ---- 决定最终轮数 ----
    if balls >= 0:
        n_rounds = balls
        print(f"  → --balls={balls} 强制指定, 覆盖检测结果 ({n_detected} 个{color_label}球)")
    else:
        n_rounds = n_detected
        print(f"  → 检测到 {n_detected} 个{color_label}球, 计划跑 {n_rounds} 轮")

    if n_rounds <= 0:
        if detect and balls < 0:
            print(f"  {log_prefix} [WARN] 检测到 0 个{color_label}球, 跳过所有调用。"
                  f" 用 --balls N 强制指定轮数 (压力测试用)。")
        elif balls == 0:
            print(f"  {log_prefix} --balls=0, 故意不执行任何轮")
        print(f"========== {log_prefix} 完成 (0 轮) ==========\n")
        return {
            "ok": True,
            "detected_balls": n_detected,
            "rounds_run": 0,
            "rounds_results": [],
        }

    # ---- 阶段 2..N: 循环调 test_run_fn ----
    # 2026-08-03: test_run_fn = pick_and_place 薄 wrapper, 接受视觉闭环参数。
    # vision=False 时 kwargs 全走默认 → 行为与旧版完全一致。
    vision_kwargs = {}
    if vision:
        vision_kwargs["vision"] = True
        vision_kwargs["vision_fallback"] = vision_fallback
        vision_kwargs["sign_arm"] = sign_arm
        vision_kwargs["sign_x"] = sign_x
        vision_kwargs["vision_timeout"] = vision_timeout
        if grasp_y_mm is not None:
            vision_kwargs["grasp_y_mm"] = grasp_y_mm

    rounds_results: list = []
    for r in range(1, n_rounds + 1):
        print(f"\n----- {log_prefix} [轮 {r}/{n_rounds}] -----")
        print(f"  调 {test_log_prefix}.run(hold_s={hold_s:.1f}"
              f"{', vision=True' if vision else ''})")
        try:
            r_res = test_run_fn(client, runner, hold_s=hold_s, **vision_kwargs)
            rounds_results.append({"round": r, "ok": True, "result": r_res})
            print(f"  [轮 {r}/{n_rounds}] OK")
        except Exception as e:
            err = f"{type(e).__name__}: {str(e)[:200]}"
            print(f"  [轮 {r}/{n_rounds}] FAIL  {err}")
            rounds_results.append({"round": r, "ok": False, "error": err})
            # 2026-07-30 失败兜底: test1/2/3/4 的吸气逻辑是
            #   grasp(True) → sleep(hold_s) → tower_run → grasp(False) → reset_x
            # 其中 tower_run (high_tower / low_tower) 一旦 raise (belt-slip / x 卡死 /
            # 编码器 stall), grasp(False) 就跳过 → 球留在吸盘上。
            # _last_loop 默认 continue (失败不中断), 如果失败的恰好是最后一轮,
            # 整个 last_*_to_{high,low} 返回 OK, the_final 退出 → 球吸到下个脚本。
            # 兜底: 失败时强制放气。⚠️ 不能走 client._call_arm("grasp", ...)：
            #   _call_arm 的 timeout 是位置形参, bool 位置传进去会被当 timeout →
            #   "got multiple values for argument 'timeout'" TypeError (memory
            #   [[arm-grasp-call-arm-base]])。正路: client.http.execute_arm_action
            #   直调位置参 bool(on)。
            try:
                client.http.execute_arm_action(
                    "grasp", False, timeout=5.0, sync=True,
                )
                print(f"  [轮 {r}/{n_rounds}] 强制 grasp(False) 放气 (失败兜底)")
            except Exception as cleanup_err:  # pragma: no cover
                print(f"  [轮 {r}/{n_rounds}] 强制放气也失败: "
                      f"{type(cleanup_err).__name__}: {str(cleanup_err)[:120]}")
            continue  # 失败不中断 (瞬时硬件问题不该让整批失败); 想失败即停 → raise

    n_ok = sum(1 for x in rounds_results if x["ok"])
    print(f"\n========== {log_prefix} 完成 "
          f"({n_rounds} 轮, 成功 {n_ok}/{n_rounds}) ==========\n")
    return {
        "ok": True,
        "detected_balls": n_detected,
        "rounds_run": n_rounds,
        "rounds_results": rounds_results,
    }


def build_last_parser(log_prefix: str,
                      color_label: str,
                      test_log_prefix: str,
                      detect_help_extra: str = "") -> argparse.ArgumentParser:
    """给四个 last_*_to_{high,low}.py 共享的 build_parser。

    所有阈值参数默认 = None, 由 main_with_args 在运行时从 target_module.DETECT_*
    填入 (CLI 显式传值时覆盖)。这样 target_* 阈值后续被改, 自动跟上, 不需要改
    这里硬编码。
    """
    p = argparse.ArgumentParser(
        description=f"{log_prefix} 检测{color_label}球 → 循环 N 次 {test_log_prefix.split('/')[-1]}",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--balls", type=int, default=-1,
                   help="强制执行轮数 (覆盖检测结果)。-1=用检测到的球数; "
                        "0=不执行; N>0=强制 N 轮 (压力测试用)")
    p.add_argument("--no-detect", action="store_true", dest="no_detect",
                   help="跳过检测, 必须配合 --balls 使用")
    p.add_argument("--hold", type=float, default=5.0,
                   help="每轮吸气独立保持秒数 (透传给 test_run, 默认 5.0)")
    p.add_argument("--detect-timeout", type=float, default=None,
                   dest="detect_timeout",
                   help="识别轮询总时长 (秒), 拿到球提前返回 (默认 = target_*.DETECT_TIMEOUT_S)")
    p.add_argument("--score-min", type=float, default=None,
                   dest="score_min", help=f"识别最低置信度{detect_help_extra}")
    p.add_argument("--area-min", type=float, default=None,
                   dest="area_min", help="最小归一化面积 (默认 = target_*.DETECT_AREA_MIN)")
    p.add_argument("--area-max", type=float, default=None,
                   dest="area_max", help="最大归一化面积 (默认 = target_*.DETECT_AREA_MAX)")
    p.add_argument("--aspect-tol", type=float, default=None,
                   dest="aspect_tol",
                   help="宽高比容差 |aspect-1|≤tol (默认 = target_*.DETECT_ASPECT_TOL)")
    p.add_argument("--no-prep", action="store_true", dest="no_prep",
                   help="跳过检测前的摆臂 (默认开: 仿 target_* 5 步前 4 步)")
    # ---- 2026-08-03 新增: 视觉闭环取球 (透传给 test_run_fn / pick_and_place) ----
    p.add_argument("--vision", action="store_true",
                   help="启用视觉闭环取球 (track_velocity_pick); "
                        "⚠️ sign 参数是 task1 姿态标定值, task5 位姿首跑先确认方向")
    p.add_argument("--grasp-y", type=float, default=None, dest="grasp_y",
                   help="视觉模式吸气 y (mm); 不传用 pick_and_place 默认 (-70, 不下探)")
    p.add_argument("--no-vision-fallback", dest="vision_fallback", action="store_false",
                   help="视觉失败不回退开环盲吸 (默认回退)")
    p.add_argument("--sign-arm", type=float, default=1.0, dest="sign_arm",
                   help="视觉伺服大臂轴符号 (±1, 现场标定)")
    p.add_argument("--sign-x", type=float, default=-1.0, dest="sign_x",
                   help="视觉伺服 x 轴符号 (±1, 现场标定)")
    p.add_argument("--vision-timeout", type=float, default=20.0,
                   dest="vision_timeout", help="视觉伺服总超时 (秒)")
    p.set_defaults(vision_fallback=True)
    return p


def _resolve(args: argparse.Namespace, attr: str, target_module, default_name: str) -> float:
    """CLI 显式传值时用 args.attr; 否则从 target_module.{default_name} 取。"""
    v = getattr(args, attr)
    if v is not None:
        return float(v)
    return float(getattr(target_module, default_name))


def main_with_args(args: argparse.Namespace,
                   log_prefix: str,
                   target_module,
                   test_run_fn: Callable,
                   test_log_prefix: str,
                   color_label: str,
                   prep_y_mm: float = None,
                   prep_x_mm: float = None,
                   prep_arm_deg: float = None,
                   prep_hand_deg: float = None) -> int:
    """给四个 wrapper 的 main() 复用: 解析 + 错误检查 + 计时 + 调 run_last_loop。

    prep_y_mm/prep_x_mm/prep_arm_deg/prep_hand_deg: 显式覆盖 prep_pose 默认参数
    (默认从 target_module.TARGET1_* 拿)。None = 用默认。

    Returns:
        process exit code (0 = OK, 2 = 参数错误)。
    """
    if args.no_detect and args.balls < 0:
        print(f"  {log_prefix} [ERROR] --no-detect 必须配合 --balls N 使用 "
              f"(不然脚本不知道该跑几轮)", file=sys.stderr)
        return 2
    client = ArmClient.connect()
    runner = ArmRunner(client)
    t_total_start = time.perf_counter()
    run_last_loop(
        client, runner,
        log_prefix=log_prefix,
        target_module=target_module,
        test_run_fn=test_run_fn,
        test_log_prefix=test_log_prefix,
        color_label=color_label,
        balls=args.balls,
        detect=not args.no_detect,
        prep_pose=not args.no_prep,
        prep_y_mm=prep_y_mm,
        prep_x_mm=prep_x_mm,
        prep_arm_deg=prep_arm_deg,
        prep_hand_deg=prep_hand_deg,
        hold_s=args.hold,
        detect_timeout_s=_resolve(args, "detect_timeout", target_module, "DETECT_TIMEOUT_S"),
        score_min=_resolve(args, "score_min", target_module, "DETECT_SCORE_MIN"),
        area_min=_resolve(args, "area_min", target_module, "DETECT_AREA_MIN"),
        area_max=_resolve(args, "area_max", target_module, "DETECT_AREA_MAX"),
        aspect_tol=_resolve(args, "aspect_tol", target_module, "DETECT_ASPECT_TOL"),
        # 2026-08-03: 视觉闭环取球参数透传 (vision=False 时全走默认, 行为不变)
        vision=getattr(args, "vision", False),
        grasp_y_mm=getattr(args, "grasp_y", None),
        vision_fallback=getattr(args, "vision_fallback", True),
        sign_arm=getattr(args, "sign_arm", 1.0),
        sign_x=getattr(args, "sign_x", -1.0),
        vision_timeout=getattr(args, "vision_timeout", 20.0),
    )
    elapsed = time.perf_counter() - t_total_start
    print(f"========== {log_prefix} 总耗时: {elapsed:.3f} s ==========")
    return 0


__all__ = ["_prep_pose", "run_last_loop", "build_last_parser", "main_with_args"]