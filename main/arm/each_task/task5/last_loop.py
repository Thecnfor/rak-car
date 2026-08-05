"""task5 / last_loop —— 通用循环: prep_pose → 检测 → 循环 pick→place。

替代原来的 5 个文件:
  _last_loop.py
  last_blue_to_high.py
  last_yellow_to_high.py
  last_blue_to_low.py
  last_yellow_to_low.py

通过两个参数区分行为:
  - color: "blue" | "yellow"    → 检测哪个颜色的球 / 用哪个 get_* 位姿
  - tower: "high" | "low"       → 放到高仓还是低仓

用法:
    python -m main.arm.each_task.task5.last_loop --color blue --tower high --balls 2
    python -m main.arm.each_task.task5.last_loop --color yellow --tower low --vision
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Callable

from main.arm import ArmClient, ArmRunner  # noqa: E402


LOG_PREFIX = "[task5/last_loop]"

# ---------- 位姿映射 (颜色 × 目标仓 → 对应模块) ----------

# 每个 color 对应:
#   target_module: 检测用 (提供 _move_x_with_split + detect_balls + 默认位姿)
#   get_module:    取球位姿 (get_blue / get_yellow)
#   color_label:   显示用
_COLOR_TABLE: dict[str, dict] = {
    "blue": {
        "target_module": "main.arm.each_task.task5.target_blue",
        "get_module": "main.arm.each_task.task5.get_blue",
        "label": "blue",
    },
    "yellow": {
        "target_module": "main.arm.each_task.task5.target_yellow",
        "get_module": "main.arm.each_task.task5.get_yellow",
        "label": "yellow",
    },
}

# 每个 tower 对应:
#   tower_fn: 放仓位姿函数
#   tower_name: 显示用
_TOWER_TABLE: dict[str, dict] = {
    "high": {
        "tower_fn": None,  # lazy import
        "tower_name": "high_tower",
    },
    "low": {
        "tower_fn": None,  # lazy import
        "tower_name": "low_tower",
    },
}


# ---------- 摆检测位姿 (复用 _last_loop 的 _prep_pose) ----------

def _prep_pose(client: ArmClient, runner: ArmRunner,
               target_module, log_prefix: str,
               y_mm: float = None, x_mm: float = None,
               arm_deg: float = None, hand_deg: float = None) -> None:
    """检测前摆臂: 仿照目标模块 (target_* 或 get_*) 的前 4 步。"""
    if y_mm is None: y_mm = target_module.TARGET1_Y_MM
    if x_mm is None: x_mm = target_module.TARGET1_X_MM
    if arm_deg is None: arm_deg = target_module.TARGET1_ARM_DEG
    if hand_deg is None: hand_deg = target_module.TARGET1_HAND_DEG
    print(f"  {log_prefix} [prep] 摆臂到检测位姿 (仿 {target_module.__name__.split('.')[-1]} 前 4 步)")
    print(f"  [prep 1/4] move_y({y_mm}mm)  抬出保护区 [0,-30]")
    runner.move_y(y_mm, timeout=30.0)
    print(f"  [prep 2/4] move_x({x_mm}mm)  belt-slip 安全")
    x_info = target_module._move_x_with_split(client, x_mm)
    print(f"             x_info={x_info}")
    print(f"  [prep 3/4] set_arm_angle({arm_deg:.1f}°)")
    client.set_arm_angle(arm_deg, speed=80, timeout=10.0)
    print(f"  [prep 4/4] 手爪 → {hand_deg:.1f}° (DOWN, 底层直调)")
    client._call_arm(
        "set_hand_angle", timeout=10.0, sync=True,
        angle=hand_deg, speed=80,
    )
    print(f"  {log_prefix} [prep] 完成\n")


# ---------- 主循环 ----------

def run_last_loop(
    client: ArmClient,
    runner: ArmRunner,
    *,
    color: str,               # "blue" | "yellow"
    tower: str,               # "high" | "low"
    balls: int = -1,
    detect: bool = True,
    prep_pose: bool = True,
    hold_s: float = 5.0,
    detect_timeout_s: float = None,
    score_min: float = None,
    area_min: float = None,
    area_max: float = None,
    aspect_tol: float = None,
    vision: bool = False,
    grasp_y_mm: float = None,
    vision_fallback: bool = True,
    sign_arm: float = 1.0,
    sign_x: float = -1.0,
    vision_timeout: float = 20.0,
) -> dict:
    """通用循环: prep_pose → 检测 → 循环调 pick_and_place。

    Args:
        color: "blue" | "yellow" — 检测哪个颜色的球 / 用哪个 get_* 位姿
        tower: "high" | "low" — 放到高仓还是低仓
        balls: 强制执行轮数 (-1=用检测结果, 0=不执行, N>0=强制 N 轮)
        detect: 是否做 prep_pose + 球识别
        prep_pose: 是否摆检测位姿
        vision: 是否启用视觉闭环取球
        grasp_y_mm: 视觉模式吸气 y (mm)
        vision_fallback: 视觉失败是否回退开环
        sign_arm / sign_x: 视觉伺服符号 (task1 姿态标定值)
        vision_timeout: 视觉伺服总超时 (秒)
        hold_s: 开环模式吸气保持秒数
        detect_timeout_s / score_min / area_min / area_max / aspect_tol:
            检测阈值 (None = 从 target_module 取默认值)

    Returns:
        {"ok": True, "detected_balls": int, "rounds_run": int, "rounds_results": list}
    """
    # 校验
    if color not in _COLOR_TABLE:
        raise ValueError(f"color 必须是 blue/yellow,  got={color!r}")
    if tower not in _TOWER_TABLE:
        raise ValueError(f"tower 必须是 high/low,  got={tower!r}")

    color_info = _COLOR_TABLE[color]
    target_module = __import__(color_info["target_module"], fromlist=[""])
    get_module = __import__(color_info["get_module"], fromlist=[""])
    color_label = color_info["label"]

    tower_info = _TOWER_TABLE[tower]
    if tower_info["tower_fn"] is None:
        if tower == "high":
            from main.arm.each_task.task5.high_tower import run as _tower_fn
        else:
            from main.arm.each_task.task5.low_tower import run as _tower_fn
        _TOWER_TABLE[tower]["tower_fn"] = _tower_fn
    tower_fn = tower_info["tower_fn"]
    tower_name = tower_info["tower_name"]

    # 复用 pick_and_place 的执行器
    from main.arm.each_task.task5.pick_and_place import run_pick_and_place

    # 阈值 None → 从 target_module 取默认
    if detect_timeout_s is None:
        detect_timeout_s = target_module.DETECT_TIMEOUT_S
    if score_min is None:
        score_min = target_module.DETECT_SCORE_MIN
    if area_min is None:
        area_min = target_module.DETECT_AREA_MIN
    if area_max is None:
        area_max = target_module.DETECT_AREA_MAX
    if aspect_tol is None:
        aspect_tol = target_module.DETECT_ASPECT_TOL

    log_prefix = f"[task5/last_{color}_to_{tower}]"
    print(f"\n========== {log_prefix} run ==========")
    print(f"  color={color}  tower={tower}  balls={balls}  vision={vision}")

    # ---- 阶段 1: 检测 (prep_pose 在检测前) ----
    detected: list = []
    if detect:
        if prep_pose:
            print(f"  [1/?] prep_pose: 摆臂到检测位姿")
            _prep_pose(client, runner, target_module, log_prefix)
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

    # ---- 阶段 2..N: 循环调 pick_and_place ----
    vision_kwargs = {}
    if vision:
        vision_kwargs["vision"] = True
        vision_kwargs["vision_fallback"] = vision_fallback
        vision_kwargs["sign_arm"] = sign_arm
        vision_kwargs["sign_x"] = sign_x
        vision_kwargs["vision_timeout"] = vision_timeout
        if grasp_y_mm is not None:
            vision_kwargs["grasp_y_mm"] = grasp_y_mm

    # 视觉模式 label
    vision_label = f"ball_{color}"

    rounds_results: list = []
    for r in range(1, n_rounds + 1):
        print(f"\n----- {log_prefix} [轮 {r}/{n_rounds}] -----")
        print(f"  pick={get_module.__name__.split('.')[-1]}  tower={tower_name}  "
              f"{'vision=True' if vision else 'open_loop'}")
        try:
            r_res = run_pick_and_place(
                client, runner,
                log_prefix=f"{log_prefix}[轮{r}/{n_rounds}]",
                pick_fn=get_module.run,
                pick_name=get_module.__name__.split('.')[-1],
                tower_fn=tower_fn,
                tower_name=tower_name,
                vision=vision,
                vision_label=vision_label,
                grasp_y_mm=grasp_y_mm,
                hold_s=hold_s,
                vision_fallback=vision_fallback,
                sign_arm=sign_arm,
                sign_x=sign_x,
                vision_timeout=vision_timeout,
            )
            rounds_results.append({"round": r, "ok": True, "result": r_res})
            print(f"  [轮 {r}/{n_rounds}] OK")
        except Exception as e:
            err = f"{type(e).__name__}: {str(e)[:200]}"
            print(f"  [轮 {r}/{n_rounds}] FAIL  {err}")
            rounds_results.append({"round": r, "ok": False, "error": err})
            # 失败兜底: 强制放气
            try:
                client.http.execute_arm_action(
                    "grasp", False, timeout=5.0, sync=True,
                )
                print(f"  [轮 {r}/{n_rounds}] 强制 grasp(False) 放气 (失败兜底)")
            except Exception as cleanup_err:
                print(f"  [轮 {r}/{n_rounds}] 强制放气也失败: "
                      f"{type(cleanup_err).__name__}: {str(cleanup_err)[:120]}")
            continue  # 失败不中断

    n_ok = sum(1 for x in rounds_results if x["ok"])
    print(f"\n========== {log_prefix} 完成 "
          f"({n_rounds} 轮, 成功 {n_ok}/{n_rounds}) ==========\n")
    return {
        "ok": True,
        "detected_balls": n_detected,
        "rounds_run": n_rounds,
        "rounds_results": rounds_results,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task5 last_loop: 检测球 → 循环 pick→place (color × tower)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--color", choices=("blue", "yellow"), required=True,
                   help="球颜色 (决定检测过滤 + get_* 位姿 + 视觉 label)")
    p.add_argument("--tower", choices=("high", "low"), required=True,
                   help="目标仓: high=高位仓 / low=低位仓")
    p.add_argument("--balls", type=int, default=-1,
                   help="强制执行轮数 (覆盖检测结果)。-1=用检测到的球数; "
                        "0=不执行; N>0=强制 N 轮 (压力测试用)")
    p.add_argument("--no-detect", action="store_true", dest="no_detect",
                   help="跳过检测, 必须配合 --balls 使用")
    p.add_argument("--no-prep", action="store_true", dest="no_prep",
                   help="跳过检测前的摆臂 (默认开: 仿 target_* 5 步前 4 步)")
    p.add_argument("--hold", type=float, default=5.0,
                   help="每轮吸气独立保持秒数 (透传给 pick_and_place, 默认 5.0)")
    p.add_argument("--detect-timeout", type=float, default=None,
                   dest="detect_timeout",
                   help="识别轮询总时长 (秒), 拿到球提前返回")
    p.add_argument("--score-min", type=float, default=None,
                   dest="score_min", help="识别最低置信度")
    p.add_argument("--area-min", type=float, default=None,
                   dest="area_min", help="最小归一化面积")
    p.add_argument("--area-max", type=float, default=None,
                   dest="area_max", help="最大归一化面积")
    p.add_argument("--aspect-tol", type=float, default=None,
                   dest="aspect_tol",
                   help="宽高比容差 |aspect-1|≤tol")
    # 视觉闭环取球 (透传给 pick_and_place)
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


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.no_detect and args.balls < 0:
        print(f"  {LOG_PREFIX} [ERROR] --no-detect 必须配合 --balls N 使用 "
              f"(不然脚本不知道该跑几轮)", file=sys.stderr)
        return 2

    client = ArmClient.connect()
    runner = ArmRunner(client)
    t_total_start = time.perf_counter()

    # 解析阈值参数 (CLI 显式传值时覆盖 target_module 默认)
    color_info = _COLOR_TABLE[args.color]
    target_module = __import__(color_info["target_module"], fromlist=[""])

    def _resolve(attr: str, default_name: str):
        v = getattr(args, attr)
        if v is not None:
            return float(v)
        return float(getattr(target_module, default_name))

    run_last_loop(
        client, runner,
        color=args.color,
        tower=args.tower,
        balls=args.balls,
        detect=not args.no_detect,
        prep_pose=not args.no_prep,
        hold_s=args.hold,
        detect_timeout_s=_resolve("detect_timeout", "DETECT_TIMEOUT_S"),
        score_min=_resolve("score_min", "DETECT_SCORE_MIN"),
        area_min=_resolve("area_min", "DETECT_AREA_MIN"),
        area_max=_resolve("area_max", "DETECT_AREA_MAX"),
        aspect_tol=_resolve("aspect_tol", "DETECT_ASPECT_TOL"),
        vision=getattr(args, "vision", False),
        grasp_y_mm=getattr(args, "grasp_y", None),
        vision_fallback=getattr(args, "vision_fallback", True),
        sign_arm=getattr(args, "sign_arm", 1.0),
        sign_x=getattr(args, "sign_x", -1.0),
        vision_timeout=getattr(args, "vision_timeout", 20.0),
    )

    elapsed = time.perf_counter() - t_total_start
    print(f"========== {LOG_PREFIX} 总耗时: {elapsed:.3f} s ==========")
    return 0


if __name__ == "__main__":
    sys.exit(main())
