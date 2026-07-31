"""task5 / blue_target —— **高仓色标识别 → 蓝/黄分支分发到 last_*_to_high**。

(2026-07-30 新建; PPT Slide 10 全流程"识别 → 决策 → 执行"的 dispatcher。)

业务流程 (用户 2026-07-30 指定):
  1. **先识别高仓色标**: 走 `target.py` 的 detect_high_tower_color,
     抓侧摄 JPEG → HSV 阈值 → 判定 "blue" / "yellow" / "unknown"。
  2. **决策分发**:
     - blue   → 调 ``last_blue_to_high.main()`` (从车载仓取蓝球 → 放高仓)
     - yellow → 调 ``last_yellow_to_high.main()`` (从车载仓取黄球 → 放高仓)
     - unknown → 退出码 3 (色标未识别, 让现场人工重摆车体或调 ROI)

⚠️ **target 识别的语义** (重要, 别和球识别搞混):
  - **target = 高仓色标** (高仓本身贴的蓝/黄标签, PPT Slide 10)
  - 与 ball detection (target_blue/target_yellow 的 detect_balls) **无关**
  - 检测函数: `target.detect_high_tower_color` (HSV 阈值, 不是模型推理)

⚠️ **last_*_to_high 内部仍会再做一次球识别** (球 detection, 不是高仓 detection):
  - last_blue_to_high → target_blue.detect_balls (color_filter="blue")
  - last_yellow_to_high → target_yellow.detect_balls (color_filter="yellow")
  - 即: 先确定高仓是哪个色 (本脚本) → 再从车载仓取同色球放进高仓 (last_*_to_high)
  - 这是 PPT Slide 10 的完整任务链

⚠️ **位姿继承**:
  - 本脚本运行 `target.run()` 会把车摆到 x=0 / arm=90° / hand=-90° / y=0。
  - 之后 last_blue/last_yellow 的 _prep_pose 会再走一次
    y → x → 大臂 → 手爪 (target_blue/yellow 位姿), 中间会自然过渡
    (手爪 -90° → 0° / 大臂 90° → 85° 等)。
  - 不要担心"摆臂到 target 位姿后立即被 last_* 覆盖"——这是预期流程。

⚠️ **手动覆盖 `--color`**:
  - 默认 auto = 靠 target 检测; --color blue/yellow = 强制指定, 跳过检测
  - 必须配 `--no-detect` (检测关了就没数据源, 必须显式给)
  - 用于: ROI 调试阶段 / 现场色标被遮时绕过

⚠️ **本文件自包含** (task5/__init__.py 提到的"曾被外部清空"防御):
  - 只依赖 ``main.arm`` (ArmClient/ArmRunner) + ``task5/target.py``
    (高仓色标识别) + ``task5/last_blue_to_high.py`` / ``last_yellow_to_high.py``
    (球抓取 + 放置)
  - 不重复实现 detect_high_tower_color / _last_loop 骨架

跑法:
    python main/arm/each_task/task5/blue_target.py                # 自动识别 → 分发
    python main/arm/each_task/task5/blue_target.py --balls 2      # 强制 2 轮 (覆盖检测)
    python main/arm/each_task/task5/blue_target.py --color blue   # 手动指定蓝
    python main/arm/each_task/task5/blue_target.py --no-detect --color yellow --balls 1
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

import main.arm.each_task.task5.target as target_module  # noqa: E402
import main.arm.each_task.task5.last_blue_to_high as last_blue_module  # noqa: E402
import main.arm.each_task.task5.last_yellow_to_high as last_yellow_module  # noqa: E402


# ---------- 常量 ----------

LOG_PREFIX: str = "[task5/blue_target]"

VALID_COLORS = ("blue", "yellow", "auto")
"""--color 合法值: blue / yellow / auto (auto=靠 target 检测, 默认)"""

# last_*_to_high 的 build_parser() 接受的 flag 名, blue_target 透传。
# 其它 flag (--cam / --roi / --color-timeout) 是 target 识别专用, 不透传。
LAST_PASSTHROUGH_FLAGS = (
    "balls", "no_detect", "hold",
    "detect_timeout", "score_min", "area_min", "area_max", "aspect_tol",
    "no_prep",
)

EXIT_OK = 0
EXIT_UNKNOWN_COLOR = 3          # 色标未识别
EXIT_COLOR_MISMATCH = 4         # 检测色 ≠ --color 强制值
EXIT_DISPATCH_FAIL = 5          # 调 last_* 失败


# ---------- 主流程 ----------

def _detect_target_color(client: ArmClient, runner: ArmRunner,
                         args: argparse.Namespace) -> str:
    """跑 target.run() 取高仓色标, 返回 "blue" / "yellow" / "unknown"。

    target.run() 内部会做 x=0 / arm=90 / hand=-90 / y=0 全套摆位 + 色标识别。
    即便色标识别失败, 它仍然返回 (color_info 兜底为 {"color": "unknown", "error": ...})。

    Args:
        client / runner: ArmClient / ArmRunner 实例。
        args: blue_target 解析后的 Namespace (取 cam / roi / color_timeout 透传给 target)。

    Returns:
        "blue" / "yellow" / "unknown"
    """
    print(f"\n========== {LOG_PREFIX} [阶段 1/2] 高仓色标识别 ==========")
    print(f"  cam={args.cam}  roi={args.roi}  timeout={args.color_timeout}s")
    # target.run() 返回 {"ok": bool, "color_info": dict | None, ...}
    result = target_module.run(
        client, runner,
        # target 默认值 (x=0 / arm=90 / hand=-90 / y=0) 直接走模块常量,
        # 不暴露给 blue_target CLI —— 检测前的摆位是 target 自己的事。
        detect_color=True,
        cam=args.cam,
        roi=args.roi,
        color_timeout=args.color_timeout,
    )
    color_info = result.get("color_info") if isinstance(result, dict) else None
    if not isinstance(color_info, dict):
        print(f"  {LOG_PREFIX} [FAIL] target.run() 没返回 color_info: {color_info!r}")
        return "unknown"
    color = str(color_info.get("color", "unknown")).lower()
    print(f"  {LOG_PREFIX} 检测结果: color={color!r}  "
          f"blue_ratio={color_info.get('blue_ratio', 0.0):.3f}  "
          f"yellow_ratio={color_info.get('yellow_ratio', 0.0):.3f}")
    if "error" in color_info:
        print(f"  {LOG_PREFIX} [WARN] target.run() 内部识别报错: {color_info['error']}")
    return color


def _build_last_argv(args: argparse.Namespace) -> list:
    """把 blue_target Namespace 转成 last_blue_to_high / last_yellow_to_high
    接受的 argv list (只保留 LAST_PASSTHROUGH_FLAGS)。

    关键:
      - --no-detect 在 blue_target 是 "跳过 target 识别" (手动 --color 必传);
        在 last_* 是 "跳过球识别" (--balls 必传)。
        ⚠️ 同一个 flag 不同语义 → 不能直接透传, 必须按需重设。
        处理: blue_target 的 --no-detect 强制设 True (让 last_* 用 --balls);
        否则 (默认) → 让 last_* 自己 detect (蓝色才决定几轮)。
    """
    argv = []
    if args.balls is not None and args.balls >= 0:
        argv += ["--balls", str(args.balls)]
    if args.no_detect:
        # blue_target 关掉了 target 识别 → 但 last_* 的 --no-detect
        # 是关球识别, 语义不同。这里**不**透传 --no-detect 给 last_*,
        # 让它照常做球识别 (更稳)。如果 blue_target 用户要"两阶段都跳过识别",
        # 应该用 --balls 强制指定轮数。
        pass
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
    return argv


def _dispatch(color: str, args: argparse.Namespace) -> int:
    """根据色标结果分发到 last_blue_to_high 或 last_yellow_to_high。

    Returns:
        last_* main() 返回码 (0=OK); 调用失败 → EXIT_DISPATCH_FAIL。
    """
    print(f"\n========== {LOG_PREFIX} [阶段 2/2] 分发执行 ==========")
    last_argv = _build_last_argv(args)
    if color == "blue":
        print(f"  分发: blue → last_blue_to_high (argv={last_argv})")
        try:
            rc = last_blue_module.main(last_argv)
        except Exception as e:
            print(f"  {LOG_PREFIX} [FAIL] last_blue_to_high 抛异常: "
                  f"{type(e).__name__}: {e}")
            return EXIT_DISPATCH_FAIL
        return int(rc) if rc is not None else EXIT_OK
    if color == "yellow":
        print(f"  分发: yellow → last_yellow_to_high (argv={last_argv})")
        try:
            rc = last_yellow_module.main(last_argv)
        except Exception as e:
            print(f"  {LOG_PREFIX} [FAIL] last_yellow_to_high 抛异常: "
                  f"{type(e).__name__}: {e}")
            return EXIT_DISPATCH_FAIL
        return int(rc) if rc is not None else EXIT_OK
    print(f"  {LOG_PREFIX} [FAIL] 未识别色标 ({color!r}), 不分发")
    return EXIT_UNKNOWN_COLOR


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task5 blue_target: 高仓色标识别 → 蓝/黄分支 → last_*_to_high",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # ---- 高仓色标识别 ----
    p.add_argument("--color", choices=VALID_COLORS, default="auto",
                   help="高仓色标: blue/yellow/auto (auto=target 检测)")
    p.add_argument("--no-detect", action="store_true", dest="no_detect",
                   help="跳过 target 识别 (必须配合 --color blue/yellow)")
    p.add_argument("--cam", default=target_module.DEFAULT_CAM,
                   help=f"色标识别相机 (默认 {target_module.DEFAULT_CAM}=side)")
    p.add_argument("--roi", type=int, nargs=4, default=None,
                   metavar=("X", "Y", "W", "H"),
                   help=f"色标识别 ROI (像素), 默认 = target.DEFAULT_ROI "
                        f"({target_module.DEFAULT_ROI})")
    p.add_argument("--color-timeout", type=float,
                   default=target_module.JPEG_FETCH_TIMEOUT_S,
                   dest="color_timeout",
                   help="抓 JPEG HTTP 超时 (秒)")
    # ---- 透传给 last_*_to_high 的字段 ----
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
    return p


def main(argv=None) -> int:
    t_total = time.perf_counter()
    args = build_parser().parse_args(argv)
    print(f"========== {LOG_PREFIX} run ==========")
    print(f"  --color={args.color}  --no_detect={args.no_detect}  "
          f"--balls={args.balls}")

    # ---- 校验 --no-detect 必带 --color ----
    if args.no_detect and args.color == "auto":
        print(f"  {LOG_PREFIX} [ERROR] --no-detect 必须配合 --color blue/yellow "
              f"(auto 没数据源)", file=sys.stderr)
        return 2

    client = ArmClient.connect()
    runner = ArmRunner(client)

    # ---- 阶段 1: 决定色标 ----
    if args.no_detect:
        color = args.color
        print(f"  跳过 target 识别, 用 --color={color}")
    else:
        color = _detect_target_color(client, runner, args)
        # 检测结果与 --color 强制值冲突 → 拒绝执行 (防止覆盖现场配置)
        if args.color != "auto" and color != args.color:
            print(f"  {LOG_PREFIX} [ERROR] 检测 {color!r} ≠ --color {args.color!r}, "
                  f"拒绝执行 (避免覆盖现场决策)", file=sys.stderr)
            elapsed = time.perf_counter() - t_total
            print(f"========== {LOG_PREFIX} 总耗时: {elapsed:.3f} s ==========")
            return EXIT_COLOR_MISMATCH

    # ---- 阶段 2: 分发 ----
    rc = _dispatch(color, args)

    elapsed = time.perf_counter() - t_total
    print(f"========== {LOG_PREFIX} 完成 (color={color}, rc={rc}, {elapsed:.3f}s) ==========\n")
    return rc


if __name__ == "__main__":
    sys.exit(main())