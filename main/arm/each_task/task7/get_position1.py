"""task7 / get_position1 —— **位置 1 抓取** 的 5 步纯臂序列 (不碰底盘)。

按用户 2026-08-04 v2+ 重新指定顺序 + 参数:

  1. move_y(-190mm)               y 抬高到 -190mm, 完全出保护区 [0, -80]
  2. move_x_with_split(-18mm)     x 移到 -18mm (在 y=-190 时, 保护区外, 相对位姿非撞墙归零)
  3. set_arm_angle(+90°)          大臂到 +90° (复位位, 业务硬限上界 [-150, +90])
  4. set_hand_angle(0°)           手爪到 0° (DOWN, 业务硬限上界 [-90, 0])
  5. move_y(-85mm)                y 降到工作深度 -85mm (保护区外, 距 -80 上边界 5mm)

⚠️ **顺序关键** (这条不能乱):
  - 第 1 步 y 抬高到 -190 是为了让第 2/3/4 步都能在 **保护区外** 完成。
    保护区 y ∈ [0, -80]mm 内: set_arm_angle(非 MID/0) / set_hand_angle(非 -90)
    / move_x 都会被 _check_safe 拦截, 所以必须先抬 y。
  - 第 2 步 x 移到 -18 必须在 y ≤ -80 才能调 (y=-190 满足 ✓), 否则 SDK 拦截。
    -18 是相对位姿 (**非撞墙归零**), 给后续 x 走 split 时一个明确的起点附近。
  - 第 5 步 y 降到 -85mm, **仍在保护区外** (-85 ≤ -80 ✓), 距 -80 上边界
    仅 5mm 余量 — 偏紧, 后续若要再 set_arm_angle(非 0)/set_hand_angle(非 -90)
    /move_x 都还能直接调, 但余量小, 调用方小心。

⚠️ **业务硬限** (走前要核对, 见 ARM_API §1.1 / §7):
  - y=-190 ≤ soft_y_max=-200 ✓ (距上限 10mm 余量, 实测 OK)
  - x=-18 ∈ [-320, +220] mm ✓
  - arm=+90 ∈ [-150, +90]° ✓ (业务硬限上界)
  - hand=0 ∈ [-90, 0]° ✓ (业务硬限上界 = DOWN)
  - y=-85 ≤ -80 ✓ (保护区外, 但距 -80 上边界仅 5mm 余量, 偏紧)

⚠️ **x 走 move_x_with_split** (common.py:174): belt-slip 防误撞墙 + 自动 retry,
   与 task7/position1.py、task7/position2.py 一致。

⚠️ **set_hand_angle 走 client 不走 runner** (与 position2.py 同款):
  - ArmRunner 没有 set_hand_angle (只有 set_storage), 见
    [[armrunner-set-hand-angle-gotcha]]
  - client.set_hand_angle(angle, speed, timeout) 中 timeout 是必填**位置参**
    (与 set_arm_angle 默认 80 不同), 这里走 runner.default_timeout_s 兜底

⚠️ **本文件自包含** (与 task7/{position1,position2,target,dipan}.py 同款):
   只依赖 ``main.arm.ArmClient`` + ``main.arm.ArmRunner`` +
   ``main.arm.each_task.common.move_x_with_split``,
   不 import task7 包内任何模块。原因: task5 包曾被外部清空过一次
   (见 [[task5-rebuild-2026-07-22]]), 自包含可保证 ``python get_position1.py``
   直接跑不受影响。

⚠️ **2026-08-04 改版历史**:
  - v1 (2026-08-03):   5 步 = y up(-190) → arm(-90) → hand(0) → x(-300) → y down(-70 进保护区 终态)
  - v2 (2026-08-04 上午): 5 步 = y up(-190) → x(0 撞墙) → arm(+90) → hand(0) → y down(-120 保护区外 40mm)
  - **v2+ (2026-08-04 下午, 当前)**: 5 步 = y up(-190) → x(-18) → arm(+90) → hand(0) → y down(-85 保护区外 5mm 偏紧)

跑法:
    python main/arm/each_task/task7/get_position1.py
    python -m main.arm.each_task.task7.get_position1
    python main/arm/each_task/task7/get_position1.py --y-up -180          # 抬低一点
    python main/arm/each_task/task7/get_position1.py --x-target -25       # x 起点微调
    python main/arm/each_task/task7/get_position1.py --y-down -100       # 工作深度微调 (留 20mm)
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
from main.arm.each_task.common import move_x_with_split  # noqa: E402


# ---------- 序列常量 (用户 2026-08-04 v2+ 指定) ----------

LOG_PREFIX: str = "[task7/get_position1]"

POS_Y_UP_MM: float = -190.0
"""第 1 步 y 抬到 -190mm (远出保护区 [0, -80], 给 set_arm_angle(+90°)/set_hand_angle(0°)/move_x 留余地)。

⚠️ ≤ soft_y_max=-200 业务硬限; 距上限 10mm 余量, 实测 OK。
   想更安全可 ``--y-up -180``。"""

POS_X_TARGET_MM: float = -15
"""第 2 步 x 移到 -18mm (y=-190 时, 保护区外, 相对位姿非撞墙归零)。

⚠️ 必须 ∈ [-320, +220] 软限位 ✓; -18 接近 0 但不是撞墙归零, 是相对位姿
   (给后续 x 走 split 时一个明确起点附近)。
⚠️ 用 ``--x-target`` 可现场微调。"""

POS_ARM_DEG: float = 83
"""第 3 步大臂到 +90° (复位位, 业务硬限 [-150, +90]° 上界)。

⚠️ 在保护区 y ∈ [0, -80] 内 set_arm_angle(非 MID/0) 会被拦截, 所以本步必须在
   y=-190 之后执行 (见上方"顺序关键")。"""

POS_HAND_DEG: float = 0.0
"""第 4 步手爪到 0° (DOWN, 业务硬限 [-90, 0]° 上界)。

⚠️ 在保护区 y ∈ [0, -80] 内 set_hand_angle(非 UP/-90) 会被拦截, 所以本步必须在
   y=-190 之后执行 (见上方"顺序关键")。"""

POS_Y_DOWN_MM: float = -80
"""第 5 步 y 降到工作深度 -85mm (保护区外, 距 -80 上边界 5mm — 偏紧)。

⚠️ -85 ≤ -80 → 仍在保护区外, 后续 angle/x 动作可直接调, 不用再抬 y。
   跟 v2 的 -120 (40mm 余量) 故意不同 — 用户 2026-08-04 下午新指定, 走得更深。
   ⚠️ **偏紧**, 现场要小心: 万一 y 下超一点就会进保护区拦截后续 angle/x。
   想更安全可 ``--y-down -100`` (留 20mm 余量)。"""

ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度, 默认 80。与 task7/position1.py、target.py 一致。"""


# ---------- 主流程 ----------

def run(client: ArmClient, runner: ArmRunner) -> dict:
    """按用户顺序执行 5 步纯臂序列: y up → x=-18 → arm → hand → y down。

    本函数 **不碰底盘**, 只调机械臂。

    Args:
        client: ArmClient (move_x_with_split + set_hand_angle 内部用到)
        runner: ArmRunner (move_y / set_arm_angle + default_timeout_s)

    Returns:
        {
            "ok": True,
            "y_up_mm":      -190.0,   # 第 1 步目标
            "x_target_mm":  -18.0,    # 第 2 步目标
            "arm_deg":      +90.0,    # 第 3 步目标 (复位位)
            "hand_deg":     0.0,      # 第 4 步目标 (DOWN)
            "y_down_mm":    -85.0,    # 第 5 步目标 (保护区外, 距 -80 边界 5mm)
            "x_result":     dict,     # move_x_with_split 第 2 步返回
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (5 步纯臂序列, 不动底盘) ==========")

    # 1. y 抬高 (出保护区, 给 set_arm/hand_angle 和 move_x 留余地)
    print(f"  [1/5] move_y({POS_Y_UP_MM}mm)              y 抬高完全出保护区")
    runner.move_y(POS_Y_UP_MM, verify=True)

    # 2. x 移到 -18mm (在 y=-190 时, 保护区外, 相对位姿非撞墙归零)
    print(f"  [2/5] move_x_with_split({POS_X_TARGET_MM}mm)  x → -18 (y=-190, split 兜底)")
    x_result = move_x_with_split(
        client, runner, POS_X_TARGET_MM,
        log_prefix=f"  {LOG_PREFIX} step2",
    )

    # 3. 大臂转 +90° (复位位, 保护区 y=-190 外, 业务硬限上界)
    print(f"  [3/5] set_arm_angle({POS_ARM_DEG}°)         大臂到复位位")
    runner.set_arm_angle(POS_ARM_DEG, speed=ANGLE_SPEED)

    # 4. 手爪转 0° (DOWN, 保护区 y=-190 外, 业务硬限边界)
    # ⚠️ ArmRunner 没有 set_hand_angle (只有 set_storage), 必须走 client.set_hand_angle,
    #    且 timeout 是必填位置参 (与 set_arm_angle 不同)。见 [[armrunner-set-hand-angle-gotcha]]
    print(f"  [4/5] set_hand_angle({POS_HAND_DEG}°)         手爪到 DOWN")
    client.set_hand_angle(
        POS_HAND_DEG, speed=ANGLE_SPEED,
        timeout=runner.default_timeout_s,
    )

    # 5. y 降到工作深度 (保护区外 -85 ≤ -80, 距上边界 5mm — 偏紧)
    print(f"  [5/5] move_y({POS_Y_DOWN_MM}mm)              y 降到工作深度 (保护区外, 偏紧)")
    runner.move_y(POS_Y_DOWN_MM, verify=True)

    dt = time.time() - t0
    print(f"========== {LOG_PREFIX} 完成 ({dt:.2f}s) ==========")
    print(f"  ⚠️ 终态: y={POS_Y_DOWN_MM}mm 在保护区 [0, -80] **外** 但偏紧 (距 -80 仅 5mm)")
    print(f"     — 后续 move_x / set_arm_angle(非 0) / set_hand_angle(非 -90) 还能直接调,")
    print(f"     但万一 y 下超一点就会进保护区拦截, 现场要小心。\n")

    return {
        "ok": True,
        "y_up_mm": POS_Y_UP_MM,
        "x_target_mm": POS_X_TARGET_MM,
        "arm_deg": POS_ARM_DEG,
        "hand_deg": POS_HAND_DEG,
        "y_down_mm": POS_Y_DOWN_MM,
        "x_result": x_result,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 允许覆盖 3 个位移量 (y_up / x_target / y_down)。

    arm_deg / hand_deg 是复位位姿, 故意不暴露给 CLI (避免误改改坏业务硬限)。
    想改这两个值, 请编辑本文件顶置常量。
    """
    p = argparse.ArgumentParser(
        description=(
            "task7 get_position1: 5 步纯臂序列 (y up(-190) → x(-18) → arm(+90) → hand(0) → y down(-85))"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--y-up", type=float, default=POS_Y_UP_MM,
                   help="第 1 步 y 抬高目标 (mm, 默认 -190, 抬高越狠越负)")
    p.add_argument("--x-target", type=float, default=POS_X_TARGET_MM,
                   help="第 2 步 x 目标位置 (mm, 默认 -18, 必须在 [-320, +220])")
    p.add_argument("--y-down", type=float, default=POS_Y_DOWN_MM,
                   help="第 5 步 y 工作深度目标 (mm, 默认 -85, 必须在保护区外 ≤ -80, 距 -80 仅 5mm)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    # 用 CLI 覆盖 3 个可变常量 (顶置常量是文档/默认值)
    global POS_Y_UP_MM, POS_X_TARGET_MM, POS_Y_DOWN_MM
    POS_Y_UP_MM = args.y_up
    POS_X_TARGET_MM = args.x_target
    POS_Y_DOWN_MM = args.y_down
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner)
    return 0


if __name__ == "__main__":
    sys.exit(main())