"""task6 / position2 —— **任务六位置 2** 的 5 步纯臂序列 (顺序固定, 不动底盘)。

按用户 2026-08-04 指定顺序 (与 position1.py **完全一致**, 仅 x 目标不同):

  Step 1: runner.move_y(-190mm)                y 抬高到 -190mm (距 soft_y_max=-200 留 10mm 余量)
            ↓
  Step 2: runner.set_arm_angle(+83°)           大臂旋转至 +83° (距业务硬限上界 +90° 留 7° 余量)
            ↓
  Step 3: move_x_with_split(-75mm)             x 滑到 -75mm (距下界 -320 留 245mm 余量) ← **本脚本唯一与 position1 不同**
            ↓
  Step 4: client.set_hand_angle(0°)            手爪舵机 0° (DOWN, 业务硬限上界 0°)
            ↓
  Step 5: runner.move_y(-95mm)                 y 下降到 -95mm (距保护区上界 -80 留 15mm 余量)
            ↓
  终态: y=-95 + arm=+83° + x=-75 + hand=0°

⚠️ **业务硬限 / 保护区逐项核对** (走前要核对, 见 ARM_API §1.1 / §7):
  - y=-190 ≤ soft_y_max=-200 ✓           距上限 10mm 余量
  - arm=+83 ∈ [-150, +90]° ✓              距上界 +90° 还有 7°, 距下界 -150° 远
  - x=-75 ∈ [-320, +220] mm ✓             距下界 -320 还有 245mm, 距上界 +220 还有 295mm
  - hand=0 ∈ [-90, 0]° ✓                  正好上界 (DOWN 位)
  - y=-95 ≤ -80 ✓                          保护区外 (保护区 y ∈ [0, -80])

⚠️ **顺序关键** (与 position1.py 完全一致, 不可调换):
  - Step 1 y=-190 抬高到保护区外 + 离上限 10mm, 让后续 arm/x/hand 全在保护区外执行。
  - Step 2 arm=+83°: y=-190 保护区外, wrapper 放行。
  - Step 3 x=-75: y=-190 保护区外, move_x_with_split wrapper 放行。
  - Step 4 hand=0° (DOWN): 此时 y=-190 保护区外, wrapper 放行。
                            hand=0° (DOWN) 是保护区 [0, -80] 内的**唯一例外**, 但
                            这里 y=-190 始终在外, 走 wrapper 完全合法。
  - Step 5 y=-95: move_y 从不被保护区拦, 终点 y=-95 仍在保护区外, 安全。

⚠️ **为什么 hand=0° 走 client 不走 runner**:
  ArmRunner 没有 ``set_hand_angle`` (只有 ``set_storage_angle``), 必须走
  ``client.set_hand_angle(angle, speed, timeout=...)``, 且 ``timeout`` 是必填位置参
  (与 ``set_arm_angle`` 默认值不同)。见 [[armrunner-set-hand-angle-gotcha]]。

⚠️ **与 task6/position1.py 的唯一区别** (用户 2026-08-04 指定):
  - position1.py: x=-18mm (近 0 位, 适合"近点"场景)
  - position2.py: **x=-75mm** (中距离, 适合"中点"场景)
  - 其余 4 步 (y / arm / hand / y 终态) **完全一致**。
  - 两个脚本同款 5 步纯臂, 共享同一套 5 步骨架, 仅 X 目标不同。

⚠️ **与 task6/target2.py 的区别**:
  - target2.py = 4 步 (y → arm → hand → x), 终态保持 y 不下降, 后续做 OCR 写 liaobiao2
  - position2.py = 5 步 (y → arm → x → hand → y), 终态 y=-95 (近保护区), 无 OCR
  - 两者场景不同: target2 = 主抓取位姿, position2 = 备选/调整位姿。

⚠️ **本文件自包含** (与 task6/{tuigan, wenzishibie, position1}.py + task7/{position*.py} 同款):
   只依赖 ``main.arm.ArmClient`` + ``main.arm.ArmRunner`` +
   ``main.arm.each_task.common.move_x_with_split``,
   不 import task6 包内任何模块。原因: task5 包曾被外部清空过一次
   (见 [[task5-rebuild-2026-07-22]]), 自包含可保证 ``python position2.py``
   直接跑不受影响。

跑法:
    python main/arm/each_task/task6/position2.py
    python -m main.arm.each_task.task6.position2
    python main/arm/each_task/task6/position2.py --x-target -50   # x 不到那么远
    python main/arm/each_task/task6/position2.py --x-target -100  # x 更远
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


# ---------- 默认参数 (用户 2026-08-04 现场指定, 与 position1.py 仅 x 不同) ----------

LOG_PREFIX: str = "[task6/position2]"

# ==== Step 1: y 抬高 (出保护区, 离上限留 10mm 余量) ====
POS_Y_HIGH_MM: float = -190.0
"""Step 1: y 抬到 -190mm (距业务硬限上界 soft_y_max=-200 留 10mm 余量)。

⚠️ -190 ≤ soft_y_max=-200 ✓ (10mm buffer)
⚠️ -190 ≤ -80 ✓ (保护区外, 后续 arm/x/hand 全安全)
⚠️ 与 position1.py 完全一致。
⚠️ 起点 y 若已在保护区 (y > -80), move_y(-190) 自动抬出保护区, 无需软抢答。"""

# ==== Step 2: 大臂到 +83° (大扭矩动作, runner 默认 sleep 由 SDK 处理) ====
POS_ARM_DEG: float = 83.0
"""Step 2: 大臂旋转至 +83° (距业务硬限上界 +90° 留 7° 余量)。

⚠️ +83 ∈ [-150, +90]° ✓ (距上界 7°, 距下界 233°)
⚠️ 与 position1.py 完全一致。
⚠️ runner.set_arm_angle 默认 speed=80, 大扭矩动作; 用户 CLI 暂不暴露 speed。"""

# ==== Step 3: x 滑到 -75mm (本脚本与 position1.py 唯一不同) ====
POS_X_TARGET_MM: float = -75.0
"""Step 3: x 滑到 -75mm (距下界 -320 留 245mm 余量)。

⚠️ **本脚本与 position1.py 的唯一区别**: position1 x=-18 (近 0 位),
   本脚本 x=-75 (中距离, 距 position1 多走 57mm)。
⚠️ -75 ∈ [-320, +220] mm ✓
⚠️ 走 ``move_x_with_split`` (belt-slip / wall_hit / overshoot 检测),
   兜底 ARM_API §9.1。"""

# ==== Step 4: 手爪到 0° (DOWN 位, 业务硬限上界) ====
POS_HAND_DEG: float = 0.0
"""Step 4: 手爪舵机 0° (DOWN 位, 正好业务硬限上界 [-90, 0]°)。

⚠️ hand=0° = DOWN, 是保护区 y ∈ [0, -80] 内 set_hand_angle **唯一例外**。
   但本脚本 y=-190 始终在保护区外, wrapper 放行无问题, 不需要 tuigan.py
   那种底层直调。
⚠️ 与 position1.py 完全一致。
⚠️ 必须走 ``client.set_hand_angle(angle, speed, timeout=...)`` (ArmRunner 没
   set_hand_angle, timeout 必填位置参)。见 [[armrunner-set-hand-angle-gotcha]]"""

# ==== Step 5: y 下降到 -95mm (终态, 保护区外留 15mm 余量) ====
POS_Y_LOW_MM: float = -95.0
"""Step 5: y 下降到 -95mm (距保护区上界 -80 留 15mm 余量)。

⚠️ -95 ≤ -80 ✓ (保护区外, 后续可以再调 move_y 不被拦)
⚠️ 终态保持 y=-95 + arm=+83° + x=-75 + hand=0°, 适合"贴近地面"动作。
⚠️ y 从 -190 → -95 是 +95mm 下降, runner.move_y 内部 PID 闭环。
⚠️ 与 position1.py 完全一致。"""

# ==== 时序常量 ====
ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度, 默认 80。与 task6/{target1, target2, tuigan, position1}.py、
task7/*.py 一致。"""


# ---------- 主流程 ----------

def run(client: ArmClient, runner: ArmRunner,
        y_high_mm: float = POS_Y_HIGH_MM,
        arm_deg: float = POS_ARM_DEG,
        x_target_mm: float = POS_X_TARGET_MM,
        hand_deg: float = POS_HAND_DEG,
        y_low_mm: float = POS_Y_LOW_MM) -> dict:
    """5 步纯臂序列 (顺序固定 y → arm → x → hand → y, 用户 2026-08-04 硬指定)。

    与 task6/position1.py 共享 5 步骨架, **仅 x 目标不同**:
      - position1.run(x_target_mm=-18)
      - position2.run(x_target_mm=-75)  ← 本脚本默认

    与 task6/target2.py 的核心区别:
      - target2 = 4 步 (y → arm → hand → x), 终态 y 不下降, 后续做 OCR 写 liaobiao2
      - position2 = 5 步 (y → arm → x → hand → y), 终态 y=-95 (近保护区), 无 OCR

    Args:
        client:    ArmClient (move_x_with_split + set_hand_angle)
        runner:    ArmRunner (move_y + set_arm_angle)
        y_high_mm:  Step 1 y 抬高目标 (mm, 默认 -190, 距上限 10mm)
        arm_deg:   Step 2 大臂角度 (°, 默认 +83, 距上界 7°)
        x_target_mm: Step 3 x 目标位置 (mm, 默认 **-75**, 与 position1 的 -18 不同)
        hand_deg:  Step 4 手爪角度 (°, 默认 0 = DOWN)
        y_low_mm:  Step 5 y 下降目标 (mm, 默认 -95, 距保护区 15mm)

    Returns:
        {
            "ok": True,
            "y_high_mm": float,
            "arm_deg":   float,
            "x_target_mm": float,
            "hand_deg":  float,
            "y_low_mm":  float,
            "x_result":  dict,    # Step 3 split 结果
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (5 步纯臂: y → arm → x → hand → y) ==========")
    print(f"  Step 1: y={y_high_mm:.0f}mm")
    print(f"  Step 2: arm={arm_deg:.0f}°")
    print(f"  Step 3: x={x_target_mm:.0f}mm  ← position2 vs position1 唯一不同 (-75 vs -18)")
    print(f"  Step 4: hand={hand_deg:.0f}°")
    print(f"  Step 5: y={y_low_mm:.0f}mm")

    # ==== Step 1: y 抬高到 -190 (出保护区 + 离上限 10mm) ====
    print(f"\n  ── Step 1: y → {y_high_mm:.0f}mm (出保护区, 距上限 10mm) ──")
    runner.move_y(y_high_mm, verify=True)

    # ==== Step 2: 大臂旋转至 +83° (保护区外, wrapper 放行) ====
    print(f"\n  ── Step 2: arm → {arm_deg:.0f}° (距上界 +90° 留 7°) ──")
    runner.set_arm_angle(arm_deg, speed=ANGLE_SPEED)

    # ==== Step 3: x 滑到 -75 (y=-190 保护区外, split 兜底) ====
    print(f"\n  ── Step 3: x → {x_target_mm:.0f}mm (中距离, 距下界 -320 留 245mm, split 兜底) ──")
    x_result = move_x_with_split(
        client, runner, x_target_mm,
        log_prefix=f"  {LOG_PREFIX} step3",
    )

    # ==== Step 4: 手爪 0° (DOWN, y=-190 保护区外, 走 client) ====
    # ⚠️ ArmRunner 没有 set_hand_angle, 必须走 client.set_hand_angle,
    #    且 timeout 是必填位置参 (与 set_arm_angle 默认值不同)。
    #    见 [[armrunner-set-hand-angle-gotcha]]
    print(f"\n  ── Step 4: hand → {hand_deg:.0f}° (DOWN, y=-190 保护区外) ──")
    client.set_hand_angle(
        hand_deg, speed=ANGLE_SPEED,
        timeout=runner.default_timeout_s,
    )

    # ==== Step 5: y 下降到 -95 (终态, 距保护区 15mm) ====
    print(f"\n  ── Step 5: y → {y_low_mm:.0f}mm (距保护区上界 -80 留 15mm) ──")
    runner.move_y(y_low_mm, verify=True)

    dt = time.time() - t0
    print(f"\n========== {LOG_PREFIX} 完成 ({dt:.2f}s) ==========")
    print(f"  终态: y={y_low_mm:.0f}mm + arm={arm_deg:.0f}° + "
          f"x={x_target_mm:.0f}mm + hand={hand_deg:.0f}°")
    print(f"  ⚠️ y=-95 距保护区上界 -80 留 15mm 余量, 后续可继续 runner.move_y(...) 微调。\n")

    return {
        "ok": True,
        "y_high_mm": y_high_mm,
        "arm_deg": arm_deg,
        "x_target_mm": x_target_mm,
        "hand_deg": hand_deg,
        "y_low_mm": y_low_mm,
        "x_result": x_result,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 5 个位置常量全暴露, 默认值与模块常量一致 (x=-75, 与 position1 的 -18 不同)。

    参数命名跟模块常量对齐:
      --y-high   / --arm   / --x-target   / --hand   / --y-low
    """
    p = argparse.ArgumentParser(
        description=(
            "task6 position2: 5 步纯臂序列 "
            "(y=-190 → arm=+83° → x=-75mm → hand=0° → y=-95), 不动底盘, 不做 OCR"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--y-high", type=float, default=POS_Y_HIGH_MM,
                   dest="y_high",
                   help="Step 1 y 抬高目标 (mm, 默认 -190, 距 soft_y_max=-200 留 10mm)")
    p.add_argument("--arm", type=float, default=POS_ARM_DEG,
                   help="Step 2 大臂角度 (°, 默认 +83, 距业务硬限上界 +90° 留 7°)")
    p.add_argument("--x-target", type=float, default=POS_X_TARGET_MM,
                   dest="x_target",
                   help=("Step 3 x 目标位置 (mm, 默认 **-75**, 与 position1.py 的 -18 不同, "
                         "必须在 [-320, +220])"))
    p.add_argument("--hand", type=float, default=POS_HAND_DEG,
                   help="Step 4 手爪角度 (°, 默认 0 = DOWN)")
    p.add_argument("--y-low", type=float, default=POS_Y_LOW_MM,
                   dest="y_low",
                   help="Step 5 y 下降目标 (mm, 默认 -95, 距保护区上界 -80 留 15mm)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args()
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner,
        y_high_mm=args.y_high,
        arm_deg=args.arm,
        x_target_mm=args.x_target,
        hand_deg=args.hand,
        y_low_mm=args.y_low)
    return 0


if __name__ == "__main__":
    sys.exit(main())
