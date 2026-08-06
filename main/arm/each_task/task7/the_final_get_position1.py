"""task7 / the_final_get_position1 —— **位置 1 抓取** 的 4 步纯臂脚本 (composite_run → y_down → suck → y_up)。

按用户 2026-08-06 新建, 本文件是 ``get_position1.py v4`` 的 1:1 镜像
(命名风格跟 ``the_final.py`` 编排器一致):

  ========== [task7/the_final_get_position1] run (4 步: composite_run → y_down → suck → y_up) ==========
    [1/4] composite_run: arm=86.0° x=0.0mm y=-190.0mm hand=10.0°  speed=80 timeout=30s
    [1/4] ✅ 4 轴并发到位 (~2-3s)

    [2/4] runner.move_y(-92.0mm)  y 下降到抓取深度 (从 -190 → -92, 距离 98mm, 保护区外 12mm, hand=+10° UP 不撞保护区)

    [3/4] runner.suck()  吸气 (开始抓取, y=-92mm + hand=+10° + x=0mm)

    [4/4] runner.move_y(-190.0mm)  y 上升回 init (从 -92 → -190, 距离 98mm, 真空保持, 货物随臂上升)
  ========== 完成 (~3-4s) ==========

逻辑同 ``get_position1.py v4`` —— 4 机联动到位高位 + y 下降到抓取深度 + 吸气 + y 抬回高位。
本文件以 ``the_final_get_position1`` 命名 (跟 ``the_final.py`` / ``the_final_position1-6`` 命名风格统一)。

⚠️ **业务硬限** (走前要核对, 见 ARM_API §1.1 / §7 + setters.py):
  - y=-190 ≤ soft_y_max=-200 ✓ (距上限 10mm, 偏紧但合法)
  - y=-92 ≤ soft_y_max=-200 ✓ (距上限 108mm 余量, 充裕)
  - y=-92 ≤ -80 ✓ (**保护区 [0, -80] 外 12mm**, 后续 suck 安全)
  - x=0 ∈ [-320, +220] mm ✓ (中位, 双向余地最大)
  - arm=+86 ∈ [-150, +150]° ✓ (**距 RIGHT=+90° 边界 4° 余量, 偏紧**)
  - hand=+10 ∈ [-90, +10]° ✓ (业务硬限上界, **正好踩边界**)

⚠️ **Step 2/4 move_y 不撞保护区** (y=-92 在 [0, -80] **外** 12mm):
  - move_y 是纯 y 平移, 不走 set_*_angle, 所以**不查** _check_y_protected。
  - hand=+10° 是 P 姿态上限 (上界), 不是 DOWN 姿态, 不在 _check_y_protected 触发条件。
  - 所以 Step 2 move_y(-92) 和 Step 4 move_y(-190) 都安全。
  - Step 3 runner.suck() 在 y=-92 + hand=+10° 状态下, 不查保护区。

⚠️ **不走 move_x_with_split** (与 task1_seeding / position*.py v5 / target.py / the_final_position*.py 同款):
  - Step 1 composite_run 内部走 SDK move_x_position, **不带** belt-slip retry。
  - Step 2/4 runner.move_y 是纯 y 平移, 不涉及 belt-slip。
  - Step 3 runner.suck 是真空阀, 不涉及 belt-slip。
  - belt-slip 一般表现为单步撞墙/失步, **并发 4 轴** 过程中概率极低
    (大臂/手爪走完后 x/y 同时在动, 编码器基本同步)。

⚠️ **vacuum 保持**:
  - Step 3 runner.suck() 开启真空后, 真空保持, 直到 runner.drop_object() 才放气。
  - 终态 (Step 4 后): y=-190, hand=+10°, **真空仍然 ON**。
  - 后续如需放气, 调 ``runner.drop_object()`` 单独一次即可。

⚠️ **本文件自包含** (与 task7/{position1-3 v5, position5 v4+, target, dipan, get_position1 v4, the_final_position1-6}.py 同款):
  只依赖 ``main.arm.ArmClient`` + ``main.arm.ArmRunner``,
  **不 import** ``main.arm.each_task.common.move_x_with_split`` (本版不用),
  也不 import task7 包内任何模块 (包含 ``get_position1`` 也不引用 —— 本文件是镜像不是 alias)。
  原因: task5 包曾被外部清空过一次 (见 [[task5-rebuild-2026-07-22]]),
  自包含可保证 ``python the_final_get_position1.py`` 直接跑不受影响。

⚠️ **跟 get_position1.py v4 关系** (1:1 镜像, 命名风格同 the_final.py):
  - get_position1.py: 老文件名, v4 是 2026-08-06 现场实测通过的 4 步抓取脚本
  - **the_final_get_position1.py**: 1:1 镜像, 文件名跟 task7/the_final.py 编排器命名风格一致
  - 两个文件**逻辑 + 常量完全相同**, 区别只是文件名 + LOG_PREFIX + 内部 docstring 引用
  - 现场用哪个都行; 如果将来要把位置 1 抓取单独抽出用, 推荐用 ``the_final_get_position1.py``
  - **不要**为了对齐结构去引用 ``from . import get_position1`` —— 违反自包含约定
  - (跟 the_final_position5.py vs position5.py / the_final_position2.py vs position2.py 是同款镜像关系)

⚠️ **改版历史**:
  - **v1 (2026-08-06)**: 首次新建 —— 1:1 镜像 get_position1.py v4 (4 步抓取脚本)。
    跟 the_final_position1-6 是同一命名家族 ("the_final_*"), 但功能定位不同:
    the_final_position* = "位置 N 投递" (含放气), the_final_get_position* = "位置 N 抓取" (含吸气)。

跑法:
    python main/arm/each_task/task7/the_final_get_position1.py
    python -m main.arm.each_task.task7.the_final_get_position1
    python main/arm/each_task/task7/the_final_get_position1.py --y-down -100   # 改 Step 2 grab 深度 (-92 → -100)
    python main/arm/each_task/task7/the_final_get_position1.py --y-up -130      # 改 Step 1/4 高位 (-190 → -130)
    python main/arm/each_task/task7/the_final_get_position1.py --x-target -25   # 改 Step 1 x (0 → -25)
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


# ---------- 终态常量 (用户 2026-08-06 v1 新建, 1:1 镜像 get_position1.py v4) ----------

LOG_PREFIX: str = "[task7/the_final_get_position1]"

# ====== Step 1: 4 机联动 composite_run 终态 ======

POS_Y_UP_MM: float = -190.0
"""Step 1 composite_run 的 y_mm 终态 + Step 4 move_y 回归目标 (-190mm, 高位)。

⚠️ ≤ soft_y_max=-200 ✓; 距上限 10mm, 偏紧但合法。
⚠️ 保护区 [0, -80] **外** 110mm, 给 Step 2 move_y(-92) 留 98mm 下降距离。
⚠️ 用 ``--y-up`` 可现场微调 (更浅 = 离保护区更远)。
⚠️ 跟 get_position1.py v4 的 POS_Y_UP_MM 完全相同 (本文件是 1:1 镜像)。"""

POS_X_TARGET_MM: float = 0.0
"""Step 1 composite_run 的 x_mm 终态 (0mm, 中位)。

⚠️ 必须 ∈ [-320, +220] 软限位 ✓; 0 是正中央, 后续动作双向余地最大。
⚠️ 用 ``--x-target`` 可现场微调。
⚠️ 跟 get_position1.py v4 的 POS_X_TARGET_MM 完全相同。"""

POS_ARM_DEG: float = 86.0
"""Step 1 composite_run 的 arm 角度终态 (+86°, 近 RIGHT 位)。

⚠️ 业务硬限 [-150, +150]°; 距 RIGHT=+90° 边界仅 4° 余量, 偏紧; 改大会抛 ValueError。
⚠️ 故意不暴露给 CLI (避免误改改坏业务硬限边界)。想改请编辑本常量。
⚠️ 跟 get_position1.py v4 的 POS_ARM_DEG 完全相同 (+86°, 近 RIGHT 位)。"""

POS_HAND_DEG: float = 10.0
"""Step 1 composite_run 的 hand 角度终态 (+10°, P 姿态上限)。

⚠️ +10° 正好踩业务硬限上界 [-90, +10]° (2026-08-05 放宽), 改大会抛 ValueError。
⚠️ 故意不暴露给 CLI (避免误改改坏业务硬限边界)。想改请编辑本常量。
⚠️ 跟 get_position1.py v4 的 POS_HAND_DEG 完全相同。"""

# ====== Step 2: y 下降到抓取深度 ======

POS_Y_DOWN_MM: float = -92.0
"""Step 2 move_y 目标 (-92mm, 抓取深度)。

⚠️ **保护区 [0, -80] 外 12mm** (y=-92 < -80), move_y 不查保护区 ✓。
⚠️ ≤ soft_y_max=-200 ✓; 距上限 108mm 余量, 充裕。
⚠️ 与 position5 v4+ POS_Y_DOWN_MM=-85 不同 (position5 是 -85 保护区外 5mm 偏紧,
   the_final_get_position1 是 -92 保护区外 12mm 更宽, **留 hand=+10° UP 状态下更安全的余量**)。
⚠️ 用 ``--y-down`` 可现场微调。
⚠️ 跟 get_position1.py v4 的 POS_Y_DOWN_MM 完全相同。"""

# ====== 通用 ======

ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度 + xy PID speed, 默认 80。与 task7/position*.py / target.py / position5 v4+ / get_position1 v4 一致。"""

COMPOSITE_TIMEOUT_S: float = 30.0
"""Step 1 composite_run 同步超时 (秒)。4 轴并发到位一般 ~2-3s, 给 30s 兜底
(含网络 + job_queue + SDK 内部 4 路 as_completed)。"""

Y_MOVE_TIMEOUT_S: float = 10.0
"""Step 2/4 runner.move_y 同步超时 (秒)。单 y 移动 98mm (-190↔-92) 一般 ~1-2s, 给 10s 兜底。"""

SUCK_TIMEOUT_S: float = 10.0
"""Step 3 runner.suck 同步超时 (秒)。真空阀开启一般 <1s, 给 10s 兜底。"""


# ---------- 主流程 ----------

def run(client: ArmClient, runner: ArmRunner) -> dict:
    """按用户顺序执行 4 步: composite_run → move_y_down → suck → move_y_up。

    本函数 **不碰底盘**, 只调机械臂。

    终态: x=0, y=-190, arm=+86°, hand=+10°, **真空保持 ON**
    (需 runner.drop_object() 才会放气)。

    ⚠️ **2026-08-06 v1 新建**: 1:1 镜像 get_position1.py v4, 命名风格统一为
       ``the_final_*.py``。作为 "位置 1 抓取" 的纯臂脚本 (不碰底盘)。

    Args:
        client: ArmClient (composite_run 在这里)
        runner: ArmRunner (move_y + suck 在这里)

    Returns:
        {
            "ok":              True / False,    # Step 1 composite_run 返回 ok (后续 3 步走 SDK 同步, 异常就抛)
            "step1_composite": dict,            # Step 1 composite_run(arm=+86°, x=0, y=-190, hand=+10°) 原始 job dict
            "step2_y_down":    dict,            # Step 2 move_y(-92) 原始 job dict
            "step3_suck":      dict,            # Step 3 runner.suck() 原始 job dict
            "step4_y_up":      dict,            # Step 4 move_y(-190) 原始 job dict
            "final_pose": {                      # 终态 (预期值, 不重读 state)
                "x_mm":    0.0,
                "y_mm":    -190.0,
                "arm_deg": 86.0,
                "hand_deg": 10.0,
            },
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (4 步: composite_run → y_down → suck → y_up) ==========")

    # ========== Step 1: 4 机联动 composite_run ==========
    # 仿 task1_seeding.py 模式, 4 轴并发到位。y=-190 在保护区 [0, -80] **外** 110mm。
    print(f"  [1/4] composite_run: arm={POS_ARM_DEG}° x={POS_X_TARGET_MM}mm "
          f"y={POS_Y_UP_MM}mm hand={POS_HAND_DEG}°  speed={ANGLE_SPEED} "
          f"timeout={COMPOSITE_TIMEOUT_S:.0f}s")
    step1 = client.composite_run(
        arm=POS_ARM_DEG,
        x_mm=POS_X_TARGET_MM,
        y_mm=POS_Y_UP_MM,
        hand=POS_HAND_DEG,
        speed=ANGLE_SPEED,
        timeout=COMPOSITE_TIMEOUT_S,
    )
    ok1 = (
        isinstance(step1, dict)
        and step1.get("status") == "succeeded"
        and isinstance(step1.get("result"), dict)
        and step1["result"].get("ok", False)
    )
    if not ok1:
        print(f"  [1/4] ❌ composite_run 失败: {step1}")
        return {
            "ok": False,
            "failed_step": "step1_composite_run",
            "step1_composite": step1,
            "step2_y_down": None,
            "step3_suck": None,
            "step4_y_up": None,
            "final_pose": None,
        }
    print(f"  [1/4] ✅ 4 轴并发到位 (~2-3s)")

    # ========== Step 2: runner.move_y(-92) 下降到抓取深度 ==========
    # y=-92 在保护区外 12mm, move_y 不查保护区 (纯 y 平移)。
    # hand=+10° 是 P 姿态上界, 不是 DOWN, 不在 _check_y_protected 触发条件。
    print(f"\n  [2/4] runner.move_y({POS_Y_DOWN_MM}mm)  y 下降到抓取深度 "
          f"(从 {POS_Y_UP_MM:.0f} → {POS_Y_DOWN_MM:.0f}, 距离 {abs(POS_Y_DOWN_MM - POS_Y_UP_MM):.0f}mm, "
          f"保护区外 12mm, hand={POS_HAND_DEG:+.0f}° UP 不撞保护区)")
    step2 = runner.move_y(
        y_mm=POS_Y_DOWN_MM,
        timeout=Y_MOVE_TIMEOUT_S,
        verify=True,
    )

    # ========== Step 3: runner.suck() 吸气 ==========
    # 在抓取位 (y=-92 + hand=+10° + x=0) 启动真空。
    # 真空一旦开启, 会保持 ON 直到 runner.drop_object() 才放气。
    print(f"\n  [3/4] runner.suck()  吸气 (开始抓取, y={POS_Y_DOWN_MM:.0f}mm + hand={POS_HAND_DEG:+.0f}° + x={POS_X_TARGET_MM:.0f}mm)")
    step3 = runner.suck(timeout=SUCK_TIMEOUT_S)

    # ========== Step 4: runner.move_y(-190) 上升回 init ==========
    # 把整个臂抬回高位 (-190), 货物被真空吸住随臂上升。
    # y=-190 在保护区外 110mm, move_y 安全。
    print(f"\n  [4/4] runner.move_y({POS_Y_UP_MM}mm)  y 上升回 init "
          f"(从 {POS_Y_DOWN_MM:.0f} → {POS_Y_UP_MM:.0f}, 距离 {abs(POS_Y_UP_MM - POS_Y_DOWN_MM):.0f}mm, "
          f"真空保持, 货物随臂上升)")
    step4 = runner.move_y(
        y_mm=POS_Y_UP_MM,
        timeout=Y_MOVE_TIMEOUT_S,
        verify=True,
    )

    dt = time.time() - t0
    print(f"\n========== {LOG_PREFIX} 完成 ({dt:.2f}s) ==========")
    print(f"  终态: y={POS_Y_UP_MM:.0f}mm (保护区 [0, -80] **外** 110mm) "
          f"x={POS_X_TARGET_MM:.0f}mm arm={POS_ARM_DEG:+.0f}° hand={POS_HAND_DEG:+.0f}°")
    print(f"     真空保持 ON (需 runner.drop_object() 才会放气)。\n")

    return {
        "ok": True,
        "step1_composite": step1,
        "step2_y_down": step2,
        "step3_suck": step3,
        "step4_y_up": step4,
        "final_pose": {
            "x_mm": POS_X_TARGET_MM,
            "y_mm": POS_Y_UP_MM,
            "arm_deg": POS_ARM_DEG,
            "hand_deg": POS_HAND_DEG,
        },
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 暴露 3 个位移量 (y_up / y_down / x_target) 供现场微调。

    arm_deg / hand_deg 是复位位姿, 故意不暴露给 CLI (避免误改改坏业务硬限)。
    想改这两个值, 请编辑本文件顶置常量。
    """
    p = argparse.ArgumentParser(
        description=(
            "task7 the_final_get_position1 v1: 4 步 = composite_run(arm=+86° x=0 y=-190 hand=+10°)\n"
            "  → move_y(-92) → suck() → move_y(-190)  (1:1 镜像 get_position1.py v4)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--y-up", type=float, default=POS_Y_UP_MM,
                   help="Step 1/4 y_mm 终态 (mm, 默认 -190, 高位, 必须 ≤ -200)")
    p.add_argument("--y-down", type=float, default=POS_Y_DOWN_MM,
                   help="Step 2 move_y 目标 (mm, 默认 -92, 抓取深度, 必须 ≤ -80 保护区外)")
    p.add_argument("--x-target", type=float, default=POS_X_TARGET_MM,
                   help="Step 1 composite_run 的 x_mm 终态 (mm, 默认 0 中位, 必须在 [-320, +220])")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    # 用 CLI 覆盖 3 个可变常量 (顶置常量是文档/默认值)
    global POS_Y_UP_MM, POS_Y_DOWN_MM, POS_X_TARGET_MM
    POS_Y_UP_MM = args.y_up
    POS_Y_DOWN_MM = args.y_down
    POS_X_TARGET_MM = args.x_target
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner)
    return 0


if __name__ == "__main__":
    sys.exit(main())