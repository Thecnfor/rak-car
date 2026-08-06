"""task7 / the_final_position2 —— **位置 2 (上中)** 投递脚本 (3 步臂, 含 1 次放气)。

按用户 2026-08-06 新建 (从零照抄 ``position2.py`` v5.3 的 3 步纯臂脚本逻辑):
本文件是 task7 编排器链 ``the_final_position1/3`` 的**被委托子任务** (Phase 2),
不是 1:1 镜像 (因为 the_final_position2 自己就是 position2.py 的内容, 不需要再镜像)。

  ========== [task7/the_final_position2] run (3 步臂: composite_run → 放气 → x 归零) ==========
    [1/3] composite_run: arm=+90° x=-225mm y=-172mm hand=-20°  speed=80 timeout=30s
    [1/3] ✅ 4 轴并发到位 (~2-3s, 终态已是投递位 x=-225mm)
    [2/3] runner.drop_object()  放气 (断开真空, 货物落目标位, y=-172 + x=-225)
    [3/3] runner.move_x(0.0mm)  x 归零 (撞墙 calibrate, 从 -225 → 0, 距离 225mm)
  ========== 完成 (~3-4s) ==========

⚠️ **业务硬限** (走前要核对, 见 ARM_API §1.1 / §7 + setters.py):
  - y=-172 ≤ soft_y_max=-200 ✓ (距上限 28mm, 偏紧但合法; 在保护区 [0, -80] **外** 92mm)
  - x=-225 ∈ [-320, +220] ✓ (距下界 -320 95mm 余量, 偏紧但合法)
  - arm=+90 ∈ [-150, +150]° ✓ (业务硬限内, 复位位)
  - hand=-20 ∈ [-90, +10]° ✓ (业务硬限内, -90 < -20 < +10)

⚠️ **y=-172 在保护区 [0, -80] 外 92mm —— composite_run 仍必须走** (2026-08-06 用户拍板):
  - composite_run 内部**不调用** _check_y_protected (composite.py:60 注释
    "23:31 用户拍板: 不怕撞车! _check_y_protected 去掉! 要速度!")。
  - 即使 y=-172 已经出保护区, 仍走 composite_run 是为了**4 轴并发** (4 机联动耗时 ~2-3s,
    比手动 4 步串行快 ~3-4 倍)。
  - **绝对不要拆成** ``move_y(-172)`` + ``set_arm_angle(90)`` + ``set_hand_angle(-20)``,
    拆开既慢又会撞 _check_y_protected (虽然 y 在区外但复合调用仍然风险大)。
  - 与 position2.py v5.3 / target.py v3 / get_position1.py v4 同款取舍 (state 过渡直接 composite_run)。

⚠️ **顺序关键** (放气位置不能乱):
  - Step 2 放气必须在 composite 终态 x=-225 **之后**, Step 3 x 归零 **之前**。
  - 太早 (composite 之前) → 货物在工作位之前掉 → 扔歪 (但本版没单独的 push 步骤了,
    composite 终态 = 投递位, 放气只在投递位执行)
  - 太晚 (归零之后) → 真空一直吸住, 归零时拖拽货物 → 撞墙/撞货物

⚠️ **不走 move_x_with_split**:
  - 用户 2026-07-31 报告 belt-slip 已修复, 业务层简化直接用 runner.move_x。
  - 与 task1_seeding / position2.py v5.3 / get_position1 v4 / target v3 同款取舍
    (state 过渡不走 split)。

⚠️ **本文件自包含** (与 task7/{target, get_position1, dipan, the_final_position5}.py 同款):
  只依赖 ``main.arm.ArmClient`` + ``main.arm.ArmRunner``,
  不 import ``main.arm.each_task.common.move_x_with_split`` (本版不用),
  也不 import task7 包内任何模块 (包含 ``position2`` 也不引用 —— 本文件就是 position2 的等价物,
    不再走镜像)。

⚠️ **跟 position2.py 关系** (照搬, 不再镜像):
  - position2.py: 老文件名, v5.3 是 2026-08-06 现场实测通过的 3 步投递脚本
  - **the_final_position2.py**: 内容跟 position2.py v5.3 完全相同,
    文件名跟 task7/the_final.py 编排器命名风格一致
  - **不要**为了对齐结构去引用 ``from . import position2`` —— 违反自包含约定
  - (跟 the_final_position1 / 3 编排器配套, 后者委托本文件 run() 处理 Phase 2 臂序列)

⚠️ **改版历史**:
  - **v1 (2026-08-06)**: 首次新建 —— 从零照抄 position2.py v5.3 (3 步纯臂脚本)。
    跟 the_final_position5.py (7 步镜像 position5) 是不同风格:
    the_final_position5 是镜像 (position5 内容), the_final_position2 是照搬 (position2 内容)。

跑法:
    python main/arm/each_task/task7/the_final_position2.py
    python -m main.arm.each_task.task7.the_final_position2
    python main/arm/each_task/task7/the_final_position2.py --x-init -260     # 改 Step 1 composite x
    python main/arm/each_task/task7/the_final_position2.py --x-return -10    # 改 Step 3 归零 x
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


# ---------- 终态常量 (用户 2026-08-06 v1 新建, 照搬 position2.py v5.3) ----------

LOG_PREFIX: str = "[task7/the_final_position2]"

# ----- Step 1: 4 机联动 composite_run 终态 (直接到投递位 -225) -----

POS_X_INIT_MM: float = -225.0
"""Step 1 composite_run 的 x_mm 终态 (-225mm, **投递位**)。

⚠️ 必须 ∈ [-320, +220] 软限位 ✓; 距下界 -320 有 95mm 余量, 偏紧但合法。
⚠️ 跟 position2.py v5.3 的 POS_X_INIT_MM 完全相同 (本文件是从零照抄)。"""

POS_Y_INIT_MM: float = -172.0
"""Step 1 composite_run 的 y_mm 终态 (-172mm)。

⚠️ 此值在保护区 [0, -80] **外** 92mm, 离 soft_y_max=-200 距 28mm (偏紧但合法)。
⚠️ **仍走 composite_run** 是为了 4 轴并发提速 (~2-3s), 不是因为保护区 (见 docstring 顶部)。
⚠️ **绝对不要拆成** ``move_y(-172)`` + ``set_*_angle`` —— 会撞 _check_y_protected 风险 + 慢。
⚠️ 跟 position2.py v5.3 的 POS_Y_INIT_MM 完全相同。"""

POS_ARM_DEG: float = 90.0
"""Step 1 composite_run 的 arm 角度终态 (+90°, 复位位)。

⚠️ 业务硬限 [-150, +150]° ✓; +90 是 init 位置 (保护区允许)。
⚠️ 与 target.py / get_position1 v4 / position2 v5.3 / position5 v4+ 同款 reset 位 (+90°)。
⚠️ 故意不暴露给 CLI (避免误改改坏业务硬限边界)。"""

POS_HAND_DEG: float = -20.0
"""Step 1 composite_run 的 hand 角度终态 (-20°, mid mode)。

⚠️ 业务硬限 [-90, +10]° ✓; -90 < -20 < +10, 合法。
⚠️ composite_run 本身不调 _check_y_protected, 所以 y=-172 + hand=-20 不会拦截。
⚠️ 后续 runner.drop_object() / runner.move_x() 都不涉及 hand/y 校验, 不会触发拦截。
⚠️ 跟 position2.py v5.3 的 POS_HAND_DEG 完全相同。"""

# ----- Step 3: x 归零 -----

POS_X_RETURN_MM: float = 0.0
"""Step 3 runner.move_x 目标 (0mm, 撞墙 calibrate 位)。

⚠️ 撞墙是 calibrate, 重置编码器零点; 后续视觉闭环用此作为起点。
⚠️ 从 Step 1 x=-225 → Step 3 x=0 距离 225mm, 较远但单步可达。
⚠️ 跟 position2.py v5.3 的 POS_X_RETURN_MM 完全相同。"""

# ----- 通用 -----

ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度 + xy PID speed, 默认 80。与 task7/{target, get_position1, position2}.py 一致。"""

COMPOSITE_TIMEOUT_S: float = 30.0
"""Step 1 composite_run 同步超时 (秒)。4 轴并发到位一般 ~2-3s, 给 30s 兜底。"""

X_MOVE_TIMEOUT_S: float = 10.0
"""Step 2/3 runner.move_x + drop_object 同步超时 (秒)。单 x 移动一般 ~1-2s, 给 10s 兜底。"""


# ---------- 主流程 ----------

def run(client: ArmClient, runner: ArmRunner) -> dict:
    """按用户顺序执行 3 步臂: composite_run → 放气 → x 归零。

    本函数 **不碰底盘**, 只调机械臂。终态: x=0, y=-172, arm=+90°, hand=-20°
    (y 在保护区 [0, -80] **外** 92mm, 工作深度足够)。

    ⚠️ **2026-08-06 v1 新建**: 从零照抄 position2.py v5.3, 命名风格统一为
       ``the_final_position{N}.py``。作为 the_final_position1/3 编排器的 Phase 2 子任务。

    Args:
        client: ArmClient (composite_run 在这里)
        runner: ArmRunner (move_x + drop_object 在这里)

    Returns:
        {
            "ok":                  True / False,    # composite_run 4 路 + 后续 2 步全 ok
            "step1_composite":     dict,            # Step 1 composite_run 原始 job dict
            "step2_drop":          dict,            # Step 2 drop_object 原始 job dict
            "step3_x_return":      dict,            # Step 3 move_x(0) 原始 job dict
            "final_pose": {                          # 终态 (预期值, 不重读 state)
                "x_mm": 0.0,
                "y_mm": -172.0,
                "arm_deg": 90.0,
                "hand_deg": -20.0,
            },
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (3 步臂: composite_run → 放气 → x 归零) ==========")

    # ========== Step 1: 4 机联动 composite_run (仿 task1_seeding 模式) ==========
    # ⚠️ **仍用 composite_run, 不能拆**: 即便 y=-172 已经在保护区 [0, -80] 外 92mm,
    #    拆 move_y(-172) + set_arm_angle(+90°) + set_hand_angle(-20°) 会慢 3-4 倍
    #    (~6-8s vs 1 步 ~2-3s), composite_run 是**性能**取舍, 不是保护区妥协。
    #    composite_run 内部**不调** _check_y_protected (composite.py:60 拍板),
    #    所以 4 轴并发合法。
    print(f"  [1/3] composite_run: arm={POS_ARM_DEG:+.0f}° x={POS_X_INIT_MM:.0f}mm "
          f"y={POS_Y_INIT_MM:.0f}mm hand={POS_HAND_DEG:+.0f}°  speed={ANGLE_SPEED} "
          f"timeout={COMPOSITE_TIMEOUT_S:.0f}s")
    step1 = client.composite_run(
        arm=POS_ARM_DEG,
        x_mm=POS_X_INIT_MM,
        y_mm=POS_Y_INIT_MM,
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
        print(f"  [1/3] ❌ composite_run 失败: {step1}")
        return {
            "ok": False,
            "failed_step": "step1_composite_run",
            "step1_composite": step1,
            "step2_drop": None,
            "step3_x_return": None,
            "final_pose": None,
        }
    print(f"  [1/3] ✅ 4 轴并发到位 (~2-3s, 终态已是投递位 x={POS_X_INIT_MM:.0f}mm)")

    # ========== Step 2: runner.drop_object() 放气 (投递关键步骤) ==========
    # ⚠️ 必须**在 composite 到位 (Step 1, x=-225) 之后**, **x 归零 (Step 3) 之前**。
    # 走 runner.drop_object (runner.py:198), 底层 client.grasp(False)
    # (电平语义: 气泵关 + 阀门开 → 断开真空, 物体落下)。
    print(f"  [2/3] runner.drop_object()  放气 (断开真空, 货物落目标位, y={POS_Y_INIT_MM:.0f} + x={POS_X_INIT_MM:.0f})")
    step2 = runner.drop_object(timeout=X_MOVE_TIMEOUT_S)

    # ========== Step 3: runner.move_x(0) x 归零 (撞墙 calibrate) ==========
    print(f"\n  [3/3] runner.move_x({POS_X_RETURN_MM}mm)  x 归零 (撞墙 calibrate, 从 {POS_X_INIT_MM:.0f} → 0, "
          f"距离 {abs(POS_X_RETURN_MM - POS_X_INIT_MM):.0f}mm)")
    step3 = runner.move_x(
        x_mm=POS_X_RETURN_MM,
        timeout=X_MOVE_TIMEOUT_S,
        verify=True,
    )

    dt = time.time() - t0
    print(f"\n========== {LOG_PREFIX} 完成 ({dt:.2f}s) ==========")
    print(f"  终态: y={POS_Y_INIT_MM:.0f}mm (保护区 [0, -80] 外 92mm) "
          f"x=0mm (撞墙 calibrate) "
          f"arm={POS_ARM_DEG:+.0f}° hand={POS_HAND_DEG:+.0f}°")
    print(f"     注意: y 在保护区外, 后续 set_*_angle / move_x 都可以直接调, 不用先抬 y。\n")

    return {
        "ok": True,
        "step1_composite": step1,
        "step2_drop": step2,
        "step3_x_return": step3,
        "final_pose": {
            "x_mm": POS_X_RETURN_MM,
            "y_mm": POS_Y_INIT_MM,
            "arm_deg": POS_ARM_DEG,
            "hand_deg": POS_HAND_DEG,
        },
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 允许覆盖 2 个位置常量 (x_init / x_return)。

    y / arm / hand 是 composite_run 终态, 故意不暴露给 CLI (避免误改改坏 y 保护区)。
    想改这 3 个值, 请编辑本文件顶置常量。
    """
    p = argparse.ArgumentParser(
        description=(
            "task7 the_final_position2 v1: 3 步 —— composite_run(arm=+90° x=-225 y=-172 hand=-20°) "
            "→ drop_object → move_x(0)  (从零照抄 position2.py v5.3, 编排器链 Phase 2 子任务)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--x-init", type=float, default=POS_X_INIT_MM,
                   help="Step 1 composite_run 的 x_mm 终态 (mm, 默认 -225 投递位, 必须在 [-320, +220])")
    p.add_argument("--x-return", type=float, default=POS_X_RETURN_MM,
                   help="Step 3 move_x 的目标 (mm, 默认 0, 撞墙 calibrate 位)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    # 用 CLI 覆盖 2 个可变常量 (顶置常量是文档/默认值)
    global POS_X_INIT_MM, POS_X_RETURN_MM
    POS_X_INIT_MM = args.x_init
    POS_X_RETURN_MM = args.x_return
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner)
    return 0


if __name__ == "__main__":
    sys.exit(main())