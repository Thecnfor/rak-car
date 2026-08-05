"""task6 / the_final —— **任务六完整编排** 11 步序列 (tuigan → targets → 双 pick-and-place)。

⚠️ **文件名说明**: 用户 2026-08-04 口语说 "the final" 带空格, Python 模块名不允
   许空格 (import 时语法错), 故实际文件名 ``the_final.py`` (下划线), 调用方
   ``from main.arm.each_task.task6 import the_final``。

按用户 2026-08-04 指定 11 步编排顺序:

  Step  1: tuigan                                    推杆 + 扫牌 (扫到 -120mm, Y=0 触底)
  Step  2: target1                                   底盘后退 9cm + 4 步臂 + OCR → liaobiao1
  Step  3: target2                                   底盘后退 9cm + 4 步臂 + OCR → liaobiao2
  Step  4: runner.suck()                              启动吸气 #1
  Step  5: getzuowu1                                 6 步: y→x→底盘+23cm→arm→hand→y
  Step  6: position1                                 5 步纯臂 (y→arm→x→hand→y)
  Step  7: runner.drop_object()                       放气 #1
  Step  8: runner.suck()                              启动吸气 #2
  Step  9: getzuowu2                                 6 步: y→底盘+15cm→x→arm→hand→y
  Step 10: position2                                 5 步纯臂 (y→arm→x→hand→y)
  Step 11: runner.drop_object()                       放气 #2

终态: position2 完成时 y=-95 + arm=+83° + x=-75 + hand=0° + 真空已断开

⚠️ **编排器架构 (跟 task6 其它自包含脚本不同)**:
  本文件 **必须** import task6 包内其它模块, 不再"自包含"。原因: 编排的本质
  就是把别人串起来, 自包含就无意义。task6 包从未被外部清空过 (task5 清空见
  [[task5-rebuild-2026-07-22]], 但 task6 一直完整), 故跨模块 import 安全。

⚠️ **错误处理策略 (硬件安全优先)**:
  任意 Step 失败 (异常 / return.ok=False) → **立即中止**, 不继续后续步骤,
  不回滚前面已成功的步骤。原因: 硬件在未知状态继续走极易撞墙/撞机构,
  让用户人工复位 + 调日志后再继续比自动跳过更安全。
  - 调用方拿到 ok=False + failed_step + 已成功的 results, 自行判断下一步。

⚠️ **各子模块默认参数来源**:
  本脚本不暴露任何子模块参数 (CLI 极简), 全部走子模块的模块常量默认值:
    - tuigan.PUSH_X_MM = -200, PUSH_Y_MM = 0, ...
    - target1.DEFAULT_BACK_MM = 90, POS_Y_MM = -143, POS_ARM_DEG = -95,
      POS_HAND_DEG = -55, POS_X_TARGET_MM = -121
    - target2.**** (跟 target1 同款)
    - getzuowu1.POS_X_TARGET_MM = -175 (用户改), POS_Y_LOW_MM = -15 (用户改),
      DEFAULT_FORWARD_MM = 230, POS_Y_HIGH_MM = -190, POS_ARM_DEG = -95,
      POS_HAND_DEG = 0
    - getzuowu2.POS_X_TARGET_MM = -170, POS_Y_LOW_MM = -10 (进保护区 ⚠️),
      DEFAULT_FORWARD_MM = 150, POS_Y_HIGH_MM = -190, POS_ARM_DEG = -95,
      POS_HAND_DEG = 0
    - position1.POS_X_TARGET_MM = -15 (用户改), POS_Y_HIGH_MM = -190,
      POS_ARM_DEG = 83, POS_HAND_DEG = 0, POS_Y_LOW_MM = -95
    - position2.POS_X_TARGET_MM = -75, POS_Y_HIGH_MM = -190,
      POS_ARM_DEG = 83, POS_HAND_DEG = 0, POS_Y_LOW_MM = -95
  如需现场调参, 直接改对应子模块的模块常量 (CLI 不再嵌套 --forward --x-target 等)。

⚠️ **本文件自包含程度** (vs task6 其它脚本):
  其它脚本 (tuigan/target1/position1/...): 自包含, 不 import task6 包内任何模块。
  本脚本 (the_final): **编排器**, 必须 import task6 全部子模块 (见下面 import block)。
  仅依赖标准库 + main.arm + task6 子模块, 不引外部业务包。

跑法:
    python main/arm/each_task/task6/the_final.py
    python -m main.arm.each_task.task6.the_final
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

# ===== task6 包内子模块 (编排器必需) =====
# ⚠️ 这打破了 task6 其它脚本的"自包含"约定, 但编排器本质就是 import 别人
from main.arm.each_task.task6 import (  # noqa: E402
    tuigan as tuigan_mod,
    target1 as target1_mod,
    target2 as target2_mod,
    getzuowu1 as getzuowu1_mod,
    getzuowu2 as getzuowu2_mod,
    position1 as position1_mod,
    position2 as position2_mod,
)


# ---------- 序列常量 ----------

LOG_PREFIX: str = "[task6/the_final]"

# 11 步编排 (label, callable) — 按用户 2026-08-04 指定顺序
# 每个 callable 接收 (client, runner) 返回 None 或 dict
_STEPS: list[tuple[str, callable]] = [
    ("tuigan (推杆 + 扫牌, Y 触底)",
     lambda c, r: tuigan_mod.run(c, r)),
    ("target1 (底盘后退 9cm + 4 步臂 + OCR → liaobiao1)",
     lambda c, r: target1_mod.run(c, r)),
    ("target2 (底盘后退 9cm + 4 步臂 + OCR → liaobiao2)",
     lambda c, r: target2_mod.run(c, r)),
    ("suck #1 (启动吸气, 建立真空)",
     lambda c, r: r.suck()),
    ("getzuowu1 (6 步: y→x→底盘+23cm→arm→hand→y)",
     lambda c, r: getzuowu1_mod.run(c, r)),
    ("position1 (5 步纯臂: y→arm→x→hand→y, 终态 y=-95)",
     lambda c, r: position1_mod.run(c, r)),
    ("drop_object #1 (放气, 断开真空)",
     lambda c, r: r.drop_object()),
    ("suck #2 (再次吸气, 建立真空)",
     lambda c, r: r.suck()),
    ("getzuowu2 (6 步: y→底盘+15cm→x→arm→hand→y, 终态 y=-10 ⚠️进保护区)",
     lambda c, r: getzuowu2_mod.run(c, r)),
    ("position2 (5 步纯臂: y→arm→x→hand→y, 终态 y=-95)",
     lambda c, r: position2_mod.run(c, r)),
    ("drop_object #2 (放气, 断开真空, 任务六结束)",
     lambda c, r: r.drop_object()),
]
"""11 步编排定义, 按用户指定顺序。

⚠️ Step 5/9 调子模块 run() 用其模块常量默认值, 不传参 (CLI 极简)。
⚠️ Step 4/7/8/11 是 runner.suck() / runner.drop_object(), 直接走 runner 不调子模块。"""


# ---------- 主流程 ----------

def run(client: ArmClient, runner: ArmRunner) -> dict:
    """完整 11 步任务六编排: tuigan → targets → 双 pick-and-place。

    错误处理: 任意 Step 抛异常或 fail → **立即中止**, 返回 ``ok=False`` +
    ``failed_step`` + 已有 results。**不回滚**, 让用户人工复位 + 查日志。

    Args:
        client: ArmClient
        runner: ArmRunner

    Returns:
        成功时:
        {
            "ok":         True,
            "duration_s": float,                # 总耗时秒
            "results":    {step_idx: {ok, label, result, duration_s}},  # 11 项全 ok
        }
        失败时:
        {
            "ok":           False,
            "duration_s":   float,
            "failed_step":  int,                # 哪一步失败 (1-11)
            "error":        str,                # repr(exc)
            "results":      {...},              # 已成功的步骤 + 失败步骤
        }
    """
    t0 = time.time()
    total = len(_STEPS)
    print(f"\n========== {LOG_PREFIX} run ({total} 步编排) ==========")
    for i, (label, _) in enumerate(_STEPS, 1):
        print(f"  Step {i:2d}: {label}")
    print()

    results: dict = {}
    for i, (label, action) in enumerate(_STEPS, 1):
        step_t0 = time.time()
        print(f"\n{'=' * 60}")
        print(f"─── Step {i:2d}/{total}: {label} ───")
        print(f"{'=' * 60}")
        try:
            result = action(client, runner)
        except Exception as exc:                                # noqa: BLE001
            step_dt = time.time() - step_t0
            results[i] = {
                "ok": False,
                "label": label,
                "error": repr(exc),
                "duration_s": step_dt,
            }
            print(f"\n  ❌ Step {i:2d} 失败 (耗时 {step_dt:.2f}s): {exc!r}")
            print(f"\n========== {LOG_PREFIX} 中止 (Step {i:2d} 失败) ==========")
            print(f"  ⚠️ 硬件在未知状态, 已成功的 Step 不会自动回滚。")
            print(f"     请人工复位 + 查日志后决定下一步。\n")
            return {
                "ok": False,
                "duration_s": time.time() - t0,
                "failed_step": i,
                "error": repr(exc),
                "results": results,
            }

        step_dt = time.time() - step_t0
        results[i] = {
            "ok": True,
            "label": label,
            "result": result,
            "duration_s": step_dt,
        }
        print(f"\n  ✅ Step {i:2d} 完成 (耗时 {step_dt:.2f}s)")

    dt = time.time() - t0
    print(f"\n========== {LOG_PREFIX} 全部完成 ({dt:.2f}s, {total}/{total} 步成功) ==========")
    print(f"  终态: position2 完成 → y=-95 + arm=+83° + x=-75 + hand=0° + 真空已断开")
    print(f"  ⚠️ 真空断开放下位置是各子模块的终态, 后续可继续 runner.move_y/.. 微调。\n")
    return {
        "ok": True,
        "duration_s": dt,
        "results": results,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 极简 (无参数)。

    本脚本不暴露任何子模块参数 (全部走子模块模块常量默认值)。
    现场如需调参, 直接改对应子模块 (tuigan/target1/position1/.../*.py) 的
    顶置常量, 再跑本脚本。
    """
    p = argparse.ArgumentParser(
        description=(
            "task6 the_final: 11 步完整编排 (tuigan → target1 → target2 → "
            "suck+getzuowu1+position1+drop → suck+getzuowu2+position2+drop), "
            "中途失败立即中止"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    result = run(client, runner)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
