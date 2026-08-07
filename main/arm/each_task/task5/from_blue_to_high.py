"""task5 / from_blue_to_high —— 「取蓝 → 投高塔」2 阶段串联 (单文件, 非包)。

业务流程 (2026-08-08 用户指定):
  Phase A: ``new_get_blue.run(...)``  4 机联动到取蓝位姿 → 吸气 → 下探 (球吸住)
           终态: x=0mm, y=-74mm, arm=+85°, hand=+10°, 真空阀 ON (球在手)
  Phase B: ``high_tower.run(...)``    y 抬到 -115 → 4 机联动到投球位姿 → x 伸进塔
           → 结束吸气 (释放真空, 球入塔) → x 退到 -70
           终态: x=-70mm, y=-185mm, arm=90°, hand=-82°, 真空阀 OFF (球入塔)

⚠️ **使用原则 (用户要求)**:
  - **不要直接调用** new_get_blue / high_tower 内部的具体动作 (move_y / composite_run / move_x / grasp)
  - **调用它们的 run() 函数**: 这是这两个文件已封装好的"端到端流程",
    里面已包含完整的参数 / 逻辑过程 (业务硬限校验, composite_run 4 轴全传, grasp 走 runner.* 等)
  - 本文件只负责**串联 + 错误透传 + 参数透传**, 不重写任何具体步骤
  - 沿用 task5 其它文件的自包含约定 (虽 import task5 兄弟模块的 run,
    但 import 路径走 main/ 添加 sys.path, 不依赖 task5 包内部 __init__)

⚠️ **过渡位姿衔接 (隐式, 已在 new_get_blue / high_tower 内处理)**:
  - Phase A 终态: y=-74 (保护区 [0,-80] 内, 球吸住)
  - Phase B 步骤 1: y=-115 (出保护区 [0,-80] 35mm)
  - 高塔投球 y=-185 (出保护区 105mm)
  - 衔接动作: high_tower 步骤 1 move_y 自动把 y 从 -74 抬到 -115
    (move_y 走步进电机, 允许从保护区出到 [0,-80] 外)
  - ⚠️ arm/hand/x 衔接: Phase A 末态 arm=+85°/hand=+10°/x=0
    → Phase B 步骤 2 composite_run 把 arm→90°/hand=-82°/x→-70
    → composite_run 内部 4 轴并发, 接管衔接
  - 全程**真空阀不释放**: Phase A 步骤 2 grasp(True) 吸住球 → Phase B 步骤 4 grasp(False) 释放
    中间无 grasp(False), 球不会掉

⚠️ **业务硬限校验 (ARM_API.md §1.1 / setters.py:45)**:
  - new_get_blue 的位姿 (x=0, y=-74/-135, arm=85°, hand=+10°) ✓ 业务硬限内
  - high_tower 的位姿 (x=-70/-135, y=-115/-185, arm=90°, hand=-82°) ✓ 业务硬限内
  - 详见 new_get_blue.py 与 high_tower.py 的 docstring

⚠️ **本文件自包含 (task5 约定)**: 只依赖 ``main.arm`` (ArmClient/ArmRunner),
   **import task5 兄弟模块的 run() 函数** (不 import 内部常量/具体步骤),
   沿用 task5 业务层自包含约定 — task5 辅助文件曾被外部动作清空过,
   本文件 import 两个稳定的兄弟 run() 而非重写, 保证直接跑不受影响。

跑法:
    python main/arm/each_task/task5/from_blue_to_high.py
    python -m main.arm.each_task.task5.from_blue_to_high
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# 1) 把 repo 根加进 sys.path, 让 "main.arm" / "main.arm.each_task.task5.new_get_blue" 可解析
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

# 2) 业务硬限: 只 import task5 兄弟模块的 run() 函数, 不 import 内部常量 / 具体步骤
# ⚠️ 不直接 import main.arm.ArmClient/ArmRunner 之外的 task5 包内具体模块
#    (constants.py / grasp_5 / *_tower 内部步骤), 只调它们的 run() 入口。
from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.each_task.task5 import new_get_blue, high_tower  # noqa: E402


# ---------- 流程常量 (仅串联层, 内部参数沿用 new_get_blue / high_tower 默认值) ----------

LOG_PREFIX: str = "[task5/from_blue_to_high]"


# ---------- 主入口 ----------

def run(client: ArmClient, runner: ArmRunner) -> dict:
    """取蓝 → 投高塔 2 阶段串联流程。

    业务流程:
      Phase A: ``new_get_blue.run(client, runner)``  (3 步: 4 机联动 → 吸气 → 下探)
      Phase B: ``high_tower.run(client, runner)``    (5 步: y 抬升 → 4 机联动 → x 伸进
                                                      → 释放 → x 退回)

    终态 (来自 high_tower 末态):
      x=-70mm, y=-185mm, arm=90°, hand=-82°, 真空阀 OFF (球已入高塔)。

    Args:
        client: ArmClient (composite_run + http 在这里)
        runner: ArmRunner (move_y + move_x + grasp 在这里)

    Returns:
        {
            "ok": bool,                          # Phase A + Phase B 全成功
            "phase_a_get_blue": dict,            # new_get_blue.run 完整返回值
            "phase_b_high_tower": dict,          # high_tower.run 完整返回值
            "final_pose": {                      # 终态 (来自 high_tower 末态)
                "x_mm": float,                    # = -70
                "y_mm": float,                    # = -185
                "arm_deg": float,                 # = 90
                "hand_deg": float,                # = -82
            },
        }

    Raises:
        RuntimeError: Phase A new_get_blue.run 失败 (其内部已含完整错误信息)。
            Phase B high_tower.run 失败同理。
            错误透传, 不在本层包装额外 try/except (避免遮蔽原始错误)。
    """
    t_total_start = time.perf_counter()

    print(f"\n========== {LOG_PREFIX} run (取蓝 → 投高塔, 2 阶段串联) ==========")
    print(f"  Phase A: new_get_blue.run  (4 机联动 → 吸气 → 下探, 球吸住)")
    print(f"  Phase B: high_tower.run    (y 抬升 → 4 机联动 → x 伸进 → 释放 → x 退回)")
    print(f"  预期终态: x=-70mm y=-185mm arm=90° hand=-82°, 球入高塔")

    # ========== Phase A: 取蓝 (new_get_blue.run) ==========
    # 调用 new_get_blue.run() 即可, 内部参数 (x=0, y=-135, arm=85°, hand=10°,
    # grasp_y=-74) 全用 new_get_blue 默认值, 不在串联层重写
    print(f"\n--- Phase A: new_get_blue.run ---")
    phase_a = new_get_blue.run(client, runner)
    if not isinstance(phase_a, dict) or not phase_a.get("ok", False):
        # 透传 new_get_blue 内部错误信息 (它已经 print + raise RuntimeError, 这里兜底)
        print(f"  ❌ Phase A 失败: {phase_a}")
        raise RuntimeError(
            f"{LOG_PREFIX} Phase A new_get_blue.run 失败, 终止串联 (Phase B 未执行)"
        )
    a_pose = phase_a.get("final_pose", {})
    print(f"  ✅ Phase A 完成  终态: x={a_pose.get('x_mm')} y={a_pose.get('y_mm')} "
          f"arm={a_pose.get('arm_deg')}° hand={a_pose.get('hand_deg')}° (球已吸住)")

    # ========== Phase B: 投高塔 (high_tower.run) ==========
    # 调用 high_tower.run() 即可, 内部参数 (y=-115/-185, x=-70/-135, arm=90°, hand=-82°)
    # 全用 high_tower 默认值, 不在串联层重写
    print(f"\n--- Phase B: high_tower.run ---")
    phase_b = high_tower.run(client, runner)
    if not isinstance(phase_b, dict) or not phase_b.get("ok", False):
        # 透传 high_tower 内部错误信息 (它已经 print + raise RuntimeError, 这里兜底)
        print(f"  ❌ Phase B 失败: {phase_b}")
        raise RuntimeError(
            f"{LOG_PREFIX} Phase B high_tower.run 失败"
        )
    b_pose = phase_b.get("final_pose", {})
    print(f"  ✅ Phase B 完成  终态: x={b_pose.get('x_mm')} y={b_pose.get('y_mm')} "
          f"arm={b_pose.get('arm_deg')}° hand={b_pose.get('hand_deg')}° (球已入塔)")

    elapsed = time.perf_counter() - t_total_start
    print(f"\n========== {LOG_PREFIX} 完成 "
          f"(Phase A {phase_a.get('final_pose', {}).get('y_mm')}mm 保护区 → "
          f"Phase B y=-185mm 高塔投球, 总耗时 {elapsed:.3f}s) ==========\n")

    return {
        "ok": True,
        "phase_a_get_blue": phase_a,
        "phase_b_high_tower": phase_b,
        "final_pose": {
            "x_mm": b_pose.get("x_mm"),
            "y_mm": b_pose.get("y_mm"),
            "arm_deg": b_pose.get("arm_deg"),
            "hand_deg": b_pose.get("hand_deg"),
        },
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 串联层**不暴露** new_get_blue / high_tower 内部参数。

    原因: 用户明确要求"使用里面的参数和逻辑过程", 即让两个 run() 自己用默认值,
    不在串联层覆写。如果未来需要调参, 直接调 new_get_blue.run / high_tower.run 的
    关键字参数 (这两个 run 都已经把每个步骤的参数做成可调关键字)。
    """
    return argparse.ArgumentParser(
        description=(
            "task5/from_blue_to_high v1: 「取蓝 → 投高塔」2 阶段串联\n"
            "  Phase A: new_get_blue.run (4 机联动 → 吸气 → 下探)\n"
            "  Phase B: high_tower.run   (y 抬升 → 4 机联动 → x 伸进 → 释放 → x 退回)\n"
            "  默认耗时 ~8-12s (3+5 步)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    if not client.ping():
        raise RuntimeError("机械臂 runtime 未在线, 请检查 arm_feed 守护进程")
    runner = ArmRunner(client)
    result = run(client, runner)
    # 总耗时已由 run() 内部 print, 这里只再打一次状态码
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())