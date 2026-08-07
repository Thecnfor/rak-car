"""task5 / from_yellow_to_low —— 「取黄 → 投低塔」2 阶段串联 (单文件, 非包)。

业务流程 (2026-08-08 用户指定):
  Phase A: ``new_get_yellow.run(...)``  4 机联动到取黄位姿 (x=-71) → 吸气 → 下探 (球吸住)
           终态: x=-71mm, y=-74mm, arm=+85°, hand=+10°, 真空阀 ON (球在手)
  Phase B: ``low_tower.run(...)``        y 抬到 -176 → 4 机联动到投球位姿 → 结束吸气 (球入塔)
           终态: x=-150mm, y=-176mm, arm=90°, hand=0°, 真空阀 OFF (球已入低塔)

⚠️ **使用原则 (用户要求)**:
  - **不要直接调用** new_get_yellow / low_tower 内部的具体动作 (move_y / composite_run / grasp)
  - **调用它们的 run() 函数**: 端到端流程封装, 业务硬限 / composite_run 4 轴全传 /
    grasp 走 runner.* 等全部在两个 run() 内部处理
  - 本文件只负责**串联 + 错误透传**, 不重写任何具体步骤
  - 与 from_blue_to_high.py 同构, 仅 Phase A 起点 (取黄 x=-71 vs 取蓝 x=0)
    与 Phase B 终点 (低塔 vs 高塔) 不同

⚠️ **过渡位姿衔接 (隐式, 已在 new_get_yellow / low_tower 内处理)**:
  - Phase A 终态: x=-71, y=-74 (保护区 [0,-80] 内, 球吸住)
  - Phase B 步骤 1: y=-176 (出保护区 96mm, 低塔投球位姿)
  - 衔接动作: low_tower 步骤 1 move_y 自动把 y 从 -74 抬到 -176
  - arm/hand/x 衔接: Phase A 末态 arm=+85°/hand=+10°/x=-71
    → Phase B 步骤 2 composite_run 把 arm→90°/hand=0°/x→-150 (4 轴并发接管)
  - 全程**真空阀不释放**: Phase A grasp(True) 吸住 → Phase B grasp(False) 释放

⚠️ **业务硬限校验**: new_get_yellow 的位姿 (x=-71, y=-74/-135, arm=85°, hand=+10°)
   与 low_tower 的位姿 (x=-150, y=-176, arm=90°, hand=0°) 全部在
   ARM_API.md §1.1 / setters.py:45 业务硬限内。详见两个源文件的 docstring。

⚠️ **本文件自包含 (task5 约定)**: 只依赖 ``main.arm`` (ArmClient/ArmRunner),
   **import task5 兄弟模块的 run() 函数** (不 import 内部常量/具体步骤),
   沿用 task5 业务层自包含约定 — task5 辅助文件曾被外部动作清空过,
   本文件 import 两个稳定的兄弟 run() 而非重写, 保证直接跑不受影响。

跑法:
    python main/arm/each_task/task5/from_yellow_to_low.py
    python -m main.arm.each_task.task5.from_yellow_to_low
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# 1) 把 repo 根加进 sys.path, 让 "main.arm" / "main.arm.each_task.task5.new_get_yellow" 可解析
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

# 2) 只 import 兄弟模块的 run() 函数, 不 import 内部常量 / 具体步骤
from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.each_task.task5 import new_get_yellow, low_tower  # noqa: E402


# ---------- 流程常量 (仅串联层) ----------

LOG_PREFIX: str = "[task5/from_yellow_to_low]"


# ---------- 主入口 ----------

def run(client: ArmClient, runner: ArmRunner) -> dict:
    """取黄 → 投低塔 2 阶段串联流程。

    业务流程:
      Phase A: ``new_get_yellow.run(client, runner)`` (3 步: 4 机联动 → 吸气 → 下探)
      Phase B: ``low_tower.run(client, runner)``     (3 步: y 抬升 → 4 机联动 → 释放)

    终态 (来自 low_tower 末态):
      x=-150mm, y=-176mm, arm=90°, hand=0°, 真空阀 OFF (球已入低塔)。

    Args:
        client: ArmClient (composite_run + http 在这里)
        runner: ArmRunner (move_y + grasp 在这里)

    Returns:
        {
            "ok": bool,                          # Phase A + Phase B 全成功
            "phase_a_get_yellow": dict,          # new_get_yellow.run 完整返回值
            "phase_b_low_tower": dict,           # low_tower.run 完整返回值
            "final_pose": {                      # 终态 (来自 low_tower 末态)
                "x_mm": float,                    # = -150
                "y_mm": float,                    # = -176
                "arm_deg": float,                 # = 90
                "hand_deg": float,                # = 0
            },
        }

    Raises:
        RuntimeError: Phase A new_get_yellow.run 失败 (其内部已含完整错误信息)。
            Phase B low_tower.run 失败同理。
            错误透传, 不在本层包装额外 try/except (避免遮蔽原始错误)。
    """
    t_total_start = time.perf_counter()

    print(f"\n========== {LOG_PREFIX} run (取黄 → 投低塔, 2 阶段串联) ==========")
    print(f"  Phase A: new_get_yellow.run (4 机联动 x=-71 → 吸气 → 下探, 球吸住)")
    print(f"  Phase B: low_tower.run       (y 抬升 → 4 机联动 → 释放, 球入低塔)")
    print(f"  预期终态: x=-150mm y=-176mm arm=90° hand=0°, 球入低塔")

    # ========== Phase A: 取黄 (new_get_yellow.run) ==========
    print(f"\n--- Phase A: new_get_yellow.run ---")
    phase_a = new_get_yellow.run(client, runner)
    if not isinstance(phase_a, dict) or not phase_a.get("ok", False):
        print(f"  ❌ Phase A 失败: {phase_a}")
        raise RuntimeError(
            f"{LOG_PREFIX} Phase A new_get_yellow.run 失败, 终止串联 (Phase B 未执行)"
        )
    a_pose = phase_a.get("final_pose", {})
    print(f"  ✅ Phase A 完成  终态: x={a_pose.get('x_mm')} y={a_pose.get('y_mm')} "
          f"arm={a_pose.get('arm_deg')}° hand={a_pose.get('hand_deg')}° (球已吸住)")

    # ========== Phase B: 投低塔 (low_tower.run) ==========
    print(f"\n--- Phase B: low_tower.run ---")
    phase_b = low_tower.run(client, runner)
    if not isinstance(phase_b, dict) or not phase_b.get("ok", False):
        print(f"  ❌ Phase B 失败: {phase_b}")
        raise RuntimeError(
            f"{LOG_PREFIX} Phase B low_tower.run 失败"
        )
    b_pose = phase_b.get("final_pose", {})
    print(f"  ✅ Phase B 完成  终态: x={b_pose.get('x_mm')} y={b_pose.get('y_mm')} "
          f"arm={b_pose.get('arm_deg')}° hand={b_pose.get('hand_deg')}° (球已入塔)")

    elapsed = time.perf_counter() - t_total_start
    print(f"\n========== {LOG_PREFIX} 完成 "
          f"(Phase A x=-71 → Phase B x=-150 y=-176 低塔投球, 总耗时 {elapsed:.3f}s) ==========\n")

    return {
        "ok": True,
        "phase_a_get_yellow": phase_a,
        "phase_b_low_tower": phase_b,
        "final_pose": {
            "x_mm": b_pose.get("x_mm"),
            "y_mm": b_pose.get("y_mm"),
            "arm_deg": b_pose.get("arm_deg"),
            "hand_deg": b_pose.get("hand_deg"),
        },
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 串联层**不暴露** new_get_yellow / low_tower 内部参数。

    原因: 用户明确要求"使用里面的参数和逻辑过程", 即让两个 run() 自己用默认值,
    不在串联层覆写。如果未来需要调参, 直接调 new_get_yellow.run / low_tower.run 的
    关键字参数 (这两个 run 都已经把每个步骤的参数做成可调关键字)。
    """
    return argparse.ArgumentParser(
        description=(
            "task5/from_yellow_to_low v1: 「取黄 → 投低塔」2 阶段串联\n"
            "  Phase A: new_get_yellow.run (4 机联动 x=-71 → 吸气 → 下探)\n"
            "  Phase B: low_tower.run       (y 抬升 → 4 机联动 → 释放)\n"
            "  默认耗时 ~6-10s (3+3 步)"
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
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())