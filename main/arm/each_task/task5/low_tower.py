"""task5 / low_tower —— 低塔投球 3 步流程 (单文件, 非包)。

业务流程 (2026-08-08 用户指定):
  1. move_y(-176mm)               y 先抬到 -176mm (出保护区, 给复合摆位让出空间)
  2. composite_run (4 机联动)     y=-176, x=-150, arm=90°, hand=0° (~2-3s)
  3. grasp(False)                 **结束吸气** = 释放真空 (球掉进低塔)

终态: x=-150mm, y=-176mm, arm=90°, hand=0°, 真空阀 OFF (球已入低塔)。

⚠️ **业务流程 vs high_tower.py 的差异** (2026-08-08 用户指定):
  - low_tower 没有 x 推进 / 回退 (high_tower 的步骤 3/5)
    → 低塔位置比高塔矮, 不需要伸进去就能投球
  - hand=0° (DOWN) vs high_tower hand=-82° (接近 UP)
    → 低塔投球姿势: 手爪朝下, 业务硬限中点 (-90, +10)
  - y=-176 vs high_tower y=-185 (高塔更高)
    → y=-176 比 -185 离 0 (触底) 近 9mm, 符合"低塔"语义
  - 总耗时更短 (3 步 vs 5 步)

⚠️ **业务硬限校验 (ARM_API.md §1.1 / setters.py:45)**:
  - y=-176 ∈ [-200, 0] mm ✓ (出保护区 [0, -80] 96mm)
  - x=-150 ∈ [-320, +220] mm ✓ (距物理墙 ~-300mm, 安全)
  - arm=90 ∈ [-150, +150]° ✓ (复位位, 业务硬限上界)
  - hand=0 ∈ [-90, +10]° ✓ (DOWN 位, 业务硬限中点)

⚠️ **composite_run 业务硬限 (ARM_API.md §1.1 / setters.py:45)**:
  - 4 轴必须全传有效值 (2026-08-06 实测踩坑): 不接受 None 轴, SDK 会把 None 判 False
    整个 job `result.ok=False`。详见 [[composite-run-no-partial-2026-08-06]]
  - 步骤 2 y=-176 与步骤 1 末态 y=-176 相同: composite 内部走 no-op (SDK 同值 = 不动)
  - composite_run 不调 _check_y_protected (composite.py:60 拍板): hand=0°
    在 y=-176 时不会被 wrapper 拦截 (虽然本身在业务硬限内)

⚠️ **grasp 真空阀走 runner.grasp()** (loops/runner.py:182):
  - **严禁** ``client.http.execute_arm_action('grasp', ...)``
  - 现场实测 2026-08-03 SDK 内部 struct.pack 报 "required argument is not an integer"
  - **关键**: 投球动作 = ``runner.grasp(False)`` (释放真空), 不要写成 grasp(True)
  - 详见 [[arm-grasp-call-arm-base]]

⚠️ **本文件自包含**: 只依赖 ``main.arm`` (ArmClient/ArmRunner),
   **不 import task5 包内其它模块** (constants.py / grasp_5 / *_tower)。
   沿用 high_tower.py / new_get_*.py / target_blue.py 自包含约定
   — task5 辅助文件曾被外部动作清空过, 自包含保证直接跑不受影响。

跑法:
    python main/arm/each_task/task5/low_tower.py
    python -m main.arm.each_task.task5.low_tower
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


# ---------- 流程常量 (内联, 不依赖 constants.py) ----------

LOG_PREFIX: str = "[task5/low_tower]"

APPROACH_Y_MM: float = -176.0
"""步骤 1 y 目标 (mm)。

y=-176: 出保护区 [0, -80] 96mm, 给步骤 2 composite_run 让出空间。
也是 low_tower 投球的高度 (比 high_tower 的 -185 高 9mm, 离触底近 9mm)。"""

PICK_COMPOSITE_Y_MM: float = -176.0
"""步骤 2 composite_run 目标 y (mm)。

y=-176: 与步骤 1 相同, composite 内部走 no-op (SDK 同值 = 不动)。
低塔投球位姿 (比高塔 -185 高 9mm)。"""

PICK_COMPOSITE_X_MM: float = -150.0
"""步骤 2 composite_run 目标 x (mm)。

x=-150: 低塔投球位姿 (吸盘正对低塔开口)。
距物理墙 ~-300mm 还有 150mm, 安全。"""

PICK_COMPOSITE_ARM_DEG: float = 90.0
"""步骤 2 composite_run 目标大臂角度 (°)。

arm=90: 业务硬限上界 + 复位位 (init 例外位, 保护区允许)。
低塔投球大臂姿势 (与 high_tower 一致)。"""

PICK_COMPOSITE_HAND_DEG: float = 0.0
"""步骤 2 composite_run 目标手爪角度 (°)。

hand=0: DOWN 位 (手爪朝下), 业务硬限中点 (-90, +10) 范围内。
与 high_tower hand=-82° (接近 UP) 不同, low_tower 用 DOWN 投球姿势。"""

# composite_run 4 机联动参数 (沿用 high_tower.py / new_get_blue.py 同款)
COMPOSITE_TIMEOUT_S: float = 30.0
"""4 机联动 composite_run 同步超时 (秒)。4 轴并发到位一般 ~2-3s, 给 30s 兜底
(含网络 + job_queue + SDK 内部 4 路 as_completed)。"""

ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度 + xy PID speed, 默认 80。与 task5 v4 / high_tower 一致。"""

# 步骤 1 的 move_y 超时 + 步骤 3 的 grasp 超时
MOVE_TIMEOUT_S: float = 30.0
"""步骤 1 单轴 move_y 超时 (秒)。"""

GRASP_TIMEOUT_S: float = 10.0
"""步骤 3 释放真空超时 (秒)。真空阀 OFF 响应时间 ~1s, 给 10s 兜底。"""


# ---------- 主入口 ----------

def run(client: ArmClient, runner: ArmRunner,
        approach_y_mm: float = APPROACH_Y_MM,
        pick_y_mm: float = PICK_COMPOSITE_Y_MM,
        pick_x_mm: float = PICK_COMPOSITE_X_MM,
        pick_arm_deg: float = PICK_COMPOSITE_ARM_DEG,
        pick_hand_deg: float = PICK_COMPOSITE_HAND_DEG,
        *,
        move_timeout: float = MOVE_TIMEOUT_S,
        grasp_timeout: float = GRASP_TIMEOUT_S) -> dict:
    """低塔投球 3 步流程。

    业务流程:
      1. ``runner.move_y(approach_y_mm)``  y 抬到 -176mm (出保护区)
      2. ``client.composite_run(...)``      4 机联动到投球位姿 (~2-3s)
      3. ``runner.grasp(False)``            **结束吸气** = 释放真空 (球入低塔)

    Args:
        client: ArmClient (composite_run + http 在这里)
        runner: ArmRunner (move_y + grasp 在这里)
        approach_y_mm: 步骤 1 y 目标 (mm), 默认 -176
        pick_y_mm: 步骤 2 composite_run 目标 y (mm), 默认 -176
        pick_x_mm: 步骤 2 composite_run 目标 x (mm), 默认 -150
        pick_arm_deg: 步骤 2 composite_run 目标大臂角度 (°), 默认 90
        pick_hand_deg: 步骤 2 composite_run 目标手爪角度 (°), 默认 0
        move_timeout: 步骤 1 move_y 超时 (秒), 默认 30
        grasp_timeout: 步骤 3 释放真空超时 (秒), 默认 10

    Returns:
        {
            "ok": True,                          # 3 步全成功
            "step1_move_y": dict,                # 步骤 1 move_y job dict
            "step2_composite": dict,             # 步骤 2 composite_run job dict
            "step3_release": dict,               # 步骤 3 grasp(False) job dict
            "final_pose": {                      # 终态 (预期值, 不重读 state)
                "x_mm": float,                    # = pick_x_mm
                "y_mm": float,                    # = pick_y_mm
                "arm_deg": float,                 # = pick_arm_deg
                "hand_deg": float,                # = pick_hand_deg
            },
        }

    Raises:
        RuntimeError: 步骤 2 composite_run 失败 (status != "succeeded" 或 result.ok=False)。
            步骤 1 move 失败走 runner.move_y 抛错逻辑。
            步骤 3 release 失败走 runner.grasp 抛错逻辑。
    """
    print(f"\n========== {LOG_PREFIX} run (低塔投球 3 步) ==========")
    print(f"  步骤 1: y → {approach_y_mm}mm (出保护区)")
    print(f"  步骤 2: composite_run (y={pick_y_mm} x={pick_x_mm} "
          f"arm={pick_arm_deg}° hand={pick_hand_deg}°)")
    print(f"  步骤 3: grasp(False) (结束吸气 = 释放真空, 球入低塔)")

    # ========== 步骤 1: y → -176mm (出保护区) ==========
    # move_y 走步进电机, 允许保护区 [0, -30] 内调, 用于出保护区
    print(f"\n  [1/3] runner.move_y({approach_y_mm}mm)  y 抬到 -176mm")
    step1 = runner.move_y(approach_y_mm, timeout=move_timeout)

    # ========== 步骤 2: composite_run 4 机联动到投球位姿 ==========
    # 仿 high_tower.py / new_get_blue.py / new_target.py / target_blue.py 模式
    # ⚠️ composite_run 不接受 None 轴 (2026-08-06 实测): 4 轴全传有效值
    #    步骤 2 y=-176 与步骤 1 末态相同, 内部走 no-op (SDK 同值 = 不动)
    # ⚠️ composite_run 不调 _check_y_protected (composite.py:60 拍板): hand=0°
    #    在 y=-176 时不会被 wrapper 拦截 (虽然本身在业务硬限内)
    print(f"\n  [2/3] composite_run (4 机联动): arm={pick_arm_deg:+.0f}° x={pick_x_mm:.0f}mm "
          f"y={pick_y_mm:.0f}mm hand={pick_hand_deg:+.0f}°  speed={ANGLE_SPEED} "
          f"timeout={COMPOSITE_TIMEOUT_S:.0f}s")
    step2 = client.composite_run(
        arm=pick_arm_deg,
        x_mm=pick_x_mm,
        y_mm=pick_y_mm,
        hand=pick_hand_deg,
        speed=ANGLE_SPEED,
        timeout=COMPOSITE_TIMEOUT_S,
    )
    ok2 = (
        isinstance(step2, dict)
        and step2.get("status") == "succeeded"
        and isinstance(step2.get("result"), dict)
        and step2["result"].get("ok", False)
    )
    if not ok2:
        # ⚠️ 通用踩坑: job["result"]["ok"] 不是 job["ok"] — job dict 和
        # composite_run SDK 返回的 result dict 是嵌套结构, 详见
        # [[composite-run-no-partial-2026-08-06]]
        print(f"  [2/3] ❌ composite_run 失败: {step2}")
        raise RuntimeError(
            f"{LOG_PREFIX} Step 2 composite_run 4 机联动失败: {step2}"
        )
    steps2 = step2["result"].get("steps", {}) if isinstance(step2.get("result"), dict) else {}
    print(f"  [2/3] ✅ 4 轴并发到位 (~2-3s)  steps={steps2}")

    # ========== 步骤 3: grasp(False) 结束吸气 (释放真空, 球入低塔) ==========
    # ⚠️ 关键: 不是 grasp(True) (吸气 = 取球), 而是 grasp(False) (结束吸气 = 投球)
    # low_tower 投球场景: 步骤 2 摆位到低塔 → 步骤 3 释放真空 → 球掉进低塔
    # ⚠️ 通用踩坑: grasp 真空阀走 runner.grasp() / runner.suck() / runner.drop_object()
    # (loops/runner.py:182/185/194), **严禁 client.http.execute_arm_action('grasp', ...)**
    # — 现场实测 2026-08-03 SDK 内部 struct.pack 报 "required argument is not an integer"
    # (runtime ARM_ACTIONS lambda kwargs 透传 + list 整个传给 valve.set → struct.pack 格式
    # 不匹配)。详见 [[arm-grasp-call-arm-base]]。
    print(f"\n  [3/3] runner.grasp(False)  结束吸气 (释放真空, 球入低塔, "
          f"timeout={grasp_timeout:.0f}s)")
    step3 = runner.grasp(False, timeout=grasp_timeout)

    print(f"\n========== {LOG_PREFIX} 完成 "
          f"(arm={pick_arm_deg}° x={pick_x_mm}mm y={pick_y_mm}mm "
          f"hand={pick_hand_deg}°, 真空阀 OFF 球已入低塔) ==========\n")
    return {
        "ok": True,
        "step1_move_y": step1,
        "step2_composite": step2,
        "step3_release": step3,
        "final_pose": {
            "x_mm": pick_x_mm,
            "y_mm": pick_y_mm,
            "arm_deg": pick_arm_deg,
            "hand_deg": pick_hand_deg,
        },
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "task5/low_tower v1: 低塔投球 3 步流程 (place, 不是 pick)\n"
            "  1. y=-176 (出保护区)\n"
            "  2. composite_run → y=-176, x=-150, arm=90°, hand=0°\n"
            "  3. grasp(False) (结束吸气 = 释放真空, 球入低塔)\n"
            "  默认耗时 ~3-5s"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--approach-y", type=float, default=APPROACH_Y_MM,
                   dest="approach_y", help="步骤 1 y 目标 (mm)")
    p.add_argument("--pick-y", type=float, default=PICK_COMPOSITE_Y_MM,
                   dest="pick_y", help="步骤 2 composite_run 目标 y (mm)")
    p.add_argument("--pick-x", type=float, default=PICK_COMPOSITE_X_MM,
                   dest="pick_x", help="步骤 2 composite_run 目标 x (mm)")
    p.add_argument("--pick-arm", type=float, default=PICK_COMPOSITE_ARM_DEG,
                   dest="pick_arm", help="步骤 2 composite_run 目标大臂角度 (°)")
    p.add_argument("--pick-hand", type=float, default=PICK_COMPOSITE_HAND_DEG,
                   dest="pick_hand", help="步骤 2 composite_run 目标手爪角度 (°)")
    p.add_argument("--move-timeout", type=float, default=MOVE_TIMEOUT_S,
                   dest="move_timeout", help="步骤 1 move_y 超时 (秒)")
    p.add_argument("--grasp-timeout", type=float, default=GRASP_TIMEOUT_S,
                   dest="grasp_timeout", help="步骤 3 释放真空超时 (秒)")
    return p


def main(argv=None) -> int:
    t_total_start = time.perf_counter()
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    if not client.ping():
        raise RuntimeError("机械臂 runtime 未在线, 请检查 arm_feed 守护进程")
    runner = ArmRunner(client)
    result = run(client, runner,
        approach_y_mm=args.approach_y,
        pick_y_mm=args.pick_y,
        pick_x_mm=args.pick_x,
        pick_arm_deg=args.pick_arm,
        pick_hand_deg=args.pick_hand,
        move_timeout=args.move_timeout,
        grasp_timeout=args.grasp_timeout)
    elapsed = time.perf_counter() - t_total_start
    print(f"========== {LOG_PREFIX} 总耗时: {elapsed:.3f} s ==========")
    return 0


if __name__ == "__main__":
    sys.exit(main())