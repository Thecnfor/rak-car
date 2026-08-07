"""task5 / new_get_yellow —— 「取黄」位姿 + 吸气 + 下探, **仿 main/task/task1_seeding.py 的四机联动**。

业务要求 (2026-08-07 用户):
  1. 仿 ``main/task/task1_seeding.py::_switch_to_place_pose`` 的 4 机联动
     composite_run 模式, 把臂一次性摆到 "取黄位姿":
         arm = +85°
         x   = -62 mm
         y   = -135 mm
         hand = +10°
  2. 立刻 ``grasp(True)`` (吸气)
  3. ``move_y(-66)`` 下降到 grasp_y_mm (保护区 [0, -80] 内, move_y 不过保护区)

⚠️ **本文件与 new_get_blue.py 同构, 仅 4 机联动参数不同**:
  - new_get_blue: arm=+86°, x=0
  - new_get_yellow: arm=+85°, x=-62  (沿用 task5/get_yellow.py 的 x=-68 → -72 → -71 → -62)
  - 其他 (y=-135, hand=+10°, grasp=True, y=-66) 完全相同

⚠️ **composite_run 业务硬限 (ARM_API.md §1.1 / setters.py:45)**:
  - arm = +85 ∈ [-150, +150]° ✓ (业务硬限内)
  - hand = +10 ∈ [-90, +10]° ✓ (业务硬限上界, P 姿态放宽后)
  - y = -135 ≤ soft_y_max_mm (默认 -200) ✓
  - y = -135 < 0 (保护区 [0, -80] 外 55mm) ✓ — composite_run 不查保护区
  - x = -62 ∈ [-320, +220] mm ✓ (距物理墙约 -238mm, 安全)

⚠️ **composite_run 不接受 None 轴 (2026-08-06 现场实测)**:
  - 业务层 composite.py:56-68 虽然把 None 透传给底层, 但 SDK 不识别 None →
    `result.steps={None轴: False, 有值轴: True}`, 整个 job `result.ok=False`
  - **正确用法**: 4 轴全传有效值, "不动的轴"靠"传相同值"实现 (SDK 内部走 no-op)

⚠️ **composite_run 内部不调用 _check_y_protected** (composite.py:60 拍板):
  "不怕撞车! _check_y_protected 去掉! 要速度!"
  所以 hand=+10° 在 y=-135 时不会被 wrapper 拦截。

⚠️ **grasp 真空阀走 runner.grasp()** (loops/runner.py), **严禁
  client.http.execute_arm_action('grasp', ...)** — 现场实测 2026-08-03 SDK 内部
  struct.pack 报 "required argument is not an integer"。

⚠️ **move_y(-66) 进入保护区**: move_y 走步进电机, 不过 y 保护区 (api.py 注释:
  '即使在保护区 [0, -30] 也可以调, 用于出保护区')。但注意: 保护区 y ∈ [-80, 0]
  内 **不允许** set_arm/set_hand/set_pose (会撞车), 这次只 move_y, 安全。
  y=-66 距保护区下边界 -80 还有 14mm 余量。

⚠️ 本文件**自包含**: 只依赖 `main.arm` (ArmClient/ArmRunner),
   不 import task5 包内其它模块。原因: task5 辅助文件曾被外部动作清空过,
   自包含保证直接跑不受影响。

跑法:
    python main/arm/each_task/task5/new_get_yellow.py
    python -m main.arm.each_task.task5.new_get_yellow
"""
from __future__ import annotations

import argparse
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


# ---------- 目标位姿常量 (内联, 不依赖 constants.py) ----------

LOG_PREFIX: str = "[task5/new_get_yellow]"

GET_YELLOW_X_MM: float = -62.0
"""取黄 x (-62 mm)。

沿革: 2026-08-07 用户指定 (-68 → -72 → -71), 2026-08-08 用户改为 -62。
距物理墙 (-300mm 量级) 较远, 不撞墙。"""

GET_YELLOW_Y_MM: float = -135.0
"""取黄 y (吸前高距位)。出 y 保护区 [0, -80] 55mm, 远出保护区。
与 new_get_blue.py 一致 (-135)。"""

GET_YELLOW_ARM_DEG: float = 85.0
"""大臂角度 (°)。业务硬限 [+150, -150]° 内 (+85)。
用户 2026-08-07 指定 85°, 与 task5/get_blue.py 一致。
与 new_get_blue.py 86° 差 1° (取黄比取蓝略收 1°)。"""

GET_YELLOW_HAND_DEG: float = 10.0
"""手爪角度 (°, DOWN 偏后 +10°)。
业务硬限 [-90, +10]° 上界 (2026-08-05 P 姿态放宽后), 与 POSE_P_HAND_DEG 同值。
非 init 姿态 (-90 是 init), composite_run 内部不查保护区, 安全。
与 new_get_blue.py 一致 (+10)。"""

GET_YELLOW_GRASP_Y_MM: float = -66.0
"""吸气后下降到 y=-66 mm (保护区 [0, -80] 内, 只做 move_y, 不改 hand/arm,
不会被 _check_safe 拒)。

与 new_get_blue.py 一致 (-66), 2026-08-08 用户改为 -66 (2026-08-07 原 -74)。
距保护区边界 [-80, 0] 还有 14mm 余量, 保护区外一寸, 安全。"""

COMPOSITE_TIMEOUT_S: float = 30.0
"""4 机联动 composite_run 同步超时 (秒)。4 轴并发到位一般 ~2-3s, 给 30s 兜底
(含网络 + job_queue + SDK 内部 4 路 as_completed)。"""

ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度 + xy PID speed, 默认 80。与 task5/target.py / task7 一致。"""


# ---------- 主入口 ----------

def run(client: ArmClient, runner: ArmRunner,
        x_mm: float = GET_YELLOW_X_MM,
        y_mm: float = GET_YELLOW_Y_MM,
        arm_deg: float = GET_YELLOW_ARM_DEG,
        hand_deg: float = GET_YELLOW_HAND_DEG,
        grasp_y_mm: float = GET_YELLOW_GRASP_Y_MM) -> dict:
    """取黄位姿 + 吸气 + 下探的 3 步流程 (与 new_get_blue 同构)。

    1. **4 机联动** composite_run(arm=85°, x=-62, y=-135, hand=+10°)
       仿 ``main/task/task1_seeding.py::_switch_to_place_pose`` / ``_init_step2_s_pose``
       模式, 4 轴并发到位 (~2-3s)。
    2. **吸气** ``runner.grasp(True)`` 真空阀开。
    3. **下探** ``runner.move_y(-66)`` 下降到 grasp_y_mm。

    Args:
        client: ArmClient (composite_run 在这里)
        runner: ArmRunner (move_y + grasp 在这里)
        x_mm: composite_run 目标 x (mm), 默认 -62
        y_mm: composite_run 目标 y (mm), 默认 -135
        arm_deg: composite_run 目标大臂角度 (°), 默认 85
        hand_deg: composite_run 目标手爪角度 (°), 默认 +10
        grasp_y_mm: 吸气后下探目标 y (mm), 默认 -66

    Returns:
        {
            "ok": bool,                  # 4 机联动 + grasp 都成功
            "step1_composite": dict,     # 4 机联动 composite_run 原始 job dict
            "step2_grasp": dict,         # runner.grasp(True) 原始 job dict
            "step3_move_y": dict,        # runner.move_y(grasp_y_mm) 原始 job dict
            "final_pose": {              # 终态 (预期值, 不重读 state)
                "x_mm": float,
                "y_mm": float,           # = grasp_y_mm (吸气后下探)
                "arm_deg": float,
                "hand_deg": float,
            },
        }

    Raises:
        RuntimeError: Step 1 composite_run 失败 (status != "succeeded")。
            Step 2/3 失败走 runner.* 抛错逻辑。
    """
    print(f"\n========== {LOG_PREFIX} run (3 步: 4 机联动 → 吸气 → 下探) ==========")
    print(f"  目标: arm={arm_deg}° x={x_mm}mm y={y_mm}mm hand={hand_deg}°  → grasp → y={grasp_y_mm}mm")

    # ========== Step 1: 4 机联动 composite_run (仿 main/task/task1_seeding.py) ==========
    # 仿 _switch_to_place_pose / _init_step2_s_pose 同款 4 轴并发模式。
    # ⚠️ composite_run 不接受 None 轴, 4 轴全传有效值, "不动的轴"靠"传相同值"实现。
    # ⚠️ composite_run 内部不调 _check_y_protected (composite.py:60 拍板), 所以
    #    hand=+10° 在 y=-135 时不会被保护区拦截。
    print(f"  [1/3] composite_run (4 机联动): arm={arm_deg:+.0f}° x={x_mm:.0f}mm "
          f"y={y_mm:.0f}mm hand={hand_deg:+.0f}°  speed={ANGLE_SPEED} "
          f"timeout={COMPOSITE_TIMEOUT_S:.0f}s")
    step1 = client.composite_run(
        arm=arm_deg,
        x_mm=x_mm,
        y_mm=y_mm,
        hand=hand_deg,
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
        # ⚠️ 通用踩坑: job["result"]["ok"] 不是 job["ok"] — job dict 和
        # composite_run SDK 返回的 result dict 是嵌套结构, 详见
        # [[composite-run-no-partial-2026-08-06]]
        print(f"  [1/3] ❌ composite_run 失败: {step1}")
        raise RuntimeError(
            f"{LOG_PREFIX} Step 1 composite_run 4 机联动失败: {step1}"
        )
    # 检查 4 轴全部 ok (现场实测 SDK 会把 None 轴判 False, 所以这里再核一次 steps)
    steps = step1["result"].get("steps", {}) if isinstance(step1.get("result"), dict) else {}
    print(f"  [1/3] ✅ 4 轴并发到位 (~2-3s)  steps={steps}")

    # ========== Step 2: 吸气 (走 runner.grasp, 不走 http.execute_arm_action) ==========
    # ⚠️ 通用踩坑: grasp 真空阀走 runner.grasp() / runner.suck() / runner.drop_object()
    # (loops/runner.py:182/185/194), **严禁 client.http.execute_arm_action('grasp', ...)**
    # — 现场实测 2026-08-03 SDK 内部 struct.pack 报 "required argument is not an integer"
    # (runtime ARM_ACTIONS lambda kwargs 透传 + list 整个传给 valve.set → struct.pack 格式
    # 不匹配)。详见 [[arm-grasp-call-arm-base]]。
    print(f"\n  [2/3] runner.grasp(True)   吸气 (真空阀开)")
    step2 = runner.grasp(True, timeout=10.0)
    print(f"  [2/3] ✅ 吸气 job={step2}")

    # ========== Step 3: 下探 move_y(grasp_y_mm) ==========
    # move_y 走步进电机, 不过 y 保护区 (api.py 注释: '即使在保护区 [0, -30]
    # 也可以调, 用于出保护区'), 所以 y=-66 在保护区 [0, -80] 内也安全。
    # ⚠️ 不要在这里 set_arm_angle / set_hand_angle, 保护区 y 内改 hand/arm 会撞车。
    print(f"\n  [3/3] runner.move_y({grasp_y_mm}mm)   下探到 grasp_y (保护区 [0,-80] 内, move_y 不查)")
    step3 = runner.move_y(grasp_y_mm, timeout=20.0)
    print(f"  [3/3] ✅ 下探到位 job={step3}")

    print(f"========== {LOG_PREFIX} 完成 "
          f"(arm={arm_deg}° x={x_mm}mm y={grasp_y_mm}mm hand={hand_deg}°, 已吸气) ==========\n")
    return {
        "ok": True,
        "step1_composite": step1,
        "step2_grasp": step2,
        "step3_move_y": step3,
        "final_pose": {
            "x_mm": x_mm,
            "y_mm": grasp_y_mm,
            "arm_deg": arm_deg,
            "hand_deg": hand_deg,
        },
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "task5 new_get_yellow v1: 4 机联动 composite_run (arm=+85° x=-62 y=-135 hand=+10°)\n"
            "  仿 main/task/task1_seeding.py 的 4 机联动模式\n"
            "  → runner.grasp(True) 吸气 → runner.move_y(-66) 下探\n"
            "  与 new_get_blue 同构, 仅 4 机联动参数不同 (arm 86→85, x 0→-62)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--x", type=float, default=GET_YELLOW_X_MM, help="composite_run 目标 x (mm)")
    p.add_argument("--y", type=float, default=GET_YELLOW_Y_MM, help="composite_run 目标 y (mm)")
    p.add_argument("--arm", type=float, default=GET_YELLOW_ARM_DEG, help="composite_run 目标大臂角度 (°)")
    p.add_argument("--hand", type=float, default=GET_YELLOW_HAND_DEG, help="composite_run 目标手爪角度 (°)")
    p.add_argument("--grasp-y", type=float, default=GET_YELLOW_GRASP_Y_MM,
                   dest="grasp_y", help="吸气后下探目标 y (mm)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    if not client.ping():
        raise RuntimeError("机械臂 runtime 未在线, 请检查 arm_feed 守护进程")
    runner = ArmRunner(client)
    run(client, runner,
        x_mm=args.x, y_mm=args.y,
        arm_deg=args.arm, hand_deg=args.hand,
        grasp_y_mm=args.grasp_y)
    return 0


if __name__ == "__main__":
    sys.exit(main())