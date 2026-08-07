"""task5 / high_tower —— 高塔投球 5 步流程 (单文件, 非包)。

业务流程 (2026-08-08 用户指定, v4 改):
  1. move_y(-115mm)               y 先抬到 -115mm (出保护区, 给复合摆位让出空间)
  2. composite_run (4 机联动)     y=-185, x=-70, arm=90°, hand=-82° (~2-3s)
  3. move_x(-135mm)               x 推进到 -135mm (伸进高塔开口)
  4. grasp(False)                 **结束吸气** = 释放真空 (球掉进高塔)
  5. move_x(-70mm)                x 回退到 -70mm (撤回塔外)

终态: x=-70mm, y=-185mm, arm=90°, hand=-82°, 真空阀 OFF (球已入塔)。

⚠️ **v2 改: pick → place (2026-08-08 用户拍板)**:
  - 旧 v1 步骤 4 是 ``grasp(True)`` (吸气, 取球动作)
  - 新 v2 步骤 4 是 ``grasp(False)`` (**结束吸气** = 释放真空, 投球入塔)
  - 关键运动学证据: 步骤 3 x→-135 (伸进去) → 步骤 4 释放 → 步骤 5 x→-90 (退出来)
    这是 PLACE 进塔动作, 不是 PICK (PICK 应是出塔)
  - **常量对齐**:
    - APPROACH_Y_MM: -120 → -115 (v2 改)
    - PICK_COMPOSITE_X_MM: -104 → -90 → -80 → **-70** (v2 → v3 → v4 改, 与 RETRACT_X_MM 对齐)
    - RETRACT_X_MM: -104 → -90 → -80 → **-70** (v2 → v3 → v4 改)
    - 步骤 4: grasp(True) → grasp(False) (v2 改, 结束吸气)

⚠️ **业务硬限校验 (ARM_API.md §1.1 / setters.py:45)**:
  - y=-115 / -185 ∈ [-200, 0] mm ✓ (出保护区 [0, -80])
  - x=-70 / -135 ∈ [-320, +220] mm ✓ (距物理墙 ~-300mm, 安全)
  - arm=90 ∈ [-150, +150]° ✓ (复位位, 业务硬限上界)
  - hand=-82 ∈ [-90, +10]° ✓ (接近 UP 位 -90, 业务硬限下界)

⚠️ **composite_run 业务硬限 (ARM_API.md §1.1 / setters.py:45)**:
  - 4 轴必须全传有效值 (2026-08-06 实测踩坑): 不接受 None 轴, SDK 会把 None 判 False
    整个 job `result.ok=False`。详见 [[composite-run-no-partial-2026-08-06]]
  - composite_run 不调 _check_y_protected (composite.py:60 拍板): hand=-82°
    在 y=-185 时不会被 wrapper 拦截 (虽然本身也在业务硬限内)

⚠️ **步骤 3/5 单独 move_x (不用 composite_run)**:
  - composite_run 必须 4 轴全传, 步骤 3/5 只想动 x (y/arm/hand 保持 step 2 终态)
  - 用 ``runner.move_x(target, v_max_mms=80)`` 单轴即可, 4 轴开销浪费
  - belt-slip 已在 2026-07-31 修复 (target_blue / target_yellow v2 已验证),
    v_max_mms=80 与 task5 v2 提速档位一致

⚠️ **grasp 真空阀走 runner.grasp()** (loops/runner.py:182):
  - **严禁** ``client.http.execute_arm_action('grasp', ...)``
  - 现场实测 2026-08-03 SDK 内部 struct.pack 报 "required argument is not an integer"
  - **v2 关键**: 投球动作 = ``runner.grasp(False)`` (释放真空), 不要写成 grasp(True)
  - 详见 [[arm-grasp-call-arm-base]]

⚠️ **本文件自包含**: 只依赖 ``main.arm`` (ArmClient/ArmRunner),
   **不 import task5 包内其它模块** (constants.py / grasp_5 / *_tower)。
   沿用 new_get_*.py / new_target.py / target_blue.py 自包含约定
   — task5 辅助文件曾被外部动作清空过, 自包含保证直接跑不受影响。

跑法:
    python main/arm/each_task/task5/high_tower.py
    python -m main.arm.each_task.task5.high_tower
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

LOG_PREFIX: str = "[task5/high_tower]"

APPROACH_Y_MM: float = -115.0
"""步骤 1 y 目标 (mm)。

y=-115 (v2 改自 -120): 出保护区 [0, -80] 35mm, 给步骤 2 composite_run 让出 70mm 空间到 y=-185。"""

PICK_COMPOSITE_Y_MM: float = -185.0
"""步骤 2 composite_run 目标 y (mm)。

y=-185: 接近 y 软限位边界 -200 (留 15mm 余量), 高塔投球位姿。"""

PICK_COMPOSITE_X_MM: float = -70.0
"""步骤 2 composite_run 目标 x (mm)。

x=-70 (v4 改自 -80, v3 → v2 → v1 改自 -90 → -104): 高塔投球基位 (伸进塔前的 x 位置),
步骤 3 推进 / 步骤 5 回退都参考这里。"""

PICK_COMPOSITE_ARM_DEG: float = 90.0
"""步骤 2 composite_run 目标大臂角度 (°)。

arm=90: 业务硬限上界 + 复位位 (init 例外位, 保护区允许)。"""

PICK_COMPOSITE_HAND_DEG: float = -82.0
"""步骤 2 composite_run 目标手爪角度 (°)。

hand=-82: 接近 UP 位 -90, 业务硬限下界 (2026-08-05 P 姿态放宽后下界仍 -90)。
不是 init 例外位 (-90 是 init), 但在业务硬限 [-90, +10] 内合法。"""

GRASP_X_MM: float = -135.0
"""步骤 3 x 推进目标 (mm)。

x=-135: 距 step 2 基位 -90 推进 45mm, 伸进高塔开口。"""

RETRACT_X_MM: float = -70.0
"""步骤 5 x 回退目标 (mm)。

x=-70 (v4 改自 -80, v3 → v2 → v1 改自 -90 → -104): 回到 step 2 基位, 距 step 3 -135 后退 65mm。
球已掉进塔, 完成投球。"""

# composite_run 4 机联动参数 (沿用 new_get_blue.py / target_blue.py / new_target.py 同款)
COMPOSITE_TIMEOUT_S: float = 30.0
"""4 机联动 composite_run 同步超时 (秒)。4 轴并发到位一般 ~2-3s, 给 30s 兜底
(含网络 + job_queue + SDK 内部 4 路 as_completed)。"""

ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度 + xy PID speed, 默认 80。与 task5 v2 / new_get_* 一致。"""

# 步骤 3/5 的 move_x 参数 (沿用 task5 v2 提速档位, belt-slip 已修复)
MOVE_X_V_MAX_MMS: float = 80.0
"""业务限速 (2026-07-31 提速档, belt-slip 修复后验证 80mm/s 稳定)。"""

# 步骤 1 的 move_y / 步骤 3/5 的 move_x 超时
MOVE_TIMEOUT_S: float = 30.0
"""步骤 1/3/5 单轴 move_y / move_x 超时 (秒)。"""

GRASP_TIMEOUT_S: float = 10.0
"""步骤 4 吸气超时 (秒)。真空阀 ON 响应时间 ~1s, 给 10s 兜底。"""


# ---------- 主入口 ----------

def run(client: ArmClient, runner: ArmRunner,
        approach_y_mm: float = APPROACH_Y_MM,
        pick_y_mm: float = PICK_COMPOSITE_Y_MM,
        pick_x_mm: float = PICK_COMPOSITE_X_MM,
        pick_arm_deg: float = PICK_COMPOSITE_ARM_DEG,
        pick_hand_deg: float = PICK_COMPOSITE_HAND_DEG,
        grasp_x_mm: float = GRASP_X_MM,
        retract_x_mm: float = RETRACT_X_MM,
        *,
        move_timeout: float = MOVE_TIMEOUT_S,
        grasp_timeout: float = GRASP_TIMEOUT_S) -> dict:
    """高塔投球 5 步流程 (v2: place, 不是 pick)。

    业务流程:
      1. ``runner.move_y(approach_y_mm)``  y 抬到 -115mm (出保护区)
      2. ``client.composite_run(...)``      4 机联动到投球位姿 (~2-3s)
      3. ``runner.move_x(grasp_x_mm)``      x 推进到 -135mm (伸进塔)
      4. ``runner.grasp(False)``            **结束吸气** = 释放真空 (球入塔)
      5. ``runner.move_x(retract_x_mm)``    x 回退到 -90mm (退出来)

    Args:
        client: ArmClient (composite_run + http 在这里)
        runner: ArmRunner (move_y + move_x + grasp 在这里)
        approach_y_mm: 步骤 1 y 目标 (mm), 默认 -120
        pick_y_mm: 步骤 2 composite_run 目标 y (mm), 默认 -185
        pick_x_mm: 步骤 2 composite_run 目标 x (mm), 默认 -104
        pick_arm_deg: 步骤 2 composite_run 目标大臂角度 (°), 默认 90
        pick_hand_deg: 步骤 2 composite_run 目标手爪角度 (°), 默认 -82
        grasp_x_mm: 步骤 3 x 推进目标 (mm), 默认 -135
        retract_x_mm: 步骤 5 x 回退目标 (mm), 默认 -104
        move_timeout: 步骤 1/3/5 move_y / move_x 超时 (秒), 默认 30
        grasp_timeout: 步骤 4 吸气超时 (秒), 默认 10

    Returns:
        {
            "ok": True,                          # 5 步全成功
            "step1_move_y": dict,                # 步骤 1 move_y job dict
            "step2_composite": dict,             # 步骤 2 composite_run job dict
            "step3_move_x_grasp": dict,          # 步骤 3 move_x job dict
            "step4_grasp": dict,                 # 步骤 4 grasp job dict
            "step5_move_x_retract": dict,        # 步骤 5 move_x job dict
            "final_pose": {                      # 终态 (预期值, 不重读 state)
                "x_mm": float,                    # = retract_x_mm
                "y_mm": float,                    # = pick_y_mm
                "arm_deg": float,                 # = pick_arm_deg
                "hand_deg": float,                # = pick_hand_deg
            },
        }

    Raises:
        RuntimeError: 步骤 2 composite_run 失败 (status != "succeeded" 或 result.ok=False)。
            步骤 1/3/5 move 失败走 runner.* 抛错逻辑。
            步骤 4 grasp 失败走 runner.grasp 抛错逻辑。
    """
    print(f"\n========== {LOG_PREFIX} run (高塔投球 5 步, v2 place) ==========")
    print(f"  步骤 1: y → {approach_y_mm}mm (出保护区)")
    print(f"  步骤 2: composite_run (y={pick_y_mm} x={pick_x_mm} "
          f"arm={pick_arm_deg}° hand={pick_hand_deg}°)")
    print(f"  步骤 3: x → {grasp_x_mm}mm (伸进塔)")
    print(f"  步骤 4: grasp(False) (结束吸气 = 释放真空, 球入塔)")
    print(f"  步骤 5: x → {retract_x_mm}mm (退回塔外)")

    # ========== 步骤 1: y → -115mm (出保护区) ==========
    # move_y 走步进电机, 允许保护区 [0, -30] 内调, 用于出保护区
    print(f"\n  [1/5] runner.move_y({approach_y_mm}mm)  y 抬到 -115mm")
    step1 = runner.move_y(approach_y_mm, timeout=move_timeout)

    # ========== 步骤 2: composite_run 4 机联动到取球位姿 ==========
    # 仿 new_get_blue.py / new_get_yellow.py / new_target.py / target_blue.py 模式
    # ⚠️ composite_run 不接受 None 轴 (2026-08-06 实测): 4 轴全传有效值
    # ⚠️ composite_run 不调 _check_y_protected (composite.py:60 拍板): 手爪
    #    hand=-82° 在 y=-185 时不会被 wrapper 拦截 (虽然本身在业务硬限内)
    print(f"\n  [2/5] composite_run (4 机联动): arm={pick_arm_deg:+.0f}° x={pick_x_mm:.0f}mm "
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
        print(f"  [2/5] ❌ composite_run 失败: {step2}")
        raise RuntimeError(
            f"{LOG_PREFIX} Step 2 composite_run 4 机联动失败: {step2}"
        )
    steps2 = step2["result"].get("steps", {}) if isinstance(step2.get("result"), dict) else {}
    print(f"  [2/5] ✅ 4 轴并发到位 (~2-3s)  steps={steps2}")

    # ========== 步骤 3: move_x(-135mm) 推进到伸进塔 ==========
    # 单轴 move_x, 不用 composite_run (后者必须 4 轴全传, 浪费)
    # belt-slip 已修复 (2026-07-31), v_max_mms=80 与 task5 v2 提速档位一致
    print(f"\n  [3/5] runner.move_x({grasp_x_mm}mm, v_max_mms={MOVE_X_V_MAX_MMS:.0f})  "
          f"x 推进到 {grasp_x_mm}mm (伸进塔)")
    step3 = runner.move_x(grasp_x_mm, v_max_mms=MOVE_X_V_MAX_MMS, timeout=move_timeout)

    # ========== 步骤 4: grasp(False) 结束吸气 (释放真空, 球入塔) ==========
    # ⚠️ v2 关键改: 不是 grasp(True) (吸气 = 取球), 而是 grasp(False) (结束吸气 = 投球)
    # 投球入塔场景: 步骤 3 伸进塔 → 步骤 4 释放真空 → 步骤 5 退出来
    # ⚠️ 通用踩坑: grasp 真空阀走 runner.grasp() / runner.suck() / runner.drop_object()
    # (loops/runner.py:182/185/194), **严禁 client.http.execute_arm_action('grasp', ...)**
    # — 现场实测 2026-08-03 SDK 内部 struct.pack 报 "required argument is not an integer"
    # (runtime ARM_ACTIONS lambda kwargs 透传 + list 整个传给 valve.set → struct.pack 格式
    # 不匹配)。详见 [[arm-grasp-call-arm-base]]。
    print(f"\n  [4/5] runner.grasp(False)  结束吸气 (释放真空, 球入塔, "
          f"timeout={grasp_timeout:.0f}s)")
    step4 = runner.grasp(False, timeout=grasp_timeout)

    # ========== 步骤 5: move_x(-90mm) 回退 ==========
    print(f"\n  [5/5] runner.move_x({retract_x_mm}mm, v_max_mms={MOVE_X_V_MAX_MMS:.0f})  "
          f"x 回退到 {retract_x_mm}mm (退回塔外)")
    step5 = runner.move_x(retract_x_mm, v_max_mms=MOVE_X_V_MAX_MMS, timeout=move_timeout)

    print(f"\n========== {LOG_PREFIX} 完成 "
          f"(arm={pick_arm_deg}° x={retract_x_mm}mm y={pick_y_mm}mm "
          f"hand={pick_hand_deg}°, 真空阀 OFF 球已入塔) ==========\n")
    return {
        "ok": True,
        "step1_move_y": step1,
        "step2_composite": step2,
        "step3_move_x_grasp": step3,
        "step4_grasp": step4,
        "step5_move_x_retract": step5,
        "final_pose": {
            "x_mm": retract_x_mm,
            "y_mm": pick_y_mm,
            "arm_deg": pick_arm_deg,
            "hand_deg": pick_hand_deg,
        },
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "task5/high_tower v4: 高塔投球 5 步流程 (place, 不是 pick)\n"
            "  1. y=-115 (出保护区)\n"
            "  2. composite_run → y=-185, x=-70, arm=90°, hand=-82°\n"
            "  3. x=-135 (伸进塔)\n"
            "  4. grasp(False) (结束吸气 = 释放真空, 球入塔)\n"
            "  5. x=-70 (退回塔外)\n"
            "  默认耗时 ~5-7s"
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
    p.add_argument("--grasp-x", type=float, default=GRASP_X_MM,
                   dest="grasp_x", help="步骤 3 x 推进目标 (mm)")
    p.add_argument("--retract-x", type=float, default=RETRACT_X_MM,
                   dest="retract_x", help="步骤 5 x 回退目标 (mm)")
    p.add_argument("--move-timeout", type=float, default=MOVE_TIMEOUT_S,
                   dest="move_timeout", help="步骤 1/3/5 move_y / move_x 超时 (秒)")
    p.add_argument("--grasp-timeout", type=float, default=GRASP_TIMEOUT_S,
                   dest="grasp_timeout", help="步骤 4 吸气超时 (秒)")
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
        grasp_x_mm=args.grasp_x,
        retract_x_mm=args.retract_x,
        move_timeout=args.move_timeout,
        grasp_timeout=args.grasp_timeout)
    elapsed = time.perf_counter() - t_total_start
    print(f"========== {LOG_PREFIX} 总耗时: {elapsed:.3f} s ==========")
    return 0


if __name__ == "__main__":
    sys.exit(main())