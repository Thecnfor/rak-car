"""task7 / position5 —— **位置 5** 的位姿序列 (10 步动作, **含 1 次放气 Step 7**: y up → arm → hand → x mid → y down → x final → drop → x mid → y up → x return)。

按用户 2026-08-04 v3 重新指定顺序 (含 1 次放气, 在 Step 6 和原 Step 7 中间):

  1. move_y(-190mm)                y 抬到 -190mm (出保护区 [0, -30], 给 set_*_angle 留安全余量)
  2. set_arm_angle(+90°)           大臂到 90° (复位位, 业务硬限上界 [-150, +90])
  3. set_hand_angle(-30°)          手爪到 -30° (mid mode, 业务硬限内 [-90, 0])
  4. move_x_with_split(-180mm)     x 在 y=-190 时滑到中间位 -180mm (出保护区第一段 x 缓冲)
  5. move_y(-65mm)                 y 降到 -65mm (工作深度, 保护区 [0, -30] 外)
  6. move_x_with_split(-215mm)     x 在 y=-65 时滑到最终位 -215mm (push 货物到位)
  7. runner.drop_object()          🆕 v3 新增: 放气 (断开真空, 货物落到目标位)
  8. move_x_with_split(-180mm)     x 回到 x_mid (-180)  (y=-65 时, 复用 POS_X_MID_MM, 原 Step 7 前移)
  9. move_y(-190mm)                y 上升回 -190          (复用 POS_Y_UP_MM, 出工作深度给 x 归零让路, 原 Step 8 前移)
 10. move_x_with_split(0mm)        x 归零 (撞墙 calibrate, 在 y=-190 时, 原 Step 9 前移)

⚠️ **变更历史**:
  - **v1 (2026-08-03)**: 11 步原版 (含 suck() Step 0 + drop_object() Step 7)。
  - **v2 (2026-08-04 中午)**: 删掉 suck() 和 drop_object(), 原 11 步 → 9 步。
    序号前移 1 (1→1, 2→2, ... 7→6, 8→7, 9→8, 10→9)。
  - **v3 (2026-08-04 晚上)**: 在 Step 6 和 Step 7 之间加回 drop_object() (放气),
    9 步 → 10 步。序号后移 1 (7→8, 8→9, 9→10), return dict 加回 ``drop_result``
    (无 suck_result)。语义: push 货物到位后**立即放气** → 真空断开 → 货物自然
    落到目标位 → 再撤退 + 归零 (避免真空拖拽导致扔歪 / 撞货物)。

⚠️ **顺序关键** (这条不能乱):
  - 第 1 步 y 抬高是为了让 2-4 步 (set_arm/hand + move_x_mid) 不被保护区拦截。
  - 第 3 步手爪 -30° 后, 第 4 步 x_mid 在 y=-190 (出保护区) → move_x 允许 ✓。
  - 第 4 步 x_mid = -180 是缓冲: 出保护区后第一段 x 调整, 防止后续 y 降下来后
    一次性 -215 太大冲过头。
  - 第 5 步 y 降到 -65 (工作深度) → 仍在保护区外 → move_x 允许 ✓。
  - 第 6 步 x_final 在 y=-65 时调用 → 仍在保护区外 → move_x 允许 ✓ (push 货物到位)。
  - **🆕 v3 第 7 步 drop_object() 在 y=-65 (工作深度) + x=-215 (推到目标位) 时**:
    此时气阀切断真空 → 货物落到目标位。这是 "投递动作" 的关键, 必须在 x 推到位
    **之后**、x 撤退 **之前**。
    位置错误后果:
      - 太早 (push 之前) → 货物在工作位之前掉 → 扔歪
      - 太晚 (撤退之后) → 真空一直吸住, 撤退拖拽货物 → 撞墙/撞货物
  - 第 8 步 x 回到 x_mid (-180): 仍在 y=-65 → 保护区外 → move_x 允许 ✓
    (回到 x_mid 而不是直接归零, 防止大臂手爪在归零过程中撞到已投放的货物/撞墙)。
  - 第 9 步 y 上升回 -190: 准备 x 归零 (归零必须 y ≤ -80 才能调)。
  - 第 10 步 x 归零在 y=-190 时调用 → 保护区外 → move_x 允许 ✓ (撞墙 calibrate)。
  - 第 4/6/8/10 步走 move_x_with_split 防 belt-slip / wall_hit / overshoot
    (参考 main/arm/each_task/common.py:174)。
  - **4 次 x 运动分 4 次走 split**: x_mid -180 + x_final -215 + x_mid -180 + x_return 0。

⚠️ **业务硬限**:
  - y=-190 ≤ soft_y_max=-200 ✓
  - y=-65 ∈ [-200, 0] (保护区外, ≥ -30) ✓
  - arm=+90 ∈ [-150, +90] (上界) ✓
  - hand=-30 ∈ [-90, 0] ✓ (mid mode, 区别于 UP=-90)
  - x ∈ [-320, +220] 软限位 ✓ (-180/-215 距下界还有 140/105mm 余量)

⚠️ **x_mid 复用 (Step 4 + Step 8)**:
  - Step 4 (Phase A 出保护区后第一段 x) 和 Step 8 (Phase C 撤退回程, v3 由原 Step 7 前移)
    都用同一个 POS_X_MID_MM = -180 常量。
  - 现场改 x_mid 自动同步两处, 不会出现 "forward 用 x_mid, retreat 用 x_retreat" 这种
    5mm 不一致。
  - 沿用 9 步版本 (2026-08-03) 的 -180mm, 用户 2026-08-04 没说改值。

⚠️ **y_up 复用 (Step 1 + Step 9)**:
  - Step 1 (Phase A 出保护区) 和 Step 9 (Phase C 撤退抬 y, v3 由原 Step 8 前移)
    都用同一个 POS_Y_UP_MM = -190 常量。
  - 现场改 y_up 自动同步两处。

⚠️ **本文件自包含** (与 task7/target.py / dipan.py / position1/2/3.py 同款):
  只依赖 ``main.arm`` + ``main.arm.each_task.common``, 不 import task7 包内任何模块。

跑法:
    python main/arm/each_task/task7/position5.py
    python -m main.arm.each_task.task7.position5
    python main/arm/each_task/task7/position5.py --x-mid -200 --hand -60  # 现场微调
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


# ---------- 默认参数 ----------

LOG_PREFIX: str = "[task7/position5]"

# Step 1: y 抬高
POS_Y_UP_MM: float = -190.0
"""Step 1: y 抬到 -190mm (出保护区 [0, -30], 给 set_*_angle 留安全余量)。
Step 8: y 从 -65 抬回 -190mm (出工作深度, 给 x 归零让路)。

⚠️ ≤ soft_y_max=-200 才不超业务硬限。
⚠️ Step 1 + Step 8 复用同一个常量, 现场改值自动同步两处。"""

# Step 2: 大臂
POS_ARM_DEG: float = 90.0
"""Step 2: 大臂 +90° (复位位, 业务硬限上界 [-150, +90])。"""

# Step 3: 手爪
POS_HAND_DEG: float = -30.0
"""Step 3: 手爪 -30° (业务硬限内 [-90, 0], mid mode)。

⚠️ 与 position1/2/3 的 -90° (UP) 故意不同 —— 位置 5 是"已抓住货物抬起来"模式,
   -30° 是 "夹持但不完全 UP" 的工作角度 (用户 2026-08-04 实际值, 比 -45° 还 OPEN 一些)。
⚠️ 必须走 ``client.set_hand_angle(angle, speed, timeout=...)`` (ArmRunner 没 set_hand_angle,
   timeout 必填位置参)。见 [[armrunner-set-hand-angle-gotcha]]"""

# Step 4 / Step 8: x 中间位 (Step 4 在 y=-190 出保护区, Step 8 在 y=-65 撤退)
POS_X_MID_MM: float = -180.0
"""Step 4: x 在 y=-190 时滑到中间位 -180mm (出保护区第一段 x 缓冲)。
Step 8: x 在 y=-65 时回到 x_mid (-180) (防止归零冲过头; v3 由原 Step 7 前移)。

⚠️ 沿用 9 步版本 (2026-08-03) 的 -180mm, 用户 2026-08-04 没说改值。
⚠️ 必须 ≥ x_min_m=-320 软限位; -180 距下界还有 140mm 余量。
⚠️ Step 4 + Step 8 复用同一个常量, 现场改值自动同步两处。"""

# Step 5: y 工作深度
POS_Y_DOWN_MM: float = -65.0
"""Step 5: y 降到 -65mm (工作深度, 保护区 [0, -30] 外)。

⚠️ -65 距离保护区下边界 -30 还有 35mm 余量。"""

# Step 6: x 最终位 (在 y=-65 时调用)
POS_X_FINAL_MM: float = -215.0
"""Step 6: x 滑到最终位 -215mm (在 y=-65 时调用, push 货物到位)。

⚠️ 必须 ≥ x_min_m=-320 软限位; -215 距下界还有 105mm 余量。"""

# Step 9: x 归零 (在 y=-190 时调用)
POS_X_RETURN_MM: float = 0.0
"""Step 9: x 归零 (撞墙 calibrate, 重置编码器零点, 跟 position2 同款)。

⚠️ 走 split 兜底, belt-slip 时会走满 seek_timeout 返回 (不假撞墙, 见 ARM_API §9.1)。"""

# 舵机速度
ANGLE_SPEED: int = 80
"""大臂 + 手爪舵机速度, 默认 80 (与 task7 其他脚本一致)。"""


def run(client: ArmClient, runner: ArmRunner) -> dict:
    """按用户顺序执行 10 步动作 (含 1 次放气 Step 7): y up → arm → hand → x mid → y down → x final → drop → x mid → y up → x return。

    ⚠️ **2026-08-04 v3 改**: 在 Step 6 和 Step 7 之间加回 drop_object() (放气)。
       9 步 → 10 步。语义: push 货物到位后立即放气 → 真空断开 → 货物落到目标位,
       然后再撤退 + 归零 (避免真空拖拽导致扔歪 / 撞货物)。
    ⚠️ **2026-08-04 v2 改 (历史)**: 删掉 suck() + drop_object(), 原 11 步 → 9 步;
       v3 又加回 drop_object() 一次, 但**不加回 suck()** (本脚本前提: 货物已在手里)。

    Args:
        client: ArmClient (set_hand_angle + http.execute_car_action + move_x 内部用)
        runner: ArmRunner (move_y + set_arm_angle + **drop_object (Step 7)**)

    Returns:
        {
            "ok": True,
            "y_up_mm":        -190.0,
            "arm_deg":          90.0,
            "hand_deg":        -30.0,
            "x_mid_mm":       -180.0,
            "y_down_mm":       -65.0,
            "x_final_mm":     -215.0,
            "x_return_mm":       0.0,
            "x_mid_result":      dict,    # Step 4 split (Phase A 出保护区)
            "x_final_result":    dict,    # Step 6 split (Phase B push)
            "drop_result":       dict,    # 🆕 v3 Step 7 (Phase B 末尾放气, y=-65 + x=-215)
            "x_mid_return_result": dict,  # Step 8 split (Phase C 撤退, v3 由原 Step 7 前移)
            "x_return_result":   dict,    # Step 10 split (Phase D 归零, v3 由原 Step 9 前移)
        }

    Raises:
        RuntimeError: 业务层异常 (move_x 失败 / 保护区拦截)。
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (10 步动作, 含 1 次放气: 准备→投递+放气→撤退→归零) ==========")
    print(f"  Phase A 准备: y_up={POS_Y_UP_MM}mm + arm={POS_ARM_DEG}° + hand={POS_HAND_DEG}° "
          f"+ x_mid={POS_X_MID_MM}mm (y=-190 时)")
    print(f"  Phase B 投递+放气: y_down={POS_Y_DOWN_MM}mm + x_final={POS_X_FINAL_MM}mm (push) + drop_object()")
    print(f"  Phase C 撤退: x_mid={POS_X_MID_MM}mm (y=-65) + y_up={POS_Y_UP_MM}mm")
    print(f"  Phase D 归零: x_return={POS_X_RETURN_MM}mm (撞墙 calibrate, y=-190)")

    # ========== Phase A: 准备 (steps 1-4) ==========
    # Step 1: y 抬高 (出保护区, 给后续 set_*_angle 和 move_x_mid 留余地)
    print(f"\n  [1/10] move_y({POS_Y_UP_MM}mm)   y 出保护区")
    runner.move_y(POS_Y_UP_MM, verify=True)

    # Step 2: 大臂 +90° (复位位, 保护区允许)
    print(f"  [2/10] set_arm_angle({POS_ARM_DEG}°)  大臂到复位位")
    runner.set_arm_angle(POS_ARM_DEG, speed=ANGLE_SPEED)

    # Step 3: 手爪 -30° (mid mode)
    # ⚠️ ArmRunner 没有 set_hand_angle, 必须走 client.set_hand_angle, timeout 必填。
    print(f"  [3/10] set_hand_angle({POS_HAND_DEG}°)  手爪到 mid mode")
    client.set_hand_angle(
        POS_HAND_DEG, speed=ANGLE_SPEED,
        timeout=runner.default_timeout_s,
    )

    # Step 4: x_mid 在 y=-190 时滑到中间位 -180mm (出保护区第一段 x 缓冲)
    print(f"  [4/10] move_x_with_split({POS_X_MID_MM}mm)  x_mid → 中间位 (y=-190, split 兜底)")
    x_mid_result = move_x_with_split(
        client, runner, POS_X_MID_MM,
        log_prefix=f"  {LOG_PREFIX} step4",
    )

    # ========== Phase B: 投递 (steps 5-6) + 🆕 v3 放气 (step 7) ==========
    # Step 5: y 降到 -65mm (工作深度, 保护区外)
    print(f"\n  [5/10] move_y({POS_Y_DOWN_MM}mm)   y → 工作深度 (保护区外)")
    runner.move_y(POS_Y_DOWN_MM, verify=True)

    # Step 6: x 在 y=-65 时滑到最终位 -215mm (push 货物到位)
    print(f"  [6/10] move_x_with_split({POS_X_FINAL_MM}mm)  x_final → 最终位 (y=-65, split 兜底)")
    x_final_result = move_x_with_split(
        client, runner, POS_X_FINAL_MM,
        log_prefix=f"  {LOG_PREFIX} step6",
    )

    # 🆕 Step 7 (v3 新增): 放气 (断开真空, 货物落到目标位)
    # ⚠️ 必须**在 x 推到位之后** (Step 6 后), **x 撤退之前** (Step 8 前)。
    # 错误位置会导致: 提前放气 → 货物掉在工作位之前 / 推完不放气 → 真空一直吸住,
    # 撤退时拖拽货物 → 扔歪或撞墙。
    # runner.drop_object() 走 runner, 不走 client (跟 suck/drop 同款, 见
    # main/arm/loops/runner.py:185 drop_object)。
    print(f"  [7/10] drop_object()  🆕 v3 放气 (断开真空, 货物落目标位, y=-65 + x=-215)")
    drop_result = runner.drop_object()

    # ========== Phase C: 撤退 (steps 8-9) ==========
    # Step 8: x 回到 x_mid (-180) (y=-65 时, 防止归零冲过头撞到已投放的货物/撞墙)
    print(f"\n  [8/10] move_x_with_split({POS_X_MID_MM}mm)  x_mid 回归 (y=-65, 复用 POS_X_MID_MM, split 兜底)")
    x_mid_return_result = move_x_with_split(
        client, runner, POS_X_MID_MM,
        log_prefix=f"  {LOG_PREFIX} step8",
    )

    # Step 9: y 上升回 -190mm (出工作深度, 给 x 归零让路)
    print(f"  [9/10] move_y({POS_Y_UP_MM}mm)   y 出工作深度 (复用 POS_Y_UP_MM)")
    runner.move_y(POS_Y_UP_MM, verify=True)

    # ========== Phase D: 归零 (step 10) ==========
    # Step 10: x 在 y=-190 时归零 (撞墙 calibrate, 跟 position2 同款)
    print(f"\n  [10/10] move_x_with_split({POS_X_RETURN_MM}mm)  x_return → 0 位 (y=-190, split 撞墙 calibrate)")
    x_return_result = move_x_with_split(
        client, runner, POS_X_RETURN_MM,
        log_prefix=f"  {LOG_PREFIX} step10",
    )

    dt = time.time() - t0
    print(f"========== {LOG_PREFIX} 完成 ({dt:.2f}s) ==========\n")

    return {
        "ok": True,
        "y_up_mm": POS_Y_UP_MM,
        "arm_deg": POS_ARM_DEG,
        "hand_deg": POS_HAND_DEG,
        "x_mid_mm": POS_X_MID_MM,
        "y_down_mm": POS_Y_DOWN_MM,
        "x_final_mm": POS_X_FINAL_MM,
        "x_return_mm": POS_X_RETURN_MM,
        "x_mid_result": x_mid_result,
        "x_final_result": x_final_result,
        "drop_result": drop_result,                      # 🆕 v3 新增: Step 7 放气
        "x_mid_return_result": x_mid_return_result,
        "x_return_result": x_return_result,
    }


def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 7 个关键常量都可 override, 现场微调用。"""
    p = argparse.ArgumentParser(
        description=(
            "task7 position5: y up(-190) → arm 90° → hand -30° → x mid(-180) "
            "→ y down(-65) → x final(-215) → x mid(-180) → y up(-190) → x return(0) "
            "(无吸/放气)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--y-up", type=float, default=POS_Y_UP_MM,
                   help="Step 1/8 y 抬高目标 (mm, 默认 -190, 复用)")
    p.add_argument("--arm", type=float, default=POS_ARM_DEG,
                   help="Step 2 大臂角度 (°, 默认 +90)")
    p.add_argument("--hand", type=float, default=POS_HAND_DEG,
                   help="Step 3 手爪角度 (°, 默认 -30=mid mode, 不是 UP)")
    p.add_argument("--x-mid", type=float, default=POS_X_MID_MM,
                   help="Step 4/7 x 中间位 (mm, 默认 -180, 复用, 现场改自动同步两处)")
    p.add_argument("--y-down", type=float, default=POS_Y_DOWN_MM,
                   help="Step 5 工作深度 (mm, 默认 -65=保护区外)")
    p.add_argument("--x-final", type=float, default=POS_X_FINAL_MM,
                   help="Step 6 最终 x (mm, 默认 -215, 在 y=-65 时调用)")
    p.add_argument("--x-return", type=float, default=POS_X_RETURN_MM,
                   dest="x_return",
                   help="Step 9 归零 x (mm, 默认 0=撞墙, 在 y=-190 时调用)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    # 让 CLI 覆盖生效
    global POS_Y_UP_MM, POS_ARM_DEG, POS_HAND_DEG
    global POS_X_MID_MM, POS_Y_DOWN_MM, POS_X_FINAL_MM, POS_X_RETURN_MM
    POS_Y_UP_MM = args.y_up
    POS_ARM_DEG = args.arm
    POS_HAND_DEG = args.hand
    POS_X_MID_MM = args.x_mid
    POS_Y_DOWN_MM = args.y_down
    POS_X_FINAL_MM = args.x_final
    POS_X_RETURN_MM = args.x_return
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner)
    return 0


if __name__ == "__main__":
    sys.exit(main())
