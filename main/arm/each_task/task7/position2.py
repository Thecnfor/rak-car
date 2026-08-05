"""task7 / position2 —— **位置 2** 的位姿序列 (7 步臂, 含 1 次放气: y up → arm → hand → y down → x to → drop → x home)。

按用户 2026-08-04 v4 指定顺序:

  1. move_y(-190mm)              y 抬高到 -190mm, 完全出保护区 [0, -30]
  2. set_arm_angle(+90°)         大臂归为 +90° (y_up 之后立刻归位)
  3. set_hand_angle(-40°)        手爪到 -40° (mid mode, 业务硬限内)
  4. move_y(-135mm)              y 降回工作深度 -135mm (动手爪后安全)
  5. move_x_with_split(-225mm)   x 滑到位置 2 (-225mm), push 货物到位
  6. runner.drop_object()         🆕 v4 新增: 放气 (断开真空, 货物落到目标位)
  7. move_x_with_split(0mm)      x 回 0 位 (撞墙 = 0), 原 Step 6 前移

⚠️ **变更历史**:
  - **v1 (2026-08-03)**: 7 步原版 (含 suck() Step 0 + drop_object() Step 5)。
  - **v2 (2026-08-04 中午)**: 删掉 suck() 和 drop_object(), 原 7 步 → 5 步。
  - **v3 (2026-08-04 下午)**: 在 y_up 之后插入 set_arm_angle(+90°), 5 步 → 6 步。
  - **v4 (2026-08-04 晚上)**: 在 Step 5 (x_to push) 和原 Step 6 (x_return) 之间加回
    drop_object() (放气), 6 步 → 7 步。序号后移 1 (6→7), return dict 加回
    ``drop_result`` (无 suck_result)。语义: push 货物到位后立即放气 → 真空断开 →
    货物落到目标位, 然后再 x 回 0 (撞墙 calibrate)。

⚠️ **顺序关键** (这条不能乱):
  - 第 1 步 y 抬高是为了让第 2/3 步 (set_arm_angle +90° + set_hand_angle -40°) 的
    _check_y_protected 放行 (保护区 y ∈ [0, -30] 内只能允许 set_hand(UP) 和 set_arm(MID)).
  - 第 2 步大臂 +90° 必须在 y=-190 之后调 (y_up 后立刻归位, 用户 2026-08-04 要求)。
  - 第 3 步手爪 -40° 后, 第 4 步降回 -135 才安全 (手爪 OUT 后手才不刮底部东西)。
  - 第 5 步 x 滑到 -225 (push 货物到位)。
  - **🆕 v4 第 6 步 drop_object() 在 y=-135 (工作深度) + x=-225 (推到目标位) 时**:
    此时气阀切断真空 → 货物落到目标位。这是 "投递动作" 的关键, 必须在 x 推到位
    **之后**、x 归零 **之前**。
    位置错误后果:
      - 太早 (push 之前) → 货物在工作位之前掉 → 扔歪
      - 太晚 (归零之后) → 真空一直吸住, 归零时拖拽货物 → 撞墙/撞货物
  - 第 7 步 x 回 0 位 (撞墙 calibrate, v4 由原 Step 6 前移)。
  - 第 5/7 步走 move_x_with_split 防 belt-slip / wall_hit / overshoot
    (参考 main/arm/each_task/common.py:174)。

⚠️ **业务硬限** (走前要核对):
  - y=-190 ≤ soft_y_max=-200 ✓ (在 [-200, 0] 内)
  - hand=-45 ∈ [HAND_ANGLE_MIN, HAND_ANGLE_MAX] = [-90, 0] ✓ (mid mode, 业务硬限内)
  - x ∈ [-320, +220] 软限位 ✓ (-220 距下界 -320 还有 100mm 余量)
  - 大臂角度保持不变 (用户未指定, 沿用前一步 +90° reset 位)

⚠️ **不走 set_arm_angle**: 用户序列里没有, 大臂停在原位 (假设上一步是 +90°)。
   如果前一步不是 reset 位, 调用方需自己负责把大臂先摆到安全位再调本脚本。

⚠️ **本文件自包含** (与 task7/target.py / task7/dipan.py 同款):
  只依赖 ``main.arm.ArmClient`` + ``main.arm.each_task.common.move_x_with_split``,
  不 import task7 包内任何模块。

跑法:
    python main/arm/each_task/task7/position2.py
    python -m main.arm.each_task.task7.position2
    python main/arm/each_task/task7/position2.py --y-up -200   # 想抬高更狠就改
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


# ---------- 位置 2 序列常量 ----------

LOG_PREFIX: str = "[task7/position2]"

# 第 1 / 3 步: y 高度
POS_Y_UP_MM: float = -190.0
"""第 1 步 y 抬到 -190mm (远出保护区 [0, -30], 给手爪动作留 160mm 自由深度)。

⚠️ ≤ soft_y_max=-200 才不超位置硬限。"""

POS_Y_DOWN_MM: float = -135
"""第 3 步 y 降回 -135mm (工作深度, 用户 2026-08-04 实际值)。

⚠️ 此值与 task7/target.py setup (-80mm) **故意不同**, 是位置 2 的特殊深度:
   位置 2 横向较远 (-220mm), 货物位置比位置 1 高约 55mm, 所以 y 也跟加深 55mm。
   跑完整任务流程时, 跑完 position2 要回到 target.py 兼容的 y, 需要再调一次
   move_y(-80mm) (不在本脚本里, 由上层编排)。"""

# 第 2 步: 手爪
POS_HAND_DEG: float = -40
"""第 3 步手爪到 -40° (mid mode, 业务硬限内 [-90, 0])。

⚠️ 与 position5 的 -30° (work mode) **故意不同** —— position2/3/4 横向较远, 货物位置比标准位高,
   -40° 是适配的工作角度 (2026-08-04 用户统一改成 -40°)。
   mid mode = "夹持但不完全 UP" 的工作角度, 适合已经抓住货物后保持。
   改名历史: 原 POS_HAND_UP_DEG=-45, 名字 "UP" 与值 -45 矛盾, 2026-08-04 改成 POS_HAND_DEG。
⚠️ 必须走 ``client.set_hand_angle(angle, speed, timeout=...)`` (ArmRunner 没 set_hand_angle,
   timeout 必填位置参)。见 [[armrunner-set-hand-angle-gotcha]]"""

POS_ARM_DEG: float = 90
"""第 2 步大臂归为 +90° (y_up 之后立刻归位, 用户 2026-08-04 要求)。

⚠️ +90° 是业务硬限上界 [-150, +90] 内的合法值, 跟 position4/5/6 同款 (它们都有这一步)。
⚠️ 必须 y ≤ -80 (保护区外) 才能调 set_arm_angle(非 0/MID), 所以本步在 y_up=-190 之后。
⚠️ 走 ``runner.set_arm_angle(angle, speed=...)`` (ArmRunner 有这个方法, 默认 timeout 80s)。"""

# 第 5 / 7 步: x 位置
POS_X_TO_MM: float = -225
"""第 5 步 x 滑到位置 2 (向撞墙相反方向 -225mm, 用户 2026-08-04 实际值)。

⚠️ 必须 ≥ x_min_m=-320 (软限位), 否则 SDK limit_val() 钳掉。
   -225 距 -320 还有 95mm 余量, 视觉闭环 / 二次再移空间宽松。
   改动轨迹: -200 → -220 → -225 (最终值, 位置 2 横向逐步远离墙)。"""

POS_X_RETURN_MM: float = 0.0
"""第 7 步 x 回 0 位 (撞墙位 = x=0, v4 由原 Step 6 前移)。

⚠️ 走 split 兜底, belt-slip 时会走满 seek_timeout 返回 (不假撞墙, 见 ARM_API §9.1)。
   撞墙是 calibrate, 重置编码器零点; 后续视觉闭环用此作为起点。"""

# 舵机速度 (与 task7/target.py / task5/target.py 一致)
ANGLE_SPEED: int = 80
"""大臂 + 手爪舵机速度, 默认 80。"""


def run(client: ArmClient, runner: ArmRunner) -> dict:
    """按用户顺序执行 7 步臂 (含 1 次放气 Step 6): y up → arm → hand → y down → x to → drop → x home。

    ⚠️ **2026-08-04 v4 改**: 在 Step 5 (x_to push) 和原 Step 6 (x_return) 之间加回
       drop_object() (放气), 6 步 → 7 步。语义: push 货物到位后立即放气 → 真空断开
       → 货物落到目标位, 然后再 x 回 0 (撞墙 calibrate, 避免真空拖拽导致扔歪/撞货物)。
    ⚠️ **2026-08-04 v3 改 (历史)**: 在 y_up 之后插入 set_arm_angle(+90°), 5 步 → 6 步。
    ⚠️ **2026-08-04 v2 改 (历史)**: 删掉 suck() + drop_object(), 原 7 步 → 5 步;
       v4 又加回 drop_object() 一次, 但**不加回 suck()** (本脚本前提: 货物已在手里)。

    Args:
        client: ArmClient (move_x_with_split + set_hand_angle + http.execute_car_action 内部用到)
        runner: ArmRunner (move_y / set_arm_angle / **drop_object (Step 6)**)

    Returns:
        {
            "ok": True,
            "y_up_mm":    float,   # -190
            "arm_deg":    float,   # +90 (新增, 大臂归位)
            "y_down_mm":  float,   # -135 (工作深度)
            "hand_deg":   float,   # -40 (mid mode)
            "x_to_mm":    float,   # -225 (位置 2)
            "x_return_mm": float,  # 0 (撞墙)
            "x_to_result": dict,   # Step 5 move_x_with_split 位置 2 返回
            "drop_result": dict,   # 🆕 v4 Step 6 放气 (y=-135 + x=-225)
            "x_return_result": dict,  # Step 7 move_x_with_split 回 0 返回 (v4 由原 Step 6 前移)
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (7 步臂, 含 1 次放气: 准备→投递+放气→归零) ==========")

    # ========== Phase A: 准备 (steps 1-4) ==========
    # 1. y 抬高 (出保护区, 给 set_arm_angle +90° 和 set_hand_angle 留余地)
    print(f"  [1/7] move_y({POS_Y_UP_MM}mm)    y 抬高完全出保护区")
    runner.move_y(POS_Y_UP_MM, verify=True)

    # 2. 大臂归为 +90° (y_up 之后立刻归位, 用户 2026-08-04 要求)
    # ⚠️ +90° 是业务硬限上界 [-150, +90] 内合法值, 在 y=-190 (保护区外) 调 _check_safe 不会拦截。
    # ⚠️ 跟 position4/5/6 同款 (它们都有这一步)。走 runner.set_arm_angle (ArmRunner 有这个方法)。
    print(f"  [2/7] set_arm_angle({POS_ARM_DEG}°)   大臂归为 +{POS_ARM_DEG:.0f}° (y=-190, 保护区外)")
    runner.set_arm_angle(POS_ARM_DEG, speed=ANGLE_SPEED)

    # 3. 手爪 -40° (mid mode, 保护区允许 + 业务硬限内)
    # ⚠️ ArmRunner 没有 set_hand_angle (只有 set_storage), 必须走 client.set_hand_angle,
    #    且 timeout 是必填位置参 (与 set_arm_angle 不同)。见 [[armrunner-set-hand-angle-gotcha]]
    print(f"  [3/7] set_hand_angle({POS_HAND_DEG}°)   手爪到 mid mode")
    client.set_hand_angle(
        POS_HAND_DEG, speed=ANGLE_SPEED,
        timeout=runner.default_timeout_s,
    )

    # 4. y 降回工作深度 (手爪 OUT 后安全降)
    print(f"  [4/7] move_y({POS_Y_DOWN_MM}mm)    y 降回工作深度")
    runner.move_y(POS_Y_DOWN_MM, verify=True)

    # ========== Phase B: 投递 (step 5) + 🆕 v4 放气 (step 6) ==========
    # 5. x 滑到位置 2 (-225mm) (push 货物到位)
    print(f"\n  [5/7] move_x_with_split({POS_X_TO_MM}mm)  x → 位置 2 (split 兜底)")
    x_to_result = move_x_with_split(
        client, runner, POS_X_TO_MM,
        log_prefix=f"  {LOG_PREFIX} step5",
    )

    # 🆕 6. (v4 新增): 放气 (断开真空, 货物落到目标位)
    # ⚠️ 必须**在 x 推到位之后** (Step 5 后), **x 归零之前** (Step 7 前)。
    # runner.drop_object() 走 runner, 不走 client (跟 suck/drop_object 同款, 见
    # main/arm/loops/runner.py:185 drop_object)。
    print(f"  [6/7] drop_object()  🆕 v4 放气 (断开真空, 货物落目标位, y=-135 + x=-225)")
    drop_result = runner.drop_object()

    # ========== Phase C: 归零 (step 7, v4 由原 Step 6 前移) ==========
    # 7. x 回 0 位 (撞墙 calibrate)
    print(f"\n  [7/7] move_x_with_split({POS_X_RETURN_MM}mm) x → 0 位 (split 撞墙 calibrate)")
    x_return_result = move_x_with_split(
        client, runner, POS_X_RETURN_MM,
        log_prefix=f"  {LOG_PREFIX} step7",
    )

    dt = time.time() - t0
    print(f"========== {LOG_PREFIX} 完成 ({dt:.2f}s) ==========\n")

    return {
        "ok": True,
        "y_up_mm": POS_Y_UP_MM,
        "arm_deg": POS_ARM_DEG,
        "y_down_mm": POS_Y_DOWN_MM,
        "hand_deg": POS_HAND_DEG,
        "x_to_mm": POS_X_TO_MM,
        "x_return_mm": POS_X_RETURN_MM,
        "x_to_result": x_to_result,
        "drop_result": drop_result,                       # 🆕 v4 新增: Step 6 放气
        "x_return_result": x_return_result,
    }


def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 当前只暴露 --y-up (其它常量顶在文件里), 保留接口对齐 task7 其他脚本。"""
    p = argparse.ArgumentParser(
        description=(
            "task7 position2: y up → arm(+90°) → hand → y down → x to → drop → x home (位置 2 序列, 含 1 次放气)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--y-up", type=float, default=POS_Y_UP_MM,
                   help="第 1 步 y 目标 (mm, 默认 -190, 抬高越狠越负)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    # 跑 run() 时用 build_parser 的 --y-up 覆盖 (其它常量固定)
    global POS_Y_UP_MM
    POS_Y_UP_MM = args.y_up
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner)
    return 0


if __name__ == "__main__":
    sys.exit(main())