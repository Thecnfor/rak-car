"""task4 / target1 —— 摆到 'target1' 目标位姿 (用户 2026-07-22 指定)。

目标位姿:
  - y 轴  → -133 mm
  - 大臂角度 →  +90°  (MID / init / 复位位)
  - 手爪角度 →   0°   (DOWN)
  - x 轴  → -260 mm

动作顺序 (用户 2026-07-22 指定, 2026-07-27 arm 0°→90°, 2026-07-28 y -125→-133):
  1. move_y(-133)            先把 y 移出保护区 [0,-30]
  2. set_arm_angle(+90°)     MID/复位位, init 例外位, 保护区允许
  3. set_hand_angle(0°)      DOWN, arm=+90° 已在复位安全带 (>= +30°),
                              api.py:432-435 自动跳过 y 保护区拦截
  4. move_x(-260)            远距 260mm, hard_reach 模式 (split + reset_x 撞墙兜底)
                                —— 走 common.move_x_hard_reach
                                   (split 试一次 → 没到位 → reset_x 撞墙 → 再 split)
                                底层走 api.move_x (v_max_mms 透传 + _check_step_loss)
                                + api._read_x_mm_realtime 校验 (x_get_position 坏)
                                ⚠️ **目标超物理墙 (≈ -119.5mm, 见 ARM_API §7.2)**
                                   实测 motor 只走到 ≈ -80mm (belt-slip 等), split 模式
                                   反复 stall → reset_x 撞墙重置 → 再 split
                                   **最终位置 ≈ -119.5mm, 不到 -260mm**

⚠️ 本文件**自包含**: 只依赖 main.arm (ArmClient/ArmRunner) + main.arm.each_task.common
   (共享工具, 不依赖 task4 包内其它模块如 constants.py / pick_up_blue.py)。
   原因: task5 目录里的辅助文件曾被外部动作清空过 (见会话记录),
         自包含保证 `python target1.py` 直接跑不受影响。
⚠️ x 移动细节 (2026-08-01 更新以匹配 api.py 新接口):
  - 走 common.move_x_hard_reach (split + reset_x 撞墙兜底, 鲁棒模式)
  - 底层 client.move_x 现在透传 v_max_mms (2026-07-28 修) 并调 _check_step_loss
  - x 真值走 client._read_x_mm_realtime() (arm_feed 20Hz, x_get_position 坏)
  - 失败原因走 client.last_realtime_error() (统一错误上下文, 区分网络不通 / arm_feed 停 / 字段 None)

跑法 (两种都行):
    python main/arm/each_task/task4/target1.py
    python -m main.arm.each_task.task4.target1
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
from main.arm.each_task import common  # noqa: E402


# ---------- 目标位姿常量 (内联, 不依赖 constants.py) ----------

LOG_PREFIX: str = "[task4/target1]"

TARGET1_Y_MM: float = -133
"""target1 y。出 y 保护区 [0,-30]。y 软上限 = -soft_y_max_mm=-200, target1 留 67mm
余量到 -133, 不贴软限位边界; arm_base y 阈值 [-0.20, 0.0] 不变。
2026-07-28: y 校准到 -133 (用户实测球检测落点最稳, 对应 BALL_VERIFIED_*
第 7 次现场实测; 之前 -150 / -125 / -100 都已弃用)。"""

TARGET1_X_MM: float = -260.0
"""target1 x。距 init=0 远 260mm, split 模式分段。

⚠️ **超物理墙**: 实测 x 物理墙 ≈ -119.5mm (ARM_API §7.2, test_x_to_150.py docstring),
目标 -260mm 会撞墙 → wall_hit 检测 → 最终位置 ≈ -119.5mm, 不到 -260mm。
如要真到位需现场确认 rail 长度或校准原点。

⚠️ **2026-07-31 v6++ 用户反馈**: belt-slip 已修复, 速度 30→40→80 mm/s
(SDK 无硬限, arm_base.py:482-487 临时收紧 x_velocity_limit)。
trust 模式单次 move_x 不够, 改 split 模式 (move_x_with_split) 带
stall/wall/overshoot 检测, 鲁棒参数 (tol=10 / stall=10 / max_stall=5)。
"""

TARGET1_ARM_DEG: float = 90.0
"""大臂 MID / 复位位 (+90°)。init 例外位, 保护区允许 (+90° 命中 allow_init_position 分支)。
2026-07-27: arm 0° → 90°（业务硬限第三次重定义 [+90, -150]°）。"""

TARGET1_HAND_DEG: float = 0.0
"""手爪 DOWN。arm=+90° 已在复位安全带 (>= +30°), api.py:432-435 自动跳过 y 保护区拦截。
不必再走底层 _call_arm 直调（保留直调作为防御纵深）。"""

# belt-slip 安全 move_x 参数 (走 common.move_x_with_split, 鲁棒参数适配 belt-slip 修复后)
MOVE_X_V_MAX_MMS: float = 80.0
"""业务限速 (mm/s)。2026-07-31 用户反馈 belt-slip 已修复, 30→40→80 试;
SDK 无硬限 (arm_base.py:482-487 临时收紧 x_velocity_limit, 业务层任意值)。"""

# move_x_with_split 鲁棒参数 (belt-slip 修复后放宽)
MOVE_X_TOL_MM: float = 10.0           # 容差放宽 (10mm, belt-slip 刚修实时性波动)
MOVE_X_STALL_MM: float = 10.0         # stall 判定放宽 (10mm, belt-slip 残余抖动)
MOVE_X_MAX_STALL_ROUNDS: int = 5      # 容忍 5 轮 stall (旧 3, belt-slip 修复初期)
MOVE_X_KICK_SLEEP_S: float = 0.3      # 多给 0.1s 让带重咬合
MOVE_X_MAX_ROUNDS: int = 15           # 多给 3 轮机会
MOVE_X_WALL_MM: float = -300.0        # x 物理墙 (belt-slip 修好后撞墙检测生效)
MOVE_X_WALL_TOL_MM: float = 30.0      # 距墙 30mm 视为撞墙


# ---------- 主入口 ----------

def step_target1(client: ArmClient, runner: ArmRunner,
                 y_mm: float = TARGET1_Y_MM,
                 x_mm: float = TARGET1_X_MM,
                 arm_deg: float = TARGET1_ARM_DEG,
                 hand_deg: float = TARGET1_HAND_DEG) -> dict:
    """把臂摆到 target1 位姿 (y=-100→arm=+90°→hand=0°→x=-260).

    Returns:
        {"ok": True, "y_mm": float, "x_info": dict, "arm_deg": float, "hand_deg": float}

    ⚠️ **x=-260 超过物理墙 ≈ -119.5mm** (ARM_API §7.2 / test_x_to_150.py):
       stall 检测 (3 轮无进展) 兜底 → 最终位置 ≈ -119.5mm, **不到 -260mm**。
       ⚠️ **2026-07-31 实测更差**: motor 实际只走到 ≈ -80mm (belt-slip 等问题),
       trust 模式不报 stall, 业务层看不见。代码保持 -260 是用户决定。
    """
    print(f"\n========== {LOG_PREFIX} step_target1 ==========")
    if x_mm < -119.5:
        print(f"  ⚠️ x={x_mm}mm 超过物理墙 ≈ -119.5mm, 会撞墙 + stall 兜底,"
              f" 最终约停在 -119.5mm (到不了 {x_mm}mm)")
    if x_mm <= -260.0:
        print(f"  ⚠️⚠️ 2026-07-31 实测更差: motor 实际只走到 ≈ -80mm (belt-slip 等),"
              f" 不到物理墙 -119.5mm。trust 模式不报 stall, 业务层看不见。"
              f" 代码保持 -260 是用户决定, 等硬件修复后再真到位。")
    print(f"  目标: y={y_mm}mm → arm={arm_deg}° → hand={hand_deg}° → x={x_mm}mm (最后动)")

    # 1. y 出保护区
    print(f"  [1/4] move_y({y_mm}mm)  出 y 保护区 [0,-30]")
    runner.move_y(y_mm, timeout=30.0)

    # 2. 大臂 MID / 复位位 (init 例外位, 保护区允许)
    print(f"  [2/4] set_arm_angle({arm_deg}°)  MID / 复位位 (init 例外位)")
    client.set_arm_angle(arm_deg, speed=80, timeout=10.0)

    # 3. 手爪 DOWN —— arm=+90° 已在复位安全带 (>= +30°), api.py:432-435 自动跳过 y 保护区拦截
    print(f"  [3/4] 手爪 → {hand_deg}° (DOWN, arm={arm_deg}° 复位位, 保护区允许)")
    client._call_arm(
        "set_hand_angle", timeout=10.0, sync=True,
        angle=hand_deg, speed=80,
    )

    # 4. x (方案 B: hard_reach 模式 = split + reset_x 撞墙兜底; 放在最后, 摆好姿态再横移)
    #    ⚠️ 2026-07-31 v6+++: 用户确认 belt-slip 修复, 但 x 物理墙 ≈ -119.5mm, target=-260 必撞墙。
    #    走 move_x_hard_reach: 先 split 试一次, 没到位就 reset_x 撞墙重置编码器零点,
    #    再 split 一次。belt-slip 修复后 reset_x 撞墙应该稳, belt-slip 复发抛 RuntimeError。
    #    参数放宽 (tol=10 / stall=10 / max_stall=5) 适配 belt-slip 修复初期抖动。
    #    2026-08-01: hard_reach 现在内置 FUSE-rescue (FUSE 触发时先 reset_x 再抛),
    #               这里仍然 catch RuntimeError 作**最后兜底** —— 让 target1 不因
    #               单次 x 失败整个崩, 退化到 "y/arm/hand 摆好 + x 留在当前位置" 继续。
    print(f"  [4/4] move_x({x_mm}mm)  hard_reach 模式 "
          f"(split + reset_x 撞墙兜底, belt-slip 修复后)")
    x_info: dict = {"target_x": x_mm, "actual_x": None, "final_x": None,
                    "residual_mm": None, "segments": 0, "reached": False,
                    "result": "skipped", "wall_hit": False, "overshoot_mm": 0.0,
                    "reset_count": 0, "degraded": False}
    try:
        x_info = common.move_x_hard_reach(
            client, runner,
            target_x_mm=x_mm,
            log_prefix=f"{LOG_PREFIX}",
            v_max_mms=MOVE_X_V_MAX_MMS,
            tol_mm=MOVE_X_TOL_MM,
            stall_mm=MOVE_X_STALL_MM,
            max_stall_rounds=MOVE_X_MAX_STALL_ROUNDS,
            kick_sleep_s=MOVE_X_KICK_SLEEP_S,
            max_rounds=MOVE_X_MAX_ROUNDS,
            wall_mm=MOVE_X_WALL_MM,
            wall_tol_mm=MOVE_X_WALL_TOL_MM,
            reset_direction="right",  # 跟 task4 x_to_zero.py 默认一致 (init 位 / 蓝色 bin 在 x 增大方向)
        )
        print(f"        x_info={x_info}")
    except RuntimeError as fuse_err:
        # 兜底: motor 完全没响应 (FUSE-rescue 也救不回来) → 退化模式
        # y/arm/hand 已经摆好, x 留在当前位置; 让 target1 不崩, 业务层看 result 知道
        actual_x = client._read_x_mm_realtime()
        x_info = {
            "target_x": x_mm, "actual_x": actual_x, "final_x": actual_x,
            "residual_mm": (actual_x - x_mm) if actual_x is not None else None,
            "segments": 0, "reached": False,
            "result": "fuse_rescue_failed", "wall_hit": False, "overshoot_mm": 0.0,
            "reset_count": 0, "degraded": True,
            "error": str(fuse_err)[:200],
        }
        print(f"  [{LOG_PREFIX}] ⚠️⚠️ hard_reach 抛错 (FUSE-rescue 也救不回来); 退化模式继续")
        print(f"        x_info={x_info}")
        print(f"        ⚠️ target1 退化: y/arm/hand 已摆好, x 留在 {actual_x} (期望 {x_mm})")
        print(f"        ⚠️ 后续 target4 抓球可能失败 (球在 x=-260 抓取位, 当前 x={actual_x})")

    # 4.1 二次校验: realtime x 真值 + 错误上下文 (api.py §11 新接口)
    #     common.move_x_hard_reach 内部用 client._read_x_mm_realtime, 这里再读一次
    #     验证最终位置 + 暴露 last_realtime_error 给上层 (便于诊断网络/feed 问题)。
    actual_x = client._read_x_mm_realtime()
    rt_err = client.last_realtime_error()
    if rt_err:
        print(f"  [{LOG_PREFIX}] ⚠️ realtime 读数有问题: {rt_err}")
    if actual_x is not None:
        residual = actual_x - x_mm
        print(f"  [{LOG_PREFIX}] x 二次校验: realtime={actual_x:+.1f}mm "
              f"目标={x_mm:+.1f}mm 残差={residual:+.1f}mm")
        if abs(residual) > MOVE_X_TOL_MM:
            print(f"  [{LOG_PREFIX}] ⚠️ x 残差 {abs(residual):.1f}mm > "
                  f"容差 {MOVE_X_TOL_MM:.0f}mm (hard_reach 模式应该兜底, "
                  f"异常; 可能是 belt-slip 复发或物理墙挡住)")

    print(f"========== {LOG_PREFIX} 完成 "
          f"(y={y_mm} arm={arm_deg}° hand={hand_deg}° x_realtime={actual_x}) "
          f"==========\n")
    return {
        "ok": True,
        "y_mm": y_mm,
        "x_info": x_info,
        "arm_deg": arm_deg,
        "hand_deg": hand_deg,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task4 target1: 臂摆到 target1 位姿 "
                    "(y=-133→arm=+90°→hand=0°→x=-260; ⚠️ x 超过物理墙 -119.5mm, "
                    "2026-07-31 实测 motor 只走到 ≈ -80mm, 等硬件修复)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--y", type=float, default=TARGET1_Y_MM, help="y (mm)")
    p.add_argument("--x", type=float, default=TARGET1_X_MM, help="x (mm)")
    p.add_argument("--arm", type=float, default=TARGET1_ARM_DEG, help="大臂角度 (°)")
    p.add_argument("--hand", type=float, default=TARGET1_HAND_DEG, help="手爪角度 (°)")
    return p


def main(argv=None) -> int:
    t_total_start = time.perf_counter()
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_target1(client, runner, y_mm=args.y, x_mm=args.x,
                 arm_deg=args.arm, hand_deg=args.hand)
    elapsed = time.perf_counter() - t_total_start
    print(f"========== {LOG_PREFIX} 总耗时: {elapsed:.3f} s ==========")
    return 0


if __name__ == "__main__":
    sys.exit(main())