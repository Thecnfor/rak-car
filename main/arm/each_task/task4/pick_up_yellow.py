#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task4 / pick_up_yellow —— 抓+放+回抓取位 序列 (2026-07-31 v6, 黄球版)

逻辑 (9 步, 仿 pick_up_blue.py v6, 唯一区别: 步骤 5 移 x 到 -65 = YELLOW_BIN_X_MM):
  1. 记录 x 轴当前位置 (仅日志, 已知 realtime 飘, 不用于回位)
  2. 吸气
  3. y 到 -58 (抓球位)
  4. y 到 -190 (中转位, 持球抬升)
  5. 移 x 到 -65 (黄色 bin 正上方)        ← **跟 pick_up_blue 唯一区别**
  6. y 到 -155 (放球位, 命中开仓 y gate 上界)
  7. 放气
  8. y 到 -133 (最终位, 等下一阶段 target 识别)
  9. **移 x 到 RETURN_X_MM (默认 -260 = target1.py 抓取位)** —— 与步骤 8 composite_run 并行

⚠️ **跟 pick_up_blue.py v6 的区别**:
   - 唯一区别: **步骤 5** x 移 -65 (黄 bin) 而不是 0 (蓝 bin)
     (见 constants.YELLOW_BIN_X_MM / BLUE_BIN_X_MM)
   - 其他步骤 1-4, 6-9 完全一致 (吸气窗口 / y 值序列 / composite 并行 / 回抓取位)

⚠️ **跟 pick_up_blue.py 共享的设计**:
   - 2026-08 改写: 步骤 4+5 (y 抬升 ∥ x 横移) / 8+9 (y 最终位 ∥ x 回抓取位)
     改 composite_run 并行, x 仍是绝对位置指令。belt-slip + stall 现场史
     见 memory/x-axis-belt-slip。
   - grasp 必须走 runner.grasp (ArmRunner 封装), 不可走 _call_arm 直调
     (ARM_API §10: kwargs 透传到 arm_base.grasp TypeError 静默失败)
   - v6 加回抓取位步骤 9 —— 用户痛点: v5 删回位后 x 不回抓取位

⚠️ **业务流选项** (return_x_mm 参数, 同 pick_up_blue):
   - 默认 -260.0 = target1.py 抓取位 (业务流推荐)
   - -220.0 = target4.py 准备位姿 x (prep_x)
   - None = 不回位 (v5 行为兼容)
   - 其他值 = 自定义

⚠️ 本脚本**只做** pick-and-return 序列, 不做位姿 setup, 调用前必须先保证:
  - arm = +90° (MID / 复位位, 业务硬限 [+90, -150]°)
  - hand = 0°  (DOWN, 业务硬限 [-90, 0]°)
  - y 出 [0, -30] 保护区 (任意 ≤ -30 mm 即可)
  - x 已撞墙定原点 (init 时已 reset_x 过, x=0 = 蓝色 bin 正上方;
    黄 bin 在 x=-65, 距 init 65mm, belt-slip 高风险单步)

典型用法 (在 test_yellow.py 摆位姿后单独跑):
  1. python test_yellow.py      # 摆位姿到 (y=-155, arm=+90°, hand=0°, x=-65 黄 bin)
  2. python pick_up_yellow.py   # 抓→放→回 -260 (v6 默认)
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


# ---------- 步骤参数 ----------
Y_PICK_MM: float = -58.0
"""抓球 y。**非** PICK_Y_MM=-160, 用户特定场景 (现场拍板)。"""

Y_TRANSIT_MM: float = -190.0
"""中转 y。同 SAFE_Y_TRANSIT_MM。持球抬升, 远离底部 + 出 y 保护区 + 出 gate。"""

Y_PUT_MM: float = -155.0
"""放球 y。命中开仓 y gate 上界 [-205, -145] (实测 -155 命中)。"""

Y_FINAL_MM: float = -133.0
"""最终 y (回位后)。target 识别用这个 y 拍球 (跟历史 target1 校准 y 一致)。"""

# 关键区别: 黄 bin x 位置 (跟 pick_up_blue.py 的 X_BIN_MM=0 不同)
X_BIN_MM: float = -65.0
"""黄色 bin x 位置 (= constants.YELLOW_BIN_X_MM, 距 init 0 是 65mm,
belt-slip 单次有效行程 24-46mm 高风险单步, 但 trust 模式不 stall kick)。

⚠️ **跟 pick_up_blue 的唯一区别**: pick_up_blue.py 的 X_BIN_MM = 0.0 (蓝 bin),
   pick_up_yellow.py 的 X_BIN_MM = -65.0 (黄 bin)。
"""

# v6: 放完球后 x 回到的目标位置 (mm), 同 pick_up_blue
# 2026-07-31 改回 -260: 用户决定保持 -260 (等硬件修复后再真到位)
DEFAULT_RETURN_X_MM: Optional[float] = -260.0
"""放完球后 x 自动回的目标位置 (mm)。

业务流选项:
  - -260.0 (默认) = target1.py 抓取位 (业务流推荐)
  - -220.0         = target4.py 准备位姿 x (给 target4 用户用)
  - None           = 不回位 (v5 行为兼容)
  - 其他值          = 自定义 (绝对位置指令, motor 内部闭环准)

⚠️ **2026-07-31 已知 motor 走不到 -260**: 用户实测 motor 实际只走到 ≈ -80mm
   (belt-slip / 编码器失同步 / 电流保护, 不是物理墙 -119.5mm 的限制)
   trust 模式不报 stall, 业务层看不见实际停在 ≈ -80mm
   代码保持 -260 是用户决定, 等硬件修复后再真到位。

⚠️ 走 trust 模式 move_x_trust: 不 stall kick, 信任 motor PID 内部位置闭环。
   关键 (api.py:418): move_x 是绝对位置指令, motor 内部编码器闭环,
   不依赖 realtime 读数飘。
"""

# move_x 参数 (common.move_x_trust)
MOVE_X_V_MAX_MMS: float = 80.0
"""move 速度 (mm/s)。2026-07-31 用户反馈 belt-slip 已修复, 30→40→80 试;
SDK 无硬限 (arm_base.py:482-487 临时收紧 x_velocity_limit, 业务层任意值)。"""

MOVE_X_TOL_MM: float = 5.0
"""到位容差 (mm)。realtime 抖动 <1mm, 放宽给 PID 余量 (trust 模式不验证)。"""

MOVE_X_WALL_MM: float = -300.0
"""x 物理墙位置 (mm, 负值方向)。同 common.DEFAULT_WALL_MM, 留 20mm 余量防撞。"""

# grasp 参数
GRASP_TIMEOUT_S: float = 10.0
"""grasp on/off 单次超时。"""

GRASP_HOLD_AFTER_OFF_S: float = 0.2
"""放气后停 0.2s 让球稳定掉进 bin (防止反弹甩出)。"""


def step_pick_up_yellow(
    client: ArmClient,
    runner: ArmRunner,
    *,
    return_x_mm: Optional[float] = DEFAULT_RETURN_X_MM,
) -> dict:
    """抓+放+回 x (v6, 9 步, 黄球版): 记 → 吸 → 抓 → 抬 → 移 bin(-65) → 放 → 释 → 定 y → 回抓取位。

    跟 step_pick_up_blue 的**唯一区别**: 步骤 5 移到 -65 (黄 bin) 而不是 0 (蓝 bin)。

    Args:
        client: ArmClient 实例 (调 _read_x_mm_realtime + move_x_trust 间接调 move_x)。
        runner: ArmRunner 实例 (调 move_y / grasp 业务封装)。
        return_x_mm: 放完球后 x 回到的目标位置 (mm, 绝对位置指令)。
                     - 默认 -260.0 (target1.py 抓取位, 业务流推荐)
                     - None = 不回位 (v5 行为兼容)
                     - -220.0 = target4.py 准备位姿 x
                     - 其他值 = 自定义

    Returns:
        {
            "ok": True,
            "x_initial_mm": <realtime x before grasp, 仅供参考>,
            "x_after_bin_mm": <realtime x at bin (-65), 仅供参考>,
            "x_after_return_mm": <realtime x after return, 仅供参考 or None>,
            "y_pick_mm": -58.0,
            "y_transit_mm": -190.0,
            "y_put_mm": -155.0,
            "y_final_mm": -133.0,
            "move_x_to_bin_result": <common.move_x_trust result dict>,
            "move_x_to_return_result": <common.move_x_trust result dict> | None,
            "return_x_mm": <实际回位目标 or None>,
            "grasp": "picked_placed",
        }
    """
    print("=== [pick_up_yellow] 抓+放序列 (9 步, v6, 黄球版) ===")
    print(f"  [config] return_x_mm = {return_x_mm} "
          f"({'默认 target1 抓取位 (实际 motor 走不到, 停在 ≈ -80mm)' if return_x_mm == -260.0 else '自定义' if return_x_mm is not None else '不回位 (v5 行为)'})")

    # 1. 记录 x 轴当前位置 (仅日志, 已知 realtime 读数飘, 不用于回位)
    x_initial = client._read_x_mm_realtime()
    if x_initial is None:
        raise RuntimeError("realtime x_mm 读不到 (arm_feed 未启 / realtime 不可用)")
    print(f"  [record] x_initial = {x_initial:+.1f} mm "
          f"(仅日志, realtime 飘, 不用于回位)")

    # 2. 吸气 —— 真空泵开 (后续 move_y 到 -58 时吸盘贴近球, 球被吸上)
    print("  [pump]  grasp(True)                            吸气")
    runner.grasp(True, timeout=GRASP_TIMEOUT_S)

    # 3. y 到 -58 (抓球位, 用户拍板)
    print(f"  [arm]   move_y({Y_PICK_MM:+.0f})                     下到抓球位")
    runner.move_y(Y_PICK_MM)

    # 4+5. y 抬到中转位 + x 移到黄 bin (composite_run 并行, 省 ~1s)
    print(f"  [arm]   composite_run(y={Y_TRANSIT_MM:+.0f}, x={X_BIN_MM:+.0f})  抬升+横移并行")
    client.composite_run(y_mm=Y_TRANSIT_MM, x_mm=X_BIN_MM, speed=80, timeout=30.0)

    # 6. y 到 -155 (放球位, 命中开仓 y gate 上界)
    print(f"  [arm]   move_y({Y_PUT_MM:+.0f})                     降到放球位 (命中开仓 y gate 上界)")
    runner.move_y(Y_PUT_MM)

    # 7. 放气 —— 真空泵关, 球掉进 bin
    print("  [pump]  grasp(False)                           放气")
    runner.grasp(False, timeout=GRASP_TIMEOUT_S)

    # 放气后停一会, 让球稳定掉进 bin (防止反弹甩出)
    print(f"  [pump]  sleep({GRASP_HOLD_AFTER_OFF_S}s)                       "
          f"球稳定进 bin")
    time.sleep(GRASP_HOLD_AFTER_OFF_S)

    # 8+9. y 到最终位 + x 回抓取位 (composite_run 并行, 省 ~1s)
    if return_x_mm is not None:
        print(f"  [arm]   composite_run(y={Y_FINAL_MM:+.0f}, x={return_x_mm:+.0f})  回最终位+回抓取位并行")
        client.composite_run(y_mm=Y_FINAL_MM, x_mm=return_x_mm, speed=80, timeout=30.0)
    else:
        print(f"  [arm]   move_y({Y_FINAL_MM:+.0f})  升到最终位 (return_x_mm=None, 跳过回 x)")
        runner.move_y(Y_FINAL_MM)

    print("=== [pick_up_yellow] 完成 ===\n")
    return {
        "ok": True,
        "x_initial_mm": x_initial,
        "y_pick_mm": Y_PICK_MM,
        "y_transit_mm": Y_TRANSIT_MM,
        "y_put_mm": Y_PUT_MM,
        "y_final_mm": Y_FINAL_MM,
        "return_x_mm": return_x_mm,
        "grasp": "picked_placed",
    }


def _build_parser() -> "argparse.ArgumentParser":
    p = argparse.ArgumentParser(
        description="task4 pick_up_yellow: 抓黄球+放黄bin+回抓取位 (v6, 仿 pick_up_blue)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--return-x", dest="return_x", type=float, default=None,
                   help=f"放完球后 x 回的目标位置 (mm, 绝对位置指令)。"
                        f" 默认 {DEFAULT_RETURN_X_MM} (target1.py 抓取位)。"
                        f" 业务流选项: -260 (target1 抓取位), -220 (target4 prep_x)。"
                        f" 跟 --no-return 互斥。")
    p.add_argument("--no-return", dest="no_return", action="store_true",
                   help="放完球后不回 x (v5 行为兼容, 给不需要回到抓取位的场景)。"
                        " 跟 --return-x 互斥。")
    return p


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    # CLI → return_x_mm 转换 (同 pick_up_blue)
    if args.no_return:
        return_x_mm: Optional[float] = None
    elif args.return_x is not None:
        return_x_mm = float(args.return_x)
    else:
        return_x_mm = DEFAULT_RETURN_X_MM   # -260.0 默认

    print(f"[pick_up_yellow] CLI: return_x_mm = {return_x_mm} "
          f"({'默认 target1 抓取位 (实际 motor 走不到, 停在 ≈ -80mm)' if return_x_mm == -260.0 else '自定义' if return_x_mm is not None else '不回位 (v5 行为)'})")

    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_pick_up_yellow(client, runner, return_x_mm=return_x_mm)


if __name__ == "__main__":
    main()
