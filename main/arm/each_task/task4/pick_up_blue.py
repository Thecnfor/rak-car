#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task4 / pick_up_blue —— 抓+放+回抓取位 序列 (2026-07-31 v6)

逻辑 (9 步, 2026-07-31 v6 加回抓取位步骤):
  1. 记录 x 轴当前位置 (仅日志, 已知 realtime 飘, 不用于回位)
  2. 吸气
  3. y 到 -58 (抓球位)
  4. y 到 -190 (中转位, 持球抬升)
  5. 移 x 到 0 (蓝色 bin 正上方)
  6. y 到 -155 (放球位, 命中开仓 y gate 上界)
  7. 放气
  8. y 到 -133 (最终位, 等下一阶段 target 识别)
  9. **移 x 到 RETURN_X_MM (默认 -260 = target1.py 抓取位)** —— 走 trust 模式

⚠️ **2026-07-31 v6 改动**: 加回抓取位步骤 9 (move_x_trust 到 RETURN_X_MM)。
   - **用户痛点**: v5 删了回位后, pick_up_blue 后 x 留在 0, 下一轮不是
     target1/target4(例如手动跑多轮 pick) 时, x 不回到抓取位置 (-260)。
   - **解决方案**: 加可选 RETURN_X_MM 参数 (默认 -260.0 = target1.py 抓取位)。
     走 trust 模式 move_x_trust(单次 move_x, 不 stall kick, 不 reset_x),
     信任 motor PID 内部位置闭环。
   - **关键确认** (api.py:418): move_x 是**绝对位置指令** (`target=_mm_to_m(x_mm)`),
     motor 内部编码器闭环, **不依赖 realtime 读数**。所以 trust 模式可行。
   - **历史背景**:
     - v2: 9 步回 x_initial → 马达短距 stall 失败
     - v3: 8 步删回位 → 业务流断了
     - v4: 9 步 reset_x + move_x_trust + retry → reset_x 撞墙后短距仍 stall
     - **v5: 8 步不回位 → 用户反馈 x 不回抓取位**
     - **v6 (当前): 9 步回 RETURN_X_MM (默认 -260)** → 走绝对位置 trust 模式
   - **跟 v4 关键区别**:
     - v4 走 reset_x 撞墙 (副作用: 马达物理状态可能变化)
     - **v6 不走 reset_x**, 直接 trust 模式 move_x(绝对位置)
     - v4 短距 move_x(+24.5) stall
     - **v6 长距 move_x(-260) 或 move_x(0)**, 信任 motor 内部闭环

⚠️ **业务流选项** (return_x_mm 参数):
   - 默认 -260.0 = target1.py 抓取位 (业务流推荐: target1→pick→target1→pick 循环)
   - -220.0 = target4.py 准备位姿 x (prep_x, 给 target4 用户用)
   - None = 不回位 (v5 行为兼容, 给手动多轮跑但下一阶段不依赖 x 的场景)
   - 其他值 = 自定义

⚠️ **x 状态转移图** (v6):
   ┌────────────────────────────────────────────────────────────┐
   │  init: x = 0 (撞墙定原点)                                  │
   │     ↓                                                      │
   │  target1.py step 4: x → -260  (trust mode)                 │
   │     ↓                                                      │
   │  检测球 + pick_up_blue                                     │
   │     ↓                                                      │
   │  pick_up_blue step 5: x → 0  (trust mode, 蓝 bin 上方)     │
   │     ↓                                                      │
   │  pick_up_blue step 6-8: 放球 + 抬 y                         │
   │     ↓                                                      │
   │  pick_up_blue step 9: x → -260  (trust mode, 回到抓取位)   │ ← **v6 新增**
   │     ↓                                                      │
   │  (下一轮) target1.py step 4: x 已在 -260 → 跳过            │
   │  (下一轮) target4.py: prep_x = -220 → x → -220             │
   └────────────────────────────────────────────────────────────┘

⚠️ 本脚本**只做** pick-and-return 序列, 不做位姿 setup, 调用前必须先保证:
  - arm = +90° (MID / 复位位, 业务硬限 [+90, -150]°)
  - hand = 0°  (DOWN, 业务硬限 [-90, 0]°)
  - y 出 [0, -30] 保护区 (任意 ≤ -30 mm 即可)
  - x 已撞墙定原点 (init 时已 reset_x 过, x=0 = 蓝色 bin 正上方)

典型用法 (在 test_blue.py 摆位姿后单独跑):
  1. python test_blue.py       # 摆位姿到 (y=-155, arm=+90°, hand=0°, x=0 撞墙)
  2. python pick_up_blue.py    # 抓→放→回 -260 (v6 默认)

⚠️ 走 common.move_x_trust 而非 client.move_x —— belt-slip + stall 检测会让短距 move_x
   反复触发 stall kick (已知现场问题, 见 memory/x-axis-belt-slip)。trust 模式绕过
   stall 检测, 信任 motor PID 内部闭环 (move_x 是绝对位置指令, api.py:418)。

⚠️ grasp 必须走 runner.grasp (ArmRunner 封装), 不可走 _call_arm 直调
   (ARM_API §10: kwargs 透传到 arm_base.grasp TypeError 静默失败)
"""
from __future__ import annotations

import os
import sys
import time
from typing import Optional

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.each_task import common  # noqa: E402


# ---------- 步骤参数 ----------
Y_PICK_MM: float = -58.0
"""抓球 y。**非** PICK_Y_MM=-160, 用户特定场景 (现场拍板)。"""

Y_TRANSIT_MM: float = -190.0
"""中转 y。同 SAFE_Y_TRANSIT_MM。持球抬升, 远离底部 + 出 y 保护区 + 出 gate。"""

Y_PUT_MM: float = -155.0
"""放球 y。命中开仓 y gate 上界 [-205, -145] (实测 -155 命中)。"""

Y_FINAL_MM: float = -133.0
"""最终 y (回位后)。target 识别用这个 y 拍球 (跟历史 target1 校准 y 一致)。"""

X_BIN_MM: float = 0.0
"""蓝色 bin x 位置 (= BLUE_BIN_X_MM, 撞墙定原点 = bin 正上方)。"""

# v6 新增: 放完球后 x 回到的目标位置 (mm)
# 2026-07-31 改回 -260: 用户决定保持 -260 (等硬件修复后再真到位)
DEFAULT_RETURN_X_MM: Optional[float] = -260.0
"""放完球后 x 自动回的目标位置 (mm)。

业务流选项:
  - -260.0 (默认) = target1.py 抓取位 (业务流推荐)
  - -220.0         = target4.py 准备位姿 x (给 target4 用户用)
  - None           = 不回位 (v5 行为兼容, 给手动多轮跑但下一阶段不依赖 x 的场景)
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


def step_pick_up_blue(
    client: ArmClient,
    runner: ArmRunner,
    *,
    return_x_mm: Optional[float] = DEFAULT_RETURN_X_MM,
) -> dict:
    """抓+放+回 x (v6, 9 步): 记 → 吸 → 抓 → 抬 → 移 bin → 放 → 释 → 定 y → 回抓取位。

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
            "x_after_bin_mm": <realtime x at bin, 仅供参考>,
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
    print("=== [pick_up_blue] 抓+放序列 (9 步, v6 加回抓取位) ===")
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

    # 4. y 到 -190 (中转位, 持球抬升)
    print(f"  [arm]   move_y({Y_TRANSIT_MM:+.0f})                   抬到中转位 (持球)")
    runner.move_y(Y_TRANSIT_MM)

    # 5. 移 x 到 0 (蓝色 bin 正上方) —— 走 trust 模式 move_x
    move_x_to_bin_result = common.move_x_trust(
        client, runner,
        target_x_mm=X_BIN_MM,
        log_prefix="[pick_up_blue -> bin]",
        v_max_mms=MOVE_X_V_MAX_MMS,
    )
    x_after_bin = move_x_to_bin_result["actual_x"]
    print(f"  [arm]   move_x -> bin 完成, x = {x_after_bin}, "
          f"result={move_x_to_bin_result['result']!r}")
    # ⚠️ 不验证 x 是否真到位 (trust 模式, 马达 stall 业务层看不见)
    #    已知现场马达短距 move_x 经常 stall (实际不动), 但 move_x 是绝对位置指令,
    #    motor 内部闭环准, 下一次 move_x 从真实位置出发。

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

    # 8. y 到 -133 (最终位, 等下一阶段 target 识别)
    print(f"  [arm]   move_y({Y_FINAL_MM:+.0f})                     升到最终位 (target 识别 y)")
    runner.move_y(Y_FINAL_MM)

    # 9. (v6 新增) 移 x 回到抓取位 (默认 -260 = target1.py 抓取位)
    #    ⚠️ 走 trust 模式 move_x_trust —— 绝对位置指令, motor 内部闭环,
    #       不依赖 realtime 读数飘, 不 stall kick。
    #    关键: 跟 v4 失败案例的关键区别是 **不走 reset_x**, 直接 move_x(绝对位置)。
    move_x_to_return_result: Optional[dict] = None
    x_after_return: Optional[float] = None
    if return_x_mm is not None:
        print(f"  [arm]   move_x({return_x_mm:+.0f}mm)  回抓取位 "
              f"(trust 模式, 绝对位置指令)")
        move_x_to_return_result = common.move_x_trust(
            client, runner,
            target_x_mm=return_x_mm,
            log_prefix="[pick_up_blue -> return]",
            v_max_mms=MOVE_X_V_MAX_MMS,
        )
        x_after_return = move_x_to_return_result["actual_x"]
        print(f"  [arm]   move_x -> return 完成, x = {x_after_return}, "
              f"result={move_x_to_return_result['result']!r}")
        if move_x_to_return_result["result"] == "trust_failed":
            print(f"  [WARN]  回抓取位失败 (motor 无响应), x 留在当前位置"
                  f" (已知 stall, 下一阶段 target1/target4 自己会再移 x)")
    else:
        print("  [skip]  return_x_mm is None, 跳过回 x 步骤 (v5 行为兼容)")

    print("=== [pick_up_blue] 完成 "
          f"(记 + 吸 + 抓 + 抬 + 移 bin + 放 + 释 + 定 y + 回抓取位) ===\n")
    return {
        "ok": True,
        "x_initial_mm": x_initial,        # 仅日志
        "x_after_bin_mm": x_after_bin,    # 仅日志
        "x_after_return_mm": x_after_return,    # 仅日志
        "y_pick_mm": Y_PICK_MM,
        "y_transit_mm": Y_TRANSIT_MM,
        "y_put_mm": Y_PUT_MM,
        "y_final_mm": Y_FINAL_MM,
        "move_x_to_bin_result": move_x_to_bin_result,
        "move_x_to_return_result": move_x_to_return_result,
        "return_x_mm": return_x_mm,
        "grasp": "picked_placed",
    }


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    # CLI → return_x_mm 转换
    if args.no_return:
        return_x_mm: Optional[float] = None
    elif args.return_x is not None:
        return_x_mm = float(args.return_x)
    else:
        return_x_mm = DEFAULT_RETURN_X_MM   # -260.0 默认

    print(f"[pick_up_blue] CLI: return_x_mm = {return_x_mm} "
          f"({'默认 target1 抓取位 (实际 motor 走不到, 停在 ≈ -80mm)' if return_x_mm == -260.0 else '自定义' if return_x_mm is not None else '不回位 (v5 行为)'})")

    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_pick_up_blue(client, runner, return_x_mm=return_x_mm)


def _build_parser() -> "argparse.ArgumentParser":
    import argparse
    p = argparse.ArgumentParser(
        description="task4 pick_up_blue: 抓球+放球+回抓取位 (v6)",
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


if __name__ == "__main__":
    main()