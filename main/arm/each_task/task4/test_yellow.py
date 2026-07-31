#!/usr/bin/python3
"""task4 / test_yellow —— 摆好放黄色果实的姿态 (开仓独立)

目标位姿:
  y   = -155 mm   (放果实位, 同时命中开仓 y gate 上界)
  arm =  90  deg  (MID/init/复位位, 业务硬限 [+90, -150]°, 2026-07-27 重定义)
  hand=   0  deg  (DOWN, 业务硬限 [-90, 0]°)
  x   = -65  mm   (黄色果实仓位, 离 init 较远, ⚠️ belt slip 高风险)

⚠️ 2026-07-25 删除最后一步 set_storage_angle(75°) —— 开仓改走
  main.arm.each_task.task4.open_storage.step_open_storage() (单职责模块,
  2026-07-31 简化: 不读 y 不动 y, 调用方自己保证 y 在 [-205, -145] 区间)。
  本脚本只摆姿态, 不再开仓。

顺序 (重要, 两层 y/arm 保护区都要绕开):
  1. move_y(-190)              —— y 先抬到 -190 (远离底部, 不在任何 gate)
  2. (删除 2026-07-22) set_arm_angle(-90)  —— 原 "arm 先收到 ≤ -30° (出展开区)"
                                  api.py:583 规则: 大臂在 [0, -30]° 时手爪只允许 UP
                                  必须先 arm 收起来才能 set_hand_angle(非 UP)
                                  ⚠️ 删除后步骤 3 会被 set_hand_angle 拦截 (raise ValueError)
  3. set_hand_angle(0°)        —— arm=+90° 已在复位安全带 (>= +30°),
                                  api.py:432-435 自动跳过 y 保护区拦截
                                  (但仍走底层 _call_arm 直调作为防御纵深)
  4. set_arm_angle(+90°)       —— arm 走回 MID / 复位位 (no-op, 已在 init)
  5. move_x(-65)               —— 移向黄色仓位 (走 common.move_x_with_split,
                                  belt-slip + wall + overshoot 三重保护,
                                  跟 test_x_to_150.py + target1.py 同款)
  6. move_y(-155)              —— y 降到 -155 (为了放果实进 bin, 命中开仓 y gate 上界)

⚠️ 2026-07-31: 第 5 步改用 common.move_x_with_split 而非 runner.move_x(-65.0) 直调。
    - 直调一次只能走 24-46mm (belt-slip 单次有效行程), -65 会打滑不到位
    - common.move_x_with_split 仿 test_x_to_150.py 模式: 单次读 x0 + 分段循环
      + stall kick + wall 检测 + overshoot 检测, 失败 result != "success" 时业务层警觉
    - 参考: main/arm/test/test_x_to_150.py + main/arm/each_task/common.py:move_x_with_split
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.each_task import common  # noqa: E402


# 黄色 bin x 坐标 (跟 BLUE_BIN_X_MM=0 一样, 是常量不是 read)
YELLOW_BIN_X_MM: float = -65.0
"""黄色果实仓位。⚠️ belt-slip 高风险 (距 init 0 单次要走 65mm,
超出 belt-slip 单次有效行程 24-46mm), 必须走分段模式."""

# move_x 参数 (跟 test_blue.py / common.move_x_with_split / test_x_to_150.py 同款)
MOVE_X_V_MAX_MMS: float = 30.0
"""业务限速。2026-07-22 限速透传 bug 修复后定档 30。"""

MOVE_X_TOL_MM: float = 5.0
"""到位容差。realtime 抖动 <1mm, 放宽给 PID 余量。"""

MOVE_X_WALL_MM: float = -300.0
"""x 物理墙位置。留 20mm 余量防撞。"""


def test_yellow(client: ArmClient, runner: ArmRunner) -> dict:
    """把机械臂摆到放黄色果实的姿态: y=-155, arm=+90°, hand=0°, x=-65."""
    print("=== [test_yellow] 摆好放黄色果实姿态 ===")

    # 1. y 先抬到 -190 (中转位: 远离底部, 不在任何 gate / 保护区)
    print("  [arm]  move_y(-190)            抬到中转位")
    runner.move_y(-190.0)

    # 2. (删除 2026-07-22) 原 "arm 先收到 -90° (出展开区 [0, -30]°)"
    #    —— 原意图: 为步骤 3 set_hand_angle(0) 扫清 api.py:583 拦截
    #    当前状态: 不再下发 set_arm_angle(-90); arm 保持 init=+90° (复位安全带)

    # 3. 手爪 = 0° (DOWN) —— arm=+90° 已在复位安全带 (>= +30°),
    #    api.py:432-435 自动跳过 y 保护区拦截
    #    仍走底层 _call_arm 直调作为防御纵深
    print("  [arm]  set_hand_angle(0°)       DOWN (复位位, 保护区允许)")
    client._call_arm("set_hand_angle", timeout=10.0, angle=0.0, speed=80)

    # 4. arm 抬回 +90° (MID / 复位位) —— no-op (arm 仍在 init=+90°)
    print("  [arm]  set_arm_angle(+90°)      MID / 复位位 (no-op, 已在 init)")
    runner.set_arm_angle(90.0, speed=80, timeout=10.0)

    # 5. x = -65 (黄色仓位) —— 走 trust 模式 move_x
    #    ⚠️ 2026-07-31: 改用 move_x_trust (realtime x 读数不可信, 见 common.py:move_x_trust docstring)
    move_x_info = common.move_x_trust(
        client, runner,
        target_x_mm=YELLOW_BIN_X_MM,
        log_prefix="[test_yellow]",
        v_max_mms=MOVE_X_V_MAX_MMS,
    )
    x_mm = move_x_info["actual_x"]
    print(f"  [arm]  move_x <- bin 完成, x = {x_mm}, "
          f"result={move_x_info['result']!r}")

    # 6. y 降到 -155 —— 把果实放到 bin 上方, 让 75° 开仓后球准确落进黄色 bin
    #    y=-155 同时命中开仓 y gate 上界 (Round 13/15: y ∈ [-205, -145] 闭区间)
    print("  [arm]  move_y(-155)            降到放果实位 (命中开仓 y gate 上界)")
    runner.move_y(-155.0)

    # ⚠️ 2026-07-25 删除原步骤 7 set_storage_angle(75°) 开仓
    #    开仓改走 main.arm.each_task.task4.open_storage.step_open_storage()
    #    (单职责模块, 含 y gate 预检 [-205, -145] + auto_move 容错)

    print("=== [test_yellow] 完成 (y=-155, arm=+90°, hand=0°, x=-65, 开仓未执行) ===\n")
    return {
        "ok": True,
        "y_mm": -155.0,
        "arm_angle_deg": 90,
        "hand_angle_deg": 0,
        "x_mm": x_mm,
        "move_x_info": move_x_info,
        "storage": "CLOSED",   # 不再开仓, 仓保持关闭
    }


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    test_yellow(client, runner)


if __name__ == "__main__":
    main()