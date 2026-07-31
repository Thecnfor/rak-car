#!/usr/bin/python3
"""task4 / test_blue —— 摆好放蓝色果实的姿态 (开仓独立)

目标位姿:
  y   = -155 mm   (放果实位, 同时命中开仓 y gate 上界)
  arm =  90  deg  (MID/init/复位位, 业务硬限 [+90, -150]°, 2026-07-27 重定义)
  hand=   0  deg  (DOWN, 业务硬限 [-90, 0]°)
  x   =   0  mm   (init 位)

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
  5. reset_x (撞墙定 x=0 原点)  —— ⚠️ ARM_API §9 escape hatch; 绕过 wrapper 直调底层,
                                  probe_time=0.3 + reset_velocity=50mm/s
                                  (跟 main/arm/each_task/task4/x_to_zero.py 同款)
                                  ⚠️ calibrate 框架有 bug, 撞墙后 x_get_position 不可信;
                                     位置验证走 realtime (/v1/realtime/arm/state)
  6. move_y(-155)              —— y 降到 -155 (为了放果实进 bin, 命中开仓 y gate 上界)

⚠️ reset_x 行为 (ARM_API.md §9 + x_to_zero.py):
    - direction="right" (默认): 假设蓝色 bin 在 x 增大方向; 撞错改 --direction left
    - 撞墙成功后该点被定义成 x=0 (calibrate 原点)
    - 失败语义: stall 检测 (3 轮无进展) 兜底 → 优雅退出, 不硬顶烧电机
    - 撞墙后 x_get_position 读数不可信 (calibrate 框架坏, §11);
      **唯一可信位置 = realtime** /v1/realtime/arm/state
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def test_blue(client: ArmClient, runner: ArmRunner) -> dict:
    """把机械臂摆到放蓝色果实的姿态: y=-155, arm=+90°, hand=0°, x=0."""
    print("=== [test_blue] 摆好放蓝色果实姿态 ===")

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

    # 5. reset_x 撞墙定 x=0 原点 (代替 move_x(0))
    #    ⚠️ ARM_API §9 escape hatch: 绕过 ArmClient.reset_x wrapper (不透传 probe_time),
    #       直调底层 action + probe_time=0.3 + reset_velocity=50mm/s
    #       (跟 main/arm/each_task/task4/x_to_zero.py:reset_x_to_zero() 同款)
    #    ⚠️ calibrate 框架有 bug, x_get_position 不可信; 位置验证走 realtime
    print("  [arm]  reset_x(direction='right', v=50mm/s, probe_time=0.3)  撞墙定 x=0")
    client._call_arm(
        "reset_x", timeout=30.0, sync=True,
        direction="right",
        reset_velocity=0.05,   # 50 mm/s, §9.2 建议比 wrapper 默认 20 稳
        probe_time=0.3,        # ⚠️ 不设 0: 0 在 "车刚好在 selected 方向墙上" 场景会误判 stall
    )
    x_after_reset = client._read_x_mm_realtime()
    print(f"  [arm]  reset_x 完成, realtime x = {x_after_reset} (撞墙点 = x=0 原点)")

    # 6. y 降到 -155 —— 把果实放到 bin 上方, 命中开仓 y gate 上界
    #    y=-155 同时命中开仓 y gate 上界 (Round 13/15: y ∈ [-205, -145] 闭区间)
    #    ⚠️ 2026-07-25 删除原步骤 7 set_storage_angle(75°) 开仓
    #       开仓改走 main.arm.each_task.task4.open_storage.step_open_storage()
    print("  [arm]  move_y(-155)            降到放果实位 (命中开仓 y gate 上界)")
    runner.move_y(-155.0)

    print("=== [test_blue] 完成 (y=-155, arm=+90°, hand=0°, x=0 撞墙定原点, 开仓未执行) ===\n")
    return {
        "ok": True,
        "y_mm": -155.0,
        "arm_angle_deg": 90,
        "hand_angle_deg": 0,
        "x_mm": 0,                # 撞墙定原点后, x=0 是物理墙位置
        "x_mm_realtime": x_after_reset,   # realtime 校验值 (calibrate 框架坏, 不可信)
        "storage": "CLOSED",   # 不再开仓, 仓保持关闭
    }


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    test_blue(client, runner)


if __name__ == "__main__":
    main()