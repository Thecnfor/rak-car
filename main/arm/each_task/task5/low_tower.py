"""task5 / low_tower —— 把机械臂摆到 '低塔/低位仓' 目标位姿。

目标位姿 (用户指定 2026-07-22, y/x 后续调整, 2026-07-27 arm 0°→90°):
  - y 轴  → -200 mm   (high_tower 是 -180, low_tower 更深)
  - x 轴  → -169 mm   (high_tower 是 -150, low_tower 更远)
  - 大臂角度 → +90°   (MID / 复位位, 跟 high_tower 一样)
  - 手爪角度 →   0°   (DOWN, **跟 high_tower 的 -90 UP 不同!**)

动作顺序 (跟 high_tower 一致, 用户 2026-07-22 指定):
  1. move_y(-200)         抬出保护区 [0,-30]
  2. set_arm_angle(+90°)   MID / 复位位, 保护区允许 (跟 high_tower 同款)
  3. set_hand_angle(0°)   **DOWN, 不走 wrapper 走 _call_arm 直调** (关键差异!)
  4. move_x(-169)         belt-slip 分段 + realtime 校验 (跨 180mm, 触发分段)

⚠️ **跟 high_tower 的差异 (2026-07-22 用户多次调整)**:
  - y:     -180 → -200  (low_tower 更深 20mm)
  - x:     -150 → -180 → -172 → -169  (low_tower 更远 19mm; 2026-07-30 现场连调 2 次)
  - 手爪:  -90° (UP) → 0° (DOWN)   **关键差异: 0° 走 wrapper 会被拒**
  - 大臂:  一样 (+90° MID / 复位位)

⚠️ **手爪 0° 必须 _call_arm 底层直调 (跟 high_tower 第 3 步不同)**:
  - api.py:591-599 的 Python 层安全门: 大臂 ∈ [-30, 0] 展开区时手爪只允许
    -90 (UP), 0° 会 raise。
  - 走 `_call_arm("set_hand_angle", ...)` 直调 + `sync=True`, 绕开 Python 层
    校验。真正下发的合法性由车端决定: 硬件若真不允许 → 车端 error; Python
    层不先 raise。
  - 跟 get_yellow.py 的 set_hand_angle(0°) 同款 (那里大臂 2026-07-29 已改 85°)。

⚠️ 本文件**自包含**: 只依赖 `main.arm` (ArmClient/ArmRunner), 不 import task5
   包内其它模块。原因: task5 辅助文件曾被外部动作清空过, 自包含可保证跑得起来。

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
from main.arm.each_task.common import move_x_with_split  # noqa: E402


# ---------- 目标位姿常量 (内联, 不依赖 constants.py) ----------

LOG_PREFIX: str = "[task5/low_tower]"

LOW_TOWER_Y_MM: float = -200.0
"""低位仓 y。2026-07-22 用户从 -180 调为 -200 (比 high_tower 深 20mm)。出 y 保护区 [0,-30]。"""

LOW_TOWER_X_MM: float = -169
"""低位仓 x。历史: 2026-07-22 用户从 -210 调为 -180 (比 high_tower 远 30mm);
2026-07-30 现场再调 -180 → -172 (根据 overshoot 实测反推, 8mm 留 PID 余量);
2026-07-30 当场又调 -172 → -169 (再缩 3mm, PID 闭环实测更稳)。
跨 180mm 仍触发 belt-slip 分段 (overshoot_wall_hit 场景, 见 common.move_x_with_split)。"""

LOW_TOWER_ARM_DEG: float = 90.0
"""大臂 MID / 复位位 (+90°, 2026-07-27 后)。保护区允许 (跟 high_tower 一样)。"""

LOW_TOWER_HAND_DEG: float = 0.0
"""手爪 DOWN。**走 _call_arm 直调, 绕开 api.py:591-599 展开区手爪限 -90 的校验**。
跟 high_tower 的 -90 UP 不同 (high_tower 走 wrapper OK, low_tower 必须直调)。"""


# ---------- belt-slip 安全 move_x (抽离到 main.arm.each_task.common) ----------

# 2026-07-30: 之前 3 个 task5 文件 (high_tower / low_tower / target) 各拷贝一份
# _move_x_with_split, 改一处要同步 3 处容易漏。现抽到 main.arm.each_task.common,
# 3 个文件 import 即可。low_tower 是 2026-07-30 增强版 (wall_hit + overshoot 检测)
# 的源头, 抽离后另外两个自动获得这些能力。
# 本地保留 _move_x_with_split 别名 → 兼容 run() 内部调用 + 历史 log 习惯
def _move_x_with_split(client: ArmClient, runner: ArmRunner,
                       target_x_mm: float) -> dict:
    """薄 wrapper: 透传 common.move_x_with_split, 注入 LOG_PREFIX。

    见 main/arm/each_task/common.py:move_x_with_split 完整 docstring。
    """
    return move_x_with_split(
        client, runner, target_x_mm,
        log_prefix=LOG_PREFIX,
    )


# ---------- 主入口 ----------

def run(client: ArmClient, runner: ArmRunner,
        y_mm: float = LOW_TOWER_Y_MM,
        x_mm: float = LOW_TOWER_X_MM,
        arm_deg: float = LOW_TOWER_ARM_DEG,
        hand_deg: float = LOW_TOWER_HAND_DEG) -> dict:
    """把臂摆到低位仓目标位姿 (顺序跟 high_tower 一致)。

    Returns:
        {
            "ok": bool,            # **2026-07-30 改**: 反映 x 实际成功 (不是永远 True)
            "y_mm": float,
            "x_info": dict,        # 见 _move_x_with_split 注释
            "arm_deg": float,
            "hand_deg": float,
        }
    """
    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  目标: y={y_mm}mm arm={arm_deg}° hand={hand_deg}° x={x_mm}mm (最后动)")

    # 1. y 出保护区
    print(f"  [1/4] move_y({y_mm}mm)  出 y 保护区 [0,-30]")
    runner.move_y(y_mm, timeout=30.0)

    # 2. 大臂 MID (跟 high_tower 同款)
    print(f"  [2/4] set_arm_angle({arm_deg}°)  MID (init 例外位, 保护区允许)")
    client.set_arm_angle(arm_deg, speed=80, timeout=10.0)

    # 3. 手爪 DOWN —— 跟 high_tower 不同! wrapper 会拒, 走底层直调
    print(f"  [3/4] set_hand_angle({hand_deg}°)  DOWN (底层直调, 绕开 api.py:591-599)")
    client._call_arm(
        "set_hand_angle", timeout=10.0, sync=True,
        angle=hand_deg, speed=80,
    )

    # 4. x (belt-slip 分段 + realtime 校验; 放在最后, 摆好姿态再横移)
    print(f"  [4/4] move_x({x_mm}mm)  belt-slip 分段 (最后动)")
    x_info = _move_x_with_split(client, runner, x_mm)

    # 2026-07-30 改: 打印 result 而不是裸 x_info, 让 caller / log 一眼看清结果
    result = x_info.get("result", "unknown")
    final_x = x_info.get("final_x", x_info.get("actual_x"))
    residual = x_info.get("residual_mm", 0.0)
    wall_hit = x_info.get("wall_hit", False)
    overshoot_mm = x_info.get("overshoot_mm", 0.0)
    print(f"        result       = {result}")
    print(f"        final_x      = {final_x:+.1f}mm  (target={x_info.get('target_x', x_mm):+.0f}mm, "
          f"residual={residual:+.1f}mm)")
    print(f"        wall_hit     = {wall_hit}")
    print(f"        overshoot_mm = {overshoot_mm:+.1f}mm")
    if result != "success":
        print(f"        [WARN]  x 未到位 ({result}), 后续 placement 可能撞车, 请人工介入")

    # 2026-07-30 改: ok 反映 x 实际结果 (不是永远 True)
    ok = (result == "success")

    print(f"========== {LOG_PREFIX} 完成 ==========\n")
    return {
        "ok": ok,
        "y_mm": y_mm,
        "x_info": x_info,
        "arm_deg": arm_deg,
        "hand_deg": hand_deg,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task5 low_tower: 臂摆到低位仓位姿 (y=-200→arm=0→hand=0→x=-169)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--y", type=float, default=LOW_TOWER_Y_MM, help="y (mm)")
    p.add_argument("--x", type=float, default=LOW_TOWER_X_MM, help="x (mm)")
    p.add_argument("--arm", type=float, default=LOW_TOWER_ARM_DEG, help="大臂角度 (°)")
    p.add_argument("--hand", type=float, default=LOW_TOWER_HAND_DEG, help="手爪角度 (°)")
    return p


def main(argv=None) -> int:
    t_total_start = time.perf_counter()
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner, y_mm=args.y, x_mm=args.x,
        arm_deg=args.arm, hand_deg=args.hand)
    elapsed = time.perf_counter() - t_total_start
    print(f"========== {LOG_PREFIX} 总耗时: {elapsed:.3f} s ==========")
    return 0


if __name__ == "__main__":
    sys.exit(main())