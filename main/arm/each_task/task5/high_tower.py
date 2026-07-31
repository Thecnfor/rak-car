"""task5 / high_tower —— 把机械臂摆到 '高塔/高位仓' 目标位姿。

目标位姿 (用户指定 2026-07-22, 2026-07-27 arm 0°→90°):
  - y 轴  → -180 mm
  - x 轴  → -100 mm
  - 大臂角度 → +90°  (MID / 复位位)
  - 手爪角度 → -90° (UP / init)

动作顺序 (用户 2026-07-22 调整: 把 x 放最后):
  1. move_y(-180)   先把 y 移出保护区 [0,-30] (move_y 任意值都放行)
  2. set_arm_angle(+90°)   MID / 复位位, 保护区允许 (init 例外位)
  3. set_hand_angle(-90°) UP, 保护区允许 (init 例外位)
  4. move_x(-160)   y/arm/hand 都已就位, 横移到目标; 跨 150mm 超 belt-slip 单次行程,
                    分段 + realtime 校验 (§7.2.1 / §11)

⚠️ 本文件**自包含**: 只依赖 `main.arm` (ArmClient/ArmRunner), 不 import task5
   包内其它模块 (constants / b3_...)。原因: task5 目录里的辅助文件曾被外部动作
   清空过 (见会话记录), 自包含可保证 `python high_tower.py` 直接跑不受影响。
⚠️ x 位置一律走 _read_x_mm_realtime() 校验 (x_get_position 坏, §11)。

跑法 (两种都行):
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
from main.arm.each_task.common import move_x_with_split  # noqa: E402


# ---------- 目标位姿常量 (内联, 不依赖 constants.py) ----------

LOG_PREFIX: str = "[task5/high_tower]"

HIGH_TOWER_Y_MM: float = -180.0
"""高位仓 y。出 y 保护区 [0,-80]。"""

HIGH_TOWER_X_MM: float = -160
"""高位仓 x。历史: 2026-07-22 设为 -150;
2026-07-30 现场 -150 → -153 (PID 闭环实测 overshoot 补偿, 3mm 留余量);
2026-07-30 又调 -153 → -157 (再缩 4mm, 实测离墙更近但仍不撞);
2026-07-30 三调 -157 → -160 (再缩 3mm, 球更贴近 high bin 槽口)。
跨 100mm 受 belt-slip 影响, 分段。"""

HIGH_TOWER_ARM_DEG: float = 90.0
"""大臂 MID / 复位位 (+90°, 2026-07-27 后)。保护区允许。"""

HIGH_TOWER_HAND_DEG: float = -90.0
"""手爪 UP/init。保护区允许 (-90 是例外值)。"""


# ---------- belt-slip 安全 move_x (抽离到 main.arm.each_task.common) ----------

# 2026-07-30: 之前 3 个 task5 文件 (high_tower / low_tower / target) 各拷贝一份
# _move_x_with_split, 改一处要同步 3 处容易漏。现抽到 main.arm.each_task.common,
# 3 个文件 import 即可。high_tower 用的是旧版 (无 wall_hit / overshoot), 抽离后
# 自动获得这些增强能力 (来自 low_tower 2026-07-30 现场 case 的加强)。
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
        y_mm: float = HIGH_TOWER_Y_MM,
        x_mm: float = HIGH_TOWER_X_MM,
        arm_deg: float = HIGH_TOWER_ARM_DEG,
        hand_deg: float = HIGH_TOWER_HAND_DEG) -> dict:
    """把臂摆到高位仓目标位姿。

    Returns:
        {
            "ok": bool,            # **2026-07-30 改**: 反映 x 实际成功 (不是永远 True)
            "y_mm": float,
            "x_info": dict,        # 见 common.move_x_with_split 注释
            "arm_deg": float,
            "hand_deg": float,
        }
    """
    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  目标: y={y_mm}mm arm={arm_deg}° hand={hand_deg}° x={x_mm}mm (最后动)")

    # 1. y 出保护区
    print(f"  [1/4] move_y({y_mm}mm)  出 y 保护区 [0,-30]")
    runner.move_y(y_mm, timeout=30.0)

    # 2. 大臂 MID
    print(f"  [2/4] set_arm_angle({arm_deg}°)  MID (init 例外位, 保护区允许)")
    client.set_arm_angle(arm_deg, speed=80, timeout=10.0)

    # 3. 手爪 UP
    print(f"  [3/4] set_hand_angle({hand_deg}°)  UP (init 例外位, 保护区允许)")
    client.set_hand_angle(hand_deg, speed=80, timeout=10.0)

    # 4. x (belt-slip 分段 + realtime 校验; 放在最后, 摆好姿态再横移)
    print(f"  [4/4] move_x({x_mm}mm)  belt-slip 分段 (最后动)")
    x_info = _move_x_with_split(client, runner, x_mm)

    # 2026-07-30 改: 结构化打印 x_info 新字段
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
        description="task5 high_tower: 臂摆到高位仓位姿 (y=-180→arm=+90→hand=-90→x=-160)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--y", type=float, default=HIGH_TOWER_Y_MM, help="y (mm)")
    p.add_argument("--x", type=float, default=HIGH_TOWER_X_MM, help="x (mm)")
    p.add_argument("--arm", type=float, default=HIGH_TOWER_ARM_DEG, help="大臂角度 (°)")
    p.add_argument("--hand", type=float, default=HIGH_TOWER_HAND_DEG, help="手爪角度 (°)")
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
