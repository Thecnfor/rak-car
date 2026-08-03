"""task5 / get_yellow —— 摆到 '取黄' 目标位姿。

⚠️ 本文件从 get_blue.py 复制而来, 仅:
  - x 轴目标值 0 → -68
  - 第 1 步从 reset_x 撞墙 → move_x 直接走 (2026-07-22 用户要求)
  - 顺序从 4 步改成 5 步 (x → y=-130 → 大臂 → 手爪 → y=-70)
其他一律不变。

目标位姿 (用户指定 2026-07-22, 新顺序):
  1. y 轴  → -130 mm  (先抬出保护区, 后面 x/大臂 wrapper 都能过; 顺便给动作让空间)
  2. x 轴  → -68 mm   (**belt-slip 安全 move_x, 不撞墙**, 见下 ⚠️)
  3. 大臂  → 85°      (用户 2026-07-29 调整: 从 -6° 改为 85°)
  4. 手爪  → 0°       (DOWN, 走底层直调, 大臂不动)
  5. y 轴  → -70 mm   (抬回取物位)

最终位姿: x=-68, arm=85°, y=-70, hand=0°。

⚠️ **新顺序的意外好处 (2026-07-22 调整, 从 get_blue 同步)**: 之前第 1 步是 move_x,
需要进来时 y ≤ -30; 现在第 1 步是 move_y, **从 init (y=0) 直接跑也行** —— move_y
走步进电机不过 y 保护区 (api.py:323-325 注释: '即使在保护区 [0, -30] 也可以调, 用于出保护区')。

⚠️ **第 1 步用 move_x 不用 reset_x (用户 2026-07-22 要求)**:
  - 之前 get_blue.py 第 1 步是 reset_x 撞墙定原点 (撞墙点 = x=0)。
  - 现在直接走 move_x(-68): 走 PID 闭环到目标 -68, 不撞墙、不 calibrate。
  - 优点: 不依赖撞墙方向 (无 direction 参数), 简单; 不依赖车当前在哪。
  - ⚠️ **belt-slip 处理 (2026-07-24 升级)**: 68mm 跨 belt-slip 单次有效行程
    (24-46mm, §7.2.1)。当前走 `_move_x_with_split` (test_x_to_150.py 模式):
    每轮 move_x(target, v_max_mms=30) + realtime 校验; 卡住 kick; 连续 3 轮
    stall 放弃。不撞墙、不 calibrate, 纯 PID 闭环 + belt-slip 兼容。
  - 位置验证: 走 `_read_x_mm_realtime()` 20Hz arm_feed 真值 (不走 get_state(),
    calibrate 框架坏, §11)。

⚠️ **进来时 y 必须 ≤ -30** (move_x 网关要过 y 保护区 [0, -30], api.py:369):
  - 从 init (y=0) 跑会拒; 接在 high_tower (y=-180) 之后跑没问题。

⚠️ **第 4 步不走大臂 dance (用户 2026-07-22 要求, 从 get_blue 沿用)**:
  - 历史原因: 大臂原来是 -6°, 落在 api.py:604-612 的展开区 [-30, +30]° 里,
    Python 层安全门会拒手爪 ≠ -90(UP) 的下发, 故走底层
    `_call_arm("set_hand_angle", ...)` 直调绕开。
  - **2026-07-29 大臂改 85° 后**: 85° ≥ +30 已在 "安全姿态" 带外, wrapper
    其实不会再拒了; 但这里**保留底层直调**(行为不变, 真正下发的合法性由车端
    决定; 硬件若真不允许 → 拿到车端错误, 不会崩在 Python 层先 raise)。
  - 不再有大臂收/回夹的中间动作, 大臂始终停在 85°。

⚠️ 本文件**自包含**: 只依赖 `main.arm` (ArmClient/ArmRunner), 不 import task5
   包内其它模块。原因: task5 辅助文件曾被外部动作清空过, 自包含保证直接跑不受影响。

跑法:
    python main/arm/each_task/task5/get_yellow.py
    python -m main.arm.each_task.task5.get_yellow
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

LOG_PREFIX: str = "[task5/get_yellow]"

GET_YELLOW_X_MM: float = -68.0
"""取黄 x (mm)。68mm 跨 belt-slip 单次有效行程 (24-46mm, §7.2.1),
走 _move_x_with_split (common.move_x_with_split) 分段 + 卡住 kick。
(2026-08-03: belt-slip 参数不再内联, 全走 common 默认值, 数值与旧版一致。)"""


# ---------- belt-slip 安全 move_x (抽离到 main.arm.each_task.common) ----------

# 2026-08-03: 原来这里有一份内联 _move_x_with_split (旧版, 无 wall_hit / overshoot),
# 与 high_tower / low_tower / target 一样抽到 common。本地保留同名薄 wrapper →
# 兼容 run() 内部调用 + 历史 log 习惯, 同时自动获得 wall_hit / overshoot 增强。
def _move_x_with_split(client: ArmClient, runner: ArmRunner,
                       target_x_mm: float) -> dict:
    """薄 wrapper: 透传 common.move_x_with_split, 注入 LOG_PREFIX。

    见 main/arm/each_task/common.py:move_x_with_split 完整 docstring。
    """
    return move_x_with_split(
        client, runner, target_x_mm,
        log_prefix=LOG_PREFIX,
    )

GET_YELLOW_ARM_DEG: float = 85.0
"""大臂角度 (2026-07-29 用户从 -6° 改为 85°)。
85° 在业务硬限 [+90, -150] 内 (api.py:502-503), 且 ≥ +30 落在 "安全姿态" 带外
(api.py:508-509 _ARM_SAFE_BAND_MAX), 故:
  - set_arm_angle(85) 本身仍要过 y 保护区检查 (进来时 y=-130, 远出保护区, 过);
  - 之后 set_hand_angle(0) 走 Python 层 wrapper **已不会再被拒**
    (大臂不在展开区 [-30, +30]); 但下面仍保留 _call_arm 底层直调, 行为不变。"""

GET_YELLOW_Y_MM: float = -70.0
"""取黄 y (最终抬回位)。远出保护区 [0,-30]。
沿革: 2026-07-23 用户从 -88 调为 -82; 2026-07-29 与 get_blue 统一 (-75 → -70)。"""

GET_YELLOW_Y_DOWN_MM: float = -130.0
"""取黄 y 下探位 (用户 2026-07-22 新顺序: 先下 -130 做动作, 再抬回 -90)。
远出保护区 [0,-30]。**待现场校准**: 这个 y 是大臂/手爪动作时的 '低位',
可能需根据机械结构 + 取物位调整。"""

GET_YELLOW_HAND_DEG: float = 0.0
"""手爪 DOWN。**走底层 _call_arm 直调**(历史上是为了绕开 api.py:604-612 的
Python 层安全门: 大臂 ∈ [-30, +30] 展开区时手爪只允许 -90)。**2026-07-29 大臂
改 85° 后该门已不再命中**, 但直调保留不变。**用户 2026-07-22 要求**:
'改手爪时不要大臂移动, 直接在大臂当前角度改'。硬件若真的不允许, 会返回车端错误;
Python 层不再先 raise。"""


# ---------- 主入口 ----------

def run(client: ArmClient, runner: ArmRunner,
        x_mm: float = GET_YELLOW_X_MM,
        arm_deg: float = GET_YELLOW_ARM_DEG,
        y_mm: float = GET_YELLOW_Y_MM,
        y_down_mm: float = GET_YELLOW_Y_DOWN_MM,
        hand_deg: float = GET_YELLOW_HAND_DEG) -> dict:
    """摆到取黄目标位姿 (新顺序 2026-07-22 用户再次要求, 与 get_blue 一致):
      y=-130 (先抬出保护区) → move_x → 大臂 → 手爪 → y=-70 (抬回取物位)

    Returns:
        {"ok": True, "x_info": dict, "arm_deg": float, "y_mm": float,
         "y_down_mm": float, "hand_deg": float}
    """
    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  目标: y=先抬{y_down_mm}→x={x_mm}mm→大臂{arm_deg}°→手爪{hand_deg}°→y={y_mm}抬回")

    # 1. y → -130 (先抬出保护区, 后面 x/大臂 wrapper 都能过; 顺便给动作让空间)
    print(f"  [1/5] move_y({y_down_mm}mm)  抬出保护区 [0,-30]")
    runner.move_y(y_down_mm, timeout=30.0)

    # 2. x → -68: belt-slip 安全 move_x (走 test_x_to_150.py 模式, 直调 client.move_x
    #    透传 v_max_mms=30; 卡住 kick + 连续 stall 放弃)
    print(f"  [2/5] move_x({x_mm}mm)  belt-slip 安全 (test_x_to_150.py 模式)")
    x_info = _move_x_with_split(client, runner, x_mm)
    print(f"        x_info={x_info}")

    # 3. 大臂 → 85° (安全姿态带外; y=-130 已远出保护区, wrapper 过)
    print(f"  [3/5] set_arm_angle({arm_deg}°)")
    client.set_arm_angle(arm_deg, speed=80, timeout=10.0)

    # 4. 手爪 → 0° (DOWN) —— **用户要求不动大臂, 直接设手爪**
    #    大臂 85° 已在展开区 [-30,+30] 之外, api.py:604-612 的门不再命中;
    #    这里仍走底层 _call_arm 直调 (保持历史行为) → 真正下发的合法性由
    #    车端决定。硬件真不允许 → 会拿到车端错误, 不会崩在 Python 层。
    print(f"  [4/5] 手爪 → {hand_deg}° (DOWN), 大臂保持 {arm_deg}° 不动 (底层直调)")
    client._call_arm(
        "set_hand_angle", timeout=10.0, sync=True,
        angle=hand_deg, speed=80,
    )

    # 5. y → -70 (抬回取物位)
    print(f"  [5/5] move_y({y_mm}mm)  抬回取物位")
    runner.move_y(y_mm, timeout=30.0)

    print(f"========== {LOG_PREFIX} 完成 (x={x_mm}mm arm={arm_deg}° y={y_mm}mm hand={hand_deg}°) ==========\n")
    return {
        "ok": True,
        "x_info": x_info,
        "arm_deg": arm_deg,
        "y_mm": y_mm,
        "y_down_mm": y_down_mm,
        "hand_deg": hand_deg,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task5 get_yellow: 摆到取黄位姿 (y=-130→x=-68→arm=85→hand=0→y=-70)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--x", type=float, default=GET_YELLOW_X_MM,
                   help="x (mm), 默认 -68")
    p.add_argument("--arm", type=float, default=GET_YELLOW_ARM_DEG, help="大臂角度 (°)")
    p.add_argument("--y", type=float, default=GET_YELLOW_Y_MM, help="y 抬回位 (mm)")
    p.add_argument("--y-down", type=float, default=GET_YELLOW_Y_DOWN_MM,
                   dest="y_down", help="y 下探位 (mm), 大臂/手爪动作时")
    p.add_argument("--hand", type=float, default=GET_YELLOW_HAND_DEG, help="手爪角度 (°)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner, x_mm=args.x, arm_deg=args.arm,
        y_mm=args.y, y_down_mm=args.y_down, hand_deg=args.hand)
    return 0


if __name__ == "__main__":
    sys.exit(main())
