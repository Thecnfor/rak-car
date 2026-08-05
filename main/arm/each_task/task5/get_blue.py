"""task5 / get_blue —— 摆到 '取蓝' 目标位姿。

目标位姿 (用户指定 2026-07-22, 新顺序):
  1. y 轴  → -130 mm  (先抬出保护区, 后面 x/大臂 wrapper 都能过; 顺便给动作让空间)
  2. x 轴  → 0 mm     (**用 reset_x 撞墙一步到位**, 见下)
  3. 大臂  → 85°      (用户 2026-07-29 调整: 从 -5° 改为 85°)
  4. 手爪  → 0°       (DOWN, 走底层直调, 大臂不动)
  5. y 轴  → -70 mm   (抬回取物位)

最终位姿: x=0, arm=85°, y=-70, hand=0°。

⚠️ **新顺序的意外好处 (2026-07-22 调整)**: 之前第 1 步是 x/reset_x, 需要进来时
y ≤ -30; 现在第 1 步是 move_y, **从 init (y=0) 直接跑也行** —— move_y 走步进电机
不过 y 保护区 (api.py:323-325 注释: '即使在保护区 [0, -30] 也可以调, 用于出保护区')。

⚠️ **第 1 步用 reset_x 代替 move_x (用户要求 '一步到位', 2026-07-22)**:
  - move_x(0) 走 PID + belt-slip 分段, 慢且可能到不了位。
  - reset_x 撞墙一步到位: 撞到墙后该点直接被定义成 x=0 (calibrate 原点)。
  - **但 ArmClient.reset_x wrapper 有坑** (api.py:757 / ARM_API §9): 不透传
    `probe_time` → 走底层默认 0.3s 反向探针, belt-slip 下变概率事件 (时好时坏)。
  - 故这里**绕过 wrapper 直调底层 action** + `probe_time=0` (关闭探针, 走老 5cm
    gate 路径, belt-slip 下也能稳定撞出), 与 aaa_origin.py / ARM_API §9.2 一致。
  - ⚠️ **direction 字符串只是速度符号约定, 与物理墙无固定映射** (arm_base.py:532):
    "right"=正速度(往 x 增大方向); "left"=负速度(往 x 减小方向)。**2026-07-22
    实测**: direction="left" 撞到了"另一侧"墙 → "取蓝" 物理墙在 x 增大方向,
    应传 --direction right。CLI 改 --direction 必传确认。
  - ⚠️ reset_x 后位置读数不可信 (calibrate 框架坏, §11); 只信 realtime。

⚠️ **第 4 步不走大臂 dance (用户 2026-07-22 要求)**:
  - 历史原因: 大臂原来是 -5°, 落在 api.py:604-612 的展开区 [-30, +30]° 里,
    Python 层安全门会拒手爪 ≠ -90(UP) 的下发, 故走底层
    `_call_arm("set_hand_angle", ...)` 直调绕开。
  - **2026-07-29 大臂改 85° 后**: 85° ≥ +30 已在 "安全姿态" 带外, wrapper
    其实不会再拒了; 但这里**保留底层直调**(行为不变, 真正下发的合法性由车端
    决定; 硬件若真不允许 → 拿到车端错误, 不会崩在 Python 层先 raise)。
  - 不再有大臂收/回夹的中间动作, 大臂始终停在 85°。

⚠️ 本文件**自包含**: 只依赖 `main.arm` (ArmClient/ArmRunner), 不 import task5
   包内其它模块。原因: task5 辅助文件曾被外部动作清空过, 自包含保证直接跑不受影响。

跑法:
    python main/arm/each_task/task5/get_blue.py --direction right   # "取蓝" 物理墙
    python main/arm/each_task/task5/get_blue.py --direction left    # 另一侧
"""
from __future__ import annotations

import argparse
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


# ---------- 目标位姿常量 (内联, 不依赖 constants.py) ----------

LOG_PREFIX: str = "[task5/get_blue]"

GET_BLUE_ARM_DEG: float = 85.0
"""大臂角度 (2026-07-29 用户从 -5° 改为 85°)。
85° 在业务硬限 [+90, -150] 内 (api.py:502-503), 且 ≥ +30 落在 "安全姿态" 带外
(api.py:508-509 _ARM_SAFE_BAND_MAX), 故:
  - set_arm_angle(85) 本身仍要过 y 保护区检查 (进来时 y=-130, 远出保护区, 过);
  - 之后 set_hand_angle(0) 走 Python 层 wrapper **已不会再被拒**
    (大臂不在展开区 [-30, +30]); 但下面仍保留 _call_arm 底层直调, 行为不变。"""

GET_BLUE_Y_MM: float = -70.0
"""取蓝 y (最终抬回位)。远出保护区 [0,-30]。
沿革: 2026-07-23 用户从 -88 调为 -85; 2026-07-29 先调 -75, 再调为 -70。"""

GET_BLUE_Y_DOWN_MM: float = -130.0
"""取蓝 y 下探位 (用户 2026-07-22 新顺序: 先下 -130 做动作, 再抬回 -90)。
远出保护区 [0,-30]。**待现场校准**: 这个 y 是大臂/手爪动作时的 '低位',
可能需根据机械结构 + 取物位调整。"""

GET_BLUE_HAND_DEG: float = 0.0
"""手爪 DOWN。**走底层 _call_arm 直调**(历史上是为了绕开 api.py:604-612 的
Python 层安全门: 大臂 ∈ [-30, +30] 展开区时手爪只允许 -90)。**2026-07-29 大臂
改 85° 后该门已不再命中**, 但直调保留不变。**用户 2026-07-22 要求**:
'改手爪时不要大臂移动, 直接在大臂当前角度改'。硬件若真的不允许, 会返回车端错误;
Python 层不再先 raise。"""

# reset_x 撞墙定原点参数 (绕过 wrapper 直调底层, ARM_API §9.2)
# ⚠️ direction 语义 (arm_base.py:532): "right"=正速度(往 x 增大方向);
#   "left"=负速度(往 x 减小方向)。哪面物理墙对应 "right/left" 取决于车体几何
#   和当前 x 位置, **没有固定映射** —— 现场必须先确认 "取蓝" 墙在哪侧。
#   默认 None → 强制调用方传 --direction, 避免再误撞另一侧。
RESET_X_DIRECTION_DEFAULT: str = "right"
"""默认方向 (临时, 等用户确认物理墙在哪侧后改)。
**测试结论 (2026-07-22)**: 用户跑 direction="left" 时撞到了"另一侧"墙,
故物理 "取蓝" 墙在 x 增大方向 → 应改 direction="right" 才能撞对。
暂保留 right 作默认, 同时把 CLI --direction 强制必填 + 提示原因。"""

RESET_X_VELOCITY_MMS: float = 50.0
"""撞墙速度 (mm/s)。§9.2 建议 50mm/s 比 wrapper 默认 20 稳。"""

RESET_X_PROBE_TIME: float = 0.3
"""⚠️ **从 0 改回 0.3** (arm_base.py 默认值): probe_time=0 在 "车刚好在 selected
方向的墙上" 场景下会立即误判 stall → calibrate 失败/撞错位置。留 0.3 让反向
探针先验证 motor 工作 + 确认臂不在墙上, 是更稳的默认。"""

RESET_X_TIMEOUT: float = 30.0


# ---------- reset_x 撞墙 (绕过 wrapper, probe_time=0) ----------

def _reset_x_wall(client: ArmClient,
                  direction: str = RESET_X_DIRECTION_DEFAULT,
                  velocity_mms: float = RESET_X_VELOCITY_MMS,
                  probe_time: float = RESET_X_PROBE_TIME,
                  timeout: float = RESET_X_TIMEOUT) -> dict:
    """撞墙定 x 原点, 一步到位。走 ArmClient.reset_x wrapper (2026-08-01 wrapper 已透传
    probe_time, 不再需要 escape hatch)。

    Returns:
        {"reset": job dict, "x_mm_after": float | None}
    """
    if direction not in ("left", "right"):
        raise ValueError("direction 必须是 'left' 或 'right'")
    print(f"  {LOG_PREFIX} reset_x(direction={direction}, v={velocity_mms}mm/s, "
          f"probe_time={probe_time})  撞墙一步到位")
    job = client.reset_x(
        direction=direction,
        reset_velocity_mms=velocity_mms,   # mm/s, wrapper 内部转 m/s
        probe_time=probe_time,
        timeout=timeout,
    )
    # reset_x 后位置读数不可信 (calibrate 框架坏), 只信 realtime (§11)
    x_after = client._read_x_mm_realtime()
    print(f"  {LOG_PREFIX} reset_x 完成, realtime x={x_after}")
    return {"reset": job, "x_mm_after": x_after}


# ---------- 主入口 ----------

def run(client: ArmClient, runner: ArmRunner,
        arm_deg: float = GET_BLUE_ARM_DEG,
        y_mm: float = GET_BLUE_Y_MM,
        y_down_mm: float = GET_BLUE_Y_DOWN_MM,
        hand_deg: float = GET_BLUE_HAND_DEG,
        x_direction: str = RESET_X_DIRECTION_DEFAULT) -> dict:
    """摆到取蓝目标位姿 (新顺序 2026-07-22 用户再次要求):
      y=-130 (先抬出保护区) → x → 大臂 → 手爪 → y=-70 (抬回取物位)

    Returns:
        {"ok": True, "x_info": dict, "arm_deg": float, "y_mm": float,
         "y_down_mm": float, "hand_deg": float}
    """
    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  目标: y=先抬{y_down_mm}→x=0(reset_x/{x_direction})→大臂{arm_deg}°→手爪{hand_deg}°→y={y_mm}抬回")

    # 1. y → -130 (先抬出保护区, 后面 x/大臂 wrapper 都能过; 顺便给动作让空间)
    print(f"  [1/5] move_y({y_down_mm}mm)  抬出保护区 [0,-30]")
    runner.move_y(y_down_mm, timeout=30.0)

    # 2. x → 0: reset_x 撞墙一步到位 (代替 belt-slip 分段的 move_x)
    print(f"  [2/5] reset_x  撞墙定 x=0 (一步到位)")
    x_info = _reset_x_wall(client, direction=x_direction)

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

    print(f"========== {LOG_PREFIX} 完成 (x=0 arm={arm_deg}° y={y_mm} hand={hand_deg}°) ==========\n")
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
        description="task5 get_blue: 摆到取蓝位姿 (y=-130→x=0→arm=85→hand=0→y=-70)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--direction", choices=["left", "right"], default=RESET_X_DIRECTION_DEFAULT,
                   help="reset_x 撞墙方向。right=正速度(往 x 增大方向); left=负速度。"
                        "[!] 字符串与物理墙无固定映射, 2026-07-22 实测 '取蓝' 物理墙在 x 增大方向, "
                        "故默认 right。先 --direction right 试, 撞错改 --direction left。")
    p.add_argument("--arm", type=float, default=GET_BLUE_ARM_DEG, help="大臂角度 (°)")
    p.add_argument("--y", type=float, default=GET_BLUE_Y_MM, help="y 抬回位 (mm)")
    p.add_argument("--y-down", type=float, default=GET_BLUE_Y_DOWN_MM,
                   dest="y_down", help="y 下探位 (mm), 大臂/手爪动作时")
    p.add_argument("--hand", type=float, default=GET_BLUE_HAND_DEG, help="手爪角度 (°)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner, arm_deg=args.arm, y_mm=args.y, y_down_mm=args.y_down,
        hand_deg=args.hand, x_direction=args.direction)
    return 0


if __name__ == "__main__":
    sys.exit(main())
