"""task5 / test4_from_blue_to_low —— 整合冒烟脚本 (蓝→低位仓)。

吸气逻辑 (跟 test1 同款 v5, 用户 2026-07-23 第五次明确):
  - 起点: get_blue 完成 → move_y(-70) 后立即 grasp(True) 吸气
  - 吸气保持 5s (sleep 5s, 用户指定), 默认可用 --hold 改
  - low_tower 期间持续吸气 (4 步, 期间吸盘一直吸住)
  - 终点: low_tower 完成 → move_x(-169) 后 grasp(False) 放气

流程 (5 阶段, **吸气包住 low_tower + 前面 5s 独立保持 + 末尾归零**):
  阶段 1: get_blue.py 取蓝位姿 (5 步: y=-130 → reset_x(撞墙, direction=right) → arm=85° → hand=0°(底层直调) → y=-70)
  阶段 2: grasp(True) 吸气 + sleep(5s) 独立保持 (吸盘在 y=-70 高位, 吸空气)
  阶段 3: low_tower 4 步 (move_y(-200) → set_arm_angle(+90°) → set_hand_angle(0°, 底层直调) → move_x(-169) 分段),
          期间吸盘持续吸气
  阶段 4: grasp(False) 放气 (low_tower 跑完, x=-169 后立刻放)
  阶段 5: reset_x 撞墙归零 (x=-169 → 撞取蓝墙 → x=0; 兜底防止下轮 x 飘读, 2026-07-24 加)

⚠️ **关键时序演进 (跟 test1 v5 同步, 2026-07-23 第五次修正)**:
  - v1 (错误): get_blue → grasp(60s 吸气+放气) → low_tower
  - v2 (错误): get_blue → grasp(True) → low_tower → sleep(60s) → grasp(False)
  - v3 (逻辑对):  get_blue → grasp(True) → low_tower → grasp(False)  # 立刻放气
  - v4 (标注):  显式吸气窗口 = [get_blue 完成, low_tower 完成]
  - **v5 (当前, 2026-07-23 用户第五次要求)**: 在 grasp(True) 之后、low_tower 之前加
    sleep(5s) 独立保持; 总吸气时长 = 5s + low_tower 跑完耗时 (~9-13s)
  - **v6 (2026-07-24 末尾加归零)**: low_tower → grasp(False) 之后, 跑一次 reset_x
    撞取蓝墙归零 (x=-169 → x=0)。防止下一轮 get_* 进来时 x 飘读 (calibrate 框架坏)。

⚠️ **跟 test1 差异**:
  - 取物: get_yellow (move_x -68) → **get_blue (reset_x 撞墙 direction=right)**
  - 目标位: high_tower (y=-180, x=-160, hand=-90° UP) → **low_tower (y=-200, x=-169, hand=0° DOWN)**
  - low_tower 第 3 步 hand=0° 必须 _call_arm 直调 (api.py:591-599 展开区手爪限 -90)

⚠️ **吸气期间吸盘位姿**: sleep(5s) 时吸盘在 (y=-70, x=0, hand=DOWN), 离地高度待实测 (y 已改 -70),
   没贴球; 若要真吸球, 在 grasp(True) 之前先 move_y(球位, 比如 -15)。

⚠️ **不动原文件**: get_blue.py / low_tower.py 一字未改, 只 import 它们的 run() 函数。

跑法:
    python main/arm/each_task/task5/test4_from_blue_to_low.py            # 默认 hold 5s
    python main/arm/each_task/task5/test4_from_blue_to_low.py --hold 10  # 改保持秒
    python main/arm/each_task/task5/test4_from_blue_to_low.py --hold 0   # 跳过保持 (退化为 v3)
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

# 复用原脚本的 run() (别名避免重名); 原文件没改
from main.arm.each_task.task5.get_blue import run as get_blue_run  # noqa: E402
from main.arm.each_task.task5.low_tower import run as low_tower_run  # noqa: E402


LOG_PREFIX: str = "[task5/test4_from_blue_to_low]"

GRASP_HOLD_S_DEFAULT: float = 5.0
"""吸气独立保持秒数 (用户 2026-07-23 要求 5s)。在 grasp(True) 之后, low_tower 之前执行。
设 0 可跳过保持 (退化为 v3 逻辑)。"""

# reset_x 撞墙定原点参数 (内联, 跟 get_blue.py 的 _reset_x_wall 同款, ARM_API §9.2)
RESET_X_DIRECTION: str = "right"
"""终点 reset_x 方向: low_tower 在 x=-169mm, 取蓝墙在 x 增大方向 (get_blue.py
测试结论 2026-07-22), 故走 right (正速度) 撞取蓝墙 → 撞到点定义为 x=0。"""

RESET_X_VELOCITY_MMS: float = 50.0
"""撞墙速度 (mm/s)。§9.2 建议 50mm/s 比 wrapper 默认 20 稳。"""

RESET_X_PROBE_TIME: float = 0.3
"""arm_base.py 默认值: probe_time=0 在 '车刚好在 selected 方向的墙上' 场景下
会立即误判 stall → calibrate 失败/撞错位置。留 0.3 让反向探针先验证 motor 工作。"""

RESET_X_TIMEOUT: float = 30.0


# ---------- reset_x 撞墙 (内联, 跟 get_blue.py 的 _reset_x_wall 同款) ----------

def _reset_x_wall(client: ArmClient) -> dict:
    """撞墙定 x 原点, 一步到位。绕过 ArmClient.reset_x wrapper (不透传 probe_time),
    直调底层 action (ARM_API §9.2 推荐)。

    Returns:
        {"reset": job dict, "x_mm_after": float | None}
    """
    print(f"  {LOG_PREFIX} reset_x(direction={RESET_X_DIRECTION}, v={RESET_X_VELOCITY_MMS}mm/s, "
          f"probe_time={RESET_X_PROBE_TIME})  撞墙一步到位")
    job = client._call_arm(
        "reset_x", timeout=RESET_X_TIMEOUT, sync=True,
        direction=RESET_X_DIRECTION,
        reset_velocity=RESET_X_VELOCITY_MMS / 1000.0,  # m/s
        probe_time=RESET_X_PROBE_TIME,
    )
    x_after = client._read_x_mm_realtime()
    print(f"  {LOG_PREFIX} reset_x 完成, realtime x={x_after}")
    return {"reset": job, "x_mm_after": x_after}


def run(client: ArmClient, runner: ArmRunner,
        hold_s: float = GRASP_HOLD_S_DEFAULT) -> dict:
    """整合 4 阶段: get_blue → grasp(True) → sleep(hold_s) → low_tower → grasp(False)

    ⚠️ 吸气逻辑 (跟 test1 同款 v5):
      - 起点: get_blue 完成 → move_y(-70) 后 grasp(True) 吸气
      - 独立保持: sleep(hold_s) 秒 — 期间吸盘在 (y=-70, x=0, hand=DOWN) 持续吸气
      - 持续期间: low_tower 4 步 — 期间吸盘持续吸气
      - 终点: low_tower 完成 → grasp(False) 放气
      - 总吸气时长 = hold_s + low_tower 跑完耗时 (~9-13s 默认)

    Returns:
        {"ok": True, "phases": list[str], "grasp_window": str, "hold_s": float}
    """
    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  吸气窗口: get_blue 完成 (y=-70) → sleep({hold_s:.1f}s) → low_tower 完成 (x=-169)")
    print(f"  流程: get_blue → grasp(True) → sleep({hold_s:.1f}s) → low_tower(吸) → grasp(False)")
    print(f"  总吸气时长: {hold_s:.1f}s + low_tower(~4-8s) = ~{hold_s + 4:.1f}-{hold_s + 8:.1f}s")

    # 阶段 1: 取蓝位姿 (5 步) —— 吸盘摆到 DOWN 朝下取物位, **吸气前**
    get_blue_run(client, runner)
    print(f"  >>> get_blue 完成, y=-70 已就位 <<< 吸气开始触发点")

    # 阶段 2: 吸气 + 独立保持 hold_s 秒 (吸盘位姿不动, 持续吸气)
    print(f"\n----- {LOG_PREFIX} [阶段 2/4] grasp(True) + sleep({hold_s:.1f}s) -----")
    print(f"  [1] grasp(True)   吸气开始")
    runner.grasp(True, timeout=10.0)
    if hold_s > 0.0:
        print(f"  [2] sleep({hold_s:.1f}s)  吸气独立保持 ({hold_s:.1f}s, 吸盘位姿不动)")
        time.sleep(hold_s)
    else:
        print(f"  [2] hold_s=0, 跳过独立保持, 立刻调 low_tower")

    # 阶段 3: low_tower 4 步, 期间吸盘持续吸气
    print(f"\n----- {LOG_PREFIX} [阶段 3/4] low_tower(期间持续吸) -----")
    print(f"  [3] low_tower(4 步)  期间吸盘持续吸气")
    low_tower_run(client, runner)
    print(f"  >>> low_tower 完成, x=-169 已就位 <<< 吸气结束触发点")

    # 阶段 4: 放气 (low_tower 跑完立刻放, 不再 sleep)
    print(f"\n----- {LOG_PREFIX} [阶段 4/5] grasp(False) -----")
    print(f"  [4] grasp(False)  放气")
    runner.grasp(False, timeout=10.0)

    # 阶段 5: x 轴归零 (reset_x 撞墙, 球已放出, 不用保持)
    print(f"\n----- {LOG_PREFIX} [阶段 5/5] reset_x 撞墙归零 -----")
    print(f"  [5] reset_x(direction={RESET_X_DIRECTION})  从 x=-169 撞取蓝墙 → x=0")
    x_reset = _reset_x_wall(client)
    print(f"  >>> reset_x 完成, realtime x={x_reset['x_mm_after']}mm <<<")

    print(f"\n========== {LOG_PREFIX} 完成 (5 阶段) ==========\n")
    return {
        "ok": True,
        "phases": ["get_blue", "grasp+sleep", "low_tower", "release", "reset_x"],
        "grasp_window": f"get_blue_complete(y=-70) → sleep({hold_s:.1f}s) → low_tower_complete(x=-169)",
        "hold_s": hold_s,
        "x_reset": x_reset,
    }


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task5 test4: get_blue -> grasp+hold+sleep -> low_tower -> release (蓝→低位仓, 吸气包住 low_tower, 5s 独立保持)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--hold", type=float, default=GRASP_HOLD_S_DEFAULT,
                   help="吸气独立保持秒数 (默认 5.0, 设 0 跳过, 退化为 v3 逻辑)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    t_total_start = time.perf_counter()
    run(client, runner, hold_s=args.hold)
    elapsed = time.perf_counter() - t_total_start
    print(f"========== {LOG_PREFIX} 总耗时: {elapsed:.3f} s ==========")
    return 0


if __name__ == "__main__":
    sys.exit(main())