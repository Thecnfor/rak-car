#!/usr/bin/python3
"""task4 / x_to_zero —— 让 x 轴回到 0 点 (init 位 / 蓝色 bin)

用法:
    python main/arm/each_task/task4/x_to_zero.py                 # 默认 direction=right
    python main/arm/each_task/task4/x_to_zero.py --direction left  # 撞错墙时换方向

⚠️ 用 reset_x 撞墙定原点 (跟 task5/get_blue.py 同款), 不用 move_x(0) 闭环:
  - move_x(0) 受 belt-slip 影响 (§7.2.1, 单次有效行程 24-46mm):
    从 x=-150 (黄色 bin) 走到 0 要 150mm, belt-slip 严重会打滑不到位
  - reset_x 撞墙一步到位, 撞墙点直接被定义成 x=0 (calibrate 原点)
  - ArmClient.reset_x wrapper 不透传 probe_time → 走 _call_arm 底层直调 + probe_time=0.3

⚠️ direction 字符串语义 (arm_base.py:532): "right"=+速度(往 x 增大方向);
  "left"=-速度(往 x 减小方向)。哪面物理墙对应 "right/left" 取决于车体几何
  和当前 x 位置, **没有固定映射** —— 现场必须先确认 task4 的 "x=0 墙" (init 位
  / 蓝色 bin) 在哪侧。默认 right, 撞错改 --direction left。

⚠️ reset_x 后位置读数不可信 (calibrate 框架坏, §11); 只信
  /v1/realtime/arm/state (走 20Hz arm_feed)。
"""
import argparse
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


LOG_PREFIX: str = "[task4/x_to_zero]"

# reset_x 撞墙定原点参数 (跟 task5/get_blue.py 同款, ARM_API §9.2)
RESET_X_DIRECTION_DEFAULT: str = "right"
"""默认方向。direction 字符串只是速度符号约定, 与物理墙无固定映射。
task4 的 "x=0 墙" (init 位 / 蓝色 bin) 默认走 right, 撞错改 --direction left。"""

RESET_X_VELOCITY_MMS: float = 50.0
"""撞墙速度 (mm/s)。§9.2 建议 50mm/s 比 wrapper 默认 20 稳。"""

RESET_X_PROBE_TIME: float = 0.3
"""⚠️ **不设 0**: probe_time=0 在 "车刚好在 selected 方向的墙上" 场景下
会立即误判 stall → calibrate 失败/撞错位置。留 0.3 让反向探针先验证 motor
工作 + 确认臂不在墙上, 是更稳的默认。"""

RESET_X_TIMEOUT: float = 30.0


def reset_x_to_zero(client: ArmClient,
                     direction: str = RESET_X_DIRECTION_DEFAULT,
                     velocity_mms: float = RESET_X_VELOCITY_MMS,
                     probe_time: float = RESET_X_PROBE_TIME,
                     timeout: float = RESET_X_TIMEOUT) -> dict:
    """撞墙定 x=0 原点。绕过 ArmClient.reset_x wrapper (不透传 probe_time),
    直调底层 action + probe_time=0.3 (避免 stall 误判)。

    Returns:
        {"reset": job dict, "x_mm_after": float | None}
    """
    if direction not in ("left", "right"):
        raise ValueError("direction 必须是 'left' 或 'right'")
    print(f"  {LOG_PREFIX} reset_x(direction={direction}, v={velocity_mms}mm/s, "
          f"probe_time={probe_time})  撞墙定 x=0")
    job = client._call_arm(
        "reset_x", timeout=timeout, sync=True,
        direction=direction,
        reset_velocity=velocity_mms / 1000.0,  # m/s
        probe_time=probe_time,
    )
    # reset_x 后位置读数不可信 (calibrate 框架坏), 只信 realtime (§11)
    x_after = client._read_x_mm_realtime()
    print(f"  {LOG_PREFIX} reset_x 完成, realtime x={x_after}")
    return {"reset": job, "x_mm_after": x_after}


def x_to_zero(client: ArmClient, runner: ArmRunner,
              direction: str = RESET_X_DIRECTION_DEFAULT) -> dict:
    """让 x 轴回到 0 点 (撞墙定原点)。"""
    print(f"\n=== {LOG_PREFIX} 跑 ===")
    print(f"  目标: x=0 (init 位 / 蓝色 bin)")
    info = reset_x_to_zero(client, direction=direction)
    print(f"=== {LOG_PREFIX} 完成 (x=0, realtime={info.get('x_mm_after')}) ===\n")
    return {
        "ok": True,
        "x_target_mm": 0,
        "direction": direction,
        "x_mm_after_realtime": info.get("x_mm_after"),
    }


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description="task4 x_to_zero: 让 x 轴回到 0 点 (撞墙定原点, 跟 task5/get_blue.py 同款)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--direction", choices=["left", "right"], default=RESET_X_DIRECTION_DEFAULT,
                   help="reset_x 撞墙方向。right=正速度(往 x 增大方向); left=负速度。"
                        "[!] 字符串与物理墙无固定映射, 默认 right (task4 init 位 / 蓝色 bin "
                        "在 x 增大方向)。撞错改 --direction left。")
    args = p.parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    x_to_zero(client, runner, direction=args.direction)
    return 0


if __name__ == "__main__":
    sys.exit(main())