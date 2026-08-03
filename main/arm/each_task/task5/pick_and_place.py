"""task5 / pick_and_place —— 统一 pick→place 执行器 (2026-08-03 重构)。

替代 test1-4 四个脚本的公共主体。原来四个 test* 文件结构完全一致:

    get_* (开环摆位) → grasp(True) + sleep(hold_s) → tower → grasp(False) → reset_x 撞墙

现统一抽到这里, test1-4 退化成薄 wrapper (只接线: 哪个 get_* / 哪个 tower)。

两种取球模式:
  **vision=False (默认, 旧行为, 开环)**:
      get_* 位姿 → grasp(True) + sleep(hold_s) 盲吸 → tower → grasp(False) → reset_x

  **vision=True (2026-08-03 新增, 视觉闭环)**:
      get_* 位姿 → runner.track_velocity_pick("ball_blue"/"ball_yellow") 视觉伺服
      把吸盘对到球上 → y 降到 grasp_y_mm → grasp(True) → tower → grasp(False) → reset_x
      失败且 vision_fallback=True → 重跑 get_* 恢复位姿 → 退回开环盲吸 (行为不劣于旧版)。

视觉模式接线说明 (main/arm/loops/runner.py:track_velocity_pick, task1 已实车验证):
  - skip_pose_align=True: get_* 已摆好位姿, 跳过入口 composite_run (省 2-3s)。
  - lift_back=False: 吸住后不抬 y, 直接交给 tower (tower 第一步就是 move_y 出保护区)。
  - 伺服期间 track_velocity_pick 内部自动 stop_arm_feed 让出串口, 结束恢复。
  - 走 /v1/realtime/arm-velocity 速度模式 (免 arm_queue, 高频平滑)。

⚠️ 视觉模式**现场标定须知** (首跑必看):
  - sign_arm/sign_x 默认沿用 task1 在 y=-180/arm=-90 姿态的标定值; task5 取球
    位姿 (arm=85°/y=-70) 几何不同, **首次跑 --vision 前先用 --balls 0 观察方向**,
    或准备随时急停。符号反了 → 臂反向追 → 超时 → 自动回退开环 (不撞车, 只浪费时间)。
  - grasp_y_mm 默认 -70 (= 取球位姿高度, 不下探, 最保守, 与旧开环吸盘高度一致)。
    现场吸不实 → 降低 --grasp-y (如 -20 / 0), 注意仓底深度。
  - task_feed 必须运行 (runtime 默认开), 否则伺服全程 miss → 超时回退。

reset_x 撞墙归零沿用旧参数 (ARM_API §9.2): direction="right", 50mm/s, probe_time=0.3。

跑法 (一般不直接跑本文件, 跑 test1-4 wrapper):
    python -m main.arm.each_task.task5.pick_and_place   # 会打印用法说明
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Callable, Optional

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


LOG_PREFIX: str = "[task5/pick_and_place]"

GRASP_HOLD_S_DEFAULT: float = 5.0
"""开环模式吸气独立保持秒数 (沿用 test1 v5 用户指定值)。视觉模式不用它。"""

DEFAULT_GRASP_Y_MM: float = -70.0
"""视觉模式吸气 y (mm)。默认 = 取球位姿高度 (不下探, 最保守)。
**待现场校准**: 吸不实就往下调 (如 -20 / 0), 别超过仓底。"""

DEFAULT_VISION_TIMEOUT_S: float = 20.0
"""视觉伺服总超时 (秒)。超时未收敛 → 按 vision_fallback 决定回退或报错。"""

DEFAULT_VISION_HOLD_S: float = 0.5
"""视觉模式吸住后的短暂保持 (秒), 让真空建立, 随后立即进 tower。"""

# reset_x 撞墙定原点参数 (沿用 test1-4 / get_blue.py 的 _reset_x_wall, ARM_API §9.2)
RESET_X_DIRECTION: str = "right"
"""high/low tower 都在 x 负向, 取蓝墙在 x 增大方向 (get_blue.py 2026-07-22 实测),
故 right (正速度) 撞取蓝墙 → 撞到点定义为 x=0。"""

RESET_X_VELOCITY_MMS: float = 50.0
"""撞墙速度 (mm/s)。§9.2 建议 50mm/s 比 wrapper 默认 20 稳。"""

RESET_X_PROBE_TIME: float = 0.3
"""arm_base.py 默认值: probe_time=0 在 '车刚好在 selected 方向的墙上' 场景会误判 stall。"""

RESET_X_TIMEOUT: float = 30.0


# ---------- reset_x 撞墙 (单份实现, test1-4 不再各自内联) ----------

def _reset_x_wall(client: ArmClient, direction: str = RESET_X_DIRECTION) -> dict:
    """撞墙定 x 原点, 一步到位。绕过 ArmClient.reset_x wrapper (不透传 probe_time),
    直调底层 action (ARM_API §9.2 推荐)。

    Returns:
        {"reset": job dict, "x_mm_after": float | None}
    """
    print(f"  {LOG_PREFIX} reset_x(direction={direction}, v={RESET_X_VELOCITY_MMS}mm/s, "
          f"probe_time={RESET_X_PROBE_TIME})  撞墙一步到位")
    job = client._call_arm(
        "reset_x", timeout=RESET_X_TIMEOUT, sync=True,
        direction=direction,
        reset_velocity=RESET_X_VELOCITY_MMS / 1000.0,  # m/s
        probe_time=RESET_X_PROBE_TIME,
    )
    # reset_x 后位置读数不可信 (calibrate 框架坏, §11), 只信 realtime
    x_after = client._read_x_mm_realtime()
    print(f"  {LOG_PREFIX} reset_x 完成, realtime x={x_after}")
    return {"reset": job, "x_mm_after": x_after}


# ---------- 主执行器 ----------

def run_pick_and_place(
    client: ArmClient,
    runner: ArmRunner,
    *,
    log_prefix: str,
    pick_fn: Callable,          # e.g. get_yellow.run / get_blue.run
    pick_name: str,             # e.g. "get_yellow" (打印用)
    tower_fn: Callable,         # e.g. high_tower.run / low_tower.run
    tower_name: str,            # e.g. "high_tower" (打印用)
    vision: bool = False,
    vision_label: Optional[str] = None,   # e.g. "ball_yellow" / "ball_blue"
    grasp_y_mm: float = DEFAULT_GRASP_Y_MM,
    hold_s: float = GRASP_HOLD_S_DEFAULT,
    vision_fallback: bool = True,
    sign_arm: float = 1.0,
    sign_x: float = -1.0,
    vision_timeout: float = DEFAULT_VISION_TIMEOUT_S,
    reset_direction: str = RESET_X_DIRECTION,
) -> dict:
    """统一 pick→place 流程 (5 阶段)。

    阶段 1: pick_fn 摆取球位姿 (get_* 5 步)
    阶段 2: 取球
        - vision=False: grasp(True) + sleep(hold_s) 开环盲吸 (旧行为)
        - vision=True : track_velocity_pick 视觉伺服对准球 → y 到 grasp_y_mm → 吸气
                        失败 + vision_fallback → 重跑 pick_fn → 退回开环盲吸
    阶段 3: tower_fn (high_tower / low_tower), 期间持续吸气
    阶段 4: grasp(False) 放气
    阶段 5: reset_x 撞墙归零

    Returns:
        {"ok": True, "phases": list, "pick_mode": "vision"|"open_loop",
         "hold_s": float, "vision_result": dict|None, "x_reset": dict}

    Raises:
        RuntimeError: vision=True 且失败且 vision_fallback=False。
    """
    print(f"\n========== {log_prefix} run ==========")
    print(f"  模式: {'视觉闭环 (track_velocity_pick ' + str(vision_label) + ')' if vision else '开环盲吸 (旧行为)'}")
    print(f"  流程: {pick_name} → 取球 → {tower_name}(吸) → grasp(False) → reset_x({reset_direction})")

    # 阶段 1: 取球位姿
    pick_fn(client, runner)
    print(f"  >>> {pick_name} 完成 <<<")

    # 阶段 2: 取球
    pick_mode = "open_loop"
    vision_result: Optional[dict] = None
    if vision:
        print(f"\n----- {log_prefix} [阶段 2] 视觉伺服取球 ({vision_label}, "
              f"grasp_y={grasp_y_mm}mm, timeout={vision_timeout:.0f}s) -----")
        try:
            vision_result = runner.track_velocity_pick(
                vision_label,
                skip_pose_align=True,      # get_* 已摆好位姿, 跳过入口 composite_run
                lift_back=False,           # 吸住后直接交给 tower (tower 第 1 步就抬 y)
                grasp_y_mm=grasp_y_mm,
                hold_s=DEFAULT_VISION_HOLD_S,
                mode="pick",
                sign_arm=sign_arm,
                sign_x=sign_x,
                timeout=vision_timeout,
            )
        except Exception as e:
            vision_result = {"ok": False, "reason": f"exception:{type(e).__name__}: {e}"}

        if vision_result.get("ok"):
            pick_mode = "vision"
            print(f"  >>> 视觉取球成功: settled={vision_result.get('settled')} "
                  f"trace_hits={vision_result.get('trace_hits')} "
                  f"steps={vision_result.get('steps')} <<<")
        else:
            reason = vision_result.get("reason")
            print(f"  [WARN] 视觉取球失败 (reason={reason})")
            if not vision_fallback:
                raise RuntimeError(f"{log_prefix} 视觉取球失败: {reason}")
            print(f"  回退: 重跑 {pick_name} 恢复位姿 → 开环盲吸")
            pick_fn(client, runner)

    if pick_mode == "open_loop":
        print(f"\n----- {log_prefix} [阶段 2] grasp(True) + sleep({hold_s:.1f}s) 开环盲吸 -----")
        runner.grasp(True, timeout=10.0)
        if hold_s > 0.0:
            time.sleep(hold_s)

    # 阶段 3: tower, 期间吸盘持续吸气
    print(f"\n----- {log_prefix} [阶段 3] {tower_name} (期间持续吸) -----")
    tower_fn(client, runner)
    print(f"  >>> {tower_name} 完成 <<< 吸气结束触发点")

    # 阶段 4: 放气
    print(f"\n----- {log_prefix} [阶段 4] grasp(False) -----")
    runner.grasp(False, timeout=10.0)

    # 阶段 5: x 轴归零 (球已放出, 不用保持)
    print(f"\n----- {log_prefix} [阶段 5] reset_x 撞墙归零 ({reset_direction}) -----")
    x_reset = _reset_x_wall(client, direction=reset_direction)
    print(f"  >>> reset_x 完成, realtime x={x_reset['x_mm_after']}mm <<<")

    print(f"\n========== {log_prefix} 完成 (pick_mode={pick_mode}) ==========\n")
    return {
        "ok": True,
        "phases": [pick_name, f"pick({pick_mode})", tower_name, "release", "reset_x"],
        "pick_mode": pick_mode,
        "hold_s": hold_s,
        "vision_result": vision_result,
        "x_reset": x_reset,
    }


# ---------- 共享 CLI (test1-4 wrapper 复用) ----------

def build_pick_place_parser(description: str) -> argparse.ArgumentParser:
    """test1-4 wrapper 的共享 CLI: 旧 --hold 兼容 + 视觉闭环新开关。"""
    p = argparse.ArgumentParser(
        description=description,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--hold", type=float, default=GRASP_HOLD_S_DEFAULT,
                   help="开环模式吸气独立保持秒数 (设 0 跳过; --vision 时忽略)")
    p.add_argument("--vision", action="store_true",
                   help="启用视觉闭环取球 (track_velocity_pick); "
                        "⚠️ sign_arm/sign_x 是 task1 姿态标定值, task5 位姿首次跑先确认方向")
    p.add_argument("--grasp-y", type=float, default=DEFAULT_GRASP_Y_MM, dest="grasp_y",
                   help="视觉模式吸气 y (mm); 默认 -70=取球位姿高度不下探, 吸不实再往下调")
    p.add_argument("--no-vision-fallback", dest="vision_fallback", action="store_false",
                   help="视觉失败不回退开环 (默认回退: 重跑取球位姿 → 盲吸)")
    p.add_argument("--sign-arm", type=float, default=1.0, dest="sign_arm",
                   help="视觉伺服大臂轴符号 (±1, 现场标定; task1 默认 +1)")
    p.add_argument("--sign-x", type=float, default=-1.0, dest="sign_x",
                   help="视觉伺服 x 轴符号 (±1, 现场标定; task1 默认 -1)")
    p.add_argument("--vision-timeout", type=float, default=DEFAULT_VISION_TIMEOUT_S,
                   dest="vision_timeout", help="视觉伺服总超时 (秒)")
    p.set_defaults(vision_fallback=True)
    return p


def main(argv=None) -> int:
    print(f"{LOG_PREFIX} 本文件是 test1-4 的公共执行器, 不直接跑。")
    print("  跑 wrapper: test1_from_yellow_to_high.py / test2_from_blue_to_high.py /")
    print("              test3_from_yellow_to_low.py  / test4_from_blue_to_low.py")
    print("  加 --vision 启用视觉闭环取球。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
