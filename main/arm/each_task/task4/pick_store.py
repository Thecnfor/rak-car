"""task4 / target4 —— 选球判色 + 机械臂智能抓取 + 放 bin。

从 target4.py 拆出 (2026-08-10 拆分): 单一职责 = 机械臂侧"抓一个球放进对应 bin"。
- ``_color_from_track``  track 结果 final_frame.label → 球色 (兼容保留; 主流程不再用)。
- ``_pick_best_ball``    从 fetch_balls 结果选 1 球 (score 最高, 判色)。
- ``_pick_by_arm_servo`` 机械臂智能抓取 (2026-08-10): 大臂控 cx + x 十字控 cy,
                         高位伺服 → 最后盲降 → 吸气 → 抬回, 替换底盘对齐。
- ``_pick_and_store``    臂伺服抓球 + 同步放 bin (servo pick + transit → bin → release)。
"""
from __future__ import annotations

from typing import Optional

from main.arm import ArmRunner  # noqa: E402

from .constants import (  # noqa: E402
    COLOR_BLUE, COLOR_YELLOW,
    TASK4_POSE_P_ARM_DEG, TASK4_POSE_P_HAND_DEG,
    X_PICK_MM, Y_PICK_MM, X_TRANSIT_MM, Y_TRANSIT_MM, Y_PUT_MM,
    BIN_X_MM,
    # 机械臂智能抓取 (2026-08-10)
    TASK4_SETPOINT_X_NORM, TASK4_SETPOINT_Y_NORM,
    TASK4_SERVO_Y_START_MM, TASK4_SERVO_GAIN_ARM, TASK4_SERVO_GAIN_X,
    TASK4_SERVO_DEADZONE, TASK4_SERVO_MAX_VEL, TASK4_SERVO_HZ,
    TASK4_SERVO_SETTLE_HITS, TASK4_SERVO_TIMEOUT_S,
    TASK4_SERVO_ARM_MIN, TASK4_SERVO_ARM_MAX,
    TASK4_SERVO_SIGN_ARM, TASK4_SERVO_SIGN_X, TASK4_SERVO_DESCEND_HAND_DEG,
    _ts_str,
    LOG_PREFIX_TARGET4 as LOG_PREFIX,
)


def _pick_best_ball(balls: list) -> Optional[dict]:
    """从 fetch_balls 结果选 1 球: score 最高, 平局取第一个 (判色用)。"""
    candidates = [b for b in balls if b.get("color") in (COLOR_BLUE, COLOR_YELLOW)]
    if not candidates:
        return None
    return max(candidates, key=lambda b: float(b.get("score", 0.0)))


def _pick_by_arm_servo(
    runner: ArmRunner,
    *,
    color: str,
    servo_x_start_mm: float,
    servo_y_start_mm: float = TASK4_SERVO_Y_START_MM,
    servo_arm_start_deg: float = 90.0,
    servo_hand_start_deg: float = TASK4_POSE_P_HAND_DEG,
    grasp_y_mm: float = Y_PICK_MM,
    setpoint_x_norm: float = TASK4_SETPOINT_X_NORM,
    setpoint_y_norm: float = TASK4_SETPOINT_Y_NORM,
) -> dict:
    """机械臂智能抓取 (2026-08-10, 替换底盘对齐——底盘打滑不准)。

    走 runner.track_velocity_pick (velocity 模式, 免 arm_queue):
      - find_target_arm_cross: dx=cx-setpoint_x → 大臂, dy=cy-setpoint_y → x 十字
        (y 十字锁 0, hand 固定) —— 用户拍板"大臂 + x 轴, y 不动"。
      - 高位伺服 (y=servo_y_start) 收敛后 → y 盲降 grasp_y_mm → 吸气。
        (lift_back=False 不抬回; 抬高交给中转同步 composite_run)
      - setpoint 硬编码 (TASK4_SETPOINT_*), 两种球同尺寸共用一份。

    Returns:
        {"ok": bool, "error": str|None, "result": dict|None}
    """
    label = f"ball_{color}"
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] 🎯 臂伺服智能抓取 "
          f"(label={label}, 大臂+x轴, setpoint=({setpoint_x_norm:.3f},"
          f"{setpoint_y_norm:.3f}), y_start={servo_y_start_mm:.0f} -> "
          f"盲降 {grasp_y_mm:.0f})")
    result = runner.track_velocity_pick(
        label,
        x_start=servo_x_start_mm,
        y_start=servo_y_start_mm,
        arm_start=servo_arm_start_deg,
        hand_start=servo_hand_start_deg,
        grasp_y_mm=grasp_y_mm,
        descend_hand_deg=TASK4_SERVO_DESCEND_HAND_DEG,
        setpoint_x_norm=setpoint_x_norm,
        setpoint_y_norm=setpoint_y_norm,
        gain_arm=TASK4_SERVO_GAIN_ARM,
        gain_x=TASK4_SERVO_GAIN_X,
        deadzone=TASK4_SERVO_DEADZONE,
        max_vel=TASK4_SERVO_MAX_VEL,
        hz=TASK4_SERVO_HZ,
        settle_hits=TASK4_SERVO_SETTLE_HITS,
        timeout=TASK4_SERVO_TIMEOUT_S,
        arm_min=TASK4_SERVO_ARM_MIN,
        arm_max=TASK4_SERVO_ARM_MAX,
        sign_arm=TASK4_SERVO_SIGN_ARM,
        sign_x=TASK4_SERVO_SIGN_X,
        mode="pick",
        lock_first=True,
        # 抓取后不自动抬回 y_start 高位: 由 _pick_and_store 的中转同步 composite_run
        # (x=transit_x, y=transit_y) 一步抬高到中转位 (用户拍板 2026-08-10)。
        lift_back=False,
        # P 姿态 x=-295mm 靠近 runtime 下限 -300mm，负向 x_vel 会被限幅为 0。
        # 进入视觉循环前先摆到 X_PICK_MM=-240mm，给 x 轴双向调节余量。
        skip_pose_align=False,
    )
    if not result.get("ok"):
        return {"ok": False,
                "error": f"臂伺服抓取未收敛 (reason={result.get('reason')}, "
                         f"trace_hits={result.get('trace_hits')})",
                "result": result}
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] ✅ 臂伺服收敛并抓到 {color} 球")
    return {"ok": True, "error": None, "result": result}


def _pick_and_store(
    arm_client,
    runner: ArmRunner,
    *,
    color: str,
    pick_y_mm: float = Y_PICK_MM,  # 盲降抓球目标 y (臂伺服 grasp_y)
    transit_x_mm: float = X_TRANSIT_MM,
    transit_y_mm: float = Y_TRANSIT_MM,  # 中转 y (与中转 x 同步抬高, 用户拍板 -130)
    bin_x_mm: Optional[float] = None,  # None → 按 color 查 BIN_X_MM
    bin_y_mm: float = Y_PUT_MM,
    bin_hand_deg: float = TASK4_POSE_P_HAND_DEG,
    # ---- 臂伺服 pick 参数 (默认走 constants / pose_p) ----
    servo_x_start_mm: float = X_PICK_MM,
    servo_y_start_mm: float = TASK4_SERVO_Y_START_MM,
    servo_arm_start_deg: float = 90.0,
    servo_hand_start_deg: float = TASK4_POSE_P_HAND_DEG,
    setpoint_x_norm: float = TASK4_SETPOINT_X_NORM,
    setpoint_y_norm: float = TASK4_SETPOINT_Y_NORM,
) -> dict:
    """机械臂智能抓取 + 同步放 bin。

    流程 (同步, 无 sleep):
      0. 臂伺服智能抓取: 大臂+x 轴对齐 → 高位收敛 → y 盲降 pick_y → 吸气 (不抬回)
      1. composite_run x=transit_x, y=transit_y   中转同步抬高+横移 (用户拍板)
      2. composite_run x=bin_x, arm=+90           横移到 bin 上方 (显式锁大臂)
      3. composite_run y=bin_y, hand=bin_hand, arm=+90  降到放仓位 (显式锁大臂)
      4. grasp(False)                             放气

    Returns:
        {"ok": bool, "error": str|None, "release_thread": None}
    """
    if bin_x_mm is None:
        bin_x_mm = BIN_X_MM[color]  # 未显式传 → 按颜色查 bin 列

    # 0. 机械臂智能抓取 (替换旧底盘对齐 + 盲降)
    pick = _pick_by_arm_servo(
        runner,
        color=color,
        servo_x_start_mm=servo_x_start_mm,
        servo_y_start_mm=servo_y_start_mm,
        servo_arm_start_deg=servo_arm_start_deg,
        servo_hand_start_deg=servo_hand_start_deg,
        grasp_y_mm=pick_y_mm,
        setpoint_x_norm=setpoint_x_norm,
        setpoint_y_norm=setpoint_y_norm,
    )
    if not pick["ok"]:
        return {"ok": False, "error": pick["error"], "release_thread": None}

    # 1. 中转 (x+y 同步: 抬高到 transit_y + 横移到 transit_x, 一次 composite_run。
    #    用户拍板 2026-08-10: 抓取后不抬回高位, 中转即抬高, 同步走)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [1/4] composite_run(x={transit_x_mm:+.0f}, "
          f"y={transit_y_mm:+.0f})  中转同步抬高+横移")
    try:
        arm_client.composite_run(x_mm=transit_x_mm, y_mm=transit_y_mm,
                                 speed=80, timeout=30.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(x={transit_x_mm}, y={transit_y_mm}) 中转失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 2. 横移到 bin 上方 (中转 x=transit_x → bin_x_mm; 显式 arm=+90 锁定大臂)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [2/4] composite_run(x={bin_x_mm:+.0f}, "
          f"arm={TASK4_POSE_P_ARM_DEG:+.0f})  横移到 {color} bin 上方")
    try:
        arm_client.composite_run(x_mm=bin_x_mm, arm=TASK4_POSE_P_ARM_DEG,
                                 speed=80, timeout=30.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(x={bin_x_mm}) 横移失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 3. 降到放仓位 (y=bin_y_mm, hand=bin_hand_deg; 显式 arm=+90 锁定大臂)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [3/4] composite_run(y={bin_y_mm:+.0f}, "
          f"hand={bin_hand_deg:+.0f}, arm={TASK4_POSE_P_ARM_DEG:+.0f})  降到放仓位")
    try:
        arm_client.composite_run(y_mm=bin_y_mm, hand=bin_hand_deg,
                                 arm=TASK4_POSE_P_ARM_DEG,
                                 speed=80, timeout=10.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(y={bin_y_mm}, hand={bin_hand_deg}) 降放仓位失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 4. 放气 (无 sleep, 直接让下一球 goto_pose_p 回 P 姿态)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [4/4] grasp(False)  放气 (无 sleep)")
    try:
        runner.grasp(False, timeout=5.0)
    except Exception as e:
        return {"ok": False,
                "error": f"grasp(False) 失败: {type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    return {"ok": True, "error": None, "release_thread": None}
