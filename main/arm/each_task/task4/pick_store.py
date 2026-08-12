"""task4 / target4 —— 选球判色 + 机械臂视觉伺服 + 盲降抓取放 bin。

从 target4.py 拆出 (2026-08-10 拆分): 单一职责 = 机械臂侧"抓一个球放进对应 bin"。
2026-08-11 新版流程:
- ``_run_arm_servo``   调 runtime run_arm_servo (进程内闭环, 只动 x 十字 + 大臂)。
                      4s → 超时 → 加时 4s (总 8s) + 死区放大 1.5 倍 → 仍超时返回 (上层照样盲抓)。
- ``_servo_and_pick``  伺服对齐 → 保持姿势 → 盲降抓球 + 放 bin。
- ``_color_from_track``  track 结果 final_frame.label → 球色。
- ``_pick_best_ball``   从 fetch_balls 结果选 1 球 (score 最高, 兜底判色)。
"""
from __future__ import annotations

import time
from typing import Optional

from main.arm import ArmRunner  # noqa: E402

from .constants import (  # noqa: E402
    BALL_LABELS,
    COLOR_BLUE, COLOR_YELLOW,
    BIN_X_MM, BIN_HAND_DEG,
    ALIGN_ONLY,
    ARM_SERVO_SETPOINT_CX, ARM_SERVO_SETPOINT_CY,
    ARM_SERVO_GAIN_ARM, ARM_SERVO_GAIN_X, ARM_SERVO_DEADZONE,
    ARM_SERVO_RETRY_DEADZONE, ARM_SERVO_MAX_VEL,
    ARM_SERVO_TIMEOUT_S, ARM_SERVO_RETRY_TIMEOUT_S, ARM_SERVO_SETTLE_HITS,
    ARM_SERVO_SIGN_ARM, ARM_SERVO_SIGN_X, ARM_SERVO_ARM_START,
    ARM_SERVO_ARM_MIN, ARM_SERVO_ARM_MAX, ARM_SERVO_HZ,
    PICK_LOWER_Y_MM, PICK_SUCK_HAND_DEG, PICK_HOLD_S, PICK_LIFT_Y_MM,
    PICK_BIN_ARM_DEG, PICK_RELEASE_Y_MM, PICK_RELEASE_HAND_DEG,
    _ts_str,
    LOG_PREFIX_TARGET4 as LOG_PREFIX,
)


def _color_from_track(track_res) -> Optional[str]:
    """从 track 结果 final_frame.label 提球色 (ball_blue → "blue")。"""
    ff = getattr(track_res, "final_frame", None)
    label = getattr(ff, "label", None) if ff is not None else None
    if label in BALL_LABELS:
        return label.split("_", 1)[1]
    return None


def _pick_best_ball(balls: list) -> Optional[dict]:
    """从 fetch_balls 结果选 1 球: score 最高, 平局取第一个 (兜底判色用)。"""
    candidates = [b for b in balls if b.get("color") in (COLOR_BLUE, COLOR_YELLOW)]
    if not candidates:
        return None
    return max(candidates, key=lambda b: float(b.get("score", 0.0)))


def _run_arm_servo(http_client, label: str, **overrides) -> dict:
    """调 runtime run_arm_servo (进程内闭环, 只动 x 十字 + 大臂)。

    只动 x (十字) + 大臂; setpoint 走 ARM_SERVO_* 常量。
    4s → 超时 → 加时 4s (总上限 8s) + 死区放大 1.5 倍 → 仍超时返回结果 (不抛)。
    ⚠️ 重试从上一轮 end_arm 续跑 (不回起点, 否则大臂拉回初始丢已转角度, task2 同语义)。
    """
    kw = {
        "label": label,
        "setpoint_x_norm": float(ARM_SERVO_SETPOINT_CX),
        "setpoint_y_norm": float(ARM_SERVO_SETPOINT_CY),
        "gain_arm": float(ARM_SERVO_GAIN_ARM),
        "gain_x": float(ARM_SERVO_GAIN_X),
        "deadzone": float(ARM_SERVO_DEADZONE),
        "max_vel": float(ARM_SERVO_MAX_VEL),
        "servo_timeout": float(ARM_SERVO_TIMEOUT_S),
        "settle_hits": int(ARM_SERVO_SETTLE_HITS),
        "sign_arm": float(ARM_SERVO_SIGN_ARM),
        "sign_x": float(ARM_SERVO_SIGN_X),
        "arm_start": float(ARM_SERVO_ARM_START),
        "arm_min": float(ARM_SERVO_ARM_MIN),
        "arm_max": float(ARM_SERVO_ARM_MAX),
        "hz": float(ARM_SERVO_HZ),
    }
    kw.update(overrides)

    def _call_once(tag: str) -> dict:
        job = http_client.execute(
            "car", "run_arm_servo", kwargs=kw, sync=True,
            timeout=float(kw["servo_timeout"]) + 15.0,
        )
        result = (job or {}).get("result") if isinstance(job, dict) else None
        result = result if isinstance(result, dict) else {}
        print(f"  [{LOG_PREFIX}] [{_ts_str()}] 臂伺服{tag}结果: "
              f"reason={result.get('reason')} settled={result.get('settled')} "
              f"trace_hits={result.get('trace_hits')} end_arm={result.get('end_arm')}")
        return result

    result = _call_once("")
    if (not result.get("settled")) and result.get("reason") == "timeout":
        print(f"  [{LOG_PREFIX}] 臂伺服超时, 加时 {ARM_SERVO_RETRY_TIMEOUT_S:.0f}s "
              f"死区 {ARM_SERVO_DEADZONE:.3f}→{ARM_SERVO_RETRY_DEADZONE:.3f} "
              f"(总上限 {ARM_SERVO_TIMEOUT_S + ARM_SERVO_RETRY_TIMEOUT_S:.0f}s)")
        kw["deadzone"] = float(ARM_SERVO_RETRY_DEADZONE)
        kw["servo_timeout"] = float(ARM_SERVO_RETRY_TIMEOUT_S)
        if result.get("end_arm") is not None:
            kw["arm_start"] = float(result["end_arm"])  # 从当前大臂角度续跑, 不回起点
        result = _call_once("重试")
    return result


def _servo_and_pick(
    arm_client,
    http_client,
    runner: ArmRunner,
    *,
    color: str,
    bin_x: Optional[float] = None,
    release_hand: Optional[float] = None,
    dry_run: bool = False,
) -> dict:
    """机械臂视觉伺服 → 对齐后保持姿势 → 盲降抓球 → 放 bin。

    伺服: run_arm_servo (只动 x + 大臂), 超时也照样盲抓 (2026-08-11 用户)。
    抓放序列 (保持伺服后姿势):
      1. y→PICK_LOWER_Y 盲降  2. 吸气  3. 保持 PICK_HOLD_S
      4. y→PICK_LIFT_Y 抬升   5. x→bin_x 且 arm→PICK_BIN_ARM 回 +95
      6. y→PICK_RELEASE_Y + hand→放球角  7. 放气
    """
    label = f"ball_{color}"
    bin_x = float(bin_x) if bin_x is not None else BIN_X_MM.get(color, 0.0)
    release_hand = (float(release_hand) if release_hand is not None
                    else BIN_HAND_DEG.get(color, PICK_RELEASE_HAND_DEG))

    # ---- 1. 机械臂视觉伺服 ----
    if dry_run:
        print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过臂伺服 (label={label})")
    else:
        try:
            servo = _run_arm_servo(http_client, label)
            print(f"  [{LOG_PREFIX}] 臂伺服结束: settled={servo.get('settled')} "
                  f"reason={servo.get('reason')}")
        except Exception as e:
            print(f"  [{LOG_PREFIX}] ⚠️ 臂伺服异常 "
                  f"({type(e).__name__}: {str(e)[:100]}), 照样盲抓")

    if ALIGN_ONLY:
        print(f"  [{LOG_PREFIX}] [ALIGN_ONLY] 伺服完成, 不抓取")
        return {"ok": False, "error": "align_only", "release_thread": None}

    # ---- 2. 抓放序列 (保持伺服后姿势, 每步只动指定轴) ----
    steps = [
        (f"盲降 y={PICK_LOWER_Y_MM:.0f} + hand→{PICK_SUCK_HAND_DEG:.0f}°",
         lambda: arm_client.composite_run(y_mm=PICK_LOWER_Y_MM, hand=PICK_SUCK_HAND_DEG,
                                          speed=80, timeout=10.0)),
        ("吸气", lambda: runner.grasp(True, timeout=5.0)),
        (f"保持 {PICK_HOLD_S:.1f}s", lambda: time.sleep(PICK_HOLD_S)),
        (f"抬升 y={PICK_LIFT_Y_MM:.0f}",
         lambda: arm_client.composite_run(y_mm=PICK_LIFT_Y_MM, speed=80, timeout=10.0)),
        (f"横移 bin x={bin_x:.0f} + 大臂回 {PICK_BIN_ARM_DEG:.0f}",
         lambda: arm_client.composite_run(x_mm=bin_x, arm=PICK_BIN_ARM_DEG,
                                          speed=80, timeout=20.0)),
        (f"放球 y={PICK_RELEASE_Y_MM:.0f} + hand={release_hand:.0f}",
         lambda: arm_client.composite_run(y_mm=PICK_RELEASE_Y_MM, hand=release_hand,
                                          speed=80, timeout=10.0)),
        ("放气", lambda: runner.grasp(False, timeout=5.0)),
    ]
    for i, (desc, action) in enumerate(steps, 1):
        if dry_run:
            print(f"  [{LOG_PREFIX}] [DRY-RUN] [{i}/7] {desc}")
            continue
        print(f"  [{LOG_PREFIX}] [{_ts_str()}] [{i}/7] {desc}")
        try:
            r = action()
        except Exception as e:
            err = f"{desc} 异常: {type(e).__name__}: {str(e)[:120]}"
            print(f"  [{LOG_PREFIX}] ❌ [{i}/7] {err}")
            return {"ok": False, "error": err, "release_thread": None}
        if isinstance(r, dict) and r.get("ok") is False:
            err = f"{desc} 返回 ok=False (steps={r.get('steps')})"
            print(f"  [{LOG_PREFIX}] ❌ [{i}/7] {err}")
            return {"ok": False, "error": err, "release_thread": None}
    return {"ok": True, "error": None, "release_thread": None}
