"""task4 / target4 —— 选球判色 + 盲降抓取放 bin。

从 target4.py 拆出 (2026-08-10 拆分): 单一职责 = 机械臂侧"抓一个球放进对应 bin"。
- ``_color_from_track``  track 结果 final_frame.label → 球色。
- ``_pick_best_ball``    从 fetch_balls 结果选 1 球 (score 最高, 兜底判色)。
- ``_pick_and_store``    底盘对齐后盲降抓球 + 同步放 bin (6 步 composite_run/grasp)。
"""
from __future__ import annotations

from typing import Optional

from main.arm import ArmRunner  # noqa: E402

from .constants import (  # noqa: E402
    BALL_LABELS,
    COLOR_BLUE, COLOR_YELLOW,
    TASK4_POSE_P_HAND_DEG,
    X_PICK_MM, Y_PICK_MM, Y_TRANSIT_MM, X_TRANSIT_MM, Y_PUT_MM,
    BIN_X_MM, BIN_Y_MM, BIN_HAND_DEG,
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


def _pick_and_store(
    arm_client,
    runner: ArmRunner,
    *,
    color: str,
    return_x_mm: Optional[float],
    pick_timeout_s: float,  # noqa: ARG001 — 保留参数兼容性, 当前同步流程不直接用
    pick_x_mm: float = X_PICK_MM,
    pick_y_mm: float = Y_PICK_MM,
    transit_y_mm: float = Y_TRANSIT_MM,
    transit_x_mm: float = X_TRANSIT_MM,
    put_y_mm: float = Y_PUT_MM,
    bin_x_mm: float = BIN_X_MM[COLOR_BLUE],
    bin_y_mm: float = Y_PUT_MM,
    bin_hand_deg: float = TASK4_POSE_P_HAND_DEG,
) -> dict:
    """底盘对齐后盲降抓球 + 同步放 bin。

    流程 (同步, 6 步, 无 sleep):
      0. composite_run x=pick_x                盲降前横移
      1. composite_run y=Y_PICK                盲降到抓球位
      2. grasp(True)                           真空开
      3. composite_run y=Y_TRANSIT, x=X_TRANSIT  抬到中转位
      4. composite_run x=bin_x                 横移到 bin 上方
      5. composite_run y=bin_y, hand=bin_hand  降到放仓位
      6. grasp(False)                          放气

    Returns:
        {"ok": bool, "error": str|None, "release_thread": None}
    """
    bin_x = BIN_X_MM[color]

    # 0. 盲降前横移到 pick_x (待现场测; 短距, belt-slip 风险低)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [0/6] composite_run(x={pick_x_mm:+.0f})  盲降前横移到 {pick_x_mm}")
    try:
        arm_client.composite_run(x_mm=pick_x_mm, speed=80, timeout=30.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(x={pick_x_mm}) 横移失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 1. 盲降到抓球位
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [1/6] composite_run(y={pick_y_mm:+.0f})  盲降到抓球位")
    try:
        arm_client.composite_run(y_mm=pick_y_mm, speed=80, timeout=10.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(y={pick_y_mm}) 盲降失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 2. grasp + 直接下一动作 (无 sleep, SDK 真空建立自闭环)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [2/6] grasp(True)  真空开 (无 sleep)")
    try:
        runner.grasp(True, timeout=5.0)
    except Exception as e:
        return {"ok": False,
                "error": f"grasp(True) 失败: {type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 3. 抬到中转位 (y=transit_y) ∥ 横移到中转位 x=transit_x (composite_run 并行)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [3/6] composite_run(y={transit_y_mm:+.0f}, x={transit_x_mm:+.0f})  "
          f"抬升+横移到中转位")
    try:
        arm_client.composite_run(y_mm=transit_y_mm, x_mm=transit_x_mm,
                                 speed=80, timeout=30.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(y={transit_y_mm}, x={transit_x_mm}) 中转失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 4. 横移到 bin 上方 (中转 x=transit_x → bin_x)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [4/6] composite_run(x={bin_x:+.0f})  横移到 {color} bin 上方")
    try:
        arm_client.composite_run(x_mm=bin_x, speed=80, timeout=30.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(x={bin_x}) 横移失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 5. 降到放仓位 (中转 y=transit_y → bin y=bin_y_mm, 同时调整 hand=bin_hand_deg)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [5/6] composite_run(y={bin_y_mm:+.0f}, hand={bin_hand_deg:+.0f})  降到放仓位")
    try:
        arm_client.composite_run(y_mm=bin_y_mm, hand=bin_hand_deg,
                                 speed=80, timeout=10.0)
    except Exception as e:
        return {"ok": False,
                "error": f"composite_run(y={bin_y_mm}, hand={bin_hand_deg}) 降放仓位失败: "
                         f"{type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    # 6. 放气 (无 sleep, 直接让下一球 goto_pose_p 回 P 姿态)
    print(f"  [{LOG_PREFIX}] [{_ts_str()}] [6/6] grasp(False)  放气 (无 sleep)")
    try:
        runner.grasp(False, timeout=5.0)
    except Exception as e:
        return {"ok": False,
                "error": f"grasp(False) 失败: {type(e).__name__}: {str(e)[:120]}",
                "release_thread": None}

    return {"ok": True, "error": None, "release_thread": None}
