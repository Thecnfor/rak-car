#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/tasks/water_tower_task.py

Task 2: water tower fill (water_tower_task / 水塔取水).

Field layout (after entering task area, at first water tower position):
  - LEFT  (X negative side): 6 water cubes, 3 groups of 2 cubes, groups 30cm apart
  - RIGHT (X positive side): 2 water towers, 60cm apart
  - cam2 detects water tower indicator (water_l1/l2/l3)

Initial state at first water tower position:
  - Y = -150 mm  (safe carry height)
  - X = 0 mm
  - arm = +90 deg  (MID)
  - hand = -90 deg (UP)

Safety rules:
  - 大臂旋转前 X 必须在 [-300, -150], Y 必须在 [-180, -100]
  - 大臂在 [-45, -95] 时，Y 抬升前 X 必须在 [-300, -180]
  - Y 下降到 -75 吸取前，必须确认大臂已物理到位 (读 arm_state)

Per-cube sequence (from detection: X=-160, Y=-20, arm=-95, hand=-45):
  1. X 到 [-300,-180] (Rule B) → Y 到 -120 (抬升)
  2. 大臂 +90 (Rule A) → X 到 cube_x
  3. 手爪 0 → 确认大臂到位 (Rule C) → Y 到 -75
  4. grasp → Y 到 -105
  5. X 到 -180 → 大臂 -95, 手爪 -90 → X 到 -120 (投放)
  6. release → X 回 -160
"""
from __future__ import annotations

import logging
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from main.api_client import RuntimeApiClient
from main.tasks._helpers import (
    _ensure_runtime,
    _wait_infer_ready,
    _move_x,
    _move_y,
    _set_arm_angle,
    _set_hand_angle as _helpers_set_hand_angle,
    _grasp,
    _chassis_move_for,
)
from main.tasks._config import load_task_config

logger = logging.getLogger("task.water_tower")

WATER_TOWER_LABELS = ("water_l1", "water_l2", "water_l3")


# ============================================================
# 大臂旋转前置约束 (Rule A / Rule B) + Rule C 角度到位校验
# ============================================================

# Rule A: 大臂旋转前 X 必须在 [-300, -150], Y 必须在 [-180, -100]
ARM_SAFE_X_MIN = -300.0
ARM_SAFE_X_MAX = -150.0
ARM_SAFE_Y_MIN = -180.0
ARM_SAFE_Y_MAX = -100.0

# Rule B: 大臂在 [-45, -95] 时 Y 抬升前 X 必须在 [-300, -180]
ARM_LIFT_X_MIN = -300.0
ARM_LIFT_X_MAX = -180.0
ARM_LIFT_ARM_LO = -95.0
ARM_LIFT_ARM_HI = -45.0


# ── 辅助函数 ──────────────────────────────────────────────

def _read_arm_state(client: RuntimeApiClient) -> Dict[str, Any]:
    """读硬件 arm 状态 (走 /v1/execute get_arm_state, 不进 arm_feed 缓存).

    get_arm_state 返回 SDK 原始单位: x/y 为米, arm_angle/hand_angle 为度.
    这里统一转换为业务单位: x_mm / y_mm 为毫米.
    值为 None 的 key 不写入 dict, 保证 .get(key, default) 正常工作.
    """
    try:
        job = client.execute("car", "get_arm_state", sync=True, timeout=8)
    except Exception:
        return {}
    data = (job.get("result") or {}) if isinstance(job, dict) else {}
    if not isinstance(data, dict) or not data:
        return {}
    result: Dict[str, Any] = {}
    x_m = data.get("x")
    y_m = data.get("y")
    if x_m is not None:
        result["x_mm"] = x_m * 1000.0
    if y_m is not None:
        result["y_mm"] = y_m * 1000.0
    arm_angle = data.get("arm_angle")
    if arm_angle is not None:
        result["arm_angle"] = arm_angle
    hand_angle = data.get("hand_angle")
    if hand_angle is not None:
        result["hand_angle"] = hand_angle
    return result


def _ensure_rotation_position(client: RuntimeApiClient) -> None:
    """强制 X=-200, Y=-150: 大臂旋转 + 末端转到 -90° 的前置条件.

    内置 Rule B: 大臂在 [-45,-95] 时要抬升 Y 时, 先确保 X 在 [-300,-180].
    """
    state = _read_arm_state(client)
    cur_arm = state.get("arm_angle")
    cur_x = state.get("x_mm")
    cur_y = state.get("y_mm")

    # Rule B: 大臂在 [-95,-45] 且 Y 将抬升时, X 必须先到 [-300,-180]
    if (cur_arm is not None and ARM_LIFT_ARM_LO <= cur_arm <= ARM_LIFT_ARM_HI
            and cur_y is not None and cur_y > -150.0):  # Y 低于 -150, 即将抬升
        if cur_x is None or not (ARM_LIFT_X_MIN <= cur_x <= ARM_LIFT_X_MAX):
            logger.info("Rule B in rotation-position: arm=%.0f°, X=%.0f -> -180 before Y lift",
                        cur_arm, cur_x or 0.0)
            _move_x(client, -180.0, v_max_mms=80.0)

    # 现在强制 X=-200, Y=-150
    state = _read_arm_state(client)
    cur_x = state.get("x_mm")
    cur_y = state.get("y_mm")

    if cur_y is None or abs(cur_y - (-150.0)) > 3.0:
        logger.info("rotation-position: Y=%.0f -> -150", cur_y or 0.0)
        _move_y(client, -150.0)
    if cur_x is None or abs(cur_x - (-220.0)) > 3.0:
        logger.info("rotation-position: X=%.0f -> -220", cur_x or 0.0)
        _move_x(client, -220.0, v_max_mms=80.0)


def _safe_set_arm_angle(client: RuntimeApiClient, angle_deg: float,
                        speed: int = 80, timeout: float = 20.0) -> None:
    """大臂旋转前强制 X=-200, Y=-150."""
    _ensure_rotation_position(client)
    _set_arm_angle(client, angle_deg, speed=speed, timeout=timeout)


def _wait_arm_angle_reached(client: RuntimeApiClient, target_deg: float,
                            tolerance: float = 3.0, timeout: float = 10.0) -> None:
    """Rule C: 轮询 arm_state 直到大臂物理到达目标角度 (±tolerance)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        state = _read_arm_state(client)
        cur = state.get("arm_angle")
        if cur is not None and abs(cur - target_deg) <= tolerance:
            logger.info("arm angle reached: %.1f° (target %.0f° ± %.0f°)", cur, target_deg, tolerance)
            return
        time.sleep(0.15)
    raise RuntimeError(
        "arm angle did not reach {:.0f}° within {:.0f}s".format(target_deg, timeout)
    )


def _ensure_x_for_y_lift(client: RuntimeApiClient) -> None:
    """Rule B: 大臂在 [-45,-95] 且要抬升 Y 时, 确保 X 在 [-300,-180]."""
    state = _read_arm_state(client)
    cur_arm = state.get("arm_angle", 0.0)
    cur_x = state.get("x_mm", 0.0)

    if not (ARM_LIFT_ARM_LO <= cur_arm <= ARM_LIFT_ARM_HI):
        # 大臂不在约束范围, 不需要 Rule B
        return

    if ARM_LIFT_X_MIN <= cur_x <= ARM_LIFT_X_MAX:
        # X 已在安全范围
        return

    logger.info(
        "Rule B: arm=%.0f° in [%.0f,%.0f], X=%.0f not in [%.0f,%.0f], "
        "moving X to -180 before Y lift",
        cur_arm, ARM_LIFT_ARM_LO, ARM_LIFT_ARM_HI,
        cur_x, ARM_LIFT_X_MIN, ARM_LIFT_X_MAX,
    )
    _move_x(client, -180.0, v_max_mms=80.0)


def _set_hand_angle(client: RuntimeApiClient, angle_deg: float, speed: int = 80,
                   timeout: float = 10.0) -> None:
    # 委托到 main.tasks._helpers._set_hand_angle (含 retry)
    return _helpers_set_hand_angle(client, angle_deg, speed=speed, timeout=timeout)


def _pick_cube(
    client: RuntimeApiClient,
    cfg: Dict[str, Any],
    cube_x_mm: float,
) -> None:
    """抓一个水方块 (不投放).

    流程:
      1. 大臂→+95 (先到 X=-200,Y=-150 再转, Rule B 内置)
      2. Y→-120 (过渡抬升)
      3. X→cube_x_mm
      4. 手爪→0
      5. Rule C: 确认大臂物理到位
      6. Y→-75 (下降吸取)
      7. grasp + Y→-150
    """
    pick = cfg["pick_pose"]
    v_x = cfg["v_max_arm_x_mms"]

    # 1) 大臂 → 抓取角度 +95° (强制 X=-200,Y=-150, Rule B 内置)
    _safe_set_arm_angle(client, pick["arm_angle_deg"])
    _move_x(client, cube_x_mm, v_max_mms=v_x)

    # 4) 手爪 → 抓取姿态 (0°)
    _set_hand_angle(client, pick["hand_angle_deg"])

    # 5) Rule C: 确认大臂已物理到位再下降 Y
    _wait_arm_angle_reached(client, pick["arm_angle_deg"])

    # 6) Y 下降到抓取高度
    _move_y(client, pick["y_descend_mm"])

    # 7) 吸 + Y 抬升
    _grasp(client, True)
    time.sleep(cfg["vacuum_settle_s"])
    _move_y(client, pick["y_lift_mm"])


# 梯度放置: 同一个水塔内第 1/2/3 个方块的投放 Y 深度
DELIVER_Y_BY_INDEX = [-60.0, -75.0, -95.0]


def _deliver_cube(
    client: RuntimeApiClient,
    cfg: Dict[str, Any],
    cube_index: int = 0,
) -> None:
    """投放: 转大臂 → 手爪 -90° → Y 梯度下降 → X 到 -120 → 释放.

    cube_index: 0=第1个(-70), 1=第2个(-85), 2=第3个(-95).
    """
    carry = cfg["carry_pose"]
    v_x = cfg["v_max_arm_x_mms"]

    deliver_y = DELIVER_Y_BY_INDEX[min(cube_index, len(DELIVER_Y_BY_INDEX) - 1)]

    # 1) 先转大臂到运送姿态
    _safe_set_arm_angle(client, carry["arm_angle_deg"])

    # 2) 手爪到运送姿态 -90° (大臂+末端一起动, 必须在 X=-220,Y=-150)
    _ensure_rotation_position(client)
    _set_hand_angle(client, carry["hand_angle_deg"])

    # 3) Y 梯度下降到投放深度
    _move_y(client, deliver_y)

    # 4) X 移到投放位置
    _move_x(client, carry["x_mm"], v_max_mms=v_x)

    # 5) 释放
    _grasp(client, False)


def _detect_tower_count(client: RuntimeApiClient) -> int:
    """Y 降到 -20 后才检测, 1 秒超时, 失败默认 1 个方块 (不崩溃)."""
    import time as _t
    deadline = _t.time() + 1.0  # 只检测 1 秒
    while _t.time() < deadline:
        try:
            resp = client.get("/v1/realtime/vision/task", timeout=2)
        except Exception:
            _t.sleep(0.2)
            continue
        if not isinstance(resp, dict) or not resp.get("ok"):
            _t.sleep(0.2)
            continue
        task_state = resp.get("task_state") or {}
        if not task_state.get("active"):
            _t.sleep(0.2)
            continue
        dets = task_state.get("detections") or []
        for d in dets:
            label = (d or {}).get("label", "")
            if label in WATER_TOWER_LABELS:
                count_map = {"water_l1": 1, "water_l2": 2, "water_l3": 3}
                n = count_map[label]
                logger.info("water tower detect %s -> need %d cubes", label, n)
                return n
    # 超时/失败 → 默认 1 个, 不崩溃
    logger.warning("cam2 detection timeout, default to 1 cube")
    return 1


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    cfg = load_task_config("water_tower_task")
    if cfg.get("placeholder"):
        raise NotImplementedError("water_tower_task not yet implemented")

    if client is None:
        client = RuntimeApiClient()
    _ensure_runtime(client)

    _wait_infer_ready(client, timeout_s=30.0)

    completed: List[str] = []
    detection = cfg["detection_pose"]
    v_x = cfg["v_max_arm_x_mms"]
    timeout = cfg["chassis_move_timeout_s"]
    group_forward_m = cfg["group_forward_m"]
    x_target_mm = -220.0

    # ===== Step 1: X → -160mm (move_x_position PID 闭环) =====
    logger.info("Step 1: move X to %.0f mm (PID)", x_target_mm)
    _move_x(client, x_target_mm, v_max_mms=v_x)

    # ===== Step 2: 大臂 → -95°, 末端 → -45° =====
    logger.info("Step 2: arm=%s°, hand=-45°", detection["arm_angle_deg"])
    _safe_set_arm_angle(client, detection["arm_angle_deg"])
    time.sleep(2.0)

    _set_hand_angle(client, -45)
    time.sleep(1.0)

    try:
        for tower_idx, tower_label in enumerate(cfg["source_position_order"]):
            logger.info("=== processing tower %s (iteration %d) ===", tower_label, tower_idx + 1)

            if tower_idx > 0:
                tower_spacing_m = cfg.get("tower_spacing_m", 0.60)
                logger.info("chassis: from tower %d to tower %s (%.2f m forward)",
                            tower_idx, tower_label, tower_spacing_m)
                # 底盘移动前 X 复位到 -160
                _move_x(client, x_target_mm, v_max_mms=v_x)
                _chassis_move_for(client, dx_m=tower_spacing_m, timeout=timeout)
                _move_x(client, x_target_mm, v_max_mms=v_x)
                # 第二水塔检测: 末端回 -45°
                logger.info("set hand=-45° for tower %s detection", tower_label)
                _set_hand_angle(client, -45)
                time.sleep(0.5)

            logger.info("descend Y to -30 for detection")
            try:
                _move_y(client, -30)
                time.sleep(0.3)
            except Exception:
                logger.warning("Y descend failed, skip tower %s", tower_label)
                continue

            needed = _detect_tower_count(client)
            logger.info("tower %s needs %d water cubes", tower_label, needed)

            chassis_at_tower_m = 0.0  # 底盘相对水塔的偏移 (m): >0 前进, <0 后退
            picked = 0
            first_x = cfg["first_cube_x_mm"]
            second_x = cfg["second_cube_x_mm"]
            # 第一个水塔向前拿, 后面的水塔向后拿 (方块组在水塔后方)
            direction = 1.0 if tower_idx == 0 else -1.0

            while picked < needed:
                try:
                    group = picked // 2
                    target_offset = direction * group * group_forward_m

                    d = target_offset - chassis_at_tower_m
                    if abs(d) > 1e-3:
                        _move_x(client, x_target_mm, v_max_mms=v_x)
                        logger.info("chassis %.2f m to group %d", d, group + 1)
                        _chassis_move_for(client, dx_m=d, timeout=timeout)
                        chassis_at_tower_m = target_offset

                    pick_x = first_x if (picked % 2 == 0) else second_x
                    logger.info("picking cube %d at X=%s mm (group %d)", picked + 1, pick_x, group + 1)
                    _pick_cube(client, cfg, pick_x)

                    if abs(chassis_at_tower_m) > 1e-3:
                        logger.info("chassis back %.2f m to tower", -chassis_at_tower_m)
                        _chassis_move_for(client, dx_m=-chassis_at_tower_m, timeout=timeout)
                        chassis_at_tower_m = 0.0

                    _deliver_cube(client, cfg, cube_index=picked)
                    _move_x(client, x_target_mm, v_max_mms=v_x)
                except Exception:
                    logger.exception("cube %d failed, skip", picked + 1)
                picked += 1

            completed.append("tower_{}".format(tower_label))
    except Exception as exc:
        logger.exception("water_tower_task failed: %s", exc)
        return {"ok": False, "completed": completed, "error": str(exc)}

    return {"ok": True, "completed": completed}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
    result = run()
    print("water_tower_task result:", result)
