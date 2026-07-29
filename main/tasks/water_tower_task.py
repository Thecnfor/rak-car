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
  - 大臂旋转前 X 必须在 [-290, -140] 范围内
  - 底盘移动前 X 必须回到 -140 (检测位)

Per-cube sequence (from detection: X=-140, Y=-20, arm=-95, hand=-45):
  1. Y to -120   (transition)
  2. X to -140/-260, arm +90, hand 0
  3. Y to -75    (pick)
  4. grasp → Y to -105
  5. X to -180 → arm -95, hand -90 (carry) → X to -120
  6. release → X back to -140
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
from main.tasks.auto_seeding_safe import (
    _ensure_runtime,
    _wait_infer_ready,
    _chassis_move_for,
    _grasp,
    _set_arm_angle,
)
from main.tasks._config import load_task_config

logger = logging.getLogger("task.water_tower")

WATER_TOWER_LABELS = ("water_l1", "water_l2", "water_l3")


def _move_x(client: RuntimeApiClient, x_mm: float, v_max_mms: float = 80.0,
          out_time: float = 15.0, timeout: float = 30.0) -> None:
    """move_x_position PID 闭环 — 对齐 Task 1 auto_seeding.py L185-193."""
    job = client.execute(
        "arm", "move_x_position",
        args=[x_mm / 1000.0],
        kwargs={"v_max_mms": v_max_mms, "out_time": out_time},
        sync=True, timeout=timeout + 5,
    )
    if job.get("status") != "succeeded" or job.get("error"):
        raise RuntimeError(
            "arm move_x({:.0f}) failed: status={} error={}".format(
                x_mm, job.get("status"), job.get("error")
            )
        )


def _move_y(client: RuntimeApiClient, y_mm: float, timeout: float = 25.0) -> None:
    """move_y_position PID 闭环 — 对齐 Task 1 auto_seeding.py L196-207."""
    job = client.execute(
        "arm", "move_y_position",
        args=[y_mm / 1000.0],
        sync=True, timeout=timeout,
    )
    if job.get("status") != "succeeded" or job.get("error"):
        raise RuntimeError(
            "arm move_y({:.0f}) failed: status={} error={}".format(
                y_mm, job.get("status"), job.get("error")
            )
        )


# 大臂旋转安全区间: X 必须在 [-290, -140] 内才能转大臂
ARM_SAFE_X_MIN = -290.0
ARM_SAFE_X_MAX = -140.0


def _safe_set_arm_angle(client: RuntimeApiClient, angle_deg: float,
                        speed: int = 40, timeout: float = 20.0) -> None:
    """大臂旋转前确保 X 在安全区间 [-290, -140], 否则先移 X 到 -180."""
    try:
        resp = client.get("/v1/realtime/arm/state", timeout=3)
        cur_x = (resp.get("arm_state") or {}).get("x_mm", 0.0)
    except Exception:
        cur_x = 0.0
    if not (ARM_SAFE_X_MIN <= cur_x <= ARM_SAFE_X_MAX):
        logger.info("X=%.0f outside safe range [%.0f, %.0f], moving to -180 before arm rotation",
                    cur_x, ARM_SAFE_X_MIN, ARM_SAFE_X_MAX)
        _move_x(client, -180.0, v_max_mms=80.0)
    _set_arm_angle(client, angle_deg, speed=speed, timeout=timeout)


def _set_hand_angle(client: RuntimeApiClient, angle_deg: float, speed: int = 80, timeout: float = 10.0) -> None:
    """Wrapper for set_hand_angle (same retry pattern as _set_arm_angle)."""
    import time as _t
    last = None
    for attempt in range(1, 3):
        try:
            job = client.execute(
                "arm", "set_hand_angle",
                args=[angle_deg, speed],
                sync=True, timeout=timeout + 5,
            )
            if job.get("status") == "succeeded" and not job.get("error"):
                return
            last = "status={} error={}".format(job.get("status"), job.get("error"))
        except Exception as exc:
            last = "{}: {}".format(type(exc).__name__, exc)[:200]
        logger.warning("set_hand_angle(%.0f) attempt %d failed: %s", angle_deg, attempt, last)
        _t.sleep(1.0)
    raise RuntimeError("set_hand_angle({}) failed: {}".format(angle_deg, last))


def _pick_cube(
    client: RuntimeApiClient,
    cfg: Dict[str, Any],
    cube_x_mm: float,
) -> None:
    """抓一个水方块 (不投放): Y抬升 → X+大臂+末端 → Y下降 → 抓 → Y抬升."""
    pick = cfg["pick_pose"]
    v_x = cfg["v_max_arm_x_mms"]

    # 1) Y 抬升到过渡高度
    _move_y(client, pick["y_transition_mm"])

    # 2) X 移到抓取位置 (move_x_position PID)
    _move_x(client, cube_x_mm, v_max_mms=v_x)

    # 3) 大臂 + 末端到抓取姿态
    _safe_set_arm_angle(client, pick["arm_angle_deg"])
    _set_hand_angle(client, pick["hand_angle_deg"])

    # 4) Y 下降到抓取高度
    _move_y(client, pick["y_descend_mm"])

    # 5) 吸 + Y 抬升
    _grasp(client, True)
    time.sleep(cfg["vacuum_settle_s"])
    _move_y(client, pick["y_lift_mm"])


def _deliver_cube(
    client: RuntimeApiClient,
    cfg: Dict[str, Any],
) -> None:
    """投放: X→-180 → 大臂-95°+手爪-90° → X→-120 → 释放."""
    carry = cfg["carry_pose"]
    v_x = cfg["v_max_arm_x_mms"]

    # 1) X 先到安全位
    _move_x(client, -180.0, v_max_mms=v_x)

    # 2) 运送姿态
    _safe_set_arm_angle(client, carry["arm_angle_deg"])
    _set_hand_angle(client, carry["hand_angle_deg"])

    # 3) X 移到投放位置
    _move_x(client, carry["x_mm"], v_max_mms=v_x)

    # 4) 释放
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
    x_target_mm = -150.0

    # ===== Step 1: X → -150mm (move_x_position PID 闭环) =====
    logger.info("Step 1: move X to %.0f mm (PID)", x_target_mm)
    _move_x(client, x_target_mm, v_max_mms=v_x)

    # ===== Step 2: 大臂 → -90°, 末端 → -45° =====
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
                # 底盘移动前 X 复位到 -180
                _move_x(client, x_target_mm, v_max_mms=v_x)
                _chassis_move_for(client, dx_m=tower_spacing_m, timeout=timeout)
                _move_x(client, x_target_mm, v_max_mms=v_x)
                # 第二水塔检测: 末端回 -45°
                logger.info("set hand=-45° for tower %s detection", tower_label)
                _set_hand_angle(client, -45)
                time.sleep(0.5)

            logger.info("descend Y to -20 for detection")
            try:
                _move_y(client, -20)
                time.sleep(0.3)
            except Exception:
                logger.warning("Y descend failed, skip tower %s", tower_label)
                continue

            needed = _detect_tower_count(client)
            logger.info("tower %s needs %d water cubes", tower_label, needed)

            chassis_at_tower_m = 0.0  # 底盘相对水塔的前向偏移 (m)
            picked = 0
            cube_idx = 0
            first_x = cfg["first_cube_x_mm"]
            second_x = cfg["second_cube_x_mm"]

            while picked < needed:
                try:
                    group = cube_idx // 2
                    target_offset = group * group_forward_m

                    if target_offset > chassis_at_tower_m:
                        _move_x(client, x_target_mm, v_max_mms=v_x)
                        d = target_offset - chassis_at_tower_m
                        logger.info("chassis forward %.2f m to group %d", d, group + 1)
                        _chassis_move_for(client, dx_m=d, timeout=timeout)
                        chassis_at_tower_m = target_offset

                    pick_x = first_x if (cube_idx % 2 == 0) else second_x
                    logger.info("picking cube %d at X=%s mm (group %d)", cube_idx + 1, pick_x, group + 1)
                    _pick_cube(client, cfg, pick_x)

                    if chassis_at_tower_m > 0:
                        logger.info("chassis back %.2f m to tower", chassis_at_tower_m)
                        _chassis_move_for(client, dx_m=-chassis_at_tower_m, timeout=timeout)
                        chassis_at_tower_m = 0.0

                    _deliver_cube(client, cfg)
                    _move_x(client, x_target_mm, v_max_mms=v_x)
                except Exception:
                    logger.exception("cube %d failed, skip", cube_idx + 1)
                picked += 1
                cube_idx += 1

            completed.append("tower_{}".format(tower_label))
    except Exception as exc:
        logger.exception("water_tower_task failed: %s", exc)
        return {"ok": False, "completed": completed, "error": str(exc)}

    return {"ok": True, "completed": completed}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
    result = run()
    print("water_tower_task result:", result)
