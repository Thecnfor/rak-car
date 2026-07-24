#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/tasks/auto_seeding.py

Task 1: seedling transfer (right side cylinders -> left side purple circles).

Flow per source position (S1, S2, S3 in order):
  1. arm already at S1 pick pose (X=-60, arm=-150, Y=-50, hand=-10)
  2. detect label from cam2
  3. slot = target_slot_map[label]
  4. descend to pick y, grasp on, wait vacuum_settle_s, lift to carry y
  5. move chassis long-axis to align T_slot
  6. swing arm to 0 deg and slide X to -270 in parallel
  7. descend to place y, grasp off
  8. lift to carry y, swing arm back to -150, slide X back to -60
  9. move chassis to next source position

Coordinate conventions (see task_config.yml):
  x_mm: 0 = rightmost, decrease = move left
  y_mm: 0 = bottom limit, negative = up
  arm_angle: 0 = MID, -150 = rightmost
  hand_angle: -90 = UP, 0 = DOWN; pickup uses -10 (horizontal)
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
from main.arm.api import ArmClient
from main.arm.loops.runner import ArmRunner

from main.tasks._config import load_task_config

logger = logging.getLogger("task.auto_seeding")


def _ensure_runtime(client: RuntimeApiClient) -> None:
    if not client.wait_until_ready(timeout=10.0):
        raise RuntimeError("runtime not ready, check pm2 logs rak-car-api")


def _scan_labels(
    client: RuntimeApiClient,
    valid_labels: List[str],
    retries: int = 3,
    backoff_s: float = 0.5,
) -> Optional[str]:
    """cam2 task detection via /v1/realtime/vision/task (HTTP cache).

    This endpoint reads the in-memory cache populated by the task_feed
    daemon thread (10Hz by default). It does NOT call ZMQ, does NOT take
    car_lock, and does NOT crash the runtime when the inference backend
    is stuck. This bypasses the runtime bug where /v1/vision/task (POST)
    can deadlock the long-lived ZMQ REQ socket.

    task_state format:
      {active, mode, detections: [{cls_id, det_id, label, score, bbox_norm{...}}],
       count, updated_at}

    Returns the first valid label ordered by descending bbox_norm.width
    (closest cylinder). Returns None after all retries exhausted.
    """
    last_err: Optional[str] = None
    for attempt in range(1, retries + 1):
        try:
            resp = client.get("/v1/realtime/vision/task", timeout=5)
        except Exception as exc:
            last_err = f"http exception: {type(exc).__name__}: {exc}"[:200]
            logger.warning("cam2 attempt %d http failed: %s", attempt, last_err)
            if attempt < retries:
                time.sleep(backoff_s)
            continue
        if not isinstance(resp, dict) or not resp.get("ok"):
            last_err = str(resp)[:200]
            logger.warning("cam2 attempt %d not ok: %s", attempt, last_err)
            if attempt < retries:
                time.sleep(backoff_s)
            continue
        task_state = resp.get("task_state") or {}
        if not task_state.get("active"):
            last_err = f"task_feed not active (mode={task_state.get('mode')})"
            logger.info("cam2 attempt %d: %s", attempt, last_err)
            if attempt < retries:
                time.sleep(backoff_s)
            continue
        dets = task_state.get("detections") or []
        def _width(d):
            bn = d.get("bbox_norm") or {}
            try:
                return float(bn.get("width", 0.0))
            except Exception:
                return 0.0
        ordered = sorted(
            [d for d in dets if isinstance(d, dict) and d.get("label") in valid_labels],
            key=lambda d: -_width(d),
        )
        if ordered:
            return ordered[0]["label"]
        logger.info("cam2 attempt %d: ok but no valid label in %s (raw=%d, updated_at=%s)", attempt, valid_labels, len(dets), task_state.get("updated_at"))
        last_err = "no valid label"
        if attempt < retries:
            time.sleep(backoff_s)
    logger.error("cam2 giving up after %d retries, last_err=%s", retries, last_err)
    return None


def _wait_infer_ready(client: RuntimeApiClient, timeout_s: float = 30.0) -> None:
    """Probe /v1/health and wait until the task model is ready.
    If timeout, raise with hint to restart pm2 rak-car-api.
    """
    import time as _t
    deadline = _t.time() + timeout_s
    last: Any = None
    while _t.time() < deadline:
        try:
            h = client.get_health(snapshot=True)
        except Exception as exc:
            last = f"health call failed: {exc}"
            _t.sleep(1.0)
            continue
        last = h
        # health response shape: {ok, state: {infer_service: {models: [...]}}}
        state = h.get("state") or {}
        infer = state.get("infer_service") or {}
        models = infer.get("models") or []
        task = next((m for m in models if m.get("name") == "task"), None)
        if task and task.get("ready") and task.get("response"):
            logger.info("task inference backend ready (port=%s)", task.get("port"))
            return
        _t.sleep(1.0)
    raise RuntimeError(
        f"task inference backend not ready within {timeout_s}s; "
        f"on Jetson run: pm2 restart rak-car-api. last={last}"
    )


# _warmup_cam2 removed: it broke the long-lived ZMQ REQ socket


def _apply_pose(
    runner: ArmRunner,
    x_mm: float,
    y_mm: float,
    arm_angle_deg: float,
    hand_angle_deg: float,
    v_max_mms: float = 150.0,
) -> None:
    _set_arm_angle(client, arm_angle_deg)
    runner.client.set_hand_angle(hand_angle_deg, speed=80, timeout=10.0)
    runner.move_xy(x_mm=x_mm, y_mm=y_mm, v_max_mms=v_max_mms, a_max_mms2=400.0)


def _chassis_move_for(client: RuntimeApiClient, dx_m: float, dy_m: float = 0.0, dtheta_rad: float = 0.0, timeout: float = 30.0) -> None:
    job = client.execute(
        "car",
        "move_for",
        args=[[dx_m, dy_m, dtheta_rad]],
        timeout=timeout,
        sync=True,
    )
    if job.get("status") != "succeeded" or job.get("error"):
        raise RuntimeError(f"chassis move_for failed: status={job.get('status')} error={job.get('error')}")


# Bypass ArmClient.grasp: it has a positional/keyword collision in _call_arm.
# Use RuntimeApiClient directly with a fresh action call.
def _grasp(client: RuntimeApiClient, on: bool, timeout: float = 10.0) -> None:
    job = client.execute(
        "arm", "grasp", args=[bool(on)],
        sync=True, timeout=timeout,
    )
    if job.get("status") != "succeeded" or job.get("error"):
        raise RuntimeError(f"grasp({on}) failed: status={job.get('status')} error={job.get('error')}")


# Bypass ArmRunner.move_x (no v_max_mms). Use ArmClient.move_x directly with explicit speed.
def _arm_move_x(client: RuntimeApiClient, x_mm: float, v_max_mms: float = 60.0, out_time: float = 15.0, timeout: float = 30.0) -> None:
    job = client.execute(
        "arm", "move_x_position",
        args=[x_mm / 1000.0],
        kwargs={"v_max_mms": v_max_mms, "out_time": out_time, "timeout": timeout},
        sync=True, timeout=timeout + 5,
    )
    if job.get("status") != "succeeded" or job.get("error"):
        raise RuntimeError(f"arm move_x({x_mm}) failed: status={job.get('status')} error={job.get('error')}")


def _arm_move_y(client: RuntimeApiClient, y_mm: float, timeout: float = 25.0) -> None:
    """Bypass ArmClient.move_y (which calls get_state sanity check -> y/x_get_position
    can time out after big arm rotations). Talk to the runtime action directly.
    move_y_position(target) only takes a single positional target (m), no speed kwargs.
    """
    job = client.execute(
        "arm", "move_y_position",
        args=[y_mm / 1000.0],
        sync=True, timeout=timeout,
    )
    if job.get("status") != "succeeded" or job.get("error"):
        raise RuntimeError(f"arm move_y({y_mm}) failed: status={job.get('status')} error={job.get('error')}")


def _set_arm_angle(client: RuntimeApiClient, angle_deg: float, speed: int = 80, timeout: float = 20.0, retries: int = 2) -> None:
    """set_arm_angle with retry. Runtime can occasionally be slow to respond right
    after busy operations (chassis move, big arm rotations); a transient 8s timeout
    is recoverable.
    """
    import time as _t
    last = None
    for attempt in range(1, retries + 1):
        try:
            job = client.execute(
                "arm", "set_arm_angle",
                args=[angle_deg, speed],
                sync=True, timeout=timeout + 5,
            )
            if job.get("status") == "succeeded" and not job.get("error"):
                return
            last = f"status={job.get('status')} error={job.get('error')}"
        except Exception as exc:
            last = f"{type(exc).__name__}: {exc}"[:200]
        logger.warning("set_arm_angle(%.0f) attempt %d/%d failed: %s", angle_deg, attempt, retries, last)
        if attempt < retries:
            _t.sleep(1.0)
    raise RuntimeError(f"set_arm_angle({angle_deg}) failed after {retries} retries: {last}")


def _pick_one(runner: ArmRunner, client: RuntimeApiClient, cfg: Dict[str, Any]) -> str:
    pick = cfg["arm_pick_pose"]
    carry = cfg["arm_carry_pose"]
    _arm_move_y(client, pick["y_mm"])
    _grasp(client, True)
    time.sleep(cfg["vacuum_settle_s"])
    _arm_move_y(client, carry["y_mm"])
    valid = list(cfg["target_slot_map"].keys())
    label = _scan_labels(client, valid)
    if label is None:
        raise RuntimeError(f"cam2 not detecting any of {valid}")
    return label


def _transport_to_slot(
    runner: ArmRunner,
    client: RuntimeApiClient,
    cfg: Dict[str, Any],
    slot: int,
    source_idx: int,
) -> None:
    spacing = cfg["spacing_along_row_m"]
    timeout = cfg["chassis_move_timeout_s"]
    v_max = cfg["v_max_arm_lateral_mms"]
    place = cfg["arm_place_pose_T2"]

    # ????? = source_idx ???? (??????? S_{source_idx})
    source_to_pos = {1: 0.0, 2: +spacing, 3: +2 * spacing}
    current = source_to_pos[source_idx]
    # T_slot ????
    slot_to_pos = {1: -spacing, 2: 0.0, 3: +spacing}
    target = slot_to_pos[slot]
    move_m = target - current
    if abs(move_m) > 1e-3:
        logger.info("chassis transport: from S%d (pos=%.2f) to T%d (pos=%.2f), dx=%.3f m", source_idx, current, slot, target, move_m)
        _chassis_move_for(client, dx_m=move_m, timeout=timeout)

    _arm_move_x(client, place["x_mm"], v_max_mms=v_max)
    _set_arm_angle(client, place["arm_angle_deg"])


def _place_and_return(
    runner: ArmRunner,
    client: RuntimeApiClient,
    cfg: Dict[str, Any],
    next_chassis_offset_m: float,
) -> None:
    place = cfg["arm_place_pose_T2"]
    ret = cfg["arm_return_S1_pose"]
    carry_y = cfg["arm_carry_pose"]["y_mm"]
    timeout = cfg["chassis_move_timeout_s"]
    v_max = cfg["v_max_arm_lateral_mms"]

    _grasp(client, False)
    runner.move_y(carry_y, verify=False)
    _arm_move_x(client, ret["x_mm"], v_max_mms=v_max)
    _set_arm_angle(client, ret["arm_angle_deg"])
    if abs(next_chassis_offset_m) > 1e-3:
        _chassis_move_for(client, dx_m=next_chassis_offset_m, timeout=timeout)


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    cfg = load_task_config("auto_seeding")
    if cfg.get("placeholder"):
        raise NotImplementedError("auto_seeding not yet implemented")

    if client is None:
        client = RuntimeApiClient()
    _ensure_runtime(client)

    arm_client = ArmClient.connect()
    if not arm_client.ping():
        raise RuntimeError("arm runtime not online")
    runner = ArmRunner(arm_client)

    completed: List[str] = []
    spacing = cfg["spacing_along_row_m"]
    init_y_mm = cfg.get("init_y_mm", -100)

    # ===== -1. ???????? =====
    _wait_infer_ready(client, timeout_s=30.0)
    # ===== 0. ????? =====
    logger.info("init: lift arm to Y=%s mm", init_y_mm)
    _arm_move_y(client, init_y_mm)

    pick = cfg["arm_pick_pose"]
    carry = cfg["arm_carry_pose"]
    place = cfg["arm_place_pose_T2"]
    ret = cfg["arm_return_S1_pose"]
    valid_labels = list(cfg["target_slot_map"].keys())

    # 0.1 ? S1 ????: X=-60, ??=-150, ??=-10, Y ??? -100
    _set_arm_angle(client, pick["arm_angle_deg"])
    runner.client.set_hand_angle(pick["hand_angle_deg"], speed=80, timeout=10.0)
    _arm_move_x(client, pick["x_mm"], v_max_mms=60.0)

    try:
        for i, source_idx in enumerate(cfg["source_position_order"]):
            logger.info("=== processing S%d (iteration %d) ===", source_idx, i + 1)
            # ===== 0. ????? S1 ???? =====
            #   y ?? -100, arm ?? -150, x ?? -60
            #   ????? return ??? y_protect ??, ????????
            logger.info("reset to S1 detection pose: y=-100 arm=-150 x=-60")
            _arm_move_y(client, init_y_mm)
            _set_arm_angle(client, pick["arm_angle_deg"])
            runner.client.set_hand_angle(pick["hand_angle_deg"], speed=80, timeout=10.0)
            _arm_move_x(client, pick["x_mm"], v_max_mms=60.0)

            # ===== 1. ????(? -100 mm ???) =====
            label = _scan_labels(client, valid_labels)
            if label is None:
                raise RuntimeError(f"cam2 ? S{source_idx} ???? {valid_labels}")
            slot = cfg["target_slot_map"][label]
            logger.info("S%d detect %s -> T%d", source_idx, label, slot)

            # ===== 2. ?: Y ? -100 ??? -25, grasp on, 0.5s, ?? -150 =====
            _arm_move_y(client, pick["y_mm"])
            _grasp(client, True)
            time.sleep(cfg["vacuum_settle_s"])
            _arm_move_y(client, carry["y_mm"])

            # ===== 3. ??? T_slot =====
            _transport_to_slot(runner, client, cfg, slot, source_idx)

            # ===== 4. ?: Y ? -150 ??? -25, grasp off, ?? -150 =====
            _arm_move_y(client, place["y_mm"])
            _grasp(client, False)
            _arm_move_y(client, carry["y_mm"])

            # ===== 5. ? S1 ????(??????) =====
            # ??: ? X ?? -60 (arm ?? 0?), ?? arm -150
            # ??: arm 0? ?? -150 ?, ??????????
            #   ???? arm, ????? -260; ?? X ????
            #   ?? X ??, ?? arm, ?????? S1 ???
            _arm_move_x(client, ret["x_mm"], v_max_mms=60.0)
            _set_arm_angle(client, ret["arm_angle_deg"])
            logger.info("S1 pose ready (x=-60, arm=-150), proceeding to chassis offset")
            # ??(?? S1 ??,?? m):
            #   T1 = -spacing, T2 = 0, T3 = +spacing
            #   S1 = 0, S2 = +spacing, S3 = +2*spacing
            slot_to_chassis = {1: -spacing, 2: 0.0, 3: +spacing}
            current_chassis = slot_to_chassis[slot]
            if i + 1 < len(cfg["source_position_order"]):
                next_source_idx = cfg["source_position_order"][i + 1]
                source_to_chassis = {1: 0.0, 2: +spacing, 3: +2 * spacing}
                next_chassis = source_to_chassis[next_source_idx]
                next_offset_m = next_chassis - current_chassis
                if abs(next_offset_m) > 1e-3:
                    logger.info("chassis: from slot=%d (pos=%.2f) to next source S%d (pos=%.2f), dx=%.3f m", slot, current_chassis, next_source_idx, next_chassis, next_offset_m)
                    _chassis_move_for(client, dx_m=next_offset_m, timeout=cfg["chassis_move_timeout_s"])
            else:
                # ????,??? T1/T2/T3 ???;???? T1/T3,??? S1
                if abs(current_chassis) > 1e-3:
                    logger.info("chassis: end of task, return to S1 from pos=%.2f", current_chassis)
                    _chassis_move_for(client, dx_m=-current_chassis, timeout=cfg["chassis_move_timeout_s"])
            completed.append(label)
    except Exception as exc:
        logger.exception("auto_seeding failed: %s", exc)
        return {"ok": False, "completed": completed, "error": str(exc)}

    return {"ok": True, "completed": completed}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
    result = run()
    print("auto_seeding result:", result)

