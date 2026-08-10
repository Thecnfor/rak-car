#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Runtime 层 task1 核心逻辑 — 进程内直调，跳过网络栈。

完全复刻 main/task/task1_seeding.py 的业务逻辑，所有 arm/car 调用直接走 SDK 方法。

调用方式：
  POST http://<JETSON>:5050/v1/execute
  {"target": "car", "name": "run_task1", "kwargs": {}}
"""
from __future__ import annotations

import logging
import math
import os
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional, Tuple

from smartcar.whalesbot.tools import get_yaml

logger = logging.getLogger("runtime.task1")

# ══════════════════════════════════════════════════════════════════════════════
# 快速调参区（与 main/task/task1_seeding.py 完全同步）
# ══════════════════════════════════════════════════════════════════════════════

TASK1_NOZZLE_OFFSET_MAP: Dict[str, Tuple[float, float]] = {
    "cylinder_1": (0.050, -0.425),
    "cylinder_2": (0.140, -0.420),
    "cylinder_3": (0.120, -0.410),
}

PICK_SERVO_GAIN_ARM = 0.5
PICK_SERVO_GAIN_X = 0.30
PICK_SERVO_DEADZONE = 0.05
PICK_SERVO_MAX_VEL = 0.3
PICK_SERVO_SETTLE_HITS = 3
PICK_SERVO_HOLD_S = 0.05
PICK_SERVO_LIFT_BACK = True
PICK_SERVO_SKIP_POSE_ALIGN = True
PICK_SERVO_HZ = 20.0
PICK_SERVO_TIMEOUT_S_DEFAULT = 4.0
PICK_SERVO_RETRY_TIMEOUT_EXTRA_S = 3.0
PICK_SERVO_RETRY_DEADZONE = 0.10

PICK_START_HAND_DEG = -15.0

S_POSE_Y_MM = -100.0
S_POSE_X_MM = -70.0
S_POSE_HAND_DEG = 0.0

PLACE_ARM_DEG = 90.0
PLACE_HAND_DEG = 0.0
PLACE_Y_MM = -100.0
PLACE_X_MM_FALLBACK = -235.0
PLACE_ALIGN_X_MM = -235.0
PLACE_ALIGN_SETPOINT_CXCY = (0.072, -0.331)
PLACE_ALIGN_TIMEOUT_S = 5.0
PLACE_ALIGN_SERVO_ARM_MIN = 30.0
PLACE_ALIGN_SERVO_ARM_MAX = 150.0
PLACE_ALIGN_SERVO_SIGN_ARM = 1.0
PLACE_ALIGN_SERVO_SIGN_X = -1.0

PLACE_DESCEND_MM = -20.0
PLACE_LIFT_MM = -40.0

CHASSIS_CONCURRENT_Y_THRESHOLD_MM = -30.0
CHASSIS_MOVE_MAX_VEL_MPS = 0.1

COMPOSITE_SPEED_DEFAULT = 100
COMPOSITE_TIMEOUT_S_DEFAULT = 5.0

TRIGGER_SETTLE_LANE_M = 0.1
TRIGGER_SETTLE_LANE_VX = 0.1

SOURCE_POSITIONS_M = {1: 0.0, 2: 0.15, 3: 0.30}
SLOT_POSITIONS_M = {1: 0.0, 2: 0.15, 3: 0.30}
SOURCE_LABELS: tuple = ("cylinder_1", "cylinder_2", "cylinder_3")


# ══════════════════════════════════════════════════════════════════════════════
# 内部工具函数
# ══════════════════════════════════════════════════════════════════════════════

def _load_task_config() -> Dict[str, Any]:
    """加载 task_config.yml 中 auto_seeding 段。"""
    dir_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    yaml_path = os.path.join(dir_root, "task_config.yml")
    cfg = get_yaml(yaml_path)
    auto_seeding = cfg.get("task_cfg", {}).get("auto_seeding", {})
    if auto_seeding.get("placeholder"):
        raise NotImplementedError("任务 auto_seeding 配置尚未完成")
    return auto_seeding


def _read_odom_via_http(car) -> Tuple[float, float, float]:
    """通过 HTTP /v1/realtime/odom/state 读取 odom。"""
    # 尝试通过 car 的 runtime service 接口读
    # car 有 self._service 属性（CarRuntimeService）
    service = getattr(car, "_service", None)
    if service is not None:
        try:
            # CarRuntimeService 有 get_odom_feed_state 或类似方法
            feed = getattr(service, "_odom_feed_state", None)
            if feed is not None:
                odom = feed.get("odom", {}) or {}
                theta = odom.get("theta")
                return (
                    float(odom.get("x", 0.0)),
                    float(odom.get("y", 0.0)),
                    float(theta) if theta is not None else 0.0,
                )
        except Exception:
            pass
    # 直接读 car.get_odometry()
    try:
        odom = car.get_odometry() or {}
        theta = odom.get("theta")
        return (
            float(odom.get("x", 0.0)),
            float(odom.get("y", 0.0)),
            float(theta) if theta is not None else 0.0,
        )
    except Exception:
        return 0.0, 0.0, 0.0


def _read_arm_state(car) -> Dict[str, float]:
    """读 arm_state。"""
    try:
        state = car.get_arm_state() or {}
        return {
            "x_mm": float(state.get("x", 0.0)) * 1000.0,
            "y_mm": float(state.get("y", 0.0)) * 1000.0,
            "arm_angle": float(state.get("arm_angle", 0.0)),
            "hand_angle": float(state.get("hand_angle", 0.0)),
        }
    except Exception:
        return {"x_mm": 0.0, "y_mm": 0.0, "arm_angle": 0.0, "hand_angle": 0.0}


def _read_task_detections(streamer) -> List[Dict[str, Any]]:
    """读 task_feed 缓存。"""
    try:
        state = streamer.get_task_state()
        if isinstance(state, dict) and state.get("active"):
            return state.get("detections") or []
    except Exception:
        pass
    return []


def _scan_cylinder_label(
    streamer,
    valid_labels: List[str],
    setpoint_xy: Optional[Tuple[float, float]] = None,
) -> Optional[str]:
    """扫描 cylinder label。"""
    dets = _read_task_detections(streamer)
    if not dets:
        return None
    if not setpoint_xy:
        for d in dets:
            lab = (d or {}).get("label", "")
            if lab in valid_labels:
                return lab
        return None
    sx, sy = setpoint_xy
    best_label, best_d2 = None, float("inf")
    for d in dets:
        lab = (d or {}).get("label", "")
        if lab not in valid_labels:
            continue
        bb = (d or {}).get("bbox_norm") or {}
        try:
            cx = float(bb.get("cx") if "cx" in bb else bb.get("x_center", 0.0))
            cy = float(bb.get("cy") if "cy" in bb else bb.get("y_center", 0.0))
        except Exception:
            continue
        d2 = (cx - sx) ** 2 + (cy - sy) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_label = lab
    return best_label


def _scan_marker_present(streamer, marker_label: str) -> bool:
    """检查 marker 是否可见。"""
    dets = _read_task_detections(streamer)
    for d in dets:
        if (d or {}).get("label", "") == marker_label:
            return True
    return False


def _switch_to_place_pose(
    car, arm,
    x_mm: float, arm_deg: float, hand_deg: float,
) -> bool:
    """切到 PLACE 姿态。"""
    state = _read_arm_state(car)
    if state["y_mm"] > -50:
        arm.move_y_position(float(PLACE_Y_MM) / 1000.0)
    logger.info("  切 PLACE 姿态: arm=%.1f° x=%.1fmm y=%.1fmm hand=%.1f°",
                arm_deg, x_mm, PLACE_Y_MM, hand_deg)
    result = arm.composite_run(
        arm=float(arm_deg),
        x=float(x_mm) / 1000.0,
        y=float(PLACE_Y_MM) / 1000.0,
        hand=float(hand_deg),
        speed=COMPOSITE_SPEED_DEFAULT,
        timeout=COMPOSITE_TIMEOUT_S_DEFAULT,
    )
    return result.get("ok", False) if isinstance(result, dict) else bool(result)


def _switch_to_s_pose(arm, arm_deg: float, x_mm: float, hand_deg: float, y_mm: float) -> bool:
    """切到 S 姿态。"""
    result = arm.composite_run(
        arm=float(arm_deg),
        x=float(x_mm) / 1000.0,
        y=float(y_mm) / 1000.0,
        hand=float(hand_deg),
        speed=COMPOSITE_SPEED_DEFAULT,
        timeout=COMPOSITE_TIMEOUT_S_DEFAULT,
    )
    return result.get("ok", False) if isinstance(result, dict) else bool(result)


def _return_to_source_pose(arm, cfg: Dict[str, Any]) -> None:
    """防碰撞归位。"""
    ret = cfg.get("arm_return_S1_pose", {})
    arm.composite_run(
        arm=float(ret.get("arm_angle_deg", -90.0)),
        x=float(ret.get("x_mm", -100.0)) / 1000.0,
        y=float(ret.get("y_mm", -100.0)) / 1000.0,
        hand=float(ret.get("hand_angle_deg", -10.0)),
        speed=80, timeout=30.0,
    )


def _init_step0_trigger_lane_arm(car, arm, cfg: Dict[str, Any]) -> bool:
    """任务点触发后 lane follow 前移 + 臂切 PLACE 并发。"""
    settle = cfg.get("trigger_settle") or {}
    if not settle.get("enabled", False):
        return True
    lane_m = float(settle.get("lane_follow_m", TRIGGER_SETTLE_LANE_M))
    lane_vx = float(settle.get("lane_speed_mps", TRIGGER_SETTLE_LANE_VX))
    if lane_m <= 0:
        return True

    state = _read_arm_state(car)
    if state["y_mm"] > -50:
        logger.info("step0: 当前 y=%.1f 偏低, 先抬到 %s", state["y_mm"], PLACE_Y_MM)
        arm.move_y_position(float(PLACE_Y_MM) / 1000.0)

    logger.info("step0: lane follow %.2fm @ %.2fm/s 并发臂切 PLACE", lane_m, lane_vx)
    lane_done = True
    arm_ok = False
    with ThreadPoolExecutor(max_workers=2) as ex:
        f_lane = ex.submit(car.lane_dis, lane_m, lane_vx)
        f_arm = ex.submit(
            arm.composite_run,
            arm=PLACE_ARM_DEG,
            x=float(PLACE_ALIGN_X_MM) / 1000.0,
            y=float(PLACE_Y_MM) / 1000.0,
            hand=PLACE_HAND_DEG,
            speed=COMPOSITE_SPEED_DEFAULT,
            timeout=20.0,
        )
        try:
            f_lane.result()
        except Exception as exc:
            lane_done = False
            logger.warning("step0: lane follow 失败 (%s), 继续", exc)
        try:
            arm_ok = bool(f_arm.result())
        except Exception as exc:
            logger.warning("step0: 臂切 PLACE 失败 (%s), 继续", exc)
    return lane_done and arm_ok


def _init_step1_place_align_arm(car, arm, cfg: Dict[str, Any]) -> Dict[str, Any]:
    """PLACE 机械臂视觉对齐。"""
    pa = cfg.get("place_align") or {}
    if not pa.get("enabled", False):
        return {"ok": False, "arm_deg": None, "x_mm": None}

    label = str(pa.get("label", "cylinder_set"))
    sp = tuple(float(v) for v in pa.get("setpoint_cxcy", PLACE_ALIGN_SETPOINT_CXCY))
    init_x_mm = float(pa.get("init_x_mm", PLACE_ALIGN_X_MM))
    init_arm_deg = float(pa.get("init_arm_deg", PLACE_ARM_DEG))
    init_hand_deg = float(pa.get("init_hand_deg", PLACE_HAND_DEG))

    try:
        _switch_to_place_pose(car, arm, init_x_mm, init_arm_deg, init_hand_deg)
    except Exception as exc:
        logger.error("step 1: 切 place_align 初始姿态失败 (%s), 回落", exc)
        return {"ok": False, "arm_deg": None, "x_mm": None}

    logger.info("step 1: PLACE 对齐 cylinder_set — run_arm_servo(label=%s setpoint=(%.3f,%.3f) "
                "arm_start=%.0f x_init=%.0f)", label, sp[0], sp[1], init_arm_deg, init_x_mm)

    servo_result = arm.run_arm_servo(
        label=label,
        hz=float(pa.get("hz", PICK_SERVO_HZ)),
        gain_arm=float(pa.get("gain_arm", PICK_SERVO_GAIN_ARM)),
        gain_x=float(pa.get("gain_x", PICK_SERVO_GAIN_X)),
        deadzone=float(pa.get("deadzone", PICK_SERVO_DEADZONE)),
        max_vel=float(pa.get("max_vel", PICK_SERVO_MAX_VEL)),
        arm_start=init_arm_deg,
        sign_arm=float(pa.get("sign_arm", PLACE_ALIGN_SERVO_SIGN_ARM)),
        sign_x=float(pa.get("sign_x", PLACE_ALIGN_SERVO_SIGN_X)),
        setpoint_x_norm=sp[0],
        setpoint_y_norm=sp[1],
        arm_min=float(pa.get("arm_min", PLACE_ALIGN_SERVO_ARM_MIN)),
        arm_max=float(pa.get("arm_max", PLACE_ALIGN_SERVO_ARM_MAX)),
        servo_timeout=float(pa.get("timeout", PLACE_ALIGN_TIMEOUT_S)),
        settle_hits=int(pa.get("settle_hits", PICK_SERVO_SETTLE_HITS)),
    )

    logger.info("place 对齐结果: reason=%s settled=%s trace_hits=%s end_arm=%s end_x=%s",
               servo_result.get("reason"), servo_result.get("settled"),
               servo_result.get("trace_hits"), servo_result.get("end_arm"), servo_result.get("end_x"))

    # 重试
    if (not servo_result.get("settled")
            and int(servo_result.get("trace_hits", 0)) > 0):
        retry_arm_start = float(servo_result.get("end_arm", init_arm_deg))
        logger.warning("place 对齐未收敛但看得到目标 → 重试")
        servo_result = arm.run_arm_servo(
            label=label,
            hz=float(pa.get("hz", PICK_SERVO_HZ)),
            gain_arm=float(pa.get("gain_arm", PICK_SERVO_GAIN_ARM)),
            gain_x=float(pa.get("gain_x", PICK_SERVO_GAIN_X)),
            deadzone=PICK_SERVO_RETRY_DEADZONE,
            max_vel=float(pa.get("max_vel", PICK_SERVO_MAX_VEL)),
            arm_start=retry_arm_start,
            sign_arm=float(pa.get("sign_arm", PLACE_ALIGN_SERVO_SIGN_ARM)),
            sign_x=float(pa.get("sign_x", PLACE_ALIGN_SERVO_SIGN_X)),
            setpoint_x_norm=sp[0],
            setpoint_y_norm=sp[1],
            arm_min=float(pa.get("arm_min", PLACE_ALIGN_SERVO_ARM_MIN)),
            arm_max=float(pa.get("arm_max", PLACE_ALIGN_SERVO_ARM_MAX)),
            servo_timeout=float(pa.get("timeout", PLACE_ALIGN_TIMEOUT_S)) + PICK_SERVO_RETRY_TIMEOUT_EXTRA_S,
            settle_hits=int(pa.get("settle_hits", PICK_SERVO_SETTLE_HITS)),
        )
        logger.info("place 对齐重试结果: reason=%s settled=%s end_arm=%s end_x=%s",
                    servo_result.get("reason"), servo_result.get("settled"),
                    servo_result.get("end_arm"), servo_result.get("end_x"))

    arm_deg = float(servo_result["end_arm"]) if servo_result.get("end_arm") is not None else None
    x_mm = float(servo_result["end_x"]) * 1000.0 if servo_result.get("end_x") is not None else None
    ok = bool(servo_result.get("settled")) and arm_deg is not None and x_mm is not None

    if ok:
        logger.info("step 1: 记住放苗姿态 arm=%.1f° x=%.1fmm", arm_deg, x_mm)
    else:
        logger.error("step 1: place 对齐未收敛 (reason=%s), 回落写死姿态", servo_result.get("reason"))

    return {"ok": ok, "arm_deg": arm_deg, "x_mm": x_mm,
            "reason": servo_result.get("reason")}


def _init_step2_s_pose(arm, cfg: Dict[str, Any], init_y_mm: float) -> None:
    """初始化 S 姿态。"""
    state = _read_arm_state(arm)
    if state["y_mm"] > -50:
        logger.info("init step2: 当前 y=%.1f 太低, 先抬到 %s", state["y_mm"], S_POSE_Y_MM)
        arm.move_y_position(float(S_POSE_Y_MM) / 1000.0)
    pick = cfg.get("arm_pick_pose", {})
    logger.info("init step2: S 姿态 arm=%s° hand=%s° X=%smm Y=%smm",
                pick.get("arm_angle_deg"), PICK_START_HAND_DEG,
                pick.get("x_mm"), S_POSE_Y_MM)
    arm.composite_run(
        arm=float(pick.get("arm_angle_deg", -90.0)),
        x=float(pick.get("x_mm", -70.0)) / 1000.0,
        y=float(init_y_mm) / 1000.0,
        hand=PICK_START_HAND_DEG,
        speed=COMPOSITE_SPEED_DEFAULT,
        timeout=20.0,
    )


def _chassis_goto(car, target_along_m: float, pos_along: List[float],
                   chassis_move_timeout: float) -> None:
    """沿车头方向移动。"""
    dx = target_along_m - pos_along[0]
    if abs(dx) < 0.05:
        logger.info("  底盘已在相对 %.3f m, 跳过移动", target_along_m)
        return
    logger.info("  底盘: move_for(dx=%+.3f) → %.3f m", dx, target_along_m)
    try:
        car.move_for([dx, 0.0, 0.0],
                     max_velocities=[CHASSIS_MOVE_MAX_VEL_MPS,
                                     CHASSIS_MOVE_MAX_VEL_MPS,
                                     math.pi / 3.0])
        pos_along[0] = target_along_m
    except Exception as exc:
        logger.warning("  底盘 move_for 失败: %s", exc)
        raise


def _pick_at_source(
    car, arm, streamer,
    cfg: Dict[str, Any],
    column_idx: int,
    seen_state: Dict[str, Any],
) -> str:
    """扫描 label → 视觉伺服抓取 → 返回 label。"""
    time.sleep(1.0)

    setpoint_xy = next(iter(TASK1_NOZZLE_OFFSET_MAP.values()))
    logger.info("[S%d] 视觉扫描源头 cylinder label", column_idx)
    label = _scan_cylinder_label(streamer, list(SOURCE_LABELS), setpoint_xy=setpoint_xy)
    if label is None:
        raise RuntimeError(f"S{column_idx} 未检测到任何 cylinder")

    first = seen_state.get("first")
    if first is not None:
        if label == first and first in ("cylinder_1", "cylinder_3"):
            corrected = "cylinder_3" if first == "cylinder_1" else "cylinder_1"
            logger.info("  label 纠错: %s → %s", label, corrected)
            label = corrected
    else:
        seen_state["first"] = label
        logger.info("  首次识别: %s", label)

    logger.info("  -> 抓取 %s，视觉伺服对准", label)

    init_y_mm = float(cfg.get("init_y_mm", -180))
    pick_arm_start = float(cfg.get("arm_pick_pose", {}).get("arm_angle_deg", -90.0))
    nozzle_xy = TASK1_NOZZLE_OFFSET_MAP.get(label, setpoint_xy)

    vision_cfg = cfg.get("pick_vision") or {}
    servo_timeout = float(vision_cfg.get("timeout", cfg.get("pick_track_timeout_s", PICK_SERVO_TIMEOUT_S_DEFAULT)))

    servo_result = arm.run_arm_servo(
        label=label,
        hz=PICK_SERVO_HZ,
        gain_arm=PICK_SERVO_GAIN_ARM,
        gain_x=PICK_SERVO_GAIN_X,
        deadzone=PICK_SERVO_DEADZONE,
        max_vel=PICK_SERVO_MAX_VEL,
        arm_start=pick_arm_start,
        sign_arm=1.0,
        sign_x=-1.0,
        setpoint_x_norm=float(nozzle_xy[0]),
        setpoint_y_norm=float(nozzle_xy[1]),
        arm_min=-150.0,
        arm_max=90.0,
        servo_timeout=servo_timeout,
        settle_hits=PICK_SERVO_SETTLE_HITS,
    )

    if not servo_result.get("settled") and int(servo_result.get("trace_hits", 0)) > 0:
        retry_arm_start = float(servo_result.get("end_arm", pick_arm_start))
        logger.warning("S 视觉抓苗未收敛但看得到目标 → 重试")
        servo_result = arm.run_arm_servo(
            label=label,
            hz=PICK_SERVO_HZ,
            gain_arm=PICK_SERVO_GAIN_ARM,
            gain_x=PICK_SERVO_GAIN_X,
            deadzone=PICK_SERVO_RETRY_DEADZONE,
            max_vel=PICK_SERVO_MAX_VEL,
            arm_start=retry_arm_start,
            sign_arm=1.0,
            sign_x=-1.0,
            setpoint_x_norm=float(nozzle_xy[0]),
            setpoint_y_norm=float(nozzle_xy[1]),
            arm_min=-150.0,
            arm_max=90.0,
            servo_timeout=servo_timeout + PICK_SERVO_RETRY_TIMEOUT_EXTRA_S,
            settle_hits=PICK_SERVO_SETTLE_HITS,
        )

    if not servo_result.get("settled"):
        raise RuntimeError(
            f"S{column_idx} 视觉抓取未收敛 "
            f"(reason={servo_result.get('reason')}, "
            f"trace_hits={servo_result.get('trace_hits')})"
        )

    # 对齐完成 → y 降 0 → grasp → 抬回
    logger.info("  视觉对齐完成，执行抓取")
    try:
        arm.composite_run(y=0.0, speed=100, timeout=5.0)
        arm.grasp(True)
        arm.composite_run(y=float(init_y_mm) / 1000.0, speed=100, timeout=5.0)
    except Exception as exc:
        raise RuntimeError(f"抓取动作失败: {exc}") from exc

    return label


# ══════════════════════════════════════════════════════════════════════════════
# 主入口
# ══════════════════════════════════════════════════════════════════════════════

def run_task1(car) -> dict:
    """Task1 主入口：自动移苗（S1/S2/S3 → T1/T2/T3）。

    直接在 runtime 进程内执行，完全复刻 main/task/task1_seeding.py。
    """
    arm = getattr(car, "arm", None)
    streamer = getattr(car, "streamer", None)
    if arm is None:
        return {"ok": False, "completed": [], "error": "arm 未初始化"}
    if streamer is None:
        return {"ok": False, "completed": [], "error": "streamer 未初始化"}

    try:
        cfg = _load_task_config()
    except NotImplementedError as exc:
        return {"ok": False, "completed": [], "error": str(exc)}
    except Exception as exc:
        logger.error("加载配置失败: %s", exc)
        return {"ok": False, "completed": [], "error": f"配置加载失败: {exc}"}

    completed: List[str] = []
    seen_state: Dict[str, Any] = {}
    pos_along = [0.0]

    init_y_mm = float(cfg.get("init_y_mm", -180))
    source_position_order = cfg.get("source_position_order", [1, 2, 3])
    target_slot_map = cfg.get("target_slot_map", {})
    chassis_move_timeout = cfg.get("chassis_move_timeout_s", 30)

    s_arm = float(cfg.get("arm_pick_pose", {}).get("arm_angle_deg", -90.0))
    s_x_mm = float(cfg.get("arm_pick_pose", {}).get("x_mm", -70.0))

    place_arm_deg = PLACE_ARM_DEG
    place_x_mm = PLACE_X_MM_FALLBACK

    # ===== step0: lane follow 前移 + 臂切 PLACE 并发 =====
    _init_step0_trigger_lane_arm(car, arm, cfg)

    # ===== step1: place 对齐 =====
    align_arrived = False
    if cfg.get("place_align", {}).get("enabled", False):
        place_mem = _init_step1_place_align_arm(car, arm, cfg)
        align_arrived = place_mem.get("ok", False)
        if align_arrived:
            place_arm_deg = float(place_mem["arm_deg"])
            place_x_mm = float(place_mem["x_mm"])
            logger.info("  放苗姿态用记忆值: arm=%.1f° x=%.1fmm", place_arm_deg, place_x_mm)
        else:
            logger.info("  放苗姿态用兜底: arm=%.1f° x=%.1fmm", place_arm_deg, place_x_mm)
    else:
        logger.info("step 1: PLACE 机械臂对齐已禁用")

    # ===== step2: 切 S 姿态（主循环前先到 S） =====
    _init_step2_s_pose(arm, cfg, init_y_mm)

    try:
        for i, column_idx in enumerate(source_position_order):
            curr_x, _, curr_theta = _read_odom_via_http(car)
            logger.info("=== 处理底盘列 %d (S%d, odom x=%.3f theta=%.3f) ===",
                        i + 1, column_idx, curr_x, curr_theta)

            # (1) 底盘移到本列源
            if i > 0:
                target_s = SOURCE_POSITIONS_M.get(column_idx, 0.0)
                logger.info("  底盘 → S%d (相对 %.3f m)", column_idx, target_s)
                _chassis_goto(car, target_s, pos_along, chassis_move_timeout)
            else:
                logger.info("  已在 S1 列，跳过底盘移动")

            # (1.5) 切 S 姿态（先回到 S 姿态再抓取）
            logger.info("  切 S 姿态: arm=%.1f° x=%.1fmm y=%.1fmm hand=%.1f°",
                        s_arm, S_POSE_X_MM, init_y_mm, PICK_START_HAND_DEG)
            _switch_to_s_pose(arm, s_arm, S_POSE_X_MM, PICK_START_HAND_DEG, init_y_mm)

            # (2) 抓取
            try:
                label = _pick_at_source(car, arm, streamer, cfg, column_idx, seen_state)
            except Exception as exc:
                picked_so_far = set(completed)
                remaining = [l for l in SOURCE_LABELS if l not in picked_so_far]
                if remaining:
                    label = remaining[0]
                    logger.warning("  S%d 抓取失败 (%s)，兜底用 label=%s", column_idx, exc, label)
                else:
                    logger.warning("  S%d 抓取失败 (%s)，无剩余 label，跳过", column_idx, exc)
                    continue
            completed.append(label)

            # (3) 并发：底盘移到 T + 臂切 PLACE
            slot_idx = int(target_slot_map.get(label, 1))
            target_t = SLOT_POSITIONS_M.get(slot_idx, 0.0)

            st = _read_arm_state(car)
            if st["y_mm"] > CHASSIS_CONCURRENT_Y_THRESHOLD_MM:
                arm.composite_run(y=float(S_POSE_Y_MM) / 1000.0, speed=100, timeout=5.0)

            logger.info("  → T%d (label=%s) 全并发", slot_idx, label)
            with ThreadPoolExecutor(max_workers=2) as ex:
                f_chassis = ex.submit(_chassis_goto, car, target_t, pos_along, chassis_move_timeout)
                f_arm = ex.submit(
                    arm.composite_run,
                    arm=place_arm_deg,
                    x=float(place_x_mm) / 1000.0,
                    y=float(PLACE_Y_MM) / 1000.0,
                    hand=PLACE_HAND_DEG,
                    speed=COMPOSITE_SPEED_DEFAULT,
                    timeout=COMPOSITE_TIMEOUT_S_DEFAULT,
                )
                f_chassis.result()
                f_arm.result()

            # (4) 放苗：y→-20 + grasp(False) + y→-40 抬离
            logger.info("[T%d] place: y→%d + grasp(False) + y→%d",
                        slot_idx, PLACE_DESCEND_MM, PLACE_LIFT_MM)
            arm.move_y_position(float(PLACE_DESCEND_MM) / 1000.0)
            arm.grasp(False)
            arm.move_y_position(float(PLACE_LIFT_MM) / 1000.0)

            # (5) 归位 + 准备下一轮
            if i + 1 < len(source_position_order):
                next_col_idx = source_position_order[i + 1]
                logger.info("  列 %d 完成，底盘相对 %.3f m → 下一列 S%d",
                            column_idx, pos_along[0], next_col_idx)
                _return_to_source_pose(arm, cfg)

    except Exception as exc:
        logger.exception("task1 执行失败: %s", exc)
        try:
            s3_target = SOURCE_POSITIONS_M.get(3, 0.30)
            s3_dx = s3_target - pos_along[0]
            if abs(s3_dx) >= 0.05:
                car.move_for([s3_dx, 0.0, 0.0])
                pos_along[0] = s3_target
            logger.info("  异常路径已将底盘移到 S3")
        except Exception as move_exc:
            logger.warning("  异常路径移到 S3 失败: %s", move_exc)
        return {"ok": False, "completed": completed, "error": str(exc)}

    # 正常结束：移到 S3
    try:
        s3_target = SOURCE_POSITIONS_M.get(3, 0.30)
        s3_dx = s3_target - pos_along[0]
        if abs(s3_dx) >= 0.05:
            logger.info("task1 结束，底盘移到 S3 (dx=%+.3f m)", s3_dx)
            car.move_for([s3_dx, 0.0, 0.0])
            pos_along[0] = s3_target
        else:
            logger.info("task1 结束，底盘已在 S3")
    except Exception as exc:
        logger.warning("task1 末尾移到 S3 失败: %s", exc)

    return {
        "ok": True,
        "completed": completed,
        "chassis_aligned": align_arrived,
    }
