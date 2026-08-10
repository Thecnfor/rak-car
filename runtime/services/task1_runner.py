#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Runtime 层 task1 核心逻辑 — 进程内直调，跳过网络栈。

将 main/task/task1_seeding.py 的业务逻辑下沉到 runtime 进程内，
所有 arm/car 调用直接走 SDK 方法（arm.composite_run / car.move_for 等），
不经过 HTTP API，避免每帧网络往返延迟。

业务语义完全等价于 task1_seeding.py run() 函数：
  - 初始化机械臂（reset_x 撞墙校准 → 抬升 Y → 切 S 姿态）
  - 视觉扫描 cylinder label（1/2/3）
  - 视觉伺服抓取（run_arm_servo 闭环）
  - 并发运送到目标槽位
  - 视觉对齐释放
  - 循环处理 3 列

调用方式（不走网络栈）：
  car.run_task1()  # 默认阻塞，直到完成

或通过 /v1/execute：
  POST {"target": "car", "name": "run_task1", "kwargs": {...}}
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
# 快速调参区（与 main/task/task1_seeding.py 保持同步）
# ══════════════════════════════════════════════════════════════════════════════

# 吸嘴 setpoint（目标在吸嘴正下方时其 bbox 中心归一化坐标）
TASK1_NOZZLE_OFFSET_MAP: Dict[str, Tuple[float, float]] = {
    "cylinder_1": (0.050, -0.425),
    "cylinder_2": (0.140, -0.420),
    "cylinder_3": (0.120, -0.410),
}

# 视觉伺服参数（run_arm_servo）
PICK_SERVO_GAIN_ARM = 0.5
PICK_SERVO_GAIN_X = 0.30
PICK_SERVO_DEADZONE = 0.05
PICK_SERVO_MAX_VEL = 0.3
PICK_SERVO_SETTLE_HITS = 3
PICK_SERVO_HZ = 20.0
PICK_SERVO_TIMEOUT_S_DEFAULT = 4.0

# 对齐失败重试
PICK_SERVO_RETRY_TIMEOUT_EXTRA_S = 3.0
PICK_SERVO_RETRY_DEADZONE = 0.10

# 抓取起始 hand 角度
PICK_START_HAND_DEG = -15.0

# S 姿态
S_POSE_Y_MM = -100.0
S_POSE_X_MM = -70.0
S_POSE_HAND_DEG = 0.0

# PLACE 姿态
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

# 释放 y 轨迹
PLACE_DESCEND_MM = -20.0
PLACE_LIFT_MM = -40.0

# 底盘安全约束
CHASSIS_CONCURRENT_Y_THRESHOLD_MM = -30.0
CHASSIS_MOVE_MAX_VEL_MPS = 0.1

# composite_run 公共参数
COMPOSITE_SPEED_DEFAULT = 100
COMPOSITE_TIMEOUT_S_DEFAULT = 5.0

# 源/目标位置（m）
SOURCE_POSITIONS_M = {1: 0.0, 2: 0.15, 3: 0.30}
SLOT_POSITIONS_M = {1: 0.0, 2: 0.15, 3: 0.30}
SOURCE_LABELS: tuple = ("cylinder_1", "cylinder_2", "cylinder_3")


# ══════════════════════════════════════════════════════════════════════════════
# 内部工具函数
# ══════════════════════════════════════════════════════════════════════════════

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _load_task_config() -> Dict[str, Any]:
    """加载 task_config.yml 中 auto_seeding 段。"""
    # 向上两级找到 rak-car 根目录
    dir_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    yaml_path = os.path.join(dir_root, "task_config.yml")
    cfg = get_yaml(yaml_path)
    task_cfg = cfg.get("task_cfg", {})
    auto_seeding = task_cfg.get("auto_seeding", {})
    if auto_seeding.get("placeholder"):
        raise NotImplementedError("任务 auto_seeding 配置尚未完成")
    return auto_seeding


def _read_odom(car) -> Tuple[float, float, float]:
    """读 odom_state 拿 chassis 当前 x, y (m), theta (rad)。"""
    try:
        odom = car.get_odometry()
        theta = odom.get("theta")
        return (
            float(odom.get("x", 0.0)),
            float(odom.get("y", 0.0)),
            float(theta) if theta is not None else 0.0,
        )
    except Exception:
        return 0.0, 0.0, 0.0


def _read_arm_state(arm) -> Dict[str, float]:
    """读 arm_state 拿 x_mm, y_mm。"""
    try:
        state = arm.get_arm_state() or {}
        return {
            "x_mm": float(state.get("x", 0.0)) * 1000.0,
            "y_mm": float(state.get("y", 0.0)) * 1000.0,
            "arm_angle": float(state.get("arm_angle", 0.0)),
            "hand_angle": float(state.get("hand_angle", 0.0)),
        }
    except Exception:
        return {"x_mm": 0.0, "y_mm": 0.0, "arm_angle": 0.0, "hand_angle": 0.0}


def _read_task_detections(streamer) -> List[Dict[str, Any]]:
    """读 task_feed 缓存，返回 detections 列表。"""
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
    """通过 cam2 task_feed 缓存扫描 cylinder label。

    多 cylinder 同时可见时取离 setpoint_xy 最近的。
    """
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


def _switch_to_place_pose(arm, x_mm: float, arm_deg: float, hand_deg: float) -> bool:
    """切到 PLACE 姿态（y 抬升防撞保护区）。"""
    state = _read_arm_state(arm)
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
    """切到 S 姿态（抓取起始位）。"""
    result = arm.composite_run(
        arm=float(arm_deg),
        x=float(x_mm) / 1000.0,
        y=float(y_mm) / 1000.0,
        hand=float(hand_deg),
        speed=COMPOSITE_SPEED_DEFAULT,
        timeout=COMPOSITE_TIMEOUT_S_DEFAULT,
    )
    return result.get("ok", False) if isinstance(result, dict) else bool(result)


def _chassis_goto(car, target_along_m: float, pos_along: List[float],
                   chassis_move_timeout: float) -> None:
    """沿车头方向闭环移动到相对位移 target_along_m (m)。

    内部用 move_for([dx,0,0]) + 自记账，抵消 odom theta 漂移。
    """
    dx = target_along_m - pos_along[0]
    if abs(dx) < 0.05:
        logger.info("  底盘已在相对 %.3f m (|dx|=%.3f < 5cm)，跳过移动", target_along_m, abs(dx))
        return
    logger.info("  底盘闭环: move_for(dx=%+.3f) → 相对 %.3f m", dx, target_along_m)
    try:
        car.move_for([dx, 0.0, 0.0], max_velocities=[
            CHASSIS_MOVE_MAX_VEL_MPS, CHASSIS_MOVE_MAX_VEL_MPS, math.pi / 3.0
        ])
        pos_along[0] = target_along_m
    except Exception as exc:
        logger.warning("  底盘 move_for 失败: %s", exc)
        raise


def _place_cylinder(arm, slot_idx: int, place_arm_deg: float, place_x_mm: float) -> None:
    """放苗：y→-20 + grasp(False) + y→-40 抬离。"""
    logger.info("[T%d] place: y→%d + grasp(False) + y→%d",
                slot_idx, PLACE_DESCEND_MM, PLACE_LIFT_MM)

    # 5a) 下降到 -20
    arm.move_y_position(float(PLACE_DESCEND_MM) / 1000.0)

    # 5b) 释放
    arm.grasp(False)

    # 5c) 抬到 -40
    arm.move_y_position(float(PLACE_LIFT_MM) / 1000.0)


def _pick_at_source(
    arm,
    streamer,
    cfg: Dict[str, Any],
    column_idx: int,
    seen_state: Dict[str, Any],
) -> str:
    """第 i 列：扫描 label → 视觉伺服抓取 → 返回 label。"""
    time.sleep(1.0)  # 等振动稳定

    # 扫描 cylinder label
    setpoint_xy = next(iter(TASK1_NOZZLE_OFFSET_MAP.values()))
    logger.info("[S%d] 视觉扫描源头 cylinder label", column_idx)
    label = _scan_cylinder_label(streamer, list(SOURCE_LABELS), setpoint_xy=setpoint_xy)
    if label is None:
        raise RuntimeError(f"S{column_idx} 未检测到任何 cylinder")

    # 1↔3 纠错
    first = seen_state.get("first")
    if first is not None:
        if label == first and first in ("cylinder_1", "cylinder_3"):
            corrected = "cylinder_3" if first == "cylinder_1" else "cylinder_1"
            logger.info("  label 纠错: %s → %s", label, corrected)
            label = corrected
    else:
        seen_state["first"] = label
        logger.info("  首次识别: %s", label)

    # 获取当前臂状态
    state = _read_arm_state(arm)
    init_y_mm = float(cfg.get("init_y_mm", -100.0))
    pick_arm_start = float(cfg.get("arm_pick_pose", {}).get("arm_angle_deg", -90.0))

    logger.info("  -> 抓取 %s，视觉伺服对准", label)

    # 调用运行时视觉伺服（进程内闭环）
    nozzle_xy = TASK1_NOZZLE_OFFSET_MAP.get(label, setpoint_xy)
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
        servo_timeout=PICK_SERVO_TIMEOUT_S_DEFAULT,
        settle_hits=PICK_SERVO_SETTLE_HITS,
    )

    if not servo_result.get("settled"):
        # 重试
        if int(servo_result.get("trace_hits", 0)) > 0:
            retry_result = arm.run_arm_servo(
                label=label,
                hz=PICK_SERVO_HZ,
                gain_arm=PICK_SERVO_GAIN_ARM,
                gain_x=PICK_SERVO_GAIN_X,
                deadzone=PICK_SERVO_RETRY_DEADZONE,
                max_vel=PICK_SERVO_MAX_VEL,
                arm_start=float(servo_result.get("end_arm", pick_arm_start)),
                sign_arm=1.0,
                sign_x=-1.0,
                setpoint_x_norm=float(nozzle_xy[0]),
                setpoint_y_norm=float(nozzle_xy[1]),
                arm_min=-150.0,
                arm_max=90.0,
                servo_timeout=PICK_SERVO_TIMEOUT_S_DEFAULT + PICK_SERVO_RETRY_TIMEOUT_EXTRA_S,
                settle_hits=PICK_SERVO_SETTLE_HITS,
            )
            servo_result = retry_result

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

    直接在 runtime 进程内执行，所有 arm/car 调用走 SDK 方法，
    不经过 HTTP 网络栈。

    Args:
        car: MyCar 实例（持有 arm / streamer / get_odometry 等）

    Returns:
        dict: {"ok": bool, "completed": List[str], "chassis_aligned": bool,
               "error": str|None}
    """
    arm = getattr(car, "arm", None)
    streamer = getattr(car, "streamer", None)
    if arm is None:
        return {"ok": False, "completed": [], "error": "arm 未初始化"}
    if streamer is None:
        return {"ok": False, "completed": [], "error": "streamer 未初始化"}

    # 加载配置
    try:
        cfg = _load_task_config()
    except NotImplementedError as exc:
        return {"ok": False, "completed": [], "error": str(exc)}
    except Exception as exc:
        logger.error("加载 task_config.yml 失败: %s", exc)
        return {"ok": False, "completed": [], "error": f"配置加载失败: {exc}"}

    completed: List[str] = []
    seen_state: Dict[str, Any] = {}
    pos_along = [0.0]  # 底盘纵向自记账

    # place 对齐参数
    place_align_enabled = cfg.get("place_align", {}).get("enabled", False)
    place_arm_deg = float(cfg.get("arm_place_pose_T2", {}).get("arm_angle_deg", PLACE_ARM_DEG))
    place_x_mm = PLACE_X_MM_FALLBACK

    # place 对齐（2026-08-10: 放苗侧 cylinder_set 机械臂视觉对齐）
    if place_align_enabled:
        logger.info("step 1: PLACE 机械臂对齐 cylinder_set")
        try:
            _switch_to_place_pose(arm, PLACE_ALIGN_X_MM, PLACE_ALIGN_SERVO_ARM_MIN, PLACE_HAND_DEG)

            servo_result = arm.run_arm_servo(
                label=cfg.get("place_align", {}).get("label", "cylinder_set"),
                hz=float(cfg.get("place_align", {}).get("hz", PICK_SERVO_HZ)),
                gain_arm=float(cfg.get("place_align", {}).get("gain_arm", PICK_SERVO_GAIN_ARM)),
                gain_x=float(cfg.get("place_align", {}).get("gain_x", PICK_SERVO_GAIN_X)),
                deadzone=float(cfg.get("place_align", {}).get("deadzone", PICK_SERVO_DEADZONE)),
                max_vel=float(cfg.get("place_align", {}).get("max_vel", PICK_SERVO_MAX_VEL)),
                arm_start=PLACE_ALIGN_SERVO_ARM_MIN,
                sign_arm=PLACE_ALIGN_SERVO_SIGN_ARM,
                sign_x=PLACE_ALIGN_SERVO_SIGN_X,
                setpoint_x_norm=float(cfg.get("place_align", {}).get("setpoint_cxcy", PLACE_ALIGN_SETPOINT_CXCY)[0]),
                setpoint_y_norm=float(cfg.get("place_align", {}).get("setpoint_cxcy", PLACE_ALIGN_SETPOINT_CXCY)[1]),
                arm_min=PLACE_ALIGN_SERVO_ARM_MIN,
                arm_max=PLACE_ALIGN_SERVO_ARM_MAX,
                servo_timeout=PLACE_ALIGN_TIMEOUT_S,
                settle_hits=PICK_SERVO_SETTLE_HITS,
            )

            if servo_result.get("settled"):
                place_arm_deg = float(servo_result.get("end_arm", PLACE_ARM_DEG))
                end_x = servo_result.get("end_x")
                if end_x is not None:
                    place_x_mm = float(end_x) * 1000.0
                logger.info("  place 对齐成功: arm=%.1f° x=%.1fmm", place_arm_deg, place_x_mm)
            else:
                logger.warning("  place 对齐未收敛，回落写死姿态")

        except Exception as exc:
            logger.warning("  place 对齐异常: %s，回落写死姿态", exc)

    # 记住初始 odom
    align_odom_x, _, _ = _read_odom(car)

    source_position_order = cfg.get("source_position_order", [1, 2, 3])
    target_slot_map = cfg.get("target_slot_map", {})
    chassis_move_timeout = cfg.get("chassis_move_timeout_s", 30)

    # S 姿态参数
    s_arm = float(cfg.get("arm_pick_pose", {}).get("arm_angle_deg", -90.0))
    s_x_mm = float(cfg.get("arm_pick_pose", {}).get("x_mm", -70.0))
    init_y_mm = float(cfg.get("init_y_mm", -100.0))

    try:
        for i, column_idx in enumerate(source_position_order):
            curr_x, _, curr_theta = _read_odom(car)
            logger.info("=== 处理底盘列 %d (S%d, odom x=%.3f) ===", i + 1, column_idx, curr_x)

            # (1) 底盘移动到本列源
            if i > 0:
                target_s = SOURCE_POSITIONS_M.get(column_idx, 0.0)
                logger.info("  底盘 → S%d (相对 %.3f m)", column_idx, target_s)
                _chassis_goto(car, target_s, pos_along, chassis_move_timeout)
            else:
                logger.info("  已在 S1 列，跳过底盘移动")

            # (1.5) 切 S 姿态
            logger.info("  切 S 姿态: arm=%.1f° x=%.1fmm y=%.1fmm hand=%.1f°",
                        s_arm, S_POSE_X_MM, S_POSE_Y_MM, PICK_START_HAND_DEG)
            _switch_to_s_pose(arm, s_arm, S_POSE_X_MM, PICK_START_HAND_DEG, S_POSE_Y_MM)

            # (2) 抓取
            try:
                label = _pick_at_source(arm, streamer, cfg, column_idx, seen_state)
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

            # y 偏低先抬
            st = _read_arm_state(arm)
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

            # (4) 放苗
            _place_cylinder(arm, slot_idx, place_arm_deg, place_x_mm)

            # (5) 归位
            if i + 1 < len(source_position_order):
                ret = cfg.get("arm_return_S1_pose", {})
                arm.composite_run(
                    arm=float(ret.get("arm_angle_deg", -90.0)),
                    x=float(ret.get("x_mm", -100.0)) / 1000.0,
                    y=float(ret.get("y_mm", -100.0)) / 1000.0,
                    hand=float(ret.get("hand_angle_deg", -10.0)),
                    speed=80, timeout=30.0,
                )

    except Exception as exc:
        logger.exception("task1 执行失败: %s", exc)
        # 异常时尝试移到 S3
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
        "chassis_aligned": place_align_enabled,
    }
