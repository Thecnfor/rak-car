#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Runtime 层 task2 (water_tower_task) 核心逻辑 — 进程内直调，跳过网络栈。

完全复刻 main/task/task2_water_tower.py 的业务逻辑，所有 arm/car 调用直接走
SDK 方法（car.composite_run / car.run_arm_servo / car.arm.move_y_position /
car.arm.grasp 等），不经过 HTTP API。

调用方式：
  POST http://<JETSON>:5050/v1/execute
  {"target": "car", "name": "run_task2", "kwargs": {}}
  或本地：python run.py --task 2 --direct
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional, Tuple

from smartcar.whalesbot.tools import get_yaml

logger = logging.getLogger("runtime.task2")

# ══════════════════════════════════════════════════════════════════════════════
# 快速调参区（与 main/task/task2_water_tower.py 完全同步）
# ══════════════════════════════════════════════════════════════════════════════

INIT_POSE_Y_MM = -150.0
INIT_POSE_X_MM = 0.0
INIT_POSE_ARM_DEG = 90.0
INIT_POSE_HAND_DEG = -90.0

DETECT_POSE_X_MM = -200.0
DETECT_POSE_ARM_DEG = -92.0
DETECT_POSE_HAND_DEG = -60.0
DETECT_POSE_Y_MM = -10.0

PICK_POSE_Y_TRANSITION_MM = -150.0
PICK_POSE_Y_DESCEND_MM = -50.0
PICK_POSE_Y_LIFT_MM = -150.0
PICK_POSE_ARM_DEG = 90.0
PICK_POSE_HAND_DEG = -10.0

FIRST_CUBE_X_MM = -165.0
SECOND_CUBE_X_MM = -210.0
FIRST_CUBE_SAFE_X_MM = -200.0

DELIVER_RETRACT_X_MM = -260.0
FIRST_DELIVER_RETRACT_X_MM = -235.0

CARRY_POSE_X_MM = -115.0
CARRY_POSE_ARM_DEG = -92.0
CARRY_POSE_HAND_DEG = -90.0

DELIVER_Y_BY_INDEX: List[float] = [0.0, -45.0, -70.0]
DELIVER_HAND_BY_INDEX: List[float] = [-80.0, -80.0, -80.0]

TRANSIT_Y_MM = -75.0

TOWER_SPACING_M = 0.43
GROUP_FORWARD_M = 0.35
GROUP_BACKWARD_M = 0.33

PICK_VISION_LABEL = "water"
PICK_VISION_SETPOINT_CXCY: List[float] = [0.063, -0.202]
PICK_VISION_TIMEOUT_S = 3.5
PICK_BLOCK2_TIMEOUT_S: Optional[float] = 6.0
PICK_BLOCK3_TIMEOUT_S: Optional[float] = 7.0
PICK_RETRY_DEADZONE = 0.10
PICK_RETRY_EXTRA_S = 4.0
PICK_RETRY_ESCALATE_S = 1.0
PICK_VISION_HZ = 25.0
PICK_VISION_GAIN_ARM = 0.15
PICK_VISION_GAIN_X = 0.25
PICK_VISION_DEADZONE = 0.05
PICK_VISION_MAX_VEL = 0.03
PICK_VISION_SETTLE_HITS = 6
PICK_VISION_HOLD_S = 0.0

TRACK_ALIGN_TARGET = "water"
TRACK_ALIGN_SETPOINT_CXCY: List[float] = [0.148, 0.234]
TRACK_ALIGN_VX_ONLY = True
TRACK_ALIGN_SIGN_VX = +1
TRACK_ALIGN_SIGN_VY = +1
TRACK_ALIGN_KP = 0.22
TRACK_ALIGN_V_MAX = 0.11
TRACK_ALIGN_V_SLEW = 0.011
TRACK_ALIGN_HZ = 25.0
TRACK_ALIGN_DEADBAND = 0.06
TRACK_ALIGN_HOLD_FRAMES = 4
TRACK_ALIGN_MAX_LOST_FRAMES = 30
TRACK_ALIGN_MAX_SECONDS = 6.0
TRACK_ALIGN_RETRY_DEADBAND = 0.06
TRACK_ALIGN_RETRY_EXTRA_S = 3.0

DETECT_RETRY_STEP_M = 0.10
DETECT_RETRY_MAX = 2
VACUUM_SETTLE_S = 0.0
V_MAX_ARM_X_MMS = 80.0
CHASSIS_MOVE_TIMEOUT_S = 30.0

CHASSIS_MOVE_MAX_VEL_MPS = 0.10
COMPOSITE_SPEED_DEFAULT = 100
COMPOSITE_TIMEOUT_S_DEFAULT = 10.0

WATER_TOWER_LABELS = {"water_l1", "water_l2", "water_l3"}

# 大臂转动安全区 (跟 main/task2 一致)
SAFE_Y_LO, SAFE_Y_HI = -200.0, -90.0
SAFE_X_LO, SAFE_X_HI = -300.0, -200.0


# ══════════════════════════════════════════════════════════════════════════════
# 内部工具函数
# ══════════════════════════════════════════════════════════════════════════════

def _load_task_config() -> Dict[str, Any]:
    dir_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    yaml_path = os.path.join(dir_root, "task_config.yml")
    cfg = get_yaml(yaml_path)
    wcfg = cfg.get("task_cfg", {}).get("water_tower_task", {})
    if wcfg.get("placeholder"):
        raise NotImplementedError("任务 water_tower_task 配置尚未完成")
    return wcfg


def _read_arm_state(car) -> Dict[str, float]:
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


def _read_task_state(car):
    """读 task_feed 缓存，返回 (detections list, active bool)。"""
    streamer = getattr(car, "streamer", None)
    if streamer is None:
        return [], False
    try:
        state = streamer.get_task_state()
        if not isinstance(state, dict):
            return [], False
        return list(state.get("detections") or []), bool(state.get("active"))
    except Exception:
        return [], False


def _detect_tower_count(car) -> Optional[int]:
    """cam2 识别水塔等级标签，返回需要的方块数 (1/2/3)。"""
    count_map = {"water_l1": 1, "water_l2": 2, "water_l3": 3}
    deadline = time.time() + 1.0
    while time.time() < deadline:
        dets, active = _read_task_state(car)
        if not active:
            time.sleep(0.02)
            continue
        for d in dets:
            label = (d or {}).get("label", "")
            if label in WATER_TOWER_LABELS:
                n = count_map[label]
                logger.info("水塔识别 %s → 需要 %d 块", label, n)
                return n
        time.sleep(0.02)
    logger.warning("cam2 未识别到水塔等级标")
    return None


def _stop_chassis(car) -> None:
    """对齐/动作结束后显式把底盘停稳。"""
    service = getattr(car, "_service", None)
    if service is not None:
        try:
            service.set_chassis_velocity(0.0, 0.0, 0.0)
            return
        except Exception:
            pass
    try:
        car.move_for([0.0, 0.0, 0.0], max_velocities=[0.05, 0.05, math.pi / 3.0])
    except Exception:
        pass


def _chassis_move_for(car, dx_m: float, timeout: float,
                       speed_mps: float = CHASSIS_MOVE_MAX_VEL_MPS) -> None:
    """SDK 直发 move_for —— 与 main/task2 的 _chassis_move_for 等价。"""
    logger.info("底盘 move_for(dx=%.3f m, speed=%.2f m/s) → 阻塞 ≤ %.0fs",
                dx_m, speed_mps, timeout)
    car.move_for(
        [float(dx_m), 0.0, 0.0],
        max_velocities=[float(speed_mps), float(speed_mps), math.pi / 3.0],
        timeout=timeout,
    )


def _safe_composite_run(car, *, arm=None, x_mm=None, y_mm=None, hand=None,
                         speed: int = COMPOSITE_SPEED_DEFAULT,
                         timeout: float = COMPOSITE_TIMEOUT_S_DEFAULT) -> None:
    """composite_run 直发，m 单位自动转换。"""
    if arm is None and x_mm is None and y_mm is None and hand is None:
        return
    kwargs: Dict[str, Any] = dict(speed=speed, timeout=timeout)
    if arm is not None:
        kwargs["arm"] = float(arm)
    if x_mm is not None:
        kwargs["x"] = float(x_mm) / 1000.0
    if y_mm is not None:
        kwargs["y"] = float(y_mm) / 1000.0
    if hand is not None:
        kwargs["hand"] = float(hand)
    car.composite_run(**kwargs)


def _ensure_xy_in_safe_zone(car, *, timeout: float = 10.0) -> None:
    """把 X/Y 调到大臂安全区 (X∈[-300,-200], Y∈[-200,-90])。
    已满足的 no-op。
    """
    try:
        state = _read_arm_state(car)
    except Exception as exc:
        logger.warning("_ensure_xy_in_safe_zone: 读不到状态, 跳过 (%s)", exc)
        return
    cur_y = state["y_mm"]
    cur_x = state["x_mm"]
    safe_y = cur_y if SAFE_Y_LO <= cur_y <= SAFE_Y_HI else max(SAFE_Y_LO, min(SAFE_Y_HI, cur_y))
    safe_x = cur_x if SAFE_X_LO <= cur_x <= SAFE_X_HI else max(SAFE_X_LO, min(SAFE_X_HI, cur_x))

    need_x = abs(safe_x - cur_x) > 1.0
    need_y = abs(safe_y - cur_y) > 1.0

    if need_x or need_y:
        logger.info("X/Y 调安全区: Y=%.1f X=%.1f → Y=%.1f X=%.1f",
                    cur_y, cur_x, safe_y, safe_x)
        if need_y and need_x:
            if -150.0 <= cur_x <= -10.0:
                _safe_composite_run(car, x_mm=safe_x if need_x else None,
                                     y_mm=None, timeout=timeout)
                _safe_composite_run(car, x_mm=None, y_mm=safe_y if need_y else None,
                                     timeout=timeout)
            else:
                _safe_composite_run(car, x_mm=None, y_mm=safe_y, timeout=timeout)
                _safe_composite_run(car, x_mm=safe_x, y_mm=None, timeout=timeout)
        elif need_x:
            _safe_composite_run(car, x_mm=safe_x, y_mm=None, timeout=timeout)
        else:
            _safe_composite_run(car, x_mm=None, y_mm=safe_y, timeout=timeout)


def _safe_arm_rotation_sequence(
    car,
    *,
    arm_kwargs: Dict[str, Any],
    timeout: float = 10.0,
) -> None:
    """大臂转动 3 阶段 (跟 main/task2 一致): 安全位 → 转臂 → 到位。
    Y∈[-200,-90], X∈[-300,-200] clamp。
    """
    target_y = arm_kwargs.get("y_mm")
    target_x = arm_kwargs.get("x_mm")
    target_arm = arm_kwargs.get("arm")
    target_hand = arm_kwargs.get("hand")
    safe_x_override = arm_kwargs.get("safe_x_mm")

    state = _read_arm_state(car)
    cur_y = state["y_mm"]
    cur_x = state["x_mm"]
    if cur_y is None or cur_x is None:
        return

    y_in = SAFE_Y_LO <= cur_y <= SAFE_Y_HI
    x_in = SAFE_X_LO <= cur_x <= SAFE_X_HI
    safe_y = cur_y if y_in else max(SAFE_Y_LO, min(SAFE_Y_HI, cur_y))
    safe_x = cur_x if x_in else max(SAFE_X_LO, min(SAFE_X_HI, cur_x))
    if safe_x_override is not None:
        safe_x = float(safe_x_override)

    logger.info(
        "大臂 3 阶段: Y=%.1f X=%.1f → 安全位 Y=%.1f X=%.1f → 目标 Y=%s X=%s arm=%s hand=%s",
        cur_y, cur_x, safe_y, safe_x,
        target_y, target_x, target_arm, target_hand,
    )

    # 阶段 1: X/Y 调安全位
    if abs(safe_x - cur_x) > 1.0 or abs(safe_y - cur_y) > 1.0:
        logger.info("  阶段 1: X/Y 调安全位 Y=%.1f X=%.1f", safe_y, safe_x)
        _safe_composite_run(
            car, arm=None, hand=None,
            y_mm=safe_y if abs(safe_y - cur_y) > 1.0 else None,
            x_mm=safe_x if abs(safe_x - cur_x) > 1.0 else None,
            speed=100, timeout=timeout,
        )

    # 阶段 2: arm + hand (X/Y 冻结)
    if target_arm is not None or target_hand is not None:
        logger.info("  阶段 2: arm=%s hand=%s (X/Y 冻结)", target_arm, target_hand)
        _safe_composite_run(
            car, x_mm=None, y_mm=None,
            arm=target_arm, hand=target_hand,
            speed=100, timeout=timeout,
        )

    # 阶段 3: X/Y 到目标 (并发)
    need_x = target_x is not None and abs(target_x - safe_x) > 1.0
    need_y = target_y is not None and abs(target_y - safe_y) > 1.0
    if need_x or need_y:
        logger.info("  阶段 3: X/Y 并发 X=%s Y=%s", target_x, target_y)
        _safe_composite_run(
            car, arm=None, hand=None,
            x_mm=target_x if need_x else None,
            y_mm=target_y if need_y else None,
            speed=100, timeout=timeout,
        )


def _parallel_chassis_arm(
    car,
    *,
    target_dx_m: float = 0.0,
    arm_kwargs: Optional[Dict[str, Any]] = None,
    timeout: float = 10.0,
) -> None:
    """底盘 move_for + 臂动作并发 (主循环零阻塞)。
    arm_kwargs 含 arm 走 _safe_arm_rotation_sequence (3 阶段); 否则走单步 composite_run。
    """
    tasks = []
    with ThreadPoolExecutor(max_workers=2) as ex:
        if (arm_kwargs and arm_kwargs.get("arm") is None
                and abs(target_dx_m) > 1e-3):
            _ensure_xy_in_safe_zone(car, timeout=timeout)
        if abs(target_dx_m) > 1e-3:
            tasks.append(ex.submit(_chassis_move_for, car, target_dx_m, timeout))
        if arm_kwargs and arm_kwargs.get("arm") is not None:
            tasks.append(ex.submit(_safe_arm_rotation_sequence, car,
                                    arm_kwargs=arm_kwargs, timeout=timeout))
        elif arm_kwargs:
            ak = dict(arm_kwargs)
            tasks.append(ex.submit(
                _safe_composite_run, car,
                arm=ak.get("arm"), x_mm=ak.get("x_mm"), y_mm=ak.get("y_mm"),
                hand=ak.get("hand"),
                speed=ak.get("speed", 100),
                timeout=ak.get("timeout", timeout),
            ))
        for t in tasks:
            t.result()


def _deliver_prepare(
    car,
    *,
    target_dx_m: float,
    carry_x_mm: float,
    carry_arm_deg: float,
    carry_hand_deg: float,
    deliver_y_mm: float,
    retract_x_mm: float = DELIVER_RETRACT_X_MM,
    timeout: float = 10.0,
) -> None:
    """carry 切姿态 4 步: 收X → 大臂转 → Y降投放深度 → X伸出。
    底盘 move_for 回塔与臂步骤并发。
    """
    tasks = []
    with ThreadPoolExecutor(max_workers=2) as ex:
        if abs(target_dx_m) > 1e-3:
            tasks.append(ex.submit(_chassis_move_for, car, target_dx_m, timeout))

        def _arm_prep():
            _safe_composite_run(car, arm=None, x_mm=retract_x_mm, y_mm=None,
                                 hand=None, speed=100, timeout=5.0)
            _safe_arm_rotation_sequence(
                car,
                arm_kwargs=dict(arm=carry_arm_deg, hand=carry_hand_deg,
                                 speed=100, timeout=timeout),
            )
            _safe_composite_run(car, arm=None, x_mm=None, y_mm=deliver_y_mm,
                                 hand=None, speed=100, timeout=timeout)
            _safe_composite_run(car, arm=None, x_mm=carry_x_mm, y_mm=None,
                                 hand=None, speed=100, timeout=timeout)

        tasks.append(ex.submit(_arm_prep))
        for t in tasks:
            t.result()


# ══════════════════════════════════════════════════════════════════════════════
# 底盘视觉对齐（chassis-align）
# ══════════════════════════════════════════════════════════════════════════════

def _align_to_tower(car, track_cfg: Dict[str, Any]) -> Tuple[bool, str]:
    """track_chassis 闭环对齐水塔等级标，含失败重试。
    优先走 service 直调 (ChassisAlignController)；无 service 退 HTTP 自调。
    """
    service = getattr(car, "_service", None)
    base_deadband = float(track_cfg.get("deadband", TRACK_ALIGN_DEADBAND))
    base_max_seconds = float(track_cfg.get("max_seconds", TRACK_ALIGN_MAX_SECONDS))
    retry_deadband = float(track_cfg.get("retry_deadband", TRACK_ALIGN_RETRY_DEADBAND))
    retry_extra_s = float(track_cfg.get("retry_extra_s", TRACK_ALIGN_RETRY_EXTRA_S))
    common_kwargs = dict(
        target=str(track_cfg.get("target", TRACK_ALIGN_TARGET)),
        setpoint_cxcy=tuple(track_cfg.get("setpoint_cxcy", TRACK_ALIGN_SETPOINT_CXCY)),
        vx_only=bool(track_cfg.get("vx_only", TRACK_ALIGN_VX_ONLY)),
        sign_vx=int(track_cfg.get("sign_vx", TRACK_ALIGN_SIGN_VX)),
        sign_vy=int(track_cfg.get("sign_vy", TRACK_ALIGN_SIGN_VY)),
        kp=float(track_cfg.get("kp", TRACK_ALIGN_KP)),
        v_max=float(track_cfg.get("v_max", TRACK_ALIGN_V_MAX)),
        v_slew=float(track_cfg.get("v_slew", TRACK_ALIGN_V_SLEW)),
        hz=float(track_cfg.get("hz", TRACK_ALIGN_HZ)),
        hold_frames=int(track_cfg.get("hold_frames", TRACK_ALIGN_HOLD_FRAMES)),
        max_lost_frames=int(track_cfg.get("max_lost_frames",
                                          TRACK_ALIGN_MAX_LOST_FRAMES)),
        max_seconds=base_max_seconds,
    )

    def _run(deadband: float, max_seconds: float) -> Tuple[bool, str]:
        kw = dict(common_kwargs)
        kw["deadband"] = deadband
        kw["max_seconds"] = max_seconds
        if service is not None:
            from runtime.services.chassis_align import ChassisAlignController
            try:
                ctrl = ChassisAlignController(service, **kw)
                res = ctrl.run()
                logger.info("track_chassis 直调 result: %s", res)
                return bool(res.get("arrived")), str(res.get("reason", "unknown"))
            except Exception as exc:
                logger.warning("ChassisAlignController 直调失败, 退 HTTP 自调: %s", exc)
        # HTTP 自调 (跟 main/task2 原版同款)
        try:
            from main.chassis import track_chassis
            res = track_chassis(**kw)
            return bool(getattr(res, "arrived", False)), str(getattr(res, "reason", "unknown"))
        except Exception as exc:
            logger.warning("track_chassis HTTP 自调失败: %s", exc)
            return False, "error"

    arrived, reason = _run(base_deadband, base_max_seconds)
    if not arrived:
        logger.info("底盘对齐未到位 (reason=%s), 扩死区 %.3f→%.3f + 加时 %.0fs 重试",
                    reason, base_deadband, retry_deadband, retry_extra_s)
        arrived, reason = _run(retry_deadband, base_max_seconds + retry_extra_s)
    _stop_chassis(car)
    return arrived, reason


# ══════════════════════════════════════════════════════════════════════════════
# 视觉伺服抓水立方（本地闭环）
# ══════════════════════════════════════════════════════════════════════════════

def _pick_cube_servo_local(
    car,
    vision: Dict[str, Any],
    pick: Dict[str, Any],
    sp_x: Optional[float],
    sp_y: Optional[float],
    timeout_override: Optional[float] = None,
) -> Dict[str, Any]:
    """本地视觉伺服 — runtime 进程内闭环 (与 main/task2 同款, 重试规则一致)。
    """
    arm = getattr(car, "arm", None)
    if arm is None:
        raise RuntimeError("arm 未初始化")

    def _servo_kw() -> Dict[str, Any]:
        return dict(
            label=str(vision.get("label", PICK_VISION_LABEL)),
            hz=float(vision.get("hz", PICK_VISION_HZ)),
            gain_arm=float(vision.get("gain_arm", PICK_VISION_GAIN_ARM)),
            gain_x=float(vision.get("gain_x", PICK_VISION_GAIN_X)),
            deadzone=float(vision.get("deadzone", PICK_VISION_DEADZONE)),
            max_vel=float(vision.get("max_vel", PICK_VISION_MAX_VEL)),
            arm_start=float(pick["arm_angle_deg"]),
            sign_arm=float(vision.get("sign_arm", 1.0)),
            sign_x=float(vision.get("sign_x", 1.0)),
            setpoint_x_norm=sp_x if sp_x is not None else 0.0,
            setpoint_y_norm=sp_y if sp_y is not None else 0.0,
            arm_min=float(vision.get("arm_min", 60.0)),
            arm_max=float(vision.get("arm_max", 130.0)),
            servo_timeout=float(
                timeout_override if timeout_override is not None
                else vision.get("timeout", PICK_VISION_TIMEOUT_S)
            ),
            settle_hits=int(vision.get("settle_hits", PICK_VISION_SETTLE_HITS)),
        )

    kw = _servo_kw()
    logger.info(
        "cam2 本地视觉伺服: run_arm_servo(setpoint=(%.3f,%.3f) hz=%s "
        "gain_arm=%s gain_x=%s deadzone=%s max_vel=%s arm=[%s,%s] "
        "settle=%s servo_timeout=%s)",
        kw["setpoint_x_norm"], kw["setpoint_y_norm"],
        kw["hz"], kw["gain_arm"], kw["gain_x"], kw["deadzone"], kw["max_vel"],
        kw["arm_min"], kw["arm_max"], kw["settle_hits"], kw["servo_timeout"],
    )

    def _run_once(tag: str, servo_kw: Dict[str, Any]) -> Dict[str, Any]:
        result = car.run_arm_servo(**servo_kw)
        logger.info("cam2 本地视觉伺服%s: reason=%s settled=%s trace_hits=%s end_arm=%s",
                    tag, result.get("reason"), result.get("settled"),
                    result.get("trace_hits"), result.get("end_arm"))
        return result

    result = _run_once("", kw)
    if (not result.get("settled")
            and result.get("reason") == "timeout"):
        kw["deadzone"] = PICK_RETRY_DEADZONE
        kw["servo_timeout"] = float(kw["servo_timeout"]) + PICK_RETRY_EXTRA_S
        if result.get("end_arm") is not None:
            kw["arm_start"] = float(result["end_arm"])
        result = _run_once("重试", kw)
        if (not result.get("settled")
                and result.get("reason") == "timeout"
                and int(result.get("trace_hits", 0) or 0) > 0):
            kw["servo_timeout"] = float(kw["servo_timeout"]) + PICK_RETRY_ESCALATE_S
            if result.get("end_arm") is not None:
                kw["arm_start"] = float(result["end_arm"])
            result = _run_once("类推", kw)

    if not result.get("settled"):
        raise RuntimeError(
            f"cam2 本地视觉抓水立方失败 (reason={result.get('reason')}, "
            f"trace_hits={result.get('trace_hits')}, end_arm={result.get('end_arm')})"
        )

    # 对齐完成 → y 降 grasp_y + hand 转 descend_hand (并发) → 吸气 → 抬回 servo_y
    hand_param = vision.get("descend_hand_deg")
    if hand_param is not None:
        hand_param = float(hand_param)
    target_m = float(vision.get("grasp_y_mm", pick["y_descend_mm"])) / 1000.0
    car.composite_run(arm=None, x=None, y=target_m, hand=hand_param,
                       speed=100, timeout=5.0)
    arm.grasp(True)
    servo_y = float(vision.get("servo_y_mm", pick["y_transition_mm"]))
    car.composite_run(arm=None, x=None, y=servo_y / 1000.0, hand=None,
                       speed=100, timeout=5.0)
    return result


def _pick_cube(
    car,
    cfg: Dict[str, Any],
    cube_x_mm: float,
    vision_timeout_override: Optional[float] = None,
) -> None:
    """抓单个水方块 (不含投放)。"""
    pick = cfg["pick_pose"]
    vision = cfg.get("pick_vision") or {}
    arm = getattr(car, "arm", None)

    if not vision.get("enabled"):
        arm.move_y_position(float(pick["y_descend_mm"]) / 1000.0)
        arm.grasp(True)
        arm.move_y_position(float(pick["y_lift_mm"]) / 1000.0)
        return

    sp = vision.get("setpoint_cxcy")
    sp_x = float(sp[0]) if (sp and len(sp) >= 1) else None
    sp_y = float(sp[1]) if (sp and len(sp) >= 2) else None
    logger.info(
        "cam2 视觉对齐开始: cube_x_mm=%.0f, setpoint_cxcy=(%.3f, %.3f), "
        "settle_hits=%d, timeout=%.1fs, deadzone=%.3f",
        float(cube_x_mm),
        sp_x if sp_x is not None else 0.0,
        sp_y if sp_y is not None else 0.0,
        int(vision.get("settle_hits", PICK_VISION_SETTLE_HITS)),
        float(vision.get("timeout", PICK_VISION_TIMEOUT_S)),
        float(vision.get("deadzone", PICK_VISION_DEADZONE)),
    )
    t0 = time.time()
    if vision.get("local_servo", True):
        _pick_cube_servo_local(car, vision, pick, sp_x, sp_y,
                                timeout_override=vision_timeout_override)
    else:
        # 盲抓兜底
        arm.move_y_position(float(pick["y_descend_mm"]) / 1000.0)
        arm.grasp(True)
        arm.move_y_position(float(pick["y_lift_mm"]) / 1000.0)
    logger.info("cam2 视觉对齐完成: cube_x_mm=%.0f, 用时=%.2fs",
                float(cube_x_mm), time.time() - t0)

    lift_y = float(pick.get("y_lift_mm", -150.0))
    servo_y = float(vision.get("servo_y_mm", pick["y_transition_mm"]))
    if abs(lift_y - servo_y) > 1.0:
        arm.move_y_position(lift_y / 1000.0)


# ══════════════════════════════════════════════════════════════════════════════
# 主入口
# ══════════════════════════════════════════════════════════════════════════════

def run_task2(car) -> Dict[str, Any]:
    """Task2 主入口：水塔取水 (2 座水塔 × N 块水方块投放)。
    完全复刻 main/task/task2_water_tower.py，但所有 arm/car 走 SDK 直调。
    """
    arm = getattr(car, "arm", None)
    if arm is None:
        return {"ok": False, "completed": [], "error": "arm 未初始化"}
    streamer = getattr(car, "streamer", None)
    if streamer is None:
        return {"ok": False, "completed": [], "error": "streamer 未初始化"}

    try:
        cfg = _load_task_config()
    except NotImplementedError as exc:
        return {"ok": False, "completed": [], "error": str(exc)}
    except Exception as exc:
        logger.error("加载配置失败: %s", exc)
        return {"ok": False, "completed": [], "error": f"配置加载失败: {exc}"}

    detection = cfg["detection_pose"]
    track_cfg = cfg.get("track_align", {})
    timeout = float(cfg.get("chassis_move_timeout_s", CHASSIS_MOVE_TIMEOUT_S))
    group_forward_m = float(cfg.get("group_forward_m", GROUP_FORWARD_M))
    group_backward_m = float(cfg.get("group_backward_m", group_forward_m))
    tower_spacing_m = float(cfg.get("tower_spacing_m", TOWER_SPACING_M))
    x_target_mm = float(detection["x_mm"])
    first_cube_safe_x_mm = float(cfg.get("first_cube_safe_x_mm",
                                          FIRST_CUBE_SAFE_X_MM))
    pick = cfg["pick_pose"]
    carry = cfg["carry_pose"]
    vision = cfg.get("pick_vision") or {}

    completed: List[str] = []

    try:
        # ===== 初始化：底盘回退 entry_back_off_m + 切 detection 姿态并发 =====
        entry_back_off_m = float(cfg.get("entry_back_off_m", 0.0))
        # 检测是否已在 detection 姿态 (orchestrator 已预摆)
        from main.task.task3.arm_poses import TASK2_DETECTION_ARM, arm_at_pose
        try:
            detection_in_place = bool(arm_at_pose(car, TASK2_DETECTION_ARM))
        except Exception:
            detection_in_place = False
        if detection_in_place:
            init_arm_kwargs = None
            logger.info("初始化: 臂已在 detection 姿态 (预摆完成), 只做底盘回退 %.2fm",
                        entry_back_off_m)
        else:
            init_arm_kwargs = dict(
                arm=float(detection["arm_angle_deg"]),
                x_mm=x_target_mm,
                y_mm=-150.0,
                hand=float(detection["hand_angle_deg"]),
                speed=100,
                timeout=10.0,
            )
            logger.info("初始化: 底盘回退 %.2fm + 切 detection 姿态并发",
                        entry_back_off_m)
        _parallel_chassis_arm(
            car,
            target_dx_m=(-entry_back_off_m) if entry_back_off_m > 1e-3 else 0.0,
            arm_kwargs=init_arm_kwargs,
        )

        # ===== 按水塔循环 =====
        for tower_idx, tower_label in enumerate(cfg["source_position_order"]):
            logger.info("=== 处理水塔 %s (第 %d 座) ===", tower_label, tower_idx + 1)

            if tower_idx > 0:
                logger.info("底盘前进 %.2f m → 水塔 %s (并发切回 detection 姿态)",
                            tower_spacing_m, tower_label)
                _parallel_chassis_arm(
                    car,
                    target_dx_m=tower_spacing_m,
                    arm_kwargs=dict(
                        arm=float(detection["arm_angle_deg"]),
                        x_mm=x_target_mm,
                        y_mm=-150.0,
                        hand=float(detection["hand_angle_deg"]),
                        speed=100,
                        timeout=10.0,
                    ),
                )

            logger.info("Y 下降到 %.0fmm 执行检测", detection["y_mm"])
            try:
                arm.move_y_position(float(detection["y_mm"]) / 1000.0, timeout=5.0)
            except Exception:
                logger.warning("Y 下降失败, 跳过水塔 %s", tower_label)
                continue

            # 先对齐再识别 (用户 2026-08-10 新逻辑)
            _align_to_tower(car, track_cfg)
            needed = _detect_tower_count(car)
            if needed is None:
                if tower_idx > 0:
                    logger.warning("水塔 %s 全程未识别到, 跳过整座塔", tower_label)
                    continue
                logger.warning("水塔 %s 未识别到等级标, 默认取 1 块", tower_label)
                needed = 1
            logger.info("水塔 %s 需投放 %d 块水方块", tower_label, needed)

            chassis_at_tower_m = 0.0
            picked = 0
            first_x = float(cfg["first_cube_x_mm"])
            second_x = float(cfg["second_cube_x_mm"])
            direction = 1.0 if tower_idx == 0 else -1.0

            while picked < needed:
                try:
                    group = picked // 2
                    target_offset = direction * group * (
                        group_forward_m if direction > 0 else group_backward_m
                    )
                    pick_x = first_x if (picked % 2 == 0) else second_x
                    deliver_hands = cfg.get("deliver_hand_by_index",
                                            [float(carry["hand_angle_deg"])])
                    deliver_hand = deliver_hands[min(picked, len(deliver_hands) - 1)]

                    # 准备 pick: 底盘到组 + 臂切 pick 姿态 (并发)
                    d_to_group = target_offset - chassis_at_tower_m
                    logger.info("第 %d 块: 底盘 Δ=%.2f m → 第 %d 组 (并发切 pick 姿态, X=%s)",
                                picked + 1, d_to_group, group + 1, pick_x)
                    _parallel_chassis_arm(
                        car,
                        target_dx_m=d_to_group,
                        arm_kwargs=dict(
                            arm=float(pick["arm_angle_deg"]),
                            x_mm=float(pick_x),
                            y_mm=float(vision.get("servo_y_mm",
                                                   pick["y_transition_mm"])),
                            hand=float(pick["hand_angle_deg"]),
                            speed=100,
                            timeout=10.0,
                            safe_x_mm=first_cube_safe_x_mm,
                        ),
                    )
                    chassis_at_tower_m = target_offset

                    # 抓块 (含视觉伺服)
                    _block_timeout: Optional[float] = None
                    if picked == 1:
                        _block_timeout = PICK_BLOCK2_TIMEOUT_S
                    elif picked == 2:
                        _block_timeout = PICK_BLOCK3_TIMEOUT_S
                    _pick_cube(car, cfg, pick_x, vision_timeout_override=_block_timeout)

                    # 准备 deliver: 底盘回塔 + 臂切 carry 姿态
                    deliver_ys = cfg.get("deliver_y_by_index", [-50.0, -65.0, -80.0])
                    deliver_y = deliver_ys[min(picked, len(deliver_ys) - 1)]
                    d_back = -chassis_at_tower_m
                    carry_xs = cfg.get("carry_x_by_tower_mm") or []
                    if carry_xs and tower_idx < len(carry_xs):
                        carry_x_mm = float(carry_xs[tower_idx])
                    else:
                        carry_x_mm = float(carry["x_mm"])
                    retract_x = (FIRST_DELIVER_RETRACT_X_MM if picked == 0
                                 else DELIVER_RETRACT_X_MM)
                    logger.info("第 %d 块: 底盘回塔 Δ=%.2f m → carry (X收%.0f → 大臂转%s° → Y降%.0f → X伸%.0f)",
                                picked + 1, d_back, retract_x,
                                carry["arm_angle_deg"], deliver_y, carry_x_mm)
                    _deliver_prepare(
                        car,
                        target_dx_m=d_back,
                        carry_x_mm=carry_x_mm,
                        carry_arm_deg=float(carry["arm_angle_deg"]),
                        carry_hand_deg=float(deliver_hand),
                        deliver_y_mm=deliver_y,
                        retract_x_mm=retract_x,
                        timeout=10.0,
                    )
                    chassis_at_tower_m = 0.0

                    # 投放: grasp off (fire-and-forget)
                    logger.info("第 %d 块: 投放 Y=%.0f mm + grasp off",
                                picked + 1, deliver_y)
                    arm.grasp(False)

                except Exception as exc:
                    logger.exception("第 %d 块失败, 继续下一块: %s", picked + 1, exc)
                picked += 1

            completed.append("tower_{}".format(tower_label))

        # ===== 任务结束: 把 X/Y 调到大臂安全区 =====
        logger.info("任务结束: 把 X/Y 调到大臂安全区")
        _ensure_xy_in_safe_zone(car, timeout=10.0)

    except Exception as exc:
        logger.exception("water_tower_task 失败: %s", exc)
        return {"ok": False, "completed": completed, "error": str(exc)}

    return {"ok": True, "completed": completed}


if __name__ == "__main__":
    # 调试入口：直连 MyCar 跑一遍
    import logging as _logging
    _logging.basicConfig(level=_logging.INFO,
                          format="%(asctime)s [%(name)s] %(message)s")
    from runtime.services.my_car import MyCar
    _car = MyCar()
    try:
        print(run_task2(_car))
    finally:
        _car.close()