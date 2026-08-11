#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task4 底层直连实现 —— 进程内直调 MyCar，不经过 main/HTTP/WS 网络栈。

复刻 main/arm/each_task/task4/target4.py 的 2026-08-11 新版全流程:
  启动三步并发 → 第一球(蠕动→底盘对齐→臂伺服→抓放) → 后续球
  (0.1m 前进∥臂回初始 → 找球 → 臂伺服 → 抓放) → 左 IR>0.75 离区 / picks≥8 退出。

每个网络调用都映射到 MyCar / SDK 直连方法:
  /v1/realtime/chassis-velocity  → car.set_chassis_velocity()          (creep/停)
  /v1/execute move_for           → car.move_for([dx,0,0])               (0.1m 前进)
  move_along_lane(视觉循线)       → car.lane_dis_offset(vx, dis_hold)    (开始阶段前进)
  /v1/execute run_arm_servo      → car.run_arm_servo()                  (臂伺服闭环, 进程内)
  /v1/realtime/vision/task 缓存   → car.streamer.get_task_state()        (找球, 0 往返)
  /v1/realtime/chassis-align     → ChassisAlignController               (底盘对齐, 进程内)
  /v1/control/reset-stop         → car.clear_stop()
  /v1/realtime/wheels/encoders   → car.get_wheel_encoders()
  /v1/realtime/wheels/speeds     → car.set_wheel_speeds()
  start/stop_arm_feed            → car.start_arm_feed(hz) / car.stop_arm_feed(force=True)
  /v1/execute stop               → car.stop()
  业务层 mm 位姿                 → SDK 米制 (composite_run x/y 一律 /1000)

⚠️ 单位: main 侧 ArmClient.composite_run 收 mm；SDK ArmController.composite_run 收 m。
⚠️ 前提: 车端 task_feed(30Hz) 守护线程在跑 —— 本模块进入时 best-effort 自启，
   读到 streamer.task_state 缓存即可 0 往返找球；缓存不可用时回退同步
   car.get_detection_results()。

调用方式:
  python -m runtime.tasks.task4_direct            # 真跑 (不持有 car, 自建 MyCar)
  python -m runtime.tasks.task4_direct --dry-run  # 排练不动硬件
  python run.py --task 4 --direct                 # 本进程直连
  POST /v1/execute {"target":"car","name":"run_task4","kwargs":{}}  # runtime 进程内
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import Any, Dict, List, Optional

# ---------- 常量 (与 main/arm/each_task/task4/constants.py 同步) ----------

COLOR_BLUE: str = "blue"
COLOR_YELLOW: str = "yellow"
COLOR_UNKNOWN: str = "unknown"

# 球过滤 (侧摄)
TARGET_SCORE_MIN: float = 0.6
TARGET_ASPECT_TOL: float = 0.8
TARGET_AREA_MIN: float = 0.15
TARGET_AREA_MAX: float = 0.60
# cls_id → color 兜底 (task2026 实测: 17=ball_yellow, 18=ball_blue)
CLS_ID_TO_COLOR: Dict[int, str] = {17: COLOR_YELLOW, 18: COLOR_BLUE}

# P 姿态
TASK4_POSE_P_X_MM: float = -250.0
TASK4_POSE_P_Y_MM: float = -150.0
TASK4_POSE_P_ARM_DEG: float = 90.0
TASK4_POSE_P_HAND_DEG: float = -10.0
P_POSE_SKIP_TOL_X_MM: float = 10.0
P_POSE_SKIP_TOL_Y_MM: float = 10.0
P_POSE_SKIP_TOL_ARM_DEG: float = 5.0
P_POSE_SKIP_TOL_HAND_DEG: float = 5.0

# 存储仓
STORAGE_OPEN_ANGLE_DEG: int = 75
STORAGE_CLOSE_ANGLE_DEG: int = 98
STORAGE_OPEN_SPEED: int = 5

# 开始阶段 (三步并发)
START_LANE_FORWARD_M: float = 0.1
START_LANE_FORWARD_VX_MPS: float = 0.05

# 预算
DEFAULT_MAX_SECONDS: float = 9999.0
DEFAULT_CREEP_SPEED_MPS: float = 0.05
CREEP_POLL_HZ: float = 20.0
CREEP_MAX_SECONDS_S: float = 30.0
DEFAULT_TRACK_MAX_SECONDS: float = 4.0

# 新版流程 (2026-08-11)
FIRST_CREEP_MAX_M: float = 5.0
TRACK_EXTEND_SECONDS: float = 3.0

# 臂伺服
ARM_SERVO_SETPOINT_CX: float = 0.045
ARM_SERVO_SETPOINT_CY: float = -0.083
ARM_SERVO_GAIN_ARM: float = 0.2
ARM_SERVO_GAIN_X: float = 0.2
ARM_SERVO_DEADZONE: float = 0.05
ARM_SERVO_RETRY_DEADZONE: float = 0.075
ARM_SERVO_MAX_VEL: float = 0.05
ARM_SERVO_TIMEOUT_S: float = 4.0
ARM_SERVO_RETRY_TIMEOUT_S: float = 4.0
ARM_SERVO_SETTLE_HITS: int = 3
ARM_SERVO_SIGN_ARM: float = 1.0
ARM_SERVO_SIGN_X: float = 1.0
ARM_SERVO_ARM_START: float = 90.0
ARM_SERVO_ARM_MIN: float = 60.0
ARM_SERVO_ARM_MAX: float = 130.0
ARM_SERVO_HZ: float = 20.0

# 抓放序列 (mm → SDK 米制由 _composite_m 统一换算)
PICK_LOWER_Y_MM: float = -40.0
PICK_SUCK_HAND_DEG: float = 0.0
PICK_HOLD_S: float = 0.5
PICK_LIFT_Y_MM: float = -150.0
PICK_BIN_ARM_DEG: float = 95.0
PICK_RELEASE_Y_MM: float = -140.0
PICK_RELEASE_HAND_DEG: float = 10.0

# 后续球扫描
SCAN_ADVANCE_M: float = 0.10
SCAN_LOOK_S: float = 3.0
SCAN_GRAB_CX_HALF: float = 0.2
SCAN_MAX_PICKS: int = 8
SCAN_IR_FAR_M: float = 0.75

ALIGN_ONLY: bool = False
BIN_X_MM: Dict[str, float] = {COLOR_BLUE: 0.0, COLOR_YELLOW: -60.0}
BIN_HAND_DEG: Dict[str, float] = {COLOR_BLUE: 10.0, COLOR_YELLOW: 10.0}
BALL_LABELS: List[str] = ["ball_blue", "ball_yellow"]

LOG_PREFIX: str = "[task4]/direct"


# ---------- 底盘 / 传感器 helper ----------

def _odom_x(car) -> float:
    """读里程计 x (m)。兼容 tuple / dict / numpy 返回值。"""
    value = car.get_odometry()
    if isinstance(value, dict):
        value = value.get("x", value.get("odom_x", 0.0))
    if hasattr(value, "__getitem__") and not isinstance(value, str):
        try:
            value = value[0]
        except Exception:
            return 0.0
    try:
        return float(value)
    except (TypeError, ValueError, IndexError):
        return 0.0


def _ir_left(car) -> Optional[float]:
    """读左 IR 距离 (m)。读不到 / 值非法 → None。"""
    try:
        state = car.get_all_ir_distance()
    except Exception:
        return None
    if isinstance(state, dict):
        left = state.get("left")
    elif isinstance(state, (list, tuple)) and len(state) >= 1:
        left = state[0]
    else:
        return None
    if left in (None, "", "---"):
        return None
    try:
        return float(left)
    except (TypeError, ValueError):
        return None


def _ir_far(car, threshold_m: float = SCAN_IR_FAR_M) -> bool:
    """左 IR > 阈值 → 离区 (退出条件)。读不到按 False (不误退)。"""
    left = _ir_left(car)
    return left is not None and left > threshold_m


def _set_chassis_vel(car, vx: float, vy: float = 0.0) -> None:
    """下一次底盘速度 (realtime 门直连: car.set_chassis_velocity)。

    对应网络版 POST /v1/realtime/chassis-velocity。异常只 warn 不抛。
    """
    try:
        car.set_chassis_velocity(float(vx), float(vy), 0.0)
    except Exception as e:
        print(f"  [{LOG_PREFIX}] ⚠️ chassis 速度下发失败 "
              f"({type(e).__name__}: {str(e)[:60]})")


def _stopped(car) -> bool:
    """协作取消: emergency_stop / cancel_job 置位 → 真。"""
    try:
        fn = getattr(car, "_must_exit", None)
        return bool(fn() if callable(fn) else False)
    except Exception:
        return False


# ---------- 找球 (复刻 target2.fetch_balls, 直读 streamer 缓存) ----------

def _label_to_color(label, cls_id) -> str:
    """PaddleDet label / cls_id → 业务 color。"""
    if isinstance(label, str) and label:
        lo = label.strip().lower()
        if "blue" in lo:
            return COLOR_BLUE
        if "yellow" in lo:
            return COLOR_YELLOW
        return COLOR_UNKNOWN
    if isinstance(cls_id, int) and cls_id in CLS_ID_TO_COLOR:
        return CLS_ID_TO_COLOR[cls_id]
    return COLOR_UNKNOWN


def _norm_xy(bbox_norm: dict):
    """bbox_norm dict → (cx, cy, w, h)。支持三种格式 (同 target2._norm_xy)。"""
    if not isinstance(bbox_norm, dict):
        raise ValueError(f"bbox_norm 不是 dict: {bbox_norm!r}")
    try:
        if all(k in bbox_norm for k in ("x_center", "y_center", "width", "height")):
            return (float(bbox_norm["x_center"]), float(bbox_norm["y_center"]),
                    float(bbox_norm["width"]), float(bbox_norm["height"]))
        if all(k in bbox_norm for k in ("cx", "cy", "w", "h")):
            return (float(bbox_norm["cx"]), float(bbox_norm["cy"]),
                    float(bbox_norm["w"]), float(bbox_norm["h"]))
        if all(k in bbox_norm for k in ("x1", "y1", "x2", "y2")):
            x1, y1, x2, y2 = (float(bbox_norm[k]) for k in ("x1", "y1", "x2", "y2"))
            return ((x1 + x2) / 2.0, (y1 + y2) / 2.0, x2 - x1, y2 - y1)
        raise ValueError(f"bbox_norm 字段缺失: {bbox_norm!r}")
    except (TypeError, ValueError) as exc:
        raise ValueError(f"bbox_norm 解析失败: {exc}") from exc


def _det_dict_to_ball(det: dict, *, score_min, aspect_tol, area_min, area_max) -> Optional[dict]:
    """单条 task_state detection dict → ball dict; 不过滤返回 None。"""
    score_raw = det.get("score", None)
    try:
        score = float(score_raw) if score_raw is not None else 0.0
    except (TypeError, ValueError):
        return None
    if score < score_min:
        return None
    try:
        cx, cy, w, h = _norm_xy(det.get("bbox_norm") or {})
    except (ValueError, TypeError):
        return None
    if w <= 0 or h <= 0:
        return None
    if abs(w / h - 1.0) > aspect_tol:
        return None
    area = w * h
    if not (area_min <= area <= area_max):
        return None
    label = det.get("label")
    cls_id = det.get("cls_id")
    color = _label_to_color(label, cls_id)
    return {
        "color": color,
        "cx_norm": cx, "cy_norm": cy,
        "w_norm": w, "h_norm": h,
        "score": score,
        "det_id": det.get("det_id"),
        "cls_id": cls_id,
        "label": label,
    }


def _tuple_to_det(raw) -> dict:
    """get_detection_results 的 tuple 格式 → task_state detection dict。"""
    cls_id, det_id, label, score = raw[0], raw[1], raw[2], raw[3]
    x_c, y_c, w, h = raw[4], raw[5], raw[6], raw[7]
    return {
        "cls_id": cls_id, "det_id": det_id, "label": label, "score": score,
        "bbox_norm": {"x_center": x_c, "y_center": y_c, "width": w, "height": h},
    }


def _fetch_balls(
    car,
    *,
    score_min: Optional[float] = None,
    color_filter: Optional[str] = None,
    aspect_tol: Optional[float] = None,
    area_min: Optional[float] = None,
    area_max: Optional[float] = None,
    debug: bool = False,
) -> list:
    """找球: 优先读 streamer.task_state 缓存 (0 往返), 缓存不可用回退同步推理。

    对应网络版 main/arm/each_task/task4/target2.py::fetch_balls
    (GET /v1/realtime/vision/task → streamer.task_state)。
    """
    score_min = TARGET_SCORE_MIN if score_min is None else float(score_min)
    aspect_tol = TARGET_ASPECT_TOL if aspect_tol is None else float(aspect_tol)
    area_min = TARGET_AREA_MIN if area_min is None else float(area_min)
    area_max = TARGET_AREA_MAX if area_max is None else float(area_max)

    dets: list = []
    streamer = getattr(car, "streamer", None)
    if streamer is not None and hasattr(streamer, "get_task_state"):
        try:
            state = streamer.get_task_state()
            if isinstance(state, dict):
                task_state = state.get("task_state", state) if "task_state" in state else state
                if task_state.get("active"):
                    dets = task_state.get("detections") or []
        except Exception:
            dets = []
    if not dets:
        # task_feed 未启 / 无缓存 → 同步推理 (每次 ~30-100ms)
        try:
            raw = car.get_detection_results(sort_pos=(0, 0), limit_x=1, limit_y=1) or []
            dets = [_tuple_to_det(d) for d in raw if isinstance(d, (list, tuple)) and len(d) >= 8]
        except Exception:
            dets = []
    if debug:
        print(f"  [{LOG_PREFIX}] [DEBUG] raw detections={len(dets)}, "
              f"filters: score>={score_min}, |aspect-1|<={aspect_tol}, "
              f"{area_min}<=area<={area_max}")

    out: list = []
    for det in dets:
        if not isinstance(det, dict):
            continue
        ball = _det_dict_to_ball(
            det, score_min=score_min, aspect_tol=aspect_tol,
            area_min=area_min, area_max=area_max)
        if ball is None:
            continue
        if color_filter and ball["color"] != color_filter:
            continue
        out.append(ball)
    return out


def _pick_best_ball(balls: list) -> Optional[dict]:
    """选 score 最高的一球 (兜底判色)。"""
    candidates = [b for b in balls if b.get("color") in (COLOR_BLUE, COLOR_YELLOW)]
    if not candidates:
        return None
    return max(candidates, key=lambda b: float(b.get("score", 0.0)))


# ---------- 底盘对齐 (复刻 track_align.py, 进程内 ChassisAlignController) ----------

class _CarServiceAdapter:
    """把 MyCar 包成 ChassisAlignController 需要的 service 接口 (进程内直调)。

    ChassisAlignController 只依赖 5 个方法 (set_chassis_velocity / car /
    set_wheel_speeds / get_wheel_encoders / get_task_state), 全部 car 原生有。
    """

    def __init__(self, car):
        self.car = car

    def set_chassis_velocity(self, vx, vy, wz=0.0):
        return self.car.set_chassis_velocity(vx, vy, wz)

    def set_wheel_speeds(self, speeds):
        return self.car.set_wheel_speeds(speeds)

    def get_wheel_encoders(self):
        return self.car.get_wheel_encoders()

    def get_task_state(self):
        streamer = getattr(self.car, "streamer", None)
        if streamer is not None and hasattr(streamer, "get_task_state"):
            return streamer.get_task_state()
        return None


def _track_leftmost_ball(
    car,
    *,
    max_seconds: float,
    dry_run: bool,
    extend_seconds: float = 0.0,
    kp: float = 0.05,
    v_max: float = 0.04,
    deadband: float = 0.08,
    hold_frames: int = 3,
    v_slew: float = 0.01,
    decouple_xy: bool = True,
) -> dict:
    """底盘视觉伺服: 把画面最左球拉到画面中心 (复刻 track_align._track_leftmost_ball)。

    进程内跑 ChassisAlignController (读 streamer 缓存 + 直发底盘速), 等价于
    网络版 main.chassis.track_chassis → POST /v1/realtime/chassis-align。
    返回 ChassisAlignController.run() 的 dict (TrackChassisResult 字段)。
    """
    from runtime.services.chassis_align import ChassisAlignController

    def _run(max_s: float) -> dict:
        ctrl = ChassisAlignController(
            _CarServiceAdapter(car),
            target=BALL_LABELS,
            setpoint_cxcy=(0.0, 0.0),
            select_mode="leftmost",
            kp=kp, v_max=v_max, deadband=deadband,
            hold_frames=hold_frames, v_slew=v_slew,
            decouple_xy=decouple_xy,
            max_seconds=max_s, dry_run=dry_run,
        )
        return ctrl.run()

    try:
        res = _run(max_seconds)
    except Exception as e:
        print(f"  [{LOG_PREFIX}] ⚠️ 底盘对齐异常 "
              f"({type(e).__name__}: {str(e)[:100]}), 按失败继续 (臂伺服接管)")
        return {"arrived": False, "reason": "error", "final_frame": None,
                "frames": 0, "elapsed_s": 0.0}
    if (not res.get("arrived")) and res.get("reason") == "timeout" and extend_seconds > 0:
        print(f"  [{LOG_PREFIX}] track 超时 ({max_seconds:.0f}s), "
              f"加时 {extend_seconds:.0f}s (总上限 {max_seconds + extend_seconds:.0f}s)")
        try:
            retry = _run(extend_seconds)
        except Exception as e:
            print(f"  [{LOG_PREFIX}] ⚠️ track 加时异常 "
                  f"({type(e).__name__}: {str(e)[:100]}), 保留首次结果继续")
            return res
        return retry
    return res


def _color_from_track(track_res) -> Optional[str]:
    """从 track 结果 final_frame.label 提球色 (ball_blue → "blue")。"""
    ff = track_res.get("final_frame") if isinstance(track_res, dict) else None
    label = getattr(ff, "label", None) if ff is not None else None
    if label in BALL_LABELS:
        return label.split("_", 1)[1]
    return None


# ---------- 臂伺服 + 抓放 (复刻 pick_store.py) ----------

def _run_arm_servo(car, label: str, **overrides) -> dict:
    """调 car.run_arm_servo (进程内闭环, 只动 x 十字 + 大臂)。

    对应网络版 POST /v1/execute run_arm_servo。4s → 超时加时 4s + 死区放大 1.5 倍。
    重试从上一轮 end_arm 续跑 (不回起点, 保留大臂已转角度)。
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
        result = car.run_arm_servo(**kw)
        if not isinstance(result, dict):
            result = {}
        print(f"  [{LOG_PREFIX}] 臂伺服{tag}结果: "
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
            kw["arm_start"] = float(result["end_arm"])
        result = _call_once("重试")
    return result


def _composite_m(car, **kw) -> dict:
    """业务层 mm 位姿 → SDK 米制 composite_run。

    main 侧 ArmClient.composite_run 收 x_mm/y_mm (mm); SDK ArmController
    composite_run 收 x/y (m)。这里统一把 x_mm/y_mm 换算后透传, 其余原样。
    """
    sdk_kw: Dict[str, Any] = {}
    for k, v in kw.items():
        if k in ("x_mm", "y_mm"):
            sdk_kw[k.rstrip("_mm")] = float(v) / 1000.0
        else:
            sdk_kw[k] = v
    return car.arm.composite_run(**sdk_kw)


def _servo_and_pick(
    car,
    *,
    color: str,
    bin_x: Optional[float] = None,
    release_hand: Optional[float] = None,
    dry_run: bool = False,
) -> dict:
    """机械臂视觉伺服 → 对齐后保持姿势 → 盲降抓球 → 放 bin。

    复刻 main/arm/each_task/task4/pick_store.py::_servo_and_pick。抓放序列
    (保持伺服后姿势): 盲降 y→-40 + hand→0 → 吸气 → 保持 0.5s → 抬升 y→-150
    → 横移 bin x + arm→+95 → 放球 y→-140 + hand→10 → 放气。
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
            servo = _run_arm_servo(car, label)
            print(f"  [{LOG_PREFIX}] 臂伺服结束: settled={servo.get('settled')} "
                  f"reason={servo.get('reason')}")
        except Exception as e:
            print(f"  [{LOG_PREFIX}] ⚠️ 臂伺服异常 "
                  f"({type(e).__name__}: {str(e)[:100]}), 照样盲抓")

    if ALIGN_ONLY:
        print(f"  [{LOG_PREFIX}] [ALIGN_ONLY] 伺服完成, 不抓取")
        return {"ok": False, "error": "align_only"}

    # ---- 2. 抓放序列 (mm → m 换算) ----
    steps = [
        (f"盲降 y={PICK_LOWER_Y_MM:.0f} + hand→{PICK_SUCK_HAND_DEG:.0f}°",
         lambda: _composite_m(car, y_mm=PICK_LOWER_Y_MM, hand=PICK_SUCK_HAND_DEG,
                              speed=80, timeout=10.0)),
        ("吸气", lambda: car.arm.grasp(True)),
        (f"保持 {PICK_HOLD_S:.1f}s", lambda: time.sleep(PICK_HOLD_S)),
        (f"抬升 y={PICK_LIFT_Y_MM:.0f}",
         lambda: _composite_m(car, y_mm=PICK_LIFT_Y_MM, speed=80, timeout=10.0)),
        (f"横移 bin x={bin_x:.0f} + 大臂回 {PICK_BIN_ARM_DEG:.0f}",
         lambda: _composite_m(car, x_mm=bin_x, arm=PICK_BIN_ARM_DEG,
                              speed=80, timeout=20.0)),
        (f"放球 y={PICK_RELEASE_Y_MM:.0f} + hand={release_hand:.0f}",
         lambda: _composite_m(car, y_mm=PICK_RELEASE_Y_MM, hand=release_hand,
                              speed=80, timeout=10.0)),
        ("放气", lambda: car.arm.grasp(False)),
    ]
    for i, (desc, action) in enumerate(steps, 1):
        if dry_run:
            print(f"  [{LOG_PREFIX}] [DRY-RUN] [{i}/7] {desc}")
            continue
        print(f"  [{LOG_PREFIX}] [{i}/7] {desc}")
        try:
            action()
        except Exception as e:
            return {"ok": False,
                    "error": f"{desc} 失败: {type(e).__name__}: {str(e)[:120]}"}
    return {"ok": True, "error": None}


# ---------- 蠕动找球 (复刻 creep_thread.py) ----------

class _CreepThread:
    """后台线程保底盘前移 + 主线程摆臂; 见球即停 / 走满距离预算即停。

    复刻 main/arm/each_task/task4/creep_thread.py::_CreepThread, 但速度下发
    走 car.set_chassis_velocity (直连), 距离只认里程计增量, 找球读 streamer 缓存。
    """

    def __init__(self, car, *, speed_mps: float, max_distance_m: float,
                 poll_hz: float = CREEP_POLL_HZ):
        self.car = car
        self.speed_mps = speed_mps
        self.max_distance_m = max_distance_m
        self.poll_hz = poll_hz
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="task4-creep")
        self._stop_event = threading.Event()
        self.completion_event = threading.Event()
        self.distance_m = 0.0
        self.elapsed_s = 0.0
        self.balls = None
        self.found_ball = False
        self.distance_exhausted = False
        self._odo_start_x = None

    def start(self) -> None:
        try:
            self._odo_start_x = _odom_x(self.car)
        except Exception:
            self._odo_start_x = None
        self._thread.start()

    def _loop(self) -> None:
        period = 1.0 / max(self.poll_hz, 1.0)
        t0 = time.monotonic()
        try:
            while not self._stop_event.is_set():
                try:
                    self.car.set_chassis_velocity(float(self.speed_mps), 0.0, 0.0)
                except Exception:
                    pass
                time.sleep(period)

                # 走多远只认里程计增量; 读不到 / 卡死都不外推。
                if self._odo_start_x is not None:
                    try:
                        current_x = _odom_x(self.car)
                        self.distance_m = max(
                            self.distance_m, max(0.0, current_x - self._odo_start_x))
                    except Exception:
                        pass
                self.elapsed_s = time.monotonic() - t0

                # 见球即停 (运动模糊下宁可先停再对齐)
                try:
                    balls = _fetch_balls(self.car, debug=True)
                    if any(b.get("color") in (COLOR_BLUE, COLOR_YELLOW)
                           for b in balls):
                        self.balls = balls
                        self.found_ball = True
                        try:
                            self.car.set_chassis_velocity(0.0, 0.0, 0.0)
                        except Exception:
                            pass
                        break
                except Exception as e:
                    print(f"  [{LOG_PREFIX}] fetch_balls 异常: "
                          f"{type(e).__name__}: {str(e)[:100]}", file=sys.stderr)

                # 距离预算走满 → 停 (预算内没看到球 = 采区扫空, 上层判 zone_cleared)。
                if self.distance_m >= self.max_distance_m:
                    self.distance_exhausted = True
                    break
        finally:
            try:
                self.car.set_chassis_velocity(0.0, 0.0, 0.0)
            except Exception:
                pass
            self.completion_event.set()

    def wait_for_ball(self, timeout_s: float) -> dict:
        got = self.completion_event.wait(timeout=timeout_s)
        return {
            "balls": self.balls if got and self.found_ball else None,
            "distance_exhausted": bool(self.distance_exhausted),
            "distance_m": self.distance_m,
            "elapsed_s": self.elapsed_s,
        }

    def stop_and_join(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            try:
                self.car.set_chassis_velocity(0.0, 0.0, 0.0)
            except Exception:
                pass


# ---------- 后续球: 前进∥臂回初始 + 找球 (复刻 target4.py) ----------

def _advance_and_arm_init(car, *, pose_p_x_mm, pose_p_y_mm,
                          pose_p_arm_deg, pose_p_hand_deg, dry_run=False) -> None:
    """move_for 前进 SCAN_ADVANCE_M ∥ 并发 臂回初始姿势。

    对应网络版 /v1/execute move_for ∥ ArmClient.composite_run。串口抢占风险
    已接受 (2026-08-11 用户) —— 前进与臂回初始同时进行。
    """
    if dry_run:
        print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过 前进{SCAN_ADVANCE_M:.2f}m ∥ 臂回初始")
        return

    def _arm_init():
        try:
            _composite_m(car, arm=pose_p_arm_deg, x_mm=pose_p_x_mm,
                         y_mm=pose_p_y_mm, hand=pose_p_hand_deg,
                         speed=80, timeout=30.0)
        except Exception as e:
            print(f"  [{LOG_PREFIX}] ⚠️ 臂回初始失败 ({type(e).__name__}: {str(e)[:80]})")

    t = threading.Thread(target=_arm_init, name="task4-arm-init", daemon=True)
    t.start()
    try:
        car.move_for([SCAN_ADVANCE_M, 0.0, 0.0],
                     max_velocities=[0.05, 0.05, 3.14159 / 3])
        print(f"  [{LOG_PREFIX}] 前进 {SCAN_ADVANCE_M:.2f}m 完成 (臂回初始并发)")
    except Exception as e:
        print(f"  [{LOG_PREFIX}] ⚠️ move_for 前进失败 ({type(e).__name__}: {str(e)[:80]})")
    t.join(timeout=35.0)


def _look_grabbable_ball(car, *, timeout_s: float = SCAN_LOOK_S,
                         dry_run: bool = False) -> Optional[dict]:
    """找球 ≤timeout_s: 轮询 _fetch_balls, 过滤到可抓窗口
    (|cx_norm - ARM_SERVO_SETPOINT_CX| ≤ SCAN_GRAB_CX_HALF), 取最左 (cx 最小)。

    窗口内无球 → None (球在窗口外 = 下一轮的球, 上层继续前进)。
    """
    if dry_run:
        return {"color": COLOR_BLUE, "cx_norm": 0.0, "score": 1.0}
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() < deadline:
        try:
            balls = _fetch_balls(car, debug=False)
        except Exception:
            balls = []
        in_window = [
            b for b in balls
            if b.get("color") in (COLOR_BLUE, COLOR_YELLOW)
            and abs(float(b.get("cx_norm", 0.0)) - ARM_SERVO_SETPOINT_CX) <= SCAN_GRAB_CX_HALF
        ]
        if in_window:
            return min(in_window, key=lambda b: float(b.get("cx_norm", 0.0)))
        time.sleep(0.1)
    return None


def _at_p_pose(car, *, pose_p_x_mm, pose_p_y_mm,
               pose_p_arm_deg, pose_p_hand_deg) -> bool:
    """检查机械臂是否已在 P 姿态 (用于跳过开始阶段四轴联动)。

    car.get_arm_state() 返回 x/y 米制, 与 pose_p_*_mm/1000 比较。
    读不到 / 值缺失 → False (调用方重新摆臂, 保底)。
    """
    try:
        st = car.get_arm_state()
        if not isinstance(st, dict):
            return False
        x, y = st.get("x"), st.get("y")
        arm, hand = st.get("arm_angle"), st.get("hand_angle")
        if any(v is None for v in (x, y, arm, hand)):
            return False
        return (abs(float(x) - pose_p_x_mm / 1000.0) <= P_POSE_SKIP_TOL_X_MM / 1000.0
                and abs(float(y) - pose_p_y_mm / 1000.0) <= P_POSE_SKIP_TOL_Y_MM / 1000.0
                and abs(float(arm) - pose_p_arm_deg) <= P_POSE_SKIP_TOL_ARM_DEG
                and abs(float(hand) - pose_p_hand_deg) <= P_POSE_SKIP_TOL_HAND_DEG)
    except Exception:
        return False


# ---------- 主编排 (复刻 target4.py::step_target4) ----------

def step_target4(
    car,
    *,
    dry_run: bool = False,
    max_seconds: float = DEFAULT_MAX_SECONDS,
    creep_speed_mps: float = DEFAULT_CREEP_SPEED_MPS,
    track_max_seconds: float = DEFAULT_TRACK_MAX_SECONDS,
    # ---- 初始姿势参数 (与 main 侧 constants 同步, 可外部覆盖) ----
    pose_p_y_mm: float = TASK4_POSE_P_Y_MM,
    pose_p_x_mm: float = TASK4_POSE_P_X_MM,
    pose_p_arm_deg: float = TASK4_POSE_P_ARM_DEG,
    pose_p_hand_deg: float = TASK4_POSE_P_HAND_DEG,
    bin_x_blue_mm: float = BIN_X_MM[COLOR_BLUE],
    bin_x_yellow_mm: float = BIN_X_MM[COLOR_YELLOW],
    bin_hand_blue_deg: float = BIN_HAND_DEG.get(COLOR_BLUE, PICK_RELEASE_HAND_DEG),
    bin_hand_yellow_deg: float = BIN_HAND_DEG.get(COLOR_YELLOW, PICK_RELEASE_HAND_DEG),
) -> dict:
    """慢速前移搜索 + 底盘视觉定位 (最左球) + 吸嘴中心抓取 + 放 bin (进程内直连)。

    复刻 main/arm/each_task/task4/target4.py::step_target4, 所有动作直接调
    car.* / car.arm.*, 不经过 HTTP/WS/job_queue。

    Returns:
        dict: ok / picked / picks / pick_failures / total_creep_m / history /
              reason / elapsed_s。reason ∈ completed / zone_cleared /
              time_budget / keyboard_interrupt / stopped / error:... / dry_run。
    """
    print(f"\n========== {LOG_PREFIX} step_target4 (直连: 蠕动+底盘对齐+臂伺服) ==========")
    print(f"  模式: {'DRY-RUN (不动硬件)' if dry_run else 'EXECUTE (动硬件)'}")
    print(f"  初始姿势: x={pose_p_x_mm} y={pose_p_y_mm} arm={pose_p_arm_deg}° hand={pose_p_hand_deg}°")
    print(f"  第一球 creep {creep_speed_mps:.2f}m/s | 后续 前进 {SCAN_ADVANCE_M}m × "
          f"{SCAN_LOOK_S:.0f}s 找球")
    print(f"  底盘对齐 ≤{track_max_seconds:.0f}s (+超时加时 {TRACK_EXTEND_SECONDS:.0f}s, "
          f"失败也继续) | 臂伺服 setpoint=({ARM_SERVO_SETPOINT_CX},{ARM_SERVO_SETPOINT_CY})")
    print(f"  退出: 左 IR>{SCAN_IR_FAR_M:.2f}m 或 picks≥{SCAN_MAX_PICKS} | ALIGN_ONLY={ALIGN_ONLY}")

    for name, val in (("max_seconds", max_seconds), ("creep_speed_mps", creep_speed_mps),
                      ("track_max_seconds", track_max_seconds)):
        if val < 0:
            raise ValueError(f"{name} 必须 ≥ 0, 收到: {val}")

    # 保证侧摄 task_feed 在跑 (读 streamer 缓存找球的前提; 幂等, 已在跑则 no-op)。
    if not dry_run:
        try:
            car.start_task_feed(hz=30.0)
        except Exception:
            pass

    history: list = []
    n_picks = 0
    n_skips = 0
    n_pick_failures = 0
    total_creep_m = 0.0
    final_reason = "unknown"
    t_start = time.monotonic()

    def _record_pick(idx, color, res) -> int:
        history.append({"ball": idx,
                        "action": "picked" if res["ok"] else "pick_failed",
                        "color": color, "error": res["error"]})
        return 1 if res["ok"] else 0

    def _grab(color) -> dict:
        return _servo_and_pick(
            car, color=color, dry_run=dry_run,
            bin_x=(bin_x_blue_mm if color == COLOR_BLUE else bin_x_yellow_mm),
            release_hand=(bin_hand_blue_deg if color == COLOR_BLUE else bin_hand_yellow_deg),
        )

    try:
        # ---- 0. 起始臂姿态检测 (已在 P 则跳过开始阶段四轴联动) ----
        arm_at_p_pose = (
            _at_p_pose(car, pose_p_x_mm=pose_p_x_mm, pose_p_y_mm=pose_p_y_mm,
                       pose_p_arm_deg=pose_p_arm_deg, pose_p_hand_deg=pose_p_hand_deg)
            if not dry_run else False
        )
        print(f"  [{LOG_PREFIX}] 起始臂姿态: "
              f"{'已在 P 姿态 (跳过开始阶段四轴联动)' if arm_at_p_pose else '需四轴联动'}")

        # ---- 0. 停 arm_feed 守护线程 (视觉伺服前置, 20Hz 轮询会饿 arm_queue) ----
        if not dry_run:
            try:
                car.stop_arm_feed(force=True)
                print(f"  [{LOG_PREFIX}] ▶️ stop_arm_feed(force=True)")
            except Exception as e:
                print(f"  [{LOG_PREFIX}] ⚠️ stop_arm_feed 失败 "
                      f"({type(e).__name__}: {str(e)[:80]}), 继续")

        # ---- 0.b 开始阶段: 三步并发 (四轴联动到 P ∥ lane 前进 ∥ 开仓 75°) ----
        _pose_done = [arm_at_p_pose]

        def _start_pose():
            if dry_run or _pose_done[0]:
                return
            try:
                print(f"  [{LOG_PREFIX}] 开始阶段四轴联动 (arm={pose_p_arm_deg}° "
                      f"x={pose_p_x_mm}mm y={pose_p_y_mm}mm hand={pose_p_hand_deg}°)")
                _composite_m(car, arm=pose_p_arm_deg, x_mm=pose_p_x_mm,
                             y_mm=pose_p_y_mm, hand=pose_p_hand_deg,
                             speed=100, timeout=30.0)
                _pose_done[0] = True
                print(f"  [{LOG_PREFIX}] 开始阶段四轴联动完成")
            except Exception as e:
                _pose_done[0] = False
                print(f"  [{LOG_PREFIX}] ⚠️ 开始阶段四轴联动失败 "
                      f"({type(e).__name__}: {str(e)[:80]})")

        def _start_open():
            if dry_run:
                return
            try:
                print(f"  [{LOG_PREFIX}] 打开存储仓 (angle={STORAGE_OPEN_ANGLE_DEG}°)")
                car.set_storage_angle(STORAGE_OPEN_ANGLE_DEG, speed=STORAGE_OPEN_SPEED)
            except Exception as e:
                print(f"  [{LOG_PREFIX}] ⚠️ 开仓失败 ({type(e).__name__}: {str(e)[:80]})")

        t_pose = threading.Thread(target=_start_pose, name="task4-start-pose", daemon=True)
        t_open = threading.Thread(target=_start_open, name="task4-start-open", daemon=True)
        t_pose.start()
        t_open.start()
        if dry_run or START_LANE_FORWARD_M <= 0.0:
            print(f"  [{LOG_PREFIX}] {'[DRY-RUN] ' if dry_run else ''}跳过 lane 前进")
        else:
            try:
                print(f"  [{LOG_PREFIX}] 开始阶段 lane 前进 {START_LANE_FORWARD_M:.2f}m "
                      f"@ {START_LANE_FORWARD_VX_MPS:.2f}m/s")
                # 网络版走 move_along_lane (视觉循线); SDK 原生等价 = lane_dis_offset。
                car.lane_dis_offset(speed=START_LANE_FORWARD_VX_MPS,
                                    dis_hold=START_LANE_FORWARD_M)
                print(f"  [{LOG_PREFIX}] 开始阶段 lane 前进完成")
            except Exception as e:
                print(f"  [{LOG_PREFIX}] ⚠️ lane 前进失败 "
                      f"({type(e).__name__}: {str(e)[:80]}), 继续")
        t_pose.join(timeout=40.0)
        t_open.join(timeout=15.0)
        if _pose_done[0]:
            print(f"  [{LOG_PREFIX}] 开始阶段结束: 臂已在 P 姿态 (含跳过情形)")

        # ---- 2. 新版主流程 ----
        ball_idx = 0

        # ---- 2.1 第一个球: creep 找球 ----
        ball_idx += 1
        print(f"\n========== [{LOG_PREFIX}] 第 {ball_idx} 球 (首个, creep 找球) ==========")
        if dry_run:
            creep_res = {"balls": [{"color": COLOR_BLUE, "cx_norm": 0.05}],
                         "distance_m": 0.0, "elapsed_s": 0.0}
            print(f"  [{LOG_PREFIX}] [DRY-RUN] 跳过 creep, 用占位球")
        else:
            creep_thread = _CreepThread(
                car, speed_mps=creep_speed_mps,
                max_distance_m=FIRST_CREEP_MAX_M,
                poll_hz=CREEP_POLL_HZ,
            )
            creep_thread.start()
            creep_res = creep_thread.wait_for_ball(
                timeout_s=min(max(1.0, max_seconds - (time.monotonic() - t_start)),
                              CREEP_MAX_SECONDS_S + 10.0))
            creep_thread.stop_and_join()
        total_creep_m += creep_res["distance_m"]
        if creep_res["balls"] is None:
            print(f"  [{LOG_PREFIX}] 🏁 首个 creep 未见球 (前移 {creep_res['distance_m']:.3f}m)")
            final_reason = "zone_cleared"
        else:
            print(f"  [{LOG_PREFIX}] 🎯 底盘对齐 (≤{track_max_seconds:.0f}s, "
                  f"超时加时 {TRACK_EXTEND_SECONDS:.0f}s, 失败也继续)")
            track_res = _track_leftmost_ball(
                car, max_seconds=track_max_seconds,
                extend_seconds=TRACK_EXTEND_SECONDS,
                dry_run=dry_run,
            )
            print(f"  [{LOG_PREFIX}] 底盘对齐结束: arrived={track_res.get('arrived')} "
                  f"reason={track_res.get('reason')}")
            color = _color_from_track(track_res)
            if color is None and not dry_run:
                best = _pick_best_ball(creep_res["balls"] or [])
                color = best["color"] if best else None
            if dry_run and color is None:
                color = COLOR_BLUE
            if color not in (COLOR_BLUE, COLOR_YELLOW):
                print(f"  [{LOG_PREFIX}] ❌ 首个球无法定色, 收工")
                final_reason = "zone_cleared"
            else:
                print(f"  [{LOG_PREFIX}] ✓ 首个球: {color}")
                res = _grab(color)
                n_picks += _record_pick(ball_idx, color, res)
                if not res["ok"]:
                    n_pick_failures += 1

        # ---- 2.2 后续球循环 (picks<8 且 未离区) ----
        while final_reason == "unknown" and n_picks < SCAN_MAX_PICKS:
            elapsed = time.monotonic() - t_start
            if _stopped(car):
                final_reason = "stopped"
                print(f"  [{LOG_PREFIX}] ⏹ 协作取消 (emergency_stop / cancel)")
                break
            if elapsed >= max_seconds:
                final_reason = "time_budget"
                print(f"  [{LOG_PREFIX}] ⏱ 总时长 {elapsed:.1f}s 达预算, 收尾")
                break
            if _ir_far(car):
                final_reason = "zone_cleared"
                print(f"  [{LOG_PREFIX}] 🏁 左 IR > {SCAN_IR_FAR_M:.2f}m, 已离区, 收工")
                break
            ball_idx += 1
            print(f"\n========== [{LOG_PREFIX}] 第 {ball_idx} 球 (扫描前进) ==========")
            _advance_and_arm_init(
                car, pose_p_x_mm=pose_p_x_mm, pose_p_y_mm=pose_p_y_mm,
                pose_p_arm_deg=pose_p_arm_deg, pose_p_hand_deg=pose_p_hand_deg,
                dry_run=dry_run,
            )
            ball = _look_grabbable_ball(car, timeout_s=SCAN_LOOK_S, dry_run=dry_run)
            if ball is None:
                print(f"  [{LOG_PREFIX}] 未见可抓球 (窗口内无球), 下一轮继续前进")
                continue
            color = ball["color"]
            print(f"  [{LOG_PREFIX}] ✓ 锁定 {color} 球 "
                  f"(最左, cx={float(ball.get('cx_norm', 0)):.3f})")
            res = _grab(color)
            n_picks += _record_pick(ball_idx, color, res)
            if not res["ok"]:
                n_pick_failures += 1
        else:
            if final_reason == "unknown":
                final_reason = "completed"
                print(f"  [{LOG_PREFIX}] 🏁 已抓 {SCAN_MAX_PICKS} 球, 封顶收工")

    except KeyboardInterrupt:
        final_reason = "keyboard_interrupt"
        print(f"\n  [{LOG_PREFIX}] Ctrl-C 中断")
    finally:
        # 兜底清场: 速度清零 + 停底盘
        try:
            _set_chassis_vel(car, 0.0)
        except Exception:
            pass
        if not dry_run:
            try:
                car.stop()
            except Exception:
                pass
            # ---- 关仓 ----
            try:
                print(f"  [{LOG_PREFIX}] 关闭存储仓 (angle={STORAGE_CLOSE_ANGLE_DEG}°)")
                car.set_storage_angle(STORAGE_CLOSE_ANGLE_DEG, speed=STORAGE_OPEN_SPEED)
            except Exception as e:
                print(f"  [{LOG_PREFIX}] ⚠️ 关仓失败 ({type(e).__name__}: {str(e)[:80]})")
            # ---- 后台回到 P 姿态 ----
            try:
                def _return_to_pose_p():
                    try:
                        print(f"  [{LOG_PREFIX}] 后台回到 P 姿态 "
                              f"(x={pose_p_x_mm} y={pose_p_y_mm} "
                              f"arm={pose_p_arm_deg} hand={pose_p_hand_deg})")
                        _composite_m(car, arm=pose_p_arm_deg, x_mm=pose_p_x_mm,
                                     y_mm=pose_p_y_mm, hand=pose_p_hand_deg,
                                     speed=80, timeout=30.0)
                    except Exception as e:
                        print(f"  [{LOG_PREFIX}] ⚠️ 回 P 姿态失败 "
                              f"({type(e).__name__}: {str(e)[:80]})")
                threading.Thread(target=_return_to_pose_p, daemon=True).start()
            except Exception:
                pass
            # ---- 恢复 arm_feed (幂等) ----
            try:
                car.start_arm_feed(hz=20.0)
                print(f"  [{LOG_PREFIX}] ▶️ start_arm_feed(hz=20)")
            except Exception as e:
                print(f"  [{LOG_PREFIX}] ⚠️ start_arm_feed 失败 "
                      f"({type(e).__name__}: {str(e)[:80]})")

    elapsed = time.monotonic() - t_start
    print(f"\n========== {LOG_PREFIX} 完成 ==========")
    print(f"  reason={final_reason}  picks={n_picks}/{SCAN_MAX_PICKS}  "
          f"skips={n_skips}  pick_failures={n_pick_failures}  "
          f"前移={total_creep_m:.3f}m  elapsed={elapsed:.1f}s")

    return {
        "ok": final_reason in (
            "completed", "zone_cleared", "time_budget", "keyboard_interrupt",
            "align_only",
        ),
        "picked": n_picks,
        "picks": n_picks,
        "skips": n_skips,
        "pick_failures": n_pick_failures,
        "total_creep_m": total_creep_m,
        "history": history,
        "reason": final_reason,
        "elapsed_s": elapsed,
    }


# ---------- 门面 (保持旧接口: Task4Direct / run, run.py 与 actions.py 依赖) ----------

class Task4Direct:
    """用 MyCar 直接完成新版 task4 全流程 (进程内直调, 无网络栈)。"""

    def __init__(self, car, *, max_seconds: float = DEFAULT_MAX_SECONDS,
                 creep_speed_mps: float = DEFAULT_CREEP_SPEED_MPS,
                 track_max_seconds: float = DEFAULT_TRACK_MAX_SECONDS,
                 dry_run: bool = False,
                 pose_p_x_mm: float = TASK4_POSE_P_X_MM,
                 pose_p_y_mm: float = TASK4_POSE_P_Y_MM,
                 pose_p_arm_deg: float = TASK4_POSE_P_ARM_DEG,
                 pose_p_hand_deg: float = TASK4_POSE_P_HAND_DEG):
        self.car = car
        self.max_seconds = max_seconds
        self.creep_speed_mps = creep_speed_mps
        self.track_max_seconds = track_max_seconds
        self.dry_run = dry_run
        self.pose_p_x_mm = pose_p_x_mm
        self.pose_p_y_mm = pose_p_y_mm
        self.pose_p_arm_deg = pose_p_arm_deg
        self.pose_p_hand_deg = pose_p_hand_deg

    def run(self) -> Dict[str, Any]:
        return step_target4(
            self.car,
            dry_run=self.dry_run,
            max_seconds=self.max_seconds,
            creep_speed_mps=self.creep_speed_mps,
            track_max_seconds=self.track_max_seconds,
            pose_p_x_mm=self.pose_p_x_mm,
            pose_p_y_mm=self.pose_p_y_mm,
            pose_p_arm_deg=self.pose_p_arm_deg,
            pose_p_hand_deg=self.pose_p_hand_deg,
        )


def run(car=None, *, dry_run: bool = False, max_seconds: float = DEFAULT_MAX_SECONDS) -> Dict[str, Any]:
    """模块入口: 自建 MyCar (car=None) 或复用传入实例, 跑完可 close。"""
    if dry_run:
        return {"ok": True, "picked": 0, "picks": 0, "reason": "dry_run", "elapsed_s": 0.0}
    owned = car is None
    if owned:
        from runtime.services.my_car import MyCar
        car = MyCar()
    try:
        return Task4Direct(car, max_seconds=max_seconds).run()
    finally:
        if owned:
            car.close()


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        description="task4 底层直连执行 (进程内直调 MyCar, 不经过网络栈)")
    parser.add_argument("--dry-run", action="store_true", help="只打印不动硬件")
    parser.add_argument("--max-seconds", type=float, default=DEFAULT_MAX_SECONDS,
                        help="任务总时长预算 (s)")
    args = parser.parse_args(argv)
    result = run(dry_run=args.dry_run, max_seconds=args.max_seconds)
    print(result)
    return 0 if result["ok"] or args.dry_run else 1


if __name__ == "__main__":
    raise SystemExit(main())
