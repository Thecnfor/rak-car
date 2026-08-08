"""runtime/services/chassis_align.py
Server-side 视觉对齐闭环控制器。

把 main/chassis/loops/visual_track.py 的控制律搬到 runtime 单进程：
  - _expand_label_set / _select_target（纯函数，1:1 复制 client 实现）
  - ChassisAlignController.run() — 50Hz 闭环，读 task_state 内存缓存，
    下发三速直发，直到 arrived / timeout / watchdog / no_target。

调用方: runtime/api/routers/realtime.py POST /v1/realtime/chassis-align
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any, Collection, Dict, List, Optional, Tuple, Union

logger = logging.getLogger(__name__)

# ---- label 组定义 ----
# 与 main/arm/labels.py::LABEL_GROUPS 保持同步。runtime 不能 import main，
# 所以内嵌一份静态映射；部署两边一致即可。
_LABEL_GROUPS: Dict[str, Tuple[str, ...]] = {
    "animal": ("animal",),
    "ball": ("ball_blue", "ball_yellow"),
    "cylinder": ("cylinder_1", "cylinder_2", "cylinder_3"),
    "cylinder_meta": ("cylinder_set",),
    "vegetable": (
        "h_dou_jiao", "h_fan_qie", "h_jin_zhen_gu",
        "h_mo_gu", "h_qin_cai", "h_qing_jiao",
        "h_tu_dou", "h_xi_lan_hua", "h_you_cai",
    ),
    "water": ("water", "water_l1", "water_l2", "water_l3"),
}


# ---- 纯函数（1:1 对应 client 端）----


def _expand_label_set(targets: Union[str, Collection[str]]) -> set:
    """把目标 label(s) 展开成匹配集合。"""
    if isinstance(targets, str):
        raw_list = [targets]
    else:
        raw_list = list(targets)
    out: set = set()
    for t in raw_list:
        if not isinstance(t, str):
            continue
        expanded = _LABEL_GROUPS.get(t)
        if expanded:
            out.update(expanded)
        out.add(t)
    return out


def _select_target(
    detections: List[Dict[str, Any]],
    labels: set,
    setpoint_cxcy: Tuple[float, float],
    mode: str = "nearest_to_center",
) -> Optional[Dict[str, Any]]:
    """从 detections 里选一个匹配目标。返回整个 detection dict 或 None。"""
    if not detections:
        return None
    sx, sy = setpoint_cxcy

    def _bbox_cx_cy(d):
        bb = (d or {}).get("bbox_norm") or {}
        try:
            cx = float(bb.get("cx") if "cx" in bb else bb.get("x_center", 0.0))
            cy = float(bb.get("cy") if "cy" in bb else bb.get("y_center", 0.0))
            return cx, cy
        except Exception:
            return None

    def _bbox_area(d):
        bb = (d or {}).get("bbox_norm") or {}
        try:
            w = float(bb.get("width", 0.0))
            h = float(bb.get("height", 0.0))
            return max(0.0, w * h)
        except Exception:
            return 0.0

    matched = [
        d for d in detections
        if isinstance(d, dict) and (d.get("label", "") or "") in labels
    ]
    if not matched:
        return None
    if mode == "largest_area":
        return max(matched, key=_bbox_area)
    if mode == "smallest_area":
        return min(matched, key=_bbox_area)
    if mode == "leftmost":
        with_c = [(d, c) for d in matched for c in [_bbox_cx_cy(d)] if c is not None]
        if not with_c:
            return matched[0]
        return min(with_c, key=lambda dc: (dc[1][0], -_bbox_area(dc[0])))[0]
    if mode == "rightmost":
        with_c = [(d, c) for d in matched for c in [_bbox_cx_cy(d)] if c is not None]
        if not with_c:
            return matched[0]
        return max(with_c, key=lambda dc: (dc[1][0], -_bbox_area(dc[0])))[0]
    # nearest_to_center
    best_d2 = None
    best_det = None
    for d in matched:
        c = _bbox_cx_cy(d)
        if not c:
            continue
        cx, cy = c
        d2 = (cx - sx) ** 2 + (cy - sy) ** 2
        if best_d2 is None or d2 < best_d2:
            best_d2 = d2
            best_det = d
    return best_det


# ---- 结果类型 ----


@dataclass
class TrackFrame:
    """一帧追踪状态。"""
    target_found: bool = False
    label: Optional[str] = None
    cx: Optional[float] = None
    cy: Optional[float] = None
    area: Optional[float] = None
    score: Optional[float] = None
    cx_err: Optional[float] = None
    cy_err: Optional[float] = None
    vx: Optional[float] = None
    vy: Optional[float] = None
    age_ms: Optional[float] = None


@dataclass
class TrackChassisResult:
    arrived: bool = False
    reason: str = "unknown"  # arrived / timeout / watchdog / no_target
    final_frame: Optional[TrackFrame] = None
    frames: int = 0
    elapsed_s: float = 0.0


# ---- 主控制器 ----


class ChassisAlignController:
    """Server-side 视觉对齐闭环。

    1:1 复刻 main/chassis/loops/visual_track.py::track_chassis 控制律，
    但读 task_state 内存缓存（不走 HTTP GET），下发走 service 直发
    （不走 HTTP POST）。

    调用方式:
        ctrl = ChassisAlignController(service=self, target="h_tu_dou", ...)
        result = ctrl.run()  # dict
    """

    def __init__(self, service, *, target, setpoint_cxcy=(0.0, 0.0),
                 select_mode="nearest_to_center",
                 sign_vx=-1, sign_vy=1, vx_only=False,
                 kp=0.20, v_max=0.12, deadband=0.05, hold_frames=5,
                 v_slew=0.02, max_lost_frames=60, recover_after_lost=True,
                 watchdog_ms=2000.0,
                 hz=20.0, max_seconds=10.0, dry_run=False):
        self._service = service
        self._target = target
        self._setpoint_cxcy = tuple(float(x) for x in setpoint_cxcy)
        self._select_mode = select_mode
        self._sign_vx = int(sign_vx)
        self._sign_vy = int(sign_vy)
        self._vx_only = bool(vx_only)
        self._kp = float(kp)
        self._v_max = float(v_max)
        self._deadband = float(deadband)
        self._hold_frames = int(hold_frames)
        self._v_slew = float(v_slew) if v_slew is not None else None
        self._max_lost_frames = int(max_lost_frames)
        self._recover_after_lost = bool(recover_after_lost)
        self._watchdog_ms = float(watchdog_ms) if watchdog_ms is not None else None
        self._hz = float(hz)
        self._max_seconds = float(max_seconds)
        self._dry_run = bool(dry_run)

    def run(self) -> dict:
        """执行对齐闭环，返回 TrackChassisResult.__dict__。"""
        labels = _expand_label_set(self._target)
        if not labels:
            return TrackChassisResult(reason="no_target").__dict__

        period = 1.0 / max(self._hz, 1.0)
        start = time.monotonic()
        deadline = start + max(self._max_seconds, 0.0)
        next_tick = time.monotonic()

        def _set_vel(vx, vy):
            if self._dry_run:
                return
            try:
                # 优先走 service 直发（内部 IK + 命令追踪）
                self._service.set_chassis_velocity(vx, vy, 0.0)
            except Exception:
                try:
                    # 兜底: 本地 IK 直发轮速
                    car_ref = self._service.car
                    if car_ref is not None:
                        ws = list(car_ref.chassis.calculate_wheel_velocities(vx, vy, 0.0))
                        self._service.set_wheel_speeds([float(s) for s in ws])
                except Exception:
                    pass

        frames = 0
        in_band = 0
        last_vx = 0.0
        last_vy = 0.0
        lost_frames = 0
        final_frame = None
        arrived = False
        reason = "timeout"
        watchdog_triggered = False

        try:
            while True:
                now = time.monotonic()
                if now > deadline:
                    reason = "timeout"
                    break
                sleep_s = next_tick - now
                if sleep_s > 0:
                    time.sleep(sleep_s)
                next_tick += period
                if next_tick < now:
                    next_tick = now + period

                # 读 task_state 内存缓存（微秒级，不走 HTTP）
                try:
                    ts = self._service.get_task_state()
                except Exception:
                    ts = None

                now_s = time.time()
                age_ms = None
                dets: List[Dict] = []
                if isinstance(ts, dict):
                    # get_task_state() 返回 flat dict {active, detections, updated_at, ...}
                    # 兼容嵌套格式（如 {"ok": True, "task_state": {...}}）
                    inner = ts.get("task_state", ts) if "task_state" in ts else ts
                    if isinstance(inner, dict):
                        ts_updated = inner.get("updated_at")
                        if ts_updated is not None:
                            try:
                                if float(ts_updated) > 0:
                                    age_ms = max(0.0, (now_s - float(ts_updated)) * 1000.0)
                            except (TypeError, ValueError):
                                pass
                        dets = inner.get("detections") or []

                chosen = _select_target(dets, labels, self._setpoint_cxcy, self._select_mode)

                if chosen is None:
                    frm = TrackFrame(age_ms=age_ms)
                else:
                    bb = (chosen or {}).get("bbox_norm") or {}
                    try:
                        cx = float(bb.get("cx") if "cx" in bb else bb.get("x_center", 0.0))
                        cy = float(bb.get("cy") if "cy" in bb else bb.get("y_center", 0.0))
                        w = float(bb.get("width", 0.0))
                        h = float(bb.get("height", 0.0))
                    except Exception:
                        frm = TrackFrame(age_ms=age_ms)
                        chosen = None
                    else:
                        area = w * h
                        score = chosen.get("score")
                        try:
                            score = float(score) if score is not None else None
                        except Exception:
                            score = None
                        sx, sy = self._setpoint_cxcy
                        frm = TrackFrame(
                            target_found=True,
                            label=chosen.get("label"),
                            cx=cx, cy=cy,
                            area=area, score=score,
                            cx_err=sx - cx, cy_err=sy - cy,
                            age_ms=age_ms,
                        )

                frames += 1
                final_frame = frm

                # watchdog: 缓存超时
                if (self._watchdog_ms is not None
                        and frm.age_ms is not None
                        and frm.target_found):
                    if frm.age_ms > self._watchdog_ms:
                        watchdog_triggered = True
                        break

                # 目标丢失
                if not frm.target_found:
                    lost_frames += 1
                    in_band = 0
                    if (lost_frames == 1 and self._recover_after_lost
                            and (last_vx != 0.0 or last_vy != 0.0)):
                        vx, vy = -last_vx * 0.5, -last_vy * 0.5
                    else:
                        vx, vy = 0.0, 0.0
                    _set_vel(vx, vy)
                    frm.vx, frm.vy = vx, vy
                    if lost_frames > self._max_lost_frames:
                        reason = "no_target"
                        break
                    continue

                lost_frames = 0
                cx_err = frm.cx_err if frm.cx_err is not None else 0.0
                cy_err = frm.cy_err if frm.cy_err is not None else 0.0

                # P 控制律
                vx = float(self._sign_vx) * float(self._kp) * float(cx_err)
                if self._vx_only:
                    vy = 0.0
                else:
                    vy = float(self._sign_vy) * float(self._kp) * float(cy_err)

                # v_max 限幅
                if vx > self._v_max:
                    vx = self._v_max
                elif vx < -self._v_max:
                    vx = -self._v_max
                if vy > self._v_max:
                    vy = self._v_max
                elif vy < -self._v_max:
                    vy = -self._v_max

                # v_slew 限幅
                if self._v_slew is not None:
                    dvx = vx - last_vx
                    if abs(dvx) > self._v_slew:
                        vx = last_vx + self._v_slew if dvx > 0 else last_vx - self._v_slew
                    dvy = vy - last_vy
                    if abs(dvy) > self._v_slew:
                        vy = last_vy + self._v_slew if dvy > 0 else last_vy - self._v_slew

                # deadband + arrived
                if self._vx_only:
                    in_deadband = abs(cx_err) < self._deadband
                else:
                    in_deadband = abs(cx_err) < self._deadband and abs(cy_err) < self._deadband
                if in_deadband:
                    vx = 0.0
                    vy = 0.0
                    in_band += 1
                    if in_band >= self._hold_frames:
                        arrived = True
                        reason = "arrived"
                        break
                else:
                    in_band = 0

                last_vx, last_vy = vx, vy
                frm.vx, frm.vy = vx, vy
                _set_vel(vx, vy)
        finally:
            _set_vel(0.0, 0.0)

        if watchdog_triggered:
            reason = "watchdog"

        elapsed = time.monotonic() - start
        result = TrackChassisResult(
            arrived=arrived,
            reason=reason,
            final_frame=final_frame,
            frames=frames,
            elapsed_s=elapsed,
        )
        return result.__dict__
