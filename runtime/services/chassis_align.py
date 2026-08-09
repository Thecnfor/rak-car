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

import numpy as np

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
    reason: str = "unknown"  # arrived / timeout / watchdog / no_target / control_lost
    final_frame: Optional[TrackFrame] = None
    frames: int = 0
    elapsed_s: float = 0.0
    # 2026-08-09: 闭环 finally 的零速是否真的下发到轮子 (False = 主路径 + 兜底
    # 都失败, 底盘可能仍按最后非零指令滑行)。下沉后客户端看不到帧内异常, 只能
    # 靠这个字段判断"车到底停了没有"。
    stop_ok: bool = True
    # 2026-08-09: 对齐期间轮子是否物理位移 (真实编码器反馈, 非命令积分)。
    # 串口/下位机假死时 set_chassis_velocity 可能不报错但轮子不转 (HTTP 200
    # 但 no-motion) → 即使 stop_ok=True 车也没真正对齐 → motion_ok=False。
    # 有下发命令但编码器没动 = 命令路径假死; 无命令 (目标已居中) → True。
    motion_ok: bool = True
    enc_delta: Optional[float] = None


# ---- 目标轨迹 Kalman 平滑（2026-08-09）----


class _KalmanTracker:
    """常速 Kalman: 平滑 task_feed 检测的 cx/cy, 抑制 bbox 帧间抖动。

    封装 filterpy.kalman.KalmanFilter（CV 模型, 状态 [cx, cy, vcx, vcy]）。
    filterpy 是纯 numpy 库, Python 3.8 兼容; 未安装时由 ChassisAlignController
    自动禁用（kalman=False 降级回原始检测）。

    只处理**有检测帧**; 丢帧由外层原逻辑（recover/max_lost_frames）处理,
    本类不参与——不改动已验证的丢帧行为。Q 小（目标近似静止/缓动）,
    R 控制平滑强度（bbox 抖动大 → 放大 R 更平滑, 但跟踪变慢）。
    """

    def __init__(self, dt: float = 0.05, q: float = 1e-3, r: float = 1e-2):
        from filterpy.kalman import KalmanFilter  # 懒加载: 未装则上层降级
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.F = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
            dtype=float,
        )
        self.kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        self.kf.P *= 1.0
        self.kf.Q = np.eye(4) * q
        self.kf.R = np.eye(2) * r
        self._initialized = False

    def update(self, cx: float, cy: float) -> Tuple[float, float]:
        """喂一帧检测, 返回平滑后的 (cx, cy)。首帧直接初始化（不过滤）。"""
        z = np.array([[cx], [cy]], dtype=float)
        if not self._initialized:
            self.kf.x = np.array([[cx], [cy], [0.0], [0.0]], dtype=float)
            self._initialized = True
            return cx, cy
        self.kf.predict()
        self.kf.update(z)
        return float(self.kf.x[0, 0]), float(self.kf.x[1, 0])


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
                 hz=20.0, max_seconds=10.0, dry_run=False,
                 kalman=True,
                 max_control_fail_frames: int = 10,
                 decouple_xy: bool = True):
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
        # 2026-08-09 用户: 微调只动对角轮会打滑 → 4 轮一起平移。麦轮 IK 在
        # |vx|≈|vy| 的 45° 对角平移时, (vx±vy) 把一对对角轮置 0 → 只剩 2 轮
        # 提供牵引 → 打滑。decouple_xy=True 时每帧只驱动误差较大的单轴
        # (另一轴 0): 纯 x / 纯 y 平移 4 轮全动, 永不出现对角死对。
        # decouple_xy=False 保留旧对角平移 (真机对拍用)。
        self._decouple_xy = bool(decouple_xy)
        # 2026-08-09: 命令路径连续失败快速退出。串口/下位机掉线时 _set_vel 一直 False,
        # 视觉却仍活 (task_feed 独立于 MC602) → cx_err 不收敛 → 满预算 timeout 才退,
        # task 每球白烧 max_seconds。连续失败 max_control_fail_frames 帧 (默认 10 ≈ 0.5s
        # @20Hz) → reason=control_lost 提前退出, 任务层可立即重武装而不是干等。
        self._max_control_fail_frames = int(max(1, max_control_fail_frames))
        # decouple_xy 轴滞回状态 (2026-08-09): 记录上次驱动的轴, |cx|≈|cy| 不换轴。
        self._last_axis = None
        # Kalman 平滑（默认开, 2026-08-09 用户决定; kalman=False 显式关保持原始
        # 检测）: 有检测帧时平滑 cx/cy 抑制 bbox 抖动。filterpy 未安装 → 自动禁用。
        self._kalman = None
        if kalman:
            try:
                self._kalman = _KalmanTracker(dt=1.0 / max(self._hz, 1.0))
            except Exception:
                logger.warning(
                    "filterpy 未安装, kalman 禁用 (Jetson 需: pip install filterpy)"
                )
                self._kalman = None

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
            """下发底盘三速; 返回是否成功 (False = 主路径 + 兜底都失败)."""
            if self._dry_run:
                return True
            try:
                # 优先走 service 直发（内部 IK + 命令追踪）
                self._service.set_chassis_velocity(vx, vy, 0.0)
                return True
            except Exception:
                try:
                    # 兜底: 本地 IK 直发轮速
                    car_ref = self._service.car
                    if car_ref is not None:
                        ws = list(car_ref.chassis.calculate_wheel_velocities(vx, vy, 0.0))
                        self._service.set_wheel_speeds([float(s) for s in ws])
                        return True
                except Exception:
                    pass
                return False

        frames = 0
        in_band = 0
        last_vx = 0.0
        last_vy = 0.0
        lost_frames = 0
        final_frame = None
        arrived = False
        reason = "timeout"
        watchdog_triggered = False
        stop_ok = True
        _ctl_fail_streak = 0
        max_commanded = 0.0

        def _read_encoders():
            """读真实 4 轮编码器 (弧度累计, MC602 反馈); 失败/无端点 → None."""
            try:
                fn = getattr(self._service, "get_wheel_encoders", None)
                if fn is None:
                    return None
                e = fn()
                if isinstance(e, dict):
                    e = e.get("encoders")
                if isinstance(e, (list, tuple)) and len(e) == 4:
                    return [float(x) for x in e]
                return None
            except Exception:
                return None

        enc0 = _read_encoders()

        def _send(vx, vy) -> bool:
            """下发三速并跟踪连续失败; 连续失败超阈值 → reason=control_lost, 返回 False."""
            nonlocal _ctl_fail_streak, reason, max_commanded
            max_commanded = max(max_commanded, abs(vx), abs(vy))
            if _set_vel(vx, vy):
                _ctl_fail_streak = 0
                return True
            _ctl_fail_streak += 1
            if _ctl_fail_streak >= self._max_control_fail_frames:
                reason = "control_lost"
                return False
            return True

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

                # Kalman 平滑（可选）: 只处理有检测帧, 用平滑后位置重算误差。
                # 丢帧帧走原逻辑, 本处不参与。
                if self._kalman is not None and frm.target_found:
                    try:
                        cx_s, cy_s = self._kalman.update(frm.cx, frm.cy)
                        sx, sy = self._setpoint_cxcy
                        frm.cx, frm.cy = cx_s, cy_s
                        frm.cx_err = sx - cx_s
                        frm.cy_err = sy - cy_s
                    except Exception:
                        pass  # kalman 异常回退原始检测帧

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
                    # 连丢 2 帧才反向回拉 (0.25 倍) —— 单帧闪烁只停不反向,
                    # 否则 "正向(找到)↔反向(丢帧)" 交替 = 来回晃 (2026-08-09 现场)。
                    if (lost_frames == 2 and self._recover_after_lost
                            and (last_vx != 0.0 or last_vy != 0.0)):
                        vx, vy = -last_vx * 0.25, -last_vy * 0.25
                    else:
                        vx, vy = 0.0, 0.0
                    if not _send(vx, vy):
                        break
                    frm.vx, frm.vy = vx, vy
                    if lost_frames > self._max_lost_frames:
                        reason = "no_target"
                        break
                    continue

                lost_frames = 0
                cx_err = frm.cx_err if frm.cx_err is not None else 0.0
                cy_err = frm.cy_err if frm.cy_err is not None else 0.0

                # P 控制律
                if self._vx_only:
                    # task6 LLM-as-servo: 只动 x
                    vx = float(self._sign_vx) * float(self._kp) * float(cx_err)
                    vy = 0.0
                elif self._decouple_xy:
                    # 4 轮一起平移 (2026-08-09): 每帧只驱动误差较大的单轴, 另一轴 0。
                    # 同时驱动 vx+vy 且 |vx|≈|vy| 时 IK 置零一对对角轮 → 打滑。
                    # 轴滞回 (2026-08-09): |cx|≈|cy| 时避免每帧换轴 → 对角来回晃。
                    # 已选轴保持, 除非另一轴误差 > 1.2x 才切换。
                    if self._last_axis == "x" and abs(cy_err) > abs(cx_err) * 1.2:
                        self._last_axis = "y"
                    elif self._last_axis == "y" and abs(cx_err) > abs(cy_err) * 1.2:
                        self._last_axis = "x"
                    elif self._last_axis not in ("x", "y"):
                        self._last_axis = "x" if abs(cx_err) >= abs(cy_err) else "y"
                    if self._last_axis == "x":
                        vx = float(self._sign_vx) * float(self._kp) * float(cx_err)
                        vy = 0.0
                    else:
                        vx = 0.0
                        vy = float(self._sign_vy) * float(self._kp) * float(cy_err)
                else:
                    vx = float(self._sign_vx) * float(self._kp) * float(cx_err)
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
                if not _send(vx, vy):
                    break
        finally:
            stop_ok = _set_vel(0.0, 0.0)
            enc1 = _read_encoders()

        if watchdog_triggered:
            reason = "watchdog"

        # 物理位移判定: 命令路径假死 (200 但轮不转) 时编码器位移 ~0。
        enc_delta = None
        if enc0 is not None and enc1 is not None:
            enc_delta = sum(abs(enc1[i] - enc0[i]) for i in range(4))
        motion_ok = (max_commanded < 0.001) or (
            enc_delta is not None and enc_delta >= 1.0)

        elapsed = time.monotonic() - start
        result = TrackChassisResult(
            arrived=arrived,
            reason=reason,
            final_frame=final_frame,
            frames=frames,
            elapsed_s=elapsed,
            stop_ok=stop_ok,
            motion_ok=motion_ok,
            enc_delta=enc_delta,
        )
        return result.__dict__
