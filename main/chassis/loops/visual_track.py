"""main/chassis/loops/visual_track.py
通用底盘视觉追踪: 基于 cam2 task_feed 缓存, 把选定目标的 bbox_center
拉到指定 setpoint (默认画面中心 cx=0, cy=0)。

底盘通道: /v1/realtime/chassis-velocity (vx, vy, wz=0) — 实时门免 car_lock,
控制律经 mecanum_inverse IK 让 vy/cx、vx/cy 分别映射成横向/前后。

**任选目标** (main API):
    - label 传字符串: "water" = water 组(water/water_l1/l2/l3); "cylinder_2" = 具体
      label; "h_tu_dou" = 土豆; "vegetable" = 整个蔬菜组;
    - labels 传列表: 任一在列表内都算匹配；
    - setpoint 传 (cx,cy): 目标 bbox 中心要落到的画面坐标, 默认 (0,0)；
    - select_mode: "nearest_to_center" (默认, 画面中心最近) / "largest_area" (面积最大)

**轴映射（2026-08-02 现场对标）**:
    画面 cx（横向） ↔ 车前后（vx）: cx 负(画面左/靠前) → vx 负(后退)
    画面 cy（纵向） ↔ 车横向（vy）: cy 负(画面上)     → vy 正(右移)
    公式: ``vx = sign_vx * kp * cx_err``, ``vy = sign_vy * kp * cy_err``
    默认 sign_vx=-1, sign_vy=+1。换车/换摄像头只要改这两个 sign 即可。

返回 TrackChassisResult: arrived/reason/final_frame/frames/elapsed
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable, Collection, Dict, List, Optional, Tuple, Union

from ..api import ChassisClient
from ..controllers.base import mecanum_inverse


# ============ label 展开 ============


def _load_label_groups() -> Dict[str, Tuple[str, ...]]:
    """懒加载 main.arm.labels.LABEL_GROUPS。"""
    try:
        from main.arm.labels import LABEL_GROUPS, Label  # type: ignore
    except Exception:
        return {}
    out: Dict[str, Tuple[str, ...]] = {}
    for k, labels in LABEL_GROUPS.items():
        out[k] = tuple(lab.value for lab in labels if hasattr(lab, "value"))
    return out


_LABEL_GROUPS_CACHE: Optional[Dict[str, Tuple[str, ...]]] = None


def expand_label_set(targets: Union[str, Collection[str]]) -> set:
    """把目标 label(s) 展开成匹配集合。

    规则：
      - "water" → {water, water_l1, water_l2, water_l3}（若组存在）
      - "vegetable" → 整个蔬菜组
      - ["water", "cylinder_2"] → 上面四个 + cylinder_2
      - "cylinder_2" 不是组名 → 单例 {"cylinder_2"}
    """
    global _LABEL_GROUPS_CACHE
    if _LABEL_GROUPS_CACHE is None:
        _LABEL_GROUPS_CACHE = _load_label_groups()

    if isinstance(targets, str):
        raw_list = [targets]
    else:
        raw_list = list(targets)

    out: set = set()
    for t in raw_list:
        if not isinstance(t, str):
            continue
        expanded = _LABEL_GROUPS_CACHE.get(t)
        if expanded:
            out.update(expanded)
        out.add(t)
    return out


# ============ 选择策略 ============


SelectMode = str  # "nearest_to_center" | "largest_area"


def _select_target(
    detections: List[Dict[str, Any]],
    labels: set,
    setpoint_cxcy: Tuple[float, float],
    mode: SelectMode = "nearest_to_center",
) -> Optional[Dict[str, Any]]:
    """从 detections 里选一个匹配目标。返回整个 detection dict 或 None。"""
    if not detections:
        return None
    sx, sy = setpoint_cxcy

    def _bbox_cx_cy(d: Dict[str, Any]) -> Optional[Tuple[float, float]]:
        bb = (d or {}).get("bbox_norm") or {}
        try:
            cx = float(bb.get("cx") if "cx" in bb else bb.get("x_center", 0.0))
            cy = float(bb.get("cy") if "cy" in bb else bb.get("y_center", 0.0))
            return cx, cy
        except Exception:
            return None

    def _bbox_area(d: Dict[str, Any]) -> float:
        bb = (d or {}).get("bbox_norm") or {}
        try:
            w = float(bb.get("width", 0.0))
            h = float(bb.get("height", 0.0))
            return max(0.0, w * h)
        except Exception:
            return 0.0

    matched: List[Dict[str, Any]] = [
        d for d in detections
        if isinstance(d, dict) and (d.get("label", "") or "") in labels
    ]
    if not matched:
        return None
    if mode == "largest_area":
        return max(matched, key=lambda d: _bbox_area(d))
    best_d2: Optional[float] = None
    best_det: Optional[Dict[str, Any]] = None
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


# ============ 感知 ============


@dataclass
class TrackFrame:
    """一帧追踪状态（传给 on_tick / 放在结果里）。"""
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


def _sense_frame(
    api: ChassisClient,
    labels: set,
    setpoint_cxcy: Tuple[float, float],
    mode: SelectMode,
    now_s: Optional[float] = None,
) -> TrackFrame:
    """读一帧 task_feed, 选目标, 组装 TrackFrame。任何异常 → 空 frame。"""
    try:
        payload = api.http.get_vision_task_cache()
    except Exception:
        payload = None
    if now_s is None:
        now_s = time.time()

    inner_ts = None
    if isinstance(payload, dict):
        cand = payload.get("task_state")
        inner_ts = cand if isinstance(cand, dict) else None

    dets: List[Dict[str, Any]] = []
    if isinstance(inner_ts, dict):
        dets = inner_ts.get("detections") or []

    chosen = _select_target(dets, labels, setpoint_cxcy, mode)
    if chosen is None:
        return TrackFrame()

    bb = (chosen or {}).get("bbox_norm") or {}
    try:
        cx = float(bb.get("cx") if "cx" in bb else bb.get("x_center", 0.0))
        cy = float(bb.get("cy") if "cy" in bb else bb.get("y_center", 0.0))
        w = float(bb.get("width", 0.0))
        h = float(bb.get("height", 0.0))
    except Exception:
        return TrackFrame()
    area = w * h
    score = chosen.get("score")
    try:
        score = float(score) if score is not None else None
    except Exception:
        score = None
    sx, sy = setpoint_cxcy

    return TrackFrame(
        target_found=True,
        label=chosen.get("label"),
        cx=cx, cy=cy,
        area=area,
        score=score,
        cx_err=sx - cx,
        cy_err=sy - cy,
        age_ms=None,
    )


# ============ 结果 ============


@dataclass
class TrackChassisResult:
    arrived: bool = False
    reason: str = "unknown"  # arrived / timeout / stopped / watchdog / no_target
    final_frame: Optional[TrackFrame] = None
    frames: int = 0
    elapsed_s: float = 0.0


# ============ 主函数: track_chassis ============


def track_chassis(
    target: Union[str, Collection[str]] = "h_tu_dou",
    *,
    api: Optional[ChassisClient] = None,
    setpoint_cxcy: Tuple[float, float] = (0.0, 0.0),
    select_mode: SelectMode = "nearest_to_center",
    # 现场调好的轴符号（2026-08-02 现场对标：画面x↔车前后vx, 画面y↔车横向vy）
    #   sign_vx=-1, sign_vy=+1 是现场确认的方向：cx 负(画面左/靠前) → vx 负(后退)；
    #                                                    cy 负(画面上)     → vy 正(右移)
    # 如果换车/换摄像头，只要改这两个 sign 不需要动控制律
    sign_vx: int = -1,
    sign_vy: int = +1,
    # 控制律（2026-08-02 真机 cylinder_2/h_tu_dou 稳档：纯 P 振荡，所以 kp 保守 + slew 限幅）
    kp: float = 0.20,          # 比例增益（降 55% 防振荡）
    v_max: float = 0.12,        # 单向速度上限（绝对值，比前次 0.20 降 40%）
    deadband: float = 0.08,     # cx/cy 误差都 < 死区 → 判到带内
    hold_frames: int = 5,       # 连续 5 帧带内 → arrival（3 帧擦过不算）
    # 限速/平滑
    v_slew: Optional[float] = 0.02,     # 每帧 vx/vy 最多变 ±0.02 m/s（20Hz=0.4m/s²，不会爆冲）
    max_lost_frames: int = 60,          # 连续丢 60 帧(≈3s@20Hz) → 停
    recover_after_lost: bool = True,    # 短时丢帧后 1 帧反向小搜
    watchdog_ms: Optional[float] = 2000.0,  # task_feed 2s 没刷 → 停
    # 调度
    hz: float = 20.0,
    max_seconds: float = 10.0,
    dry_run: bool = False,
    on_tick: Optional[Callable[[TrackFrame, Tuple[float, float]], None]] = None,
) -> TrackChassisResult:
    """通用底盘视觉追踪: 把 target bbox 中心拉到 setpoint_cxcy。

    任选目标:
      - ``target="h_tu_dou"``  → 土豆(默认)
      - ``target="water"``     → 匹配 water / water_l1/l2/l3 任一个
      - ``target="cylinder_2"`` → 圆柱体 2 号
      - ``target=["water_l1","water_l2"]`` → 列表任一匹配
      - ``setpoint_cxcy=(-0.1, 0.1)`` → 对齐到非画面中心的标定点

    控制律（2026-08-02 现场调好的轴映射 + 符号）:
      - 画面 cx（横向） ↔ 车前后（vx）: cx 负(画面左/靠前) → vx 负(后退)
      - 画面 cy（纵向） ↔ 车横向（vy）: cy 负(画面上)     → vy 正(右移)
      - 公式: ``vx = sign_vx * kp * cx_err``, ``vy = sign_vy * kp * cy_err``
      - 如果发现反了，改 ``sign_vx=+/-1`` / ``sign_vy=+/-1`` 即可，不动控制律
      - wz = 0 全程,**不旋转**

    返回 ``TrackChassisResult``: arrived=True / 到达帧信息 / 跑了多少帧。
    """
    if api is None:
        api = ChassisClient.connect()
    labels = expand_label_set(target)
    if not labels:
        return TrackChassisResult(reason="no_target")

    period = 1.0 / max(hz, 1.0)
    start = time.monotonic()
    deadline = start + max(0.0, float(max_seconds))
    next_tick = time.monotonic()

    def _set_vel(vx: float, vy: float) -> None:
        if dry_run:
            return
        try:
            ws = mecanum_inverse(vx, vy, 0.0, 0.30)
            api.set_wheel_speeds(ws)
        except Exception:
            pass
        try:
            api.http.post(
                "/v1/realtime/chassis-velocity",
                {"vx": float(vx), "vy": float(vy), "wz": 0.0},
                timeout=1.5,
            )
        except Exception:
            pass

    frames = 0
    in_band = 0
    last_vx = 0.0
    last_vy = 0.0
    lost_frames = 0
    final_frame: Optional[TrackFrame] = None
    arrived = False
    reason = "timeout"
    stopped = False
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

            frm = _sense_frame(api, labels, setpoint_cxcy, select_mode)
            frames += 1
            final_frame = frm

            if watchdog_ms is not None and frm.age_ms is not None and frm.target_found:
                if frm.age_ms > watchdog_ms:
                    watchdog_triggered = True
                    break

            if not frm.target_found:
                lost_frames += 1
                in_band = 0
                if lost_frames == 1 and recover_after_lost and (last_vx != 0.0 or last_vy != 0.0):
                    vx, vy = -last_vx * 0.5, -last_vy * 0.5
                else:
                    vx, vy = 0.0, 0.0
                _set_vel(vx, vy)
                frm.vx, frm.vy = vx, vy
                if lost_frames > max_lost_frames:
                    reason = "no_target"
                    stopped = True
                    break
                if on_tick is not None:
                    try:
                        on_tick(frm, (vx, vy))
                    except Exception:
                        pass
                continue

            lost_frames = 0

            cx_err = frm.cx_err if frm.cx_err is not None else 0.0
            cy_err = frm.cy_err if frm.cy_err is not None else 0.0
            # 控制律（含 sign_vx / sign_vy，2026-08-02 现场调好）
            vx = float(sign_vx) * float(kp) * float(cx_err)
            vy = float(sign_vy) * float(kp) * float(cy_err)
            if vx > v_max:
                vx = v_max
            elif vx < -v_max:
                vx = -v_max
            if vy > v_max:
                vy = v_max
            elif vy < -v_max:
                vy = -v_max
            if v_slew is not None:
                dvx = vx - last_vx
                if abs(dvx) > v_slew:
                    vx = last_vx + v_slew if dvx > 0 else last_vx - v_slew
                dvy = vy - last_vy
                if abs(dvy) > v_slew:
                    vy = last_vy + v_slew if dvy > 0 else last_vy - v_slew

            in_deadband = abs(cx_err) < deadband and abs(cy_err) < deadband
            if in_deadband:
                vx = 0.0
                vy = 0.0
                in_band += 1
                if in_band >= hold_frames:
                    arrived = True
                    reason = "arrived"
                    break
            else:
                in_band = 0

            last_vx, last_vy = vx, vy
            frm.vx = vx
            frm.vy = vy
            _set_vel(vx, vy)
            if on_tick is not None:
                try:
                    on_tick(frm, (vx, vy))
                except Exception:
                    pass
    finally:
        _set_vel(0.0, 0.0)
        try:
            api.close()
        except Exception:
            pass

    if watchdog_triggered:
        reason = "watchdog"
    if stopped and reason == "unknown":
        reason = "stopped"

    elapsed = time.monotonic() - start
    return TrackChassisResult(
        arrived=arrived,
        reason=reason,
        final_frame=final_frame,
        frames=frames,
        elapsed_s=elapsed,
    )


def track_trace(every_n: int = 1) -> Callable[[TrackFrame, Tuple[float, float]], None]:
    """on_tick=track_trace(1): 每 N 帧打印一行追踪信息。"""
    counter = {"n": 0}

    def _cb(frm: TrackFrame, wheels_v: Tuple[float, float]) -> None:
        counter["n"] += 1
        n = counter["n"]
        if every_n > 1 and n % every_n != 0:
            return
        vx, vy = wheels_v
        label = frm.label or "-"
        cx = "%.3f" % frm.cx if frm.cx is not None else "-"
        cy = "%.3f" % frm.cy if frm.cy is not None else "-"
        cx_e = "%+.3f" % frm.cx_err if frm.cx_err is not None else "-"
        cy_e = "%+.3f" % frm.cy_err if frm.cy_err is not None else "-"
        score = "%.2f" % frm.score if frm.score is not None else "-"
        print(
            "[track] n=%d found=%s label=%s score=%s cx=%s cy=%s "
            "err(cx,cy)=(%s,%s) vx=%+.3f vy=%+.3f"
            % (n, frm.target_found, label, score, cx, cy, cx_e, cy_e, vx, vy)
        )

    return _cb


__all__ = [
    "track_chassis",
    "TrackChassisResult",
    "TrackFrame",
    "expand_label_set",
    "track_trace",
]