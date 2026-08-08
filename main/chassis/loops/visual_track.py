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
    - select_mode: "nearest_to_center" (默认, 画面中心最近) / "largest_area" (面积最大) /
      "leftmost" (画面横向 cx 最小 = 画面最左, 平局取面积大; task4 采收多球场景)

**轴映射（2026-08-02 现场对标）**:
    画面 cx（横向） ↔ 车前后（vx）: cx 负(画面左/靠前) → vx 负(后退)
    画面 cy（纵向） ↔ 车横向（vy）: cy 负(画面上)     → vy 正(右移)
    公式: ``vx = sign_vx * kp * cx_err``, ``vy = sign_vy * kp * cy_err``
    默认 sign_vx=-1, sign_vy=+1。换车/换摄像头只要改这两个 sign 即可。

返回 TrackChassisResult: arrived/reason/final_frame/frames/elapsed
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any, Callable, Collection, Dict, List, Optional, Tuple, Union

from ..api import ChassisClient
from ..controllers.base import mecanum_inverse

logger = logging.getLogger(__name__)


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


SelectMode = str  # "nearest_to_center" | "largest_area" | "leftmost"


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
    if mode == "smallest_area":
        # 2026-08-05 用户: 选最远球 (面积最小); 追最远目标能多前移一些
        return min(matched, key=lambda d: _bbox_area(d))
    if mode == "leftmost":
        # 画面横向 cx 最小 (画面最左); 平局取面积大的 (更可信)
        with_c = [(d, c) for d in matched for c in [_bbox_cx_cy(d)] if c is not None]
        if not with_c:
            return matched[0]
        return min(with_c, key=lambda dc: (dc[1][0], -_bbox_area(dc[0])))[0]
    if mode == "rightmost":
        # 画面横向 cx 最大 (画面最右); 平局取面积大的 (更可信)
        with_c = [(d, c) for d in matched for c in [_bbox_cx_cy(d)] if c is not None]
        if not with_c:
            return matched[0]
        return max(with_c, key=lambda dc: (dc[1][0], -_bbox_area(dc[0])))[0]
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
        # 2026-08-08：显式短超时。默认 request_timeout=10s，runtime 忙时一次 GET 就能把
        # 20Hz 闭环冻住 10s（视觉帧反正下一帧会重读，快速失败比阻塞更合理）。
        payload = api.http.get_vision_task_cache(timeout=1.5)
    except Exception:
        payload = None
    if now_s is None:
        now_s = time.time()

    inner_ts = None
    if isinstance(payload, dict):
        cand = payload.get("task_state")
        inner_ts = cand if isinstance(cand, dict) else None

    # 新鲜度: runtime 的 _StateCache 在每次刷新时写入 task_state.updated_at; 用它
    # 算 age_ms, watchdog (2s 没刷就停) 才有意义。旧实现 age_ms 硬编码 None, watchdog
    # 永远不触发 → 缓存卡死/推理端卸载 (LRU) 时车还按陈旧的 cx/cy 误差继续开。
    # payload 没 updated_at (单测 / 老 runtime) 时 age_ms=None, watchdog 不参与, 行为兼容。
    age_ms = None
    if isinstance(inner_ts, dict):
        ts = inner_ts.get("updated_at")
        try:
            ts = float(ts)
            if ts > 0:
                age_ms = max(0.0, (now_s - ts) * 1000.0)
        except (TypeError, ValueError):
            age_ms = None

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
        age_ms=age_ms,
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
    sign_vx: int = -1,
    sign_vy: int = +1,
    vx_only: bool = False,
    kp: float = 0.20,
    v_max: float = 0.12,
    deadband: float = 0.05,
    hold_frames: int = 5,
    v_slew: Optional[float] = 0.02,
    max_lost_frames: int = 60,
    recover_after_lost: bool = True,
    watchdog_ms: Optional[float] = 2000.0,
    hz: float = 20.0,
    max_seconds: float = 10.0,
    dry_run: bool = False,
    on_tick: Optional[Callable[[TrackFrame, Tuple[float, float]], None]] = None,
    sense_fn: Optional[Callable[[], TrackFrame]] = None,
) -> TrackChassisResult:
    """通用底盘视觉追踪: 把 target bbox 中心拉到 setpoint_cxcy。

    控制律已在 runtime 执行；本函数只做一次 HTTP 同步调用
    ``POST /v1/realtime/chassis-align``，阻塞 1-15s 后返回结果。

    ``on_tick`` / ``sense_fn`` 已废弃（控制律在 runtime 跑，无法注入回调）；
    传入时记录 warning 日志后忽略。
    """
    if api is None:
        api = ChassisClient.connect()
    try:
        if on_tick is not None:
            logger.warning("track_chassis: on_tick ignored (控制律已下沉到 runtime)")
        if sense_fn is not None:
            logger.warning("track_chassis: sense_fn ignored (控制律已下沉到 runtime)")
        resp = api.chassis_align(
            target=target,
            setpoint_cxcy=list(setpoint_cxcy),
            select_mode=select_mode,
            sign_vx=sign_vx, sign_vy=sign_vy, vx_only=vx_only,
            kp=kp, v_max=v_max, deadband=deadband, hold_frames=hold_frames,
            v_slew=v_slew, max_lost_frames=max_lost_frames,
            recover_after_lost=recover_after_lost,
            watchdog_ms=watchdog_ms,
            hz=hz, max_seconds=max_seconds,
            dry_run=dry_run,
        )
    finally:
        try:
            api.close()
        except Exception:
            pass
    if not isinstance(resp, dict):
        return TrackChassisResult(reason="error")
    result_data = resp.get("result", resp)
    if not isinstance(result_data, dict):
        return TrackChassisResult(reason="error")
    final_frame = result_data.get("final_frame")
    if isinstance(final_frame, dict) and final_frame:
        final_frame = TrackFrame(
            target_found=final_frame.get("target_found", False),
            label=final_frame.get("label"),
            cx=final_frame.get("cx"), cy=final_frame.get("cy"),
            area=final_frame.get("area"), score=final_frame.get("score"),
            cx_err=final_frame.get("cx_err"), cy_err=final_frame.get("cy_err"),
            vx=final_frame.get("vx"), vy=final_frame.get("vy"),
            age_ms=final_frame.get("age_ms"),
        )
    else:
        final_frame = None
    return TrackChassisResult(
        arrived=bool(result_data.get("arrived")),
        reason=result_data.get("reason", "unknown"),
        final_frame=final_frame,
        frames=int(result_data.get("frames", 0)),
        elapsed_s=float(result_data.get("elapsed_s", 0.0)),
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