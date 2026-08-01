"""main/arm/vision/parsers.py — detection JSON → Detection 解析."""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

from .types import BBoxNorm, BBoxPixels, Detection


def _norm_to_pixels(bn: Dict[str, Any], frame_shape) -> Optional[BBoxPixels]:
    """bbox_norm(相对 backend resize 输入) → bbox_pixels(相对 cam 帧).

    公式与 runtime _helpers._bbox_to_pixels 一致:
      center = (norm+1)/2 × img_wh;  box = norm/2 × img_wh.
    归一化基准是 backend resize 后的 416x416, 而 cam 帧分辨率不同;
    中心点在等比 resize 下仍正确, bbox 宽高按物理比例换算 (高度方向
    416→cam_h 的缩放与 norm 基准恰好抵消, 深度公式只消费 height_px).
    frame_shape 缺失 → None (保持旧行为, depth fallback).
    """
    if not frame_shape or len(frame_shape) < 2:
        return None
    img_h, img_w = int(frame_shape[0]), int(frame_shape[1])
    if img_h <= 0 or img_w <= 0:
        return None
    x_c = float(bn.get("x_center", 0.0))
    y_c = float(bn.get("y_center", 0.0))
    width = float(bn.get("width", 0.0))
    height = float(bn.get("height", 0.0))
    center_x = int((x_c + 1) / 2 * img_w)
    center_y = int((y_c + 1) / 2 * img_h)
    box_w = int(width * img_w / 2)
    box_h = int(height * img_h / 2)
    x1 = max(0, int(center_x - box_w / 2))
    y1 = max(0, int(center_y - box_h / 2))
    x2 = int(center_x + box_w / 2)
    y2 = int(center_y + box_h / 2)
    return BBoxPixels(x1, y1, x2, y2, box_w, box_h)


def _parse_cache(raw: Dict[str, Any]) -> List[Detection]:
    """GET /v1/realtime/vision/task 或 WS subscribe_task_detection → List[Detection].

    2026-08-01: task_feed 缓存带 frame_shape 后, bbox_pixels 由 bbox_norm 自算
    (depth-aware 增益因此可在 HTTP / WS 两条伺服路径生效); frame_shape 缺失
    时仍返回 None, 兼容旧后端.
    """
    state = raw.get("task_state") or raw.get("data") or raw
    if "detections" not in state:
        state = raw
    dets = state.get("detections") or []
    now = float(state.get("updated_at") or time.time())
    frame_shape = state.get("frame_shape")
    out: List[Detection] = []
    for d in dets:
        bn = d.get("bbox_norm") or {}
        out.append(Detection(
            label=str(d["label"]),
            score=float(d["score"]),
            track_id=d.get("det_id") or d.get("track_id"),
            class_id=d.get("cls_id") or d.get("class_id"),
            bbox_norm=BBoxNorm(
                float(bn["x_center"]), float(bn["y_center"]),
                float(bn.get("width", 0.0)), float(bn.get("height", 0.0)),
            ),
            bbox_pixels=_norm_to_pixels(bn, frame_shape),
            fetched_at=now,
        ))
    return out


def _parse_sync(raw: Dict[str, Any]) -> List[Detection]:
    """POST /v1/vision/task → List[Detection] (含 bbox_pixels)."""
    dets = raw.get("detections") or []
    now = time.time()
    out: List[Detection] = []
    for d in dets:
        bn = d.get("bbox_norm") or {}
        bp = d.get("bbox_pixels") or None
        out.append(Detection(
            label=str(d["label"]),
            score=float(d["score"]),
            track_id=d.get("track_id"),
            class_id=d.get("class_id"),
            bbox_norm=BBoxNorm(
                float(bn["x_center"]), float(bn["y_center"]),
                float(bn.get("width", 0.0)), float(bn.get("height", 0.0)),
            ),
            bbox_pixels=BBoxPixels(
                int(bp["x1"]), int(bp["y1"]), int(bp["x2"]), int(bp["y2"]),
                int(bp["width"]), int(bp["height"]),
            ) if bp else None,
            fetched_at=now,
        ))
    return out
