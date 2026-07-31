"""main/arm/vision/parsers.py — detection JSON → Detection 解析."""
from __future__ import annotations

import time
from typing import Any, Dict, List

from .types import BBoxNorm, BBoxPixels, Detection


def _parse_cache(raw: Dict[str, Any]) -> List[Detection]:
    """GET /v1/realtime/vision/task 或 WS subscribe_task_detection → List[Detection] (无 bbox_pixels)."""
    state = raw.get("task_state") or raw.get("data") or raw
    if "detections" not in state:
        state = raw
    dets = state.get("detections") or []
    now = float(state.get("updated_at") or time.time())
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
            bbox_pixels=None,
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
