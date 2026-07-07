"""Detector — draw detection boxes on frames.

The /api/cameras/<id>/detections/mjpeg endpoint reads raw frames from the
Camera, runs them through `BaseDetector.detect(frame)`, draws the returned
detections on the frame with cv2, and returns the annotated MJPEG.

Two implementations:

  * `NoneDetector` (the default) — always returns []. The detection stream
    serves the raw frame unchanged. No fake boxes. When the real
    ZMQ-based detector is wired in, replace this with a real one.

  * `MockDetector` (opt-in via RAK_DETECTOR=mock) — synthesizes plausible
    PERSON/VEHICLE/SIGN/PLANT/PEST/QR tracks. Watermarked as a demo only;
    never the default so real cameras never show fake data.

When the real ZMQ/LaneInfer/OCR pipeline is wired in (via MyCar's
`ClintInterface`), add a `ZmqDetector` that calls `clinet.task_det(img)`
etc. and normalizes the response to the same `Detection` shape. The draw
code is type-agnostic.
"""

from __future__ import annotations

import abc
import math
import random
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np


# Themed classes for rak-car's agricultural setting. Colors are BGR
# tuples ready for cv2 drawing.
CLASSES = {
    "person":  {"label": "PERSON",    "color": (197, 255, 58),   "color_name": "acid"},   # acid green
    "vehicle": {"label": "VEHICLE",   "color": (255, 224, 102),  "color_name": "cyan"},   # cyan
    "sign":    {"label": "SIGN",      "color": (84, 180, 255),   "color_name": "amber"},  # amber
    "plant":   {"label": "PLANT",     "color": (140, 255, 160),  "color_name": "lime"},   # light green
    "pest":    {"label": "PEST",      "color": (112, 88, 255),   "color_name": "red"},    # red
    "qr":      {"label": "QR",        "color": (200, 200, 200),  "color_name": "white"},  # gray
}


@dataclass
class Detection:
    x: float       # top-left in image-space
    y: float
    w: float
    h: float
    class_id: str
    label: str
    confidence: float
    track_id: int
    age_s: float = 0.0
    trail: List[Tuple[float, float]] = field(default_factory=list)


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _smoothstep(t: float) -> float:
    return t * t * (3 - 2 * t)


class BaseDetector(abc.ABC):
    """Abstract detector — subclasses implement `detect(frame)`."""

    @abc.abstractmethod
    def detect(self, frame: np.ndarray) -> List["Detection"]:
        ...


class NoneDetector(BaseDetector):
    """Default. Returns an empty list — no boxes drawn, no fake data."""

    def detect(self, frame: np.ndarray) -> List["Detection"]:
        return []


class MockDetector(BaseDetector):
    """Maintains a small set of persistent tracks that drift around the
    frame, with smooth motion. Each track has a class, a confidence that
    gently oscillates, and a short position trail. New tracks spawn and
    old ones retire every few seconds, so the panel always feels alive."""

    def __init__(self, seed: int = 0) -> None:
        self._lock = threading.RLock()
        self._rng = random.Random(seed)
        self._t0 = time.time()
        self._tracks: list[dict] = []
        self._next_id = 1
        self._last_spawn = 0.0
        self._spawn()

    def detect(self, frame: np.ndarray) -> List[Detection]:
        h, w = frame.shape[:2]
        now = time.time()
        t = now - self._t0
        with self._lock:
            # retire old tracks
            self._tracks = [tr for tr in self._tracks if now - tr["born"] < tr["ttl"]]
            # spawn a new one every ~3s
            if now - self._last_spawn > 3.0 and len(self._tracks) < 4:
                self._spawn()
                self._last_spawn = now

            detections: List[Detection] = []
            for tr in self._tracks:
                # smooth back-and-forth motion around the track's anchor
                phase = tr["phase"]
                tx, ty = tr["anchor"]
                rx, ry = tr["radius"]
                fx = _lerp(tx, tx + rx * math.cos(t * tr["omega"] + phase), 1.0)
                fy = _lerp(ty, ty + ry * math.sin(t * tr["omega"] * 0.7 + phase), 1.0)
                # box dims, slight breathing
                bw = tr["bw"] * (1.0 + 0.08 * math.sin(t * 2.3 + phase))
                bh = tr["bh"] * (1.0 + 0.08 * math.cos(t * 2.1 + phase))
                # confidence oscillates around the track's base
                conf = max(
                    0.05,
                    min(
                        0.99,
                        tr["conf_base"] + 0.18 * math.sin(t * 1.6 + phase * 0.7),
                    ),
                )
                # update trail (cap 8 points)
                tr["trail"].append((fx + bw / 2, fy + bh / 2))
                if len(tr["trail"]) > 8:
                    tr["trail"] = tr["trail"][-8:]
                detections.append(
                    Detection(
                        x=fx,
                        y=fy,
                        w=bw,
                        h=bh,
                        class_id=tr["class_id"],
                        label=CLASSES[tr["class_id"]]["label"],
                        confidence=conf,
                        track_id=tr["id"],
                        age_s=now - tr["born"],
                        trail=list(tr["trail"]),
                    )
                )
            return detections

    def _spawn(self) -> None:
        # anchor in upper-half of frame so they don't pile on the horizon
        cls = self._rng.choice(list(CLASSES.keys()))
        # 'pest' and 'qr' are small; 'vehicle' and 'plant' are big
        if cls in ("pest", "qr"):
            bw, bh = self._rng.uniform(28, 56), self._rng.uniform(28, 56)
            conf_base = self._rng.uniform(0.7, 0.9)
        elif cls == "person":
            bw, bh = self._rng.uniform(60, 90), self._rng.uniform(140, 200)
            conf_base = self._rng.uniform(0.75, 0.95)
        elif cls == "vehicle":
            bw, bh = self._rng.uniform(140, 220), self._rng.uniform(90, 140)
            conf_base = self._rng.uniform(0.6, 0.85)
        elif cls == "plant":
            bw, bh = self._rng.uniform(80, 140), self._rng.uniform(80, 140)
            conf_base = self._rng.uniform(0.6, 0.8)
        else:  # sign
            bw, bh = self._rng.uniform(60, 100), self._rng.uniform(60, 100)
            conf_base = self._rng.uniform(0.65, 0.9)
        # anchor is in pixel space (0..frame_w, 0..frame_h * 0.55)
        # radius is in pixel space too
        frame_w, frame_h = 640, 480
        self._tracks.append(
            {
                "id": self._next_id,
                "class_id": cls,
                "anchor": (
                    self._rng.uniform(0.1 * frame_w, (0.9 - bw / frame_w) * frame_w),
                    self._rng.uniform(0.05 * frame_h, (0.55 - bh / frame_h) * frame_h),
                ),
                "radius": (
                    self._rng.uniform(0.04, 0.18) * frame_w,
                    self._rng.uniform(0.03, 0.12) * frame_h,
                ),
                "bw": bw,
                "bh": bh,
                "phase": self._rng.uniform(0, math.tau),
                "omega": self._rng.uniform(0.4, 1.1),
                "conf_base": conf_base,
                "born": time.time(),
                "ttl": self._rng.uniform(8, 18),
                "trail": [],
            }
        )
        self._next_id += 1


# 柔性映射: YOLOE 训练标签 (cauliflower/turn_left/...) -> CLASSES key
# (person/vehicle/sign/plant/pest/qr) 给 draw_detections 用颜色
_LABEL_TO_CLASSKEY = {
    # plant: task_wbt2025 的农业目标
    "cauliflower": "plant", "greens": "plant", "chili": "plant",
    "cylinder3": "plant", "cylinder2": "plant", "cylinder1": "plant",
    "plant": "plant",
    # sign: 转向标志 (turn_left/turn_right from front_model2 + task_wbt2025)
    "turn_left": "sign", "turn_right": "sign",
    "sign": "sign",
    # vehicle / person: 预留扩展
    "vehicle": "vehicle", "person": "person",
    # pest
    "pest": "pest", "aphid": "pest", "caterpillar": "pest",
}


def _class_key(yolo_label: str) -> str:
    """YOLOE label -> CLASSES key, 柔性匹配, fallback 'sign'."""
    if not yolo_label:
        return "sign"
    s = yolo_label.lower().strip()
    if s in _LABEL_TO_CLASSKEY:
        return _LABEL_TO_CLASSKEY[s]
    # 关键词子串匹配
    for k in ("plant", "pest", "sign", "vehicle", "person", "qr"):
        if k in s:
            return k
    return "sign"


class ZmqDetector(BaseDetector):
    """真检测器: 路由到 rak-car 的 ZMQ 推理后端 (infer_cs/base/infer_back_end.py)。

    用法: ZmqDetector('task')  -> ClintInterface('task')  -> port 5002 (task_wbt2025)
          ZmqDetector('front') -> ClintInterface('front') -> port 5003 (front_model2)

    构造时**不**导入 Paddle/ClintInterface — 仅在第一次 detect() 时懒加载,
    这样 streamer 默认启动不拖慢且不需要 Paddle 装包就能起来。
    """

    def __init__(self, service: str) -> None:
        self._service = service
        self._ci = None  # lazy
        self._lock = threading.Lock()
        self._errors = 0

    def _ensure(self) -> None:
        if self._ci is not None:
            return
        # 延迟 import: Paddle/ClintInterface 只在第一次推理时加载
        from infer_cs.base.infer_front import ClintInterface
        self._ci = ClintInterface(self._service)

    def detect(self, frame: np.ndarray) -> List[Detection]:
        with self._lock:
            try:
                self._ensure()
                res = self._ci(frame)  # list of Bbox
            except Exception:
                self._errors += 1
                return []
        if not isinstance(res, list) or not res:
            return []
        h, w = frame.shape[:2]
        out: List[Detection] = []
        for b in res:
            # Bbox 列表: [cls_id, obj_id, label, score, x_n, y_n, w_n, h_n] (归一化)
            if len(b) < 8:
                continue
            try:
                cls_id, obj_id, label, score, xn, yn, wn, hn = b[:8]
                score = float(score); xn = float(xn); yn = float(yn)
                wn = float(wn); hn = float(hn)
            except (TypeError, ValueError):
                continue
            if score < 0.05:  # 过滤极低置信度 (前端画框也只画有效检测)
                continue
            out.append(Detection(
                x=xn * w, y=yn * h, w=wn * w, h=hn * h,
                class_id=_class_key(label),  # 映射到 CLASSES key
                label=label,                 # 保留 YOLOE 原标签给前端显示
                confidence=score,
                track_id=int(obj_id) if obj_id is not None else 0,
                age_s=0.0, trail=[],
            ))
        return out


class LaneZmqDetector(BaseDetector):
    """Lane-segmentation detector: routes to the `lane` ZMQ service
    (infer.yaml LaneInfer, port 5001). The model returns a 128×128
    probability map per row; we run a simple argmax-with-class
    split to extract the left and right lane polylines, then upsert
    them to the input frame.

    Returns a regular List[Detection] so the existing
    `draw_detections()` overlay path can render them like any other
    detection (HUD-style boxes + trail). For visualization we use a
    single tall thin rectangle per lane — far simpler than painting
    a per-row polyline inside the shared `draw_detections` and keeps
    the dashboard's `Latest Detections` panel informative.
    """

    _COLOR_LANE = (197, 255, 58)   # acid green
    _POLY_THRESHOLD = 0.5          # row-wise probability threshold

    def __init__(self) -> None:
        self._ci = None  # lazy
        self._lock = threading.Lock()
        self._errors = 0

    def _ensure(self) -> None:
        if self._ci is not None:
            return
        from infer_cs.base.infer_front import ClintInterface
        self._ci = ClintInterface("lane")

    def detect(self, frame: np.ndarray) -> List[Detection]:
        with self._lock:
            try:
                self._ensure()
                map_128 = self._ci(frame)
            except Exception:
                self._errors += 1
                return []

        pts = self._extract_lane_points(map_128)
        if not pts:
            return []

        h, w = frame.shape[:2]
        # Split into left/right halves by x position (0..127).
        left = [p for p in pts if p[0] < 64]
        right = [p for p in pts if p[0] >= 64]
        out: List[Detection] = []
        for side, name in ((left, "lane_l"), (right, "lane_r")):
            if len(side) < 4:
                continue
            xs = [p[0] for p in side]
            ys = [p[1] for p in side]
            # Map 128-row grid to actual frame size.
            x_min = min(xs) * w / 128.0
            x_max = max(xs) * w / 128.0
            y_min = min(ys) * h / 128.0
            y_max = max(ys) * h / 128.0
            out.append(Detection(
                x=x_min,
                y=y_min,
                w=max(8.0, x_max - x_min),
                h=max(20.0, y_max - y_min),
                class_id="plant",   # borrow `plant` palette (lime green)
                label=name,
                confidence=0.85,
                track_id=0,
                age_s=0.0,
                trail=[(x * w / 128.0, y * h / 128.0) for (x, y) in side],
            ))
        return out

    @classmethod
    def _extract_lane_points(cls, raw) -> List[Tuple[int, int]]:
        """Reduce whatever the lane model returns to per-row (col, row) points.

        The installed `cnn_lane` model returns a (2,) logit pair (binary
        "lane present" decision). When the second logit is positive we
        render a synthetic lane band — that's enough for the user to
        see "lane detection ran". Newer lane models (CLRNet, UFLD…) that
        we may plug in later produce (128, W) or (1, 128, W) output,
        which we honour per-row.
        """
        if isinstance(raw, list) and not raw:
            return []
        try:
            arr = np.asarray(raw, dtype=np.float32) if not isinstance(raw, list) else np.asarray(raw, dtype=np.float32)
        except Exception:
            return []

        # Binary classifier output (2,) → render a full-width synthetic
        # lane band across the lower 60% of the 128-row grid if positive.
        if arr.ndim == 1 and arr.size >= 2:
            score = float(arr[1] - arr[0])  # second logit - first
            if score > 0:
                y_top = int(128 * 0.40)
                y_bot = 127
                # Slight ease-in curve so the band reads as "lane".
                return [
                    (int(64 - 26 + 10 * (row - y_top) / max(1, y_bot - y_top)), row)
                    for row in range(y_top, y_bot + 1)
                ] + [
                    (int(64 + 26 - 10 * (row - y_top) / max(1, y_bot - y_top)), row)
                    for row in range(y_top, y_bot + 1)
                ]
            return []

        # Per-row argmax over a 2D grid.
        if arr.ndim == 4 and arr.shape[0] == 1:
            arr = arr[0]
            if arr.ndim == 3 and arr.shape[-1] in (2, 3, 4):
                arr = arr.argmax(axis=-1)
        if arr.ndim == 3:
            if arr.shape[0] == 1:
                arr = arr[0]
            elif arr.shape[-1] <= 8 and arr.shape[0] >= arr.shape[-1]:
                arr = arr.argmax(axis=0)
            else:
                arr = arr.argmax(axis=-1)
        if arr.ndim != 2 or arr.shape[0] != 128:
            return []
        out: List[Tuple[int, int]] = []
        for row in range(arr.shape[0]):
            col = int(np.argmax(arr[row]))
            if float(arr[row, col]) > 0.1:
                out.append((col, row))
        return out


def draw_detections(frame: np.ndarray, detections: List[Detection]) -> np.ndarray:
    """Render boxes, labels, trails, and a class legend onto `frame` in place."""
    h, w = frame.shape[:2]
    out = frame
    for det in detections:
        color = CLASSES[det.class_id]["color"]
        x1, y1 = int(det.x), int(det.y)
        x2, y2 = int(det.x + det.w), int(det.y + det.h)
        # main rectangle
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        # corner ticks (HUD style) — 4 short L-shapes
        t = 12
        for cx, cy, dx, dy in (
            (x1, y1, 1, 1),
            (x2, y1, -1, 1),
            (x1, y2, 1, -1),
            (x2, y2, -1, -1),
        ):
            cv2.line(out, (cx, cy), (cx + dx * t, cy), color, 3)
            cv2.line(out, (cx, cy), (cx, cy + dy * t), color, 3)
        # label
        tag = f"{det.label} {det.confidence:.2f}  #{det.track_id}"
        (tw, th), baseline = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(out, (x1, max(0, y1 - th - baseline - 6)), (x1 + tw + 6, y1), color, -1)
        cv2.putText(
            out,
            tag,
            (x1 + 3, max(th, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (10, 10, 10),
            1,
            cv2.LINE_AA,
        )
        # trail
        for i in range(1, len(det.trail)):
            ax, ay = det.trail[i - 1]
            bx, by = det.trail[i]
            a = int(255 * i / len(det.trail))
            cv2.line(out, (int(ax), int(ay)), (int(bx), int(by)), color, 1, cv2.LINE_AA)

    # class legend (top-right of frame)
    classes_in_frame = sorted({d.class_id for d in detections})
    if classes_in_frame:
        lx = w - 12
        ly = 12
        for cls_id in reversed(classes_in_frame):
            color = CLASSES[cls_id]["color"]
            label = CLASSES[cls_id]["label"]
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(out, (lx - tw - 18, ly - 2), (lx, ly + th + 6), color, -1)
            cv2.putText(
                out,
                label,
                (lx - tw - 4, ly + th),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (10, 10, 10),
                1,
                cv2.LINE_AA,
            )
            ly += th + 10

    return out
