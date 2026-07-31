"""main/arm/vision/types.py — 视觉伺服 DTO."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class BBoxNorm:
    x_center: float
    y_center: float
    width: float
    height: float

    @property
    def is_centered(self) -> bool:
        return self.is_centered_at(0.05)

    def is_centered_at(self, tol: float) -> bool:
        return abs(self.x_center) <= tol and abs(self.y_center) <= tol


@dataclass(frozen=True)
class BBoxPixels:
    x1: int
    y1: int
    x2: int
    y2: int
    width: int
    height: int


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    track_id: Optional[int]
    class_id: Optional[int]
    bbox_norm: BBoxNorm
    bbox_pixels: Optional[BBoxPixels]
    fetched_at: float

    def __repr__(self) -> str:
        return (f"Detection({self.label}#{self.track_id} "
                f"score={self.score:.2f} cx={self.bbox_norm.x_center:+.2f})")


@dataclass(frozen=True)
class ServoTrace:
    t_s: float
    iteration: int
    dx_norm: float
    dy_norm: float
    x_mm: float
    y_mm: float
    score: float
    selected_track_id: Optional[int]
    is_miss: bool = False


@dataclass(frozen=True)
class ServoResult:
    converged: bool
    selector: "TargetSelector"
    x_mm: float
    y_mm: float
    confidence: float
    iterations: int
    elapsed_s: float
    final_detection: Optional[Detection]
    trace: Tuple[ServoTrace, ...]
    settle_stable: bool = False
