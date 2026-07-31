"""main/arm/vision/selector.py — 多目标选择器."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

from ..labels import LABEL_GROUPS, Label
from .types import Detection


class SelectionStrategy(str, Enum):
    HIGHEST_SCORE      = "highest_score"
    CLOSEST_TO_CENTER  = "closest_to_center"
    LARGEST            = "largest"
    LEFTMOST           = "leftmost"
    RIGHTMOST          = "rightmost"
    TOPMOST            = "topmost"
    BOTTOMMOST         = "bottommost"
    LOCK_FIRST_SEEN    = "lock_first_seen"


@dataclass(frozen=True)
class TargetSelector:
    label: Optional[str] = None
    track_id: Optional[int] = None
    strategy: str = SelectionStrategy.HIGHEST_SCORE.value
    group: Optional[str] = None

    @classmethod
    def for_label(cls, label, *,
                  strategy: str = SelectionStrategy.HIGHEST_SCORE.value) -> "TargetSelector":
        return cls(
            label=str(label.value if isinstance(label, Label) else label),
            strategy=strategy,
        )

    @classmethod
    def for_group(cls, group: str, *,
                  strategy: str = SelectionStrategy.HIGHEST_SCORE.value) -> "TargetSelector":
        if group not in LABEL_GROUPS:
            raise ValueError(f"未知 group: {group!r} ({list(LABEL_GROUPS)})")
        return cls(label=None, strategy=strategy, group=group)

    def matches(self, det: Detection) -> bool:
        if self.group is not None:
            return det.label in [l.value for l in LABEL_GROUPS[self.group]]
        if self.label is not None:
            return det.label == self.label
        return True

    def apply_strategy(self, candidates: List[Detection]) -> Optional[Detection]:
        if not candidates:
            return None
        s = self.strategy
        if s == SelectionStrategy.HIGHEST_SCORE.value:
            return max(candidates, key=lambda d: d.score)
        if s == SelectionStrategy.CLOSEST_TO_CENTER.value:
            return min(candidates, key=lambda d: abs(d.bbox_norm.x_center) + abs(d.bbox_norm.y_center))
        if s == SelectionStrategy.LARGEST.value:
            return max(candidates, key=lambda d: d.bbox_norm.width * d.bbox_norm.height)
        if s == SelectionStrategy.LEFTMOST.value:
            return min(candidates, key=lambda d: d.bbox_norm.x_center)
        if s == SelectionStrategy.RIGHTMOST.value:
            return max(candidates, key=lambda d: d.bbox_norm.x_center)
        if s == SelectionStrategy.TOPMOST.value:
            return min(candidates, key=lambda d: d.bbox_norm.y_center)
        if s == SelectionStrategy.BOTTOMMOST.value:
            return max(candidates, key=lambda d: d.bbox_norm.y_center)
        return candidates[0]
