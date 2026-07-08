"""Utility helpers that mirror the official car's task-script idioms."""
from __future__ import annotations

import math
import time
from typing import Sequence


def delay(seconds: float) -> None:
    """Mirrors car_wrap_2026.MyCar.delay."""
    time.sleep(max(0.0, seconds))


def calculation_dis(pos_dst: Sequence[float], pos_src: Sequence[float]) -> float:
    """Euclidean distance. Mirrors MyCar.calculation_dis."""
    if len(pos_dst) < 3 or len(pos_src) < 3:
        raise ValueError('positions must be (x, y, z)')
    return math.hypot(pos_dst[0] - pos_src[0], pos_dst[1] - pos_src[1])


def get_list_by_val(lst: Sequence, index: int, val):
    """Mirrors MyCar.get_list_by_val (used by auto_seeding for cylinder_list)."""
    return [v for v in lst if (v[index] if index < len(v) else None) == val]