# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary

import numpy as np


def decode_detection_rows(rows: np.ndarray, num_dets: int, class_count: int) -> np.ndarray:
    """Convert PP-YOLOE rows [class, score, x1, y1, x2, y2] to ROS order."""
    rows = np.asarray(rows, dtype=np.float32).reshape(-1, 6)[:max(0, num_dets)]
    if rows.size == 0:
        return np.empty((0, 6), dtype=np.float32)
    classes, scores = rows[:, 0], rows[:, 1]
    x1, y1, x2, y2 = rows[:, 2], rows[:, 3], rows[:, 4], rows[:, 5]
    valid = (
        np.isfinite(rows).all(axis=1)
        & (classes >= 0) & (classes < class_count)
        & (scores >= 0.0) & (scores <= 1.0)
        & (x2 > x1) & (y2 > y1)
    )
    return np.column_stack((x1, y1, x2, y2, scores, classes))[valid]
