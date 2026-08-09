# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary

import cv2
import numpy as np


def image_data_sequence(frame):
    """Return Image.data as the integer sequence required by ROS2 Python."""
    return frame.astype(np.uint8, copy=False).reshape(-1).tolist()


def draw_lane_overlay(frame, lane):
    out = frame.copy()
    height, width = out.shape[:2]
    center_x = width // 2
    color = (0, 220, 0) if lane.valid else (0, 0, 255)
    deviation_px = int(np.clip(lane.deviation_distance * width, -width, width))
    target_x = int(np.clip(center_x + deviation_px, 0, width - 1))
    cv2.line(out, (center_x, height - 1), (center_x, height // 2), (255, 255, 0), 2)
    cv2.line(out, (center_x, height - 1), (target_x, height // 2), color, 4)
    arrow_length = max(30, min(width // 4, 100))
    arrow_dx = int(np.clip(np.sin(float(lane.deviation_angle)) * arrow_length,
                           -arrow_length, arrow_length))
    cv2.arrowedLine(out, (center_x, height // 2),
                    (center_x + arrow_dx, height // 2 - arrow_length),
                    color, 3, tipLength=0.2)
    text = (f"lane {'OK' if lane.valid else 'INVALID'} "
            f"d={lane.deviation_distance:.3f}m "
            f"a={lane.deviation_angle:.3f}rad "
            f"{lane.inference_ms:.1f}ms")
    cv2.putText(out, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                color, 2, cv2.LINE_AA)
    return out


def draw_detection_overlay(frame, detections):
    out = frame.copy()
    height, width = out.shape[:2]
    sx = width / detections.image_width if detections.image_width else 1.0
    sy = height / detections.image_height if detections.image_height else 1.0
    count = min(len(detections.xs), len(detections.ys),
                len(detections.widths), len(detections.heights),
                len(detections.scores))
    for i in range(count):
        x1 = int(np.clip(detections.xs[i] * sx, 0, width - 1))
        y1 = int(np.clip(detections.ys[i] * sy, 0, height - 1))
        x2 = int(np.clip((detections.xs[i] + detections.widths[i]) * sx,
                         x1, width - 1))
        y2 = int(np.clip((detections.ys[i] + detections.heights[i]) * sy,
                         y1, height - 1))
        name = detections.class_names[i] if i < len(detections.class_names) else f"c{i}"
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 180, 255), 2)
        cv2.putText(out, f"{name} {detections.scores[i]:.2f}",
                    (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 180, 255), 2, cv2.LINE_AA)
    cv2.putText(out, f"{detections.model_id}: {count} detections", (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 180, 255), 2,
                cv2.LINE_AA)
    return out
