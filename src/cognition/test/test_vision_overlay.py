# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary

import numpy as np
from types import SimpleNamespace

from cognition.visualization.overlay_helpers import (
    draw_detection_overlay,
    draw_lane_overlay,
    image_data_sequence,
)


def test_draw_lane_overlay_changes_frame_and_labels_result():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    lane = SimpleNamespace(
        valid=True, deviation_distance=0.2, deviation_angle=0.1,
        inference_ms=8.0)
    out = draw_lane_overlay(frame, lane)
    assert out.shape == frame.shape
    assert not np.array_equal(out, frame)


def test_draw_detection_overlay_draws_bbox_and_class():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = SimpleNamespace(
        image_width=640, image_height=480, model_id="task",
        class_names=["ball"], scores=[0.91],
        xs=[100.0], ys=[120.0], widths=[80.0], heights=[60.0])
    out = draw_detection_overlay(frame, detections)
    assert out.shape == frame.shape
    assert not np.array_equal(out, frame)


def test_image_data_sequence_converts_frame_to_ros_byte_values():
    frame = np.zeros((2, 3, 3), dtype=np.uint8)
    assert list(image_data_sequence(frame)) == [0] * 18
