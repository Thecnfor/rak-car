# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary

import numpy as np

from cognition.detector.detector_decode import decode_detection_rows


def test_decode_ppyoloe_class_score_xyxy_order():
    rows = np.array([
        [3.0, 0.58, 10.0, 20.0, 110.0, 220.0],
        [14.0, 0.42, 1.0, 2.0, 3.0, 4.0],
    ], dtype=np.float32)
    decoded = decode_detection_rows(rows, 2, 23)
    assert decoded.shape == (2, 6)
    np.testing.assert_allclose(
        decoded[0], [10.0, 20.0, 110.0, 220.0, 0.58, 3.0])


def test_decode_discards_invalid_class_score_and_box_rows():
    rows = np.array([
        [569.0, 0.8, 1.0, 2.0, 3.0, 4.0],
        [2.0, 584.5, 1.0, 2.0, 3.0, 4.0],
        [2.0, 0.8, 5.0, 5.0, 4.0, 10.0],
    ], dtype=np.float32)
    decoded = decode_detection_rows(rows, 3, 23)
    assert decoded.shape == (0, 6)
