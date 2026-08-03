"""main/chassis/tests/test_visual_track.py
通用底盘视觉追踪单测 (stdlib unittest, 离线无硬件)。

覆盖：
  - expand_label_set：water 组展开 / 单 label / list
  - track_chassis：到达 3 帧死区后 returned arrived=True
  - track_chassis：丢失目标超时返回 no_target
  - track_chassis：画面中心最近策略 vs 最大面积策略
  - track_chassis：leftmost 策略 (min cx / 平局大面积 / label 过滤 / 双球端到端)
  - track_chassis：默认 target 是 h_tu_dou (2026-08-02)
"""
import unittest
from unittest.mock import MagicMock

from main.chassis.loops.visual_track import (
    expand_label_set,
    track_chassis,
    _select_target,
)


def _d(label="water", cx=0.0, cy=0.0, w=0.2, h=0.2, score=0.9):
    return {
        "label": label,
        "score": score,
        "bbox_norm": {"cx": cx, "cy": cy, "width": w, "height": h},
    }


def _make_mock_api(payloads):
    api = MagicMock()

    def _pop():
        if payloads:
            return payloads.pop(0)
        return {"task_state": {"active": False, "detections": []}}

    http = MagicMock()
    http.get_vision_task_cache = MagicMock(side_effect=_pop)
    http.post = MagicMock()
    api.http = http
    api.set_wheel_speeds = MagicMock()
    api.close = MagicMock()
    return api


class TestExpandLabelSet(unittest.TestCase):
    def test_water_group_expands(self):
        labels = expand_label_set("water")
        self.assertIn("water", labels)
        self.assertIn("water_l1", labels)
        self.assertIn("water_l2", labels)
        self.assertIn("water_l3", labels)
        self.assertEqual(len(labels), 4)

    def test_cylinder_group_expands(self):
        labels = expand_label_set("cylinder")
        self.assertIn("cylinder_1", labels)
        self.assertIn("cylinder_2", labels)
        self.assertIn("cylinder_3", labels)

    def test_specific_label_is_single(self):
        labels = expand_label_set("cylinder_set")
        self.assertEqual(labels, {"cylinder_set"})

    def test_list_input(self):
        labels = expand_label_set(["water", "cylinder_1"])
        self.assertIn("water", labels)
        self.assertIn("water_l1", labels)
        self.assertIn("water_l2", labels)
        self.assertIn("water_l3", labels)
        self.assertIn("cylinder_1", labels)


class TestSelectTarget(unittest.TestCase):
    def test_nearest_to_center_prefers_closer(self):
        dets = [
            _d(cx=0.5, cy=0.0),
            _d(cx=0.1, cy=0.0),
            _d(cx=0.2, cy=0.0, label="other"),
        ]
        chosen = _select_target(dets, {"water"}, (0.0, 0.0), "nearest_to_center")
        self.assertIsNotNone(chosen)
        self.assertAlmostEqual(chosen["bbox_norm"]["cx"], 0.1)

    def test_largest_area(self):
        dets = [
            _d(w=0.1, h=0.1),
            _d(w=0.4, h=0.4),
            _d(w=0.2, h=0.2, label="x"),
        ]
        chosen = _select_target(dets, {"water"}, (0.0, 0.0), "largest_area")
        self.assertIsNotNone(chosen)
        bb = chosen["bbox_norm"]
        self.assertEqual(bb["width"], 0.4)

    def test_no_match_returns_none(self):
        dets = [_d(label="x")]
        self.assertIsNone(_select_target(dets, {"water"}, (0.0, 0.0)))

    def test_leftmost_prefers_min_cx(self):
        # 2026-08-03: task4 采收选画面最左球 (cx 最小); 不匹配 label 的更左也不算
        dets = [
            _d(label="ball_yellow", cx=0.3, cy=0.0),
            _d(label="ball_blue", cx=-0.4, cy=0.1),
            _d(label="ball_blue", cx=0.1, cy=-0.2),
            _d(label="other", cx=-0.9, cy=0.0),
        ]
        chosen = _select_target(
            dets, {"ball_blue", "ball_yellow"}, (0.0, 0.0), "leftmost")
        self.assertIsNotNone(chosen)
        self.assertAlmostEqual(chosen["bbox_norm"]["cx"], -0.4)
        self.assertEqual(chosen["label"], "ball_blue")

    def test_leftmost_tie_prefers_larger_area(self):
        dets = [
            _d(label="ball_blue", cx=-0.2, cy=0.0, w=0.1, h=0.1),
            _d(label="ball_yellow", cx=-0.2, cy=0.05, w=0.3, h=0.3),
        ]
        chosen = _select_target(
            dets, {"ball_blue", "ball_yellow"}, (0.0, 0.0), "leftmost")
        self.assertEqual(chosen["label"], "ball_yellow")

    def test_leftmost_no_match_returns_none(self):
        dets = [_d(label="x", cx=-0.5)]
        self.assertIsNone(
            _select_target(dets, {"ball_blue"}, (0.0, 0.0), "leftmost"))


def _steady_payloads(cx, cy, n=200, label="h_tu_dou"):
    """n 帧都停在 (cx, cy) 不动。"""
    return [
        {"task_state": {"active": True, "detections": [_d(label=label, cx=cx, cy=cy)]}}
        for _ in range(n)
    ]


def _step_payloads(seq, label="h_tu_dou"):
    """seq: [(cx,cy), ...] 前面按 seq 走,后面塞 200 帧 (0,0)。"""
    out = []
    for cx, cy in seq:
        out.append({"task_state": {"active": True, "detections": [_d(label=label, cx=cx, cy=cy)]}})
    for _ in range(200):
        out.append({"task_state": {"active": True, "detections": [_d(label=label, cx=0.0, cy=0.0)]}})
    return out


class TestTrackChassis(unittest.TestCase):
    def test_arrives_when_in_band_3_frames(self):
        seq = [(0.6, 0.5), (0.3, 0.25), (0.1, 0.1), (0.03, 0.03)] + \
              [(0.0, 0.0)] * 50
        api = _make_mock_api(_step_payloads(seq))
        result = track_chassis(
            "h_tu_dou", api=api,
            hz=200, max_seconds=5.0,
            kp=0.50, v_max=0.25, deadband=0.08, hold_frames=3,
            max_lost_frames=50, dry_run=True,
        )
        self.assertTrue(result.arrived, result)
        self.assertEqual(result.reason, "arrived")
        self.assertGreaterEqual(result.frames, 6)

    def test_no_target_timeout_reason(self):
        payloads = [
            {"task_state": {"active": True, "detections": []}}
            for _ in range(200)
        ]
        api = _make_mock_api(payloads)
        result = track_chassis(
            "water", api=api, hz=200, max_seconds=5.0,
            max_lost_frames=6, dry_run=True,
        )
        self.assertFalse(result.arrived)
        self.assertEqual(result.reason, "no_target")

    def test_default_target_is_h_tu_dou(self):
        """2026-08-02 现场:默认 target 改成了 h_tu_dou。"""
        seq = [(0.5, -0.4)] + [(0.0, 0.0)] * 50
        api = _make_mock_api(_step_payloads(seq, label="h_tu_dou"))
        result = track_chassis(
            api=api, hz=200, max_seconds=2.0,
            kp=0.50, v_max=0.25, deadband=0.08, hold_frames=3,
            max_lost_frames=50, dry_run=True,
        )
        self.assertTrue(result.arrived)

    def test_leftmost_mode_tracks_min_cx_ball(self):
        """双球场景: leftmost 全程锁定画面最左 (cx 最小) 那颗。

        前半黄球 cx=0.4 / 蓝球 cx=-0.3 → 锁蓝; 后半蓝球被拉到 0.0
        (黄球仍 0.4) → 死区到达, final_frame.label 必须是 ball_blue。
        """
        payloads = [
            {"task_state": {"active": True, "detections": [
                _d(label="ball_yellow", cx=0.4, cy=0.0),
                _d(label="ball_blue", cx=-0.3, cy=0.0),
            ]}}
            for _ in range(10)
        ] + [
            {"task_state": {"active": True, "detections": [
                _d(label="ball_yellow", cx=0.4, cy=0.0),
                _d(label="ball_blue", cx=0.0, cy=0.0),
            ]}}
            for _ in range(50)
        ]
        api = _make_mock_api(payloads)
        result = track_chassis(
            ["ball_blue", "ball_yellow"], api=api, select_mode="leftmost",
            hz=200, max_seconds=3.0,
            kp=0.50, v_max=0.25, deadband=0.08, hold_frames=3,
            max_lost_frames=50, dry_run=True,
        )
        self.assertTrue(result.arrived, result)
        self.assertEqual(result.final_frame.label, "ball_blue")

    def test_sign_vx_sign_vy_accepted(self):
        """sign_vx=+1 / sign_vy=-1 参数能被接受（2026-08-02 现场可调方向开关）。"""
        seq = [(0.0, 0.0)] * 5 + [(-0.1, 0.2)] * 100
        api = _make_mock_api(_step_payloads(seq))
        # 反号后也能跑通（方向反了照样能 arrived）
        result = track_chassis(
            "h_tu_dou", api=api, hz=200, max_seconds=2.0,
            kp=0.20, v_max=0.12, deadband=0.15, hold_frames=3,
            max_lost_frames=50, dry_run=True,
            sign_vx=+1, sign_vy=-1,
        )
        # 单纯验证参数不抛异常 + 能跑完
        self.assertIsNotNone(result)
        self.assertIsNotNone(result.frames)


if __name__ == "__main__":
    unittest.main()