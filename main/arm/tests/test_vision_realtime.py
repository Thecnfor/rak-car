"""find_target_realtime 单测 —— mock WS subscribe_task_detection 推流"""
import unittest
from unittest.mock import MagicMock, patch
from main.arm.vision import (
    ArmVisionClient, TargetSelector, Detection, BBoxNorm,
)


class FakeWsClient:
    """模拟 RuntimeWsClient.subscribe_task_detection：把 frames 推给回调"""
    def __init__(self, frames):
        self.frames = list(frames)
        self.captured_cb = None

    def subscribe_task_detection(self, on_state, hz=30.0):
        self.captured_cb = on_state
        # 同步推完所有帧（替代 asyncio loop 的异步行为）
        # WS 推送实际用 "data" 键（routes.py:1437）
        for i, frame in enumerate(self.frames):
            on_state({"data": {
                "active": True,
                "detections": [FakeWsClient._det_to_dict(d) for d in frame],
                "updated_at": float(i + 1),    # 每个帧唯一，绕过 dedup
            }})
        # 返回 stop controller
        stop = MagicMock()
        return stop

    @staticmethod
    def _det_to_dict(d):
        return {
            "label": d.label,
            "score": d.score,
            "track_id": d.track_id,
            "cls_id": d.class_id,
            "bbox_norm": {
                "x_center": d.bbox_norm.x_center,
                "y_center": d.bbox_norm.y_center,
                "width": d.bbox_norm.width,
                "height": d.bbox_norm.height,
            },
        }


def _det(cx, cy, label="cylinder_1", score=0.9, tid=0, ts=0.0):
    return Detection(
        label=label, score=score, track_id=tid, class_id=4,
        bbox_norm=BBoxNorm(cx, cy, 0.1, 0.1),
        bbox_pixels=None, fetched_at=ts,
    )


class TestFindTargetRealtime(unittest.TestCase):
    def test_converges_via_ws(self):
        # WS 推 2 帧：第一帧偏右 0.3 → move -9mm；第二帧居中 → 收敛
        ws = FakeWsClient([
            [_det(cx=0.3, cy=0.0)],
            [_det(cx=0.01, cy=0.01)],
        ])
        http = MagicMock()
        vision = ArmVisionClient(http)
        move_log = []
        result = vision.find_target_realtime(
            TargetSelector.for_label("cylinder_1"),
            x_mm=0.0, y_mm=-150.0,
            mm_per_norm=30.0, settle_tol_norm=0.05,
            timeout=5.0, ws=ws,
            move_fn=lambda x, y: move_log.append((x, y)) or {},
        )
        self.assertTrue(result.converged)
        self.assertEqual(len(move_log), 1)
        self.assertAlmostEqual(move_log[0][0], -9.0, places=1)

    def test_timeout_via_ws(self):
        # 推 100 帧都不收敛 → 超时
        ws = FakeWsClient([[_det(cx=0.5, cy=0.5)] for _ in range(100)])
        http = MagicMock()
        vision = ArmVisionClient(http)
        result = vision.find_target_realtime(
            TargetSelector.for_label("cylinder_1"),
            x_mm=0.0, y_mm=-150.0,
            mm_per_norm=30.0, settle_tol_norm=0.05,
            timeout=0.05, ws=ws,
            move_fn=lambda x, y: {},
        )
        self.assertFalse(result.converged)

    def test_label_filter_excludes_other_labels(self):
        # 推 6 帧全 animal（不匹配 cylinder_1）→ 连续 miss >= 5 → raise
        ws = FakeWsClient([[_det(cx=0.0, cy=0.0, label="animal")] for _ in range(6)])
        http = MagicMock()
        vision = ArmVisionClient(http)
        with self.assertRaises(RuntimeError):
            vision.find_target_realtime(
                TargetSelector.for_label("cylinder_1"),
                x_mm=0.0, y_mm=-150.0,
                on_missing_track="abort",
                timeout=2.0,
                ws=ws, move_fn=lambda x, y: {},
            )

    def test_builds_ws_if_none_passed(self):
        # ws=None 时内部应构造 RuntimeWsClient（导入路径在 main.ws_client）
        with patch("main.ws_client.RuntimeWsClient") as mock_ws_cls:
            mock_ws = MagicMock()
            mock_ws_cls.return_value = mock_ws
            # 模拟 WS 推 1 帧就收敛（routes.py:1437 实际用 'data' 键）
            def fake_subscribe(on_state, hz=30.0):
                on_state({"data": {
                    "detections": [{
                        "label": "cylinder_1", "score": 0.9,
                        "track_id": 0, "cls_id": 4,
                        "bbox_norm": {"x_center": 0.0, "y_center": 0.0,
                                       "width": 0.1, "height": 0.1},
                    }],
                    "updated_at": 1.0,
                }})
                return MagicMock()
            mock_ws.subscribe_task_detection.side_effect = fake_subscribe

            http = MagicMock()
            vision = ArmVisionClient(http)
            result = vision.find_target_realtime(
                TargetSelector.for_label("cylinder_1"),
                x_mm=0.0, y_mm=-150.0, ws=None, timeout=2.0,
                move_fn=lambda x, y: {},
            )
            mock_ws_cls.assert_called_once()
            self.assertTrue(result.converged)


if __name__ == "__main__":
    unittest.main()