"""find_target_track 单测 —— 持续追踪模式（不收敛停）"""
import unittest
from unittest.mock import MagicMock, patch
from main.arm.vision import (
    ArmVisionClient, TargetSelector, Detection, BBoxNorm,
)


class FakeWs:
    """推 frames 序列模拟任务持续移动"""
    def __init__(self, frames):
        self.frames = list(frames)
    def subscribe_task_detection(self, on_state, hz=30.0):
        for i, frame in enumerate(self.frames):
            on_state({"data": {
                "active": True,
                "detections": [FakeWs._det_to_dict(d) for d in frame],
                "updated_at": float(i + 1),
            }})
        return MagicMock()
    @staticmethod
    def _det_to_dict(d):
        return {
            "label": d.label, "score": d.score,
            "track_id": d.track_id, "cls_id": d.class_id,
            "bbox_norm": {
                "x_center": d.bbox_norm.x_center,
                "y_center": d.bbox_norm.y_center,
                "width": d.bbox_norm.width, "height": d.bbox_norm.height,
            },
        }


def _det(cx, cy, label="cylinder_1", score=0.9, tid=0):
    return Detection(
        label=label, score=score, track_id=tid, class_id=4,
        bbox_norm=BBoxNorm(cx, cy, 0.1, 0.1),
        bbox_pixels=None, fetched_at=0.0,
    )


class TestFindTargetTrack(unittest.TestCase):
    def test_continues_after_centered(self):
        """与 find_target_realtime 区别：居中后仍继续跑。"""
        # 推 3 帧：第一帧偏 → 移；第二帧居中（不该停！）；第三帧再偏
        ws = FakeWs([
            [_det(cx=0.3, cy=0.0)],   # 偏右
            [_det(cx=0.01, cy=0.01)],  # 居中
            [_det(cx=-0.3, cy=0.0)],   # 偏左
        ])
        http = MagicMock()
        vision = ArmVisionClient(http)
        move_log = []
        result = vision.find_target_track(
            TargetSelector.for_label("cylinder_1"),
            x_mm=0.0, y_mm=-150.0,
            mm_per_norm=30.0, settle_tol_norm=0.05,
            timeout=2.0, ws=ws,
            move_fn=lambda x, y: move_log.append((x, y)) or {},
        )
        # 不应 converged=True
        self.assertFalse(result.converged)
        # 至少 2 个 move（第三帧甚至更多）
        self.assertGreaterEqual(len(move_log), 2)

    def test_waits_for_target_when_missing(self):
        """on_missing_track='wait' (默认)：目标丢失不 abort。"""
        # 推 1 帧 target，再推 4 帧空 → on_missing_track=wait 不 raise
        ws = FakeWs([
            [_det(cx=0.3, cy=0.0, label="cylinder_1")],
            [], [], [], [],
        ])
        http = MagicMock()
        vision = ArmVisionClient(http)
        # 不应 raise
        try:
            result = vision.find_target_track(
                TargetSelector.for_label("cylinder_1"),
                x_mm=0.0, y_mm=-150.0,
                timeout=1.0, ws=ws,
                move_fn=lambda x, y: {},
            )
            self.assertFalse(result.converged)
        except RuntimeError:
            self.fail("on_missing_track='wait' 不应 raise")

    def test_aborts_when_missing_with_abort_mode(self):
        """on_missing_track='abort' + 5 帧 miss → raise。"""
        ws = FakeWs([
            [_det(cx=0.0, cy=0.0, label="animal")],  # 不匹配 cylinder_1
            [], [], [], [], [],
        ])
        http = MagicMock()
        vision = ArmVisionClient(http)
        with self.assertRaises(RuntimeError):
            vision.find_target_track(
                TargetSelector.for_label("cylinder_1"),
                x_mm=0.0, y_mm=-150.0,
                on_missing_track="abort",
                timeout=2.0, ws=ws,
                move_fn=lambda x, y: {},
            )


if __name__ == "__main__":
    unittest.main()