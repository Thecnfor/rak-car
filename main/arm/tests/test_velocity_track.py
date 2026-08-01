"""main/arm/tests/test_velocity_track.py — velocity 模式追踪单测 (07/08 封装).

覆盖: find_target_velocity (XY 方向/死区/miss/限速) + find_target_4dof
(arm/hand 增量方向/clamp/miss 保持角度)。全部离线: FakeWs 同步推帧,
post_fn 注入记录 payload, 不发真实 HTTP。
"""
import unittest
from unittest import mock

from main.arm.vision.velocity import VelocityLoop, VelocityResult, VelocityTrace


def _frame(label, x_c, y_c, score=0.9, w=0.2, h=0.2):
    return {
        "task_state": {
            "detections": [{
                "label": label, "score": score, "det_id": 1,
                "bbox_norm": {"x_center": x_c, "y_center": y_c,
                              "width": w, "height": h},
            }],
            "updated_at": 1.0,
            "frame_shape": [480, 640, 3],
        }
    }


class _FakeWs:
    """同步 WS: subscribe 时一次性推完所有帧 (确定性, 无线程)."""

    def __init__(self, frames):
        self.frames = list(frames)
        self.stopped = False

    def subscribe_task_detection(self, cb, hz=30.0):
        for raw in self.frames:
            cb(raw)
        return self._stop

    def _stop(self):
        self.stopped = True


class _FakeHttp:
    def build_url(self, path):  # post_fn 注入时不调用
        return "http://fake" + path


class _Loop(VelocityLoop):
    def __init__(self):
        self.http = _FakeHttp()


class _Runner:
    """拼出一个可调用 find_target_* 的对象: (loop, ws, posts, post_fn)."""

    @staticmethod
    def make(frames):
        loop = _Loop()
        posts = []

        def post_fn(**payload):
            posts.append(payload)
        ws = _FakeWs(frames)
        return loop, ws, posts, post_fn


class TestVelocityXY(unittest.TestCase):
    def _run(self, frames, **kw):
        loop, ws, posts, post_fn = _Runner.make(frames)
        result = loop.find_target_velocity("ball_yellow", ws=ws, post_fn=post_fn,
                                           timeout=0.3, **kw)
        return result, posts

    def test_direction_signs(self):
        # dx>0 (目标偏右) → x_vel 负; dy>0 (目标偏下) → y_vel 正 (真机实测固化)
        result, posts = self._run([_frame("ball_yellow", 0.5, 0.4)], gain=0.1)
        self.assertAlmostEqual(posts[0]["x_vel"], -0.05, places=5)
        self.assertAlmostEqual(posts[0]["y_vel"], 0.04, places=5)
        self.assertEqual(result.hits, 1)
        self.assertEqual(result.misses, 0)

    def test_deadzone_zero_vel(self):
        result, posts = self._run([_frame("ball_yellow", 0.01, -0.01)], deadzone=0.02)
        self.assertEqual(posts[0]["x_vel"], 0.0)
        self.assertEqual(posts[0]["y_vel"], 0.0)

    def test_miss_posts_zero_and_traces_miss(self):
        # 帧1 命中, 帧2 label 不匹配 → miss (发 0,0)
        result, posts = self._run([
            _frame("ball_yellow", 0.5, 0.5), _frame("animal", 0.2, 0.2)])
        self.assertEqual(result.hits, 1)
        self.assertEqual(result.misses, 1)
        self.assertEqual(posts[1]["x_vel"], 0.0)
        self.assertEqual(posts[1]["y_vel"], 0.0)
        self.assertTrue(result.trace[1].miss)

    def test_max_vel_clamp(self):
        result, posts = self._run([_frame("ball_yellow", 0.9, 0.9)],
                                  gain=0.5, max_vel=0.15)
        self.assertAlmostEqual(posts[0]["x_vel"], -0.15, places=5)
        self.assertAlmostEqual(posts[0]["y_vel"], 0.15, places=5)

    def test_wrong_label_all_miss(self):
        result, posts = self._run([_frame("animal", 0.5, 0.5)])
        self.assertEqual(result.hits, 0)
        self.assertEqual(result.misses, 1)

    def test_negate_sign(self):
        # sign_x=+1 → dx>0 → x_vel 正 (方向翻转开关)
        result, posts = self._run([_frame("ball_yellow", 0.5, 0.4)],
                                  gain=0.1, sign_x=1.0)
        self.assertGreater(posts[0]["x_vel"], 0)


class TestVelocity4Dof(unittest.TestCase):
    def _run(self, frames, **kw):
        loop, ws, posts, post_fn = _Runner.make(frames)
        result = loop.find_target_4dof("ball_yellow", ws=ws, post_fn=post_fn,
                                       timeout=0.3, **kw)
        return result, posts

    def test_arm_hand_increments(self):
        # dx>0 → arm 增大; dy>0 → hand 从 -90 增大; xy 方向同 XY 版
        result, posts = self._run([_frame("ball_yellow", 0.5, 0.5)],
                                  gain_arm=2.0, gain_hand=2.0)
        self.assertAlmostEqual(posts[0]["arm"], 1.0, places=5)    # 0 + 0.5*2
        self.assertAlmostEqual(posts[0]["hand"], -89.0, places=5)  # -90 + 0.5*2
        self.assertLess(posts[0]["x_vel"], 0)
        self.assertGreater(posts[0]["y_vel"], 0)

    def test_arm_clamp(self):
        # 大 dx 累加 → arm_target clamp 到 arm_max
        frames = [_frame("ball_yellow", 0.9, 0.0)] * 30
        result, posts = self._run(frames, gain_arm=2.0, arm_max=5.0)
        self.assertAlmostEqual(posts[-2]["arm"], 5.0, places=5)
        self.assertEqual(result.end_arm, 5.0)

    def test_miss_keeps_angle(self):
        # miss 帧: xy 停, 角度不动也不发 (payload 无 arm/hand)
        result, posts = self._run([
            _frame("ball_yellow", 0.5, 0.5), _frame("animal", 0.2, 0.2)],
            gain_arm=2.0, gain_hand=2.0)
        self.assertEqual(posts[0]["arm"], 1.0)
        self.assertEqual(posts[1]["x_vel"], 0.0)
        self.assertEqual(posts[1]["y_vel"], 0.0)
        self.assertNotIn("arm", posts[1])
        self.assertNotIn("hand", posts[1])
        self.assertEqual(result.end_arm, 1.0)
        self.assertEqual(result.end_hand, -89.0)
        self.assertEqual(result.misses, 1)

    def test_summary_and_trace_types(self):
        result, posts = self._run([_frame("ball_yellow", 0.5, 0.5)],
                                  gain_arm=2.0, gain_hand=2.0)
        self.assertIsInstance(result, VelocityResult)
        self.assertIsInstance(result.trace[0], VelocityTrace)
        self.assertIn("velocity[ball_yellow]", result.summary())


class TestDefaultPostFn(unittest.TestCase):
    """默认 post_fn 的端点契约: 内部 arm/hand → payload arm_angle/hand_angle."""

    def test_maps_arm_hand_to_angle_keys(self):
        loop = _Loop()
        post_fn = loop._default_post_fn()
        resp = mock.MagicMock()
        resp.raise_for_status = lambda: None
        resp.json = lambda: {"ok": True}
        with mock.patch("requests.post", return_value=resp) as m:
            post_fn(x_vel=0.1, y_vel=-0.2, arm=3.0, hand=-60.0)
        args, kwargs = m.call_args
        self.assertEqual(kwargs["json"]["arm_angle"], 3.0)
        self.assertEqual(kwargs["json"]["hand_angle"], -60.0)
        self.assertEqual(kwargs["json"]["x_vel"], 0.1)
        self.assertEqual(kwargs["json"]["y_vel"], -0.2)
        self.assertNotIn("arm", kwargs["json"])
        self.assertNotIn("hand", kwargs["json"])
        self.assertIn("/v1/realtime/arm-velocity", args[0])


if __name__ == "__main__":
    unittest.main()
