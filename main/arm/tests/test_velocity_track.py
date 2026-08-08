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

    def test_setpoint_aligns_nozzle_center(self):
        # 目标在吸嘴中心 (0.2, -0.5), setpoint=(0.2,-0.5) → dx=dy=0 → 停
        result, posts = self._run(
            [_frame("ball_yellow", 0.2, -0.5)],
            gain=0.1, setpoint_x_norm=0.2, setpoint_y_norm=-0.5)
        self.assertEqual(posts[0]["x_vel"], 0.0)
        self.assertEqual(posts[0]["y_vel"], 0.0)
        self.assertEqual(result.trace[0].dx, 0.0)
        self.assertEqual(result.trace[0].dy, 0.0)

    def test_setpoint_offset_direction(self):
        # 目标在画面右下方 (0.5,0.4), setpoint=(0.161,-0.519):
        #   dx = 0.5-0.161 = +0.339 → x_vel = -0.339*0.1 = -0.034
        #   dy = 0.4-(-0.519) = +0.919 → y_vel = +0.919*0.1 = +0.092
        result, posts = self._run(
            [_frame("ball_yellow", 0.5, 0.4)],
            gain=0.1, setpoint_x_norm=0.161, setpoint_y_norm=-0.519)
        self.assertAlmostEqual(posts[0]["x_vel"], -0.0339, places=4)
        self.assertAlmostEqual(posts[0]["y_vel"], 0.0919, places=4)


class TestVelocity4Dof(unittest.TestCase):
    def _run(self, frames, **kw):
        loop, ws, posts, post_fn = _Runner.make(frames)
        result = loop.find_target_4dof("ball_yellow", ws=ws, post_fn=post_fn,
                                       timeout=0.3, **kw)
        return result, posts

    def test_arm_hand_increments(self):
        # dx>0 → arm 增大; dy>0 → hand 从 -90 增大; xy 方向同 XY 版.
        # hold_y=False 显式放开 y 十字, 验证四通道全动 (默认 hold_y=True 锁 y 另测).
        result, posts = self._run([_frame("ball_yellow", 0.5, 0.5)],
                                  gain_arm=2.0, gain_hand=2.0, hold_y=False)
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

    def test_setpoint_aligns_arm_hand(self):
        # 目标在吸嘴中心 (0.2,-0.5), setpoint=(0.2,-0.5) → dx=dy=0 → xy 停, 角度不动
        result, posts = self._run(
            [_frame("ball_yellow", 0.2, -0.5)],
            gain_arm=2.0, gain_hand=2.0,
            setpoint_x_norm=0.2, setpoint_y_norm=-0.5)
        self.assertEqual(posts[0]["x_vel"], 0.0)
        self.assertEqual(posts[0]["y_vel"], 0.0)
        self.assertEqual(result.end_arm, 0.0)      # arm_start=0, 无增量
        self.assertEqual(result.end_hand, -90.0)   # hand_start=-90, 无增量

    def test_hold_y_locks_y_axis(self):
        # hold_y=True (默认): 垂直误差 dy>0 但 y_vel 强制 0; hand 增量照常
        result, posts = self._run(
            [_frame("ball_yellow", 0.0, 0.5)],
            gain_y=0.1, gain_hand=2.0,
            setpoint_x_norm=0.0, setpoint_y_norm=0.0)
        self.assertEqual(posts[0]["y_vel"], 0.0)          # y 十字锁死
        self.assertNotEqual(posts[0]["hand"], -90.0)      # hand 增量照常 (dy>0 → hand 增大)
        self.assertEqual(result.end_hand, -89.0)          # -90 + 0.5*2

    def test_hold_y_false_allows_y(self):
        # hold_y=False: y_vel 由 dy 驱动
        result, posts = self._run(
            [_frame("ball_yellow", 0.0, 0.5)],
            gain_y=0.1, gain_hand=2.0,
            setpoint_x_norm=0.0, setpoint_y_norm=0.0,
            hold_y=False)
        self.assertAlmostEqual(posts[0]["y_vel"], 0.05, places=5)


class TestArmCross(unittest.TestCase):
    """find_target_arm_cross — 本机械专用映射 (arm 控 cx, x 十字控 cy).

    实机标定 2026-08-02: cx←arm_angle, cy←x 十字. 与通用 4-DOF 不同:
    dx → arm 增量 (sign_arm=+1), dy → x_vel (sign_x=-1), y_vel 恒 0.
    """

    def _run(self, frames, **kw):
        loop, ws, posts, post_fn = _Runner.make(frames)
        result = loop.find_target_arm_cross(
            "ball_yellow", ws=ws, post_fn=post_fn, timeout=0.3, **kw)
        return result, posts

    def test_dx_drives_arm(self):
        # dx = 0.5 - 0.161 = +0.339 > 0 → arm 减小(更负): -90 + 0.339*0.4 = -89.86
        result, posts = self._run([_frame("ball_yellow", 0.5, -0.519)],
                                  gain_arm=0.4, setpoint_x_norm=0.161,
                                  setpoint_y_norm=-0.519)
        self.assertAlmostEqual(posts[0]["arm"], -89.864, places=3)
        self.assertEqual(posts[0]["y_vel"], 0.0)

    def test_dy_drives_x_vel(self):
        # dy = 0.5 - (-0.519) = +1.019 > 0 → x_vel = -1.019*0.08 = -0.0815 (往左)
        result, posts = self._run([_frame("ball_yellow", 0.161, 0.5)],
                                  gain_x=0.08, max_vel=0.15,
                                  setpoint_x_norm=0.161, setpoint_y_norm=-0.519)
        self.assertAlmostEqual(posts[0]["x_vel"], -0.0815, places=4)
        self.assertEqual(posts[0]["y_vel"], 0.0)

    def test_y_vel_always_zero(self):
        result, posts = self._run([_frame("ball_yellow", 0.5, 0.5)],
                                  gain_arm=0.4, gain_x=0.08)
        self.assertEqual(posts[0]["y_vel"], 0.0)
        self.assertNotIn("hand", posts[0])

    def test_setpoint_aligned_stops(self):
        # 目标在吸嘴中心 → dx=dy=0 → x_vel=0, arm 不动
        result, posts = self._run([_frame("ball_yellow", 0.161, -0.519)],
                                  gain_arm=0.4, gain_x=0.08,
                                  setpoint_x_norm=0.161, setpoint_y_norm=-0.519)
        self.assertEqual(posts[0]["x_vel"], 0.0)
        self.assertEqual(result.end_arm, -90.0)

    def test_miss_posts_zero(self):
        result, posts = self._run([_frame("animal", 0.5, 0.5)])
        self.assertEqual(posts[0]["x_vel"], 0.0)
        self.assertEqual(posts[0]["y_vel"], 0.0)
        self.assertEqual(result.misses, 1)


class TestTrackVelocityPickSettle(unittest.TestCase):
    """track_velocity_pick 的 settled 窗口判定 (2026-08-02 加固)."""

    def _runner_with_result(self, trace):
        from main.arm.loops.runner import ArmRunner
        client = mock.MagicMock()
        client.origin = None
        client._resolve_nozzle_setpoint = lambda *a, **k: (0.161, -0.519)
        finder = client._make_vision_with_move.return_value
        res = VelocityResult(
            label="ball_yellow", frames=len(trace), hits=sum(1 for t in trace if not t.miss),
            misses=sum(1 for t in trace if t.miss), elapsed_s=1.0,
            end_arm=-83.5, end_hand=None, max_abs_vel_mms=10.0, avg_abs_vel_mms=5.0,
            trace=tuple(trace))
        finder.find_target_arm_cross.return_value = res
        return ArmRunner(client), client, finder

    @staticmethod
    def _frames(*pts, miss=()):
        out = []
        for i, (dx, dy) in enumerate(pts):
            out.append(VelocityTrace(
                t_s=float(i), dx=dx, dy=dy, x_vel=0.0, y_vel=0.0,
                arm=-83.5, score=0.9, miss=(i in miss)))
        return out

    def test_settled_window_found_in_middle(self):
        # 末段窗口内有一处连续 3 帧收敛 (前 2 帧没收敛) → settled=True
        trace = self._frames(
            (0.10, 0.10), (0.08, 0.08),        # 未收敛
            (0.01, 0.01), (0.005, -0.005), (0.01, 0.005),  # 收敛窗口
            (0.03, 0.03),                       # 末尾抖动
        )
        runner, client, finder = self._runner_with_result(trace)
        client.composite_run.return_value = {"ok": True}
        client.get_state.return_value = mock.MagicMock()
        client.move_y.return_value = {"ok": True}
        client.grasp.return_value = {"ok": True}
        result = runner.track_velocity_pick("ball_yellow")
        self.assertTrue(result["ok"], result)

    def test_not_settled_no_window(self):
        # 全段都没连续 3 帧收敛 → settled=False
        trace = self._frames(
            (0.10, 0.10), (0.08, 0.08), (0.06, 0.06),
            (0.05, 0.05), (0.07, 0.06), (0.08, 0.07),
        )
        runner, client, finder = self._runner_with_result(trace)
        result = runner.track_velocity_pick("ball_yellow")
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "not_settled")


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


class TestKalmanVelocity(unittest.TestCase):
    """find_target_velocity 的 kalman 平滑 (2026-08-09)."""

    def _run(self, frames, **kw):
        loop, ws, posts, post_fn = _Runner.make(frames)
        result = loop.find_target_velocity("ball_yellow", ws=ws, post_fn=post_fn,
                                           timeout=0.3, **kw)
        return result, posts

    def test_kalman_first_frame_unchanged(self):
        """首帧 kalman 直接初始化不过滤 → 单帧结果与关 kalman 完全一致."""
        frames = [_frame("ball_yellow", 0.5, 0.4)]
        r_raw, p_raw = self._run(frames, gain=0.1)
        r_kf, p_kf = self._run(frames, gain=0.1, kalman=True)
        self.assertAlmostEqual(p_kf[0]["x_vel"], p_raw[0]["x_vel"], places=6)
        self.assertAlmostEqual(p_kf[0]["y_vel"], p_raw[0]["y_vel"], places=6)

    def test_kalman_smooths_jitter(self):
        """小抖动 (真值 0.5 ±0.03 交替) → kalman 后 x_vel 方差显著小于原始."""
        frames = [_frame("ball_yellow", 0.47 if i % 2 else 0.53, 0.0)
                  for i in range(12)]
        _, p_raw = self._run(frames, gain=0.1)
        _, p_kf = self._run(frames, gain=0.1, kalman=True)

        def _xvels(posts):
            # 排除 finally 的收尾 (0,0) 帧
            return [p["x_vel"] for p in posts[:-1]]

        def _std(vals):
            m = sum(vals) / len(vals)
            return (sum((v - m) ** 2 for v in vals) / len(vals)) ** 0.5

        raw_std = _std(_xvels(p_raw))
        kf_std = _std(_xvels(p_kf))
        self.assertGreater(raw_std, 0.0)
        self.assertLess(kf_std, raw_std)

    def test_kalman_miss_still_zero(self):
        """丢帧帧 (miss) 不发 kalman, 照旧发 0 停."""
        frames = [_frame("ball_yellow", 0.5, 0.5),
                  _frame("animal", 0.2, 0.2)]
        result, posts = self._run(frames, kalman=True)
        self.assertEqual(result.hits, 1)
        self.assertEqual(result.misses, 1)
        self.assertEqual(posts[1]["x_vel"], 0.0)

    def test_kalman_import_fail_disables(self):
        """filterpy 未装 (ArmKalmanTracker 构造抛 ImportError) → 自动降级, 行为同原始."""
        frames = [_frame("ball_yellow", 0.5, 0.4)]
        r_raw, p_raw = self._run(frames, gain=0.1)
        with mock.patch(
            "main.arm.vision.kalman.ArmKalmanTracker.__init__",
            side_effect=ImportError("no filterpy"),
        ):
            # _maybe_tracker 内部捕获 → 返回 None 降级, kalman=True 不炸
            r_kf, p_kf = self._run(frames, gain=0.1, kalman=True)
        self.assertAlmostEqual(p_kf[0]["x_vel"], p_raw[0]["x_vel"], places=6)

    def test_4dof_kalman_unchanged_first_frame(self):
        """4dof + kalman 首帧结果与关闭一致."""
        loop, ws, posts, post_fn = _Runner.make(
            [_frame("ball_yellow", 0.3, 0.2)])
        result = loop.find_target_4dof(
            "ball_yellow", ws=ws, post_fn=post_fn, timeout=0.3,
            gain_x=0.1, gain_y=0.1, gain_arm=2.0, gain_hand=2.0,
            kalman=True)
        self.assertEqual(result.hits, 1)
        self.assertTrue(all("x_vel" in p for p in posts))


if __name__ == "__main__":
    unittest.main()
