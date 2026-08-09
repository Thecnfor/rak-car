import unittest

from main.arm.each_task.task4.target4 import _CreepThread


class _FakeHttp:
    def __init__(self, ir_left=0.4):
        self.ir_left = ir_left
        self.odom_reads = 0
        self.posts = []

    def post(self, path, payload=None, timeout=None):
        self.posts.append((path, payload, timeout))
        return {"ok": True}

    def get_odom_state(self):
        self.odom_reads += 1
        # start() 与首轮看到触发点, 后续轮报走满 0.3m。
        x = 0.31 if self.odom_reads >= 4 else 0.0
        return {"odom_state": {"x": x}}

    def get_ir_state(self):
        return {"ir_state": {"left": self.ir_left}}

    def get_task_state(self):
        return {"task_state": {"active": True, "detections": []}}


class TestTask4IrOdomExit(unittest.TestCase):
    def test_wait_for_ball_exposes_ir_odom_terminal_state(self):
        http = _FakeHttp(ir_left=0.4)  # 近读 → 触发 IR 结束
        creep = _CreepThread(http, speed_mps=0.06, max_distance_m=0.8,
                             poll_hz=100.0)
        creep.start()
        try:
            result = creep.wait_for_ball(timeout_s=1.0)
            self.assertTrue(result["finished_by_ir_odom"])
            self.assertIsNone(result["balls"])
            self.assertTrue(any(
                payload and payload.get("vx") == 0.0
                for _, payload, _ in http.posts
            ))
        finally:
            creep.stop_and_join()

    def test_ir_odom_terminal_state_is_distinct_from_no_ball_timeout(self):
        http = _FakeHttp()
        creep = _CreepThread(http, speed_mps=0.06, max_distance_m=0.8)

        result = creep.wait_for_ball(timeout_s=0.0)

        self.assertFalse(result["finished_by_ir_odom"])
        self.assertIsNone(result["balls"])

    def test_ir_exit_disabled_on_first_ball_keeps_searching(self):
        """第 1 球 (ir_exit_enabled=False): 近读 IR 也不能触发结束。"""
        http = _FakeHttp(ir_left=0.4)
        creep = _CreepThread(http, speed_mps=0.06, max_distance_m=0.8,
                             poll_hz=100.0, ir_exit_enabled=False)
        creep.start()
        try:
            result = creep.wait_for_ball(timeout_s=0.15)
            self.assertFalse(result["finished_by_ir_odom"])
        finally:
            creep.stop_and_join()

    def test_ir_exit_ignores_far_reading(self):
        """采区中途 IR 读远 (1.5m): 不触发结束, 不吞掉搜索。"""
        http = _FakeHttp(ir_left=1.5)
        creep = _CreepThread(http, speed_mps=0.06, max_distance_m=0.8,
                             poll_hz=100.0)
        creep.start()
        try:
            result = creep.wait_for_ball(timeout_s=0.15)
            self.assertFalse(result["finished_by_ir_odom"])
        finally:
            creep.stop_and_join()

    def test_budget_exhausted_wakes_wait_for_ball(self):
        """走满距离预算 (无球/无IR): 必须唤醒 wait_for_ball, 否则主线程干等 (球9 卡死)."""
        import time as _t
        http = _FakeHttp(ir_left=1.5)
        creep = _CreepThread(http, speed_mps=0.06, max_distance_m=0.3,
                             poll_hz=100.0)
        creep.start()
        try:
            t0 = _t.monotonic()
            result = creep.wait_for_ball(timeout_s=1.0)
            elapsed = _t.monotonic() - t0
            # odom 走到 0.31 ≥ 0.3 预算 → 线程退出并唤醒, 不应等满 1s
            self.assertLess(elapsed, 0.5)
            self.assertIsNone(result["balls"])
            self.assertFalse(result["finished_by_ir_odom"])
        finally:
            creep.stop_and_join()

    def test_ir_exit_time_fallback_when_odom_frozen(self):
        """odom 冻结 (0.000m): IR 触发后靠时间兜底收尾, 不再挂死等手动停. (现场整场 odom 冻结)"""
        class _FrozenOdomHttp(_FakeHttp):
            def __init__(self):
                super().__init__(ir_left=0.4)
            def get_odom_state(self):
                return {"odom_state": {"x": 0.0}}  # 永远 0, odom 冻结

        import time as _t
        http = _FrozenOdomHttp()
        creep = _CreepThread(http, speed_mps=0.06, max_distance_m=0.8,
                             poll_hz=100.0, ir_max_seconds_s=0.2)
        creep.start()
        try:
            t0 = _t.monotonic()
            result = creep.wait_for_ball(timeout_s=1.0)
            elapsed = _t.monotonic() - t0
            # 触发后 ~0.2s 时间兜底即收尾, 不应等到 1s
            self.assertLess(elapsed, 0.8)
            self.assertTrue(result["finished_by_ir_odom"])
        finally:
            creep.stop_and_join()


if __name__ == "__main__":
    unittest.main()
