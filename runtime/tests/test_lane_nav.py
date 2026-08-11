#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""runtime/services/lane_nav.py 单测（无硬件，纯桩 service）。

覆盖：
  - RuntimeLaneIo：read_lane/get_odometry_state 缓存映射 + 同机 age 正确、
    set_wheel_speeds 委托、close 零速
  - LaneNavController：start 幂等 / pause 同步 ack / resume / stop 零速 +
    心跳 iter_count 递增（环真在跑）
"""
import time
import unittest


class _FakeSvc:
    """最小 service 桩：lane/odom 缓存 + set_wheel_speeds/emergency_stop。

    每次 get_lane_state 返回"刚刚刷新"的缓存（updated_at=now）→ 同机时钟下
    age 正确、watchdog 不误杀，让真实 DoubleLoopRunner 能稳定跑几帧。
    """

    def __init__(self):
        self.wheel_speeds = None
        self.emergencies = 0

    def get_lane_state(self):
        return {
            "active": True,
            "mode": "external_feed",
            "error_y": 0.01,
            "error_angle": 0.0,
            "updated_at": time.time(),
        }

    def get_odom_state(self):
        return {
            "active": True,
            "mode": "odom_feed",
            "x": 0.0, "y": 0.0, "theta": 0.0, "distance": 0.0,
            "updated_at": time.time(),
        }

    def set_wheel_speeds(self, speeds):
        self.wheel_speeds = [float(s) for s in speeds]
        return {"speeds": list(self.wheel_speeds)}

    def emergency_stop(self):
        self.emergencies += 1
        return True


class TestRuntimeLaneIo(unittest.TestCase):
    def _io(self):
        from runtime.services.lane_nav import RuntimeLaneIo
        return RuntimeLaneIo(_FakeSvc()), _FakeSvc()

    def test_read_lane_maps_and_age_fresh(self):
        from runtime.services.lane_nav import RuntimeLaneIo
        svc = _FakeSvc()
        io = RuntimeLaneIo(svc)
        lane = io.read_lane()
        self.assertAlmostEqual(lane.error_y, 0.01)
        self.assertTrue(lane.has_error)
        # 同机时钟 → age 几 ms，watchdog 不误杀
        self.assertTrue(lane.is_fresh)
        self.assertLess(lane.age_ms, 500.0)

    def test_read_lane_exception_returns_empty(self):
        from runtime.services.lane_nav import RuntimeLaneIo
        svc = _FakeSvc()
        io = RuntimeLaneIo(svc)
        svc.get_lane_state = lambda: (_ for _ in ()).throw(RuntimeError("boom"))
        lane = io.read_lane()
        self.assertIsNone(lane.error_y)
        self.assertFalse(lane.has_error)

    def test_get_odometry_state_maps(self):
        from runtime.services.lane_nav import RuntimeLaneIo
        svc = _FakeSvc()
        io = RuntimeLaneIo(svc)
        odom = io.get_odometry_state()
        self.assertEqual(odom.x, 0.0)
        self.assertEqual(odom.theta, 0.0)
        self.assertTrue(odom.is_fresh)

    def test_set_wheel_speeds_delegates(self):
        from runtime.services.lane_nav import RuntimeLaneIo
        svc = _FakeSvc()
        io = RuntimeLaneIo(svc)
        io.set_wheel_speeds([0.1, 0.1, -0.1, -0.1])
        self.assertEqual(svc.wheel_speeds, [0.1, 0.1, -0.1, -0.1])

    def test_emergency_stop_delegates(self):
        from runtime.services.lane_nav import RuntimeLaneIo
        svc = _FakeSvc()
        io = RuntimeLaneIo(svc)
        self.assertTrue(io.emergency_stop())
        self.assertEqual(svc.emergencies, 1)

    def test_close_zeros(self):
        from runtime.services.lane_nav import RuntimeLaneIo
        svc = _FakeSvc()
        io = RuntimeLaneIo(svc)
        io.close()
        self.assertEqual(svc.wheel_speeds, [0.0, 0.0, 0.0, 0.0])

    def test_subscription_noop_true(self):
        from runtime.services.lane_nav import RuntimeLaneIo
        io = RuntimeLaneIo(_FakeSvc())
        self.assertTrue(io.start_lane_subscription())


class TestLaneNavController(unittest.TestCase):
    def _ctrl(self):
        from runtime.services.lane_nav import LaneNavController
        svc = _FakeSvc()
        return LaneNavController(svc), svc

    def test_start_stop_lifecycle(self):
        ctrl, svc = self._ctrl()
        r = ctrl.start(hz=5.0)
        self.assertTrue(r.get("started"))
        self.assertTrue(ctrl.state()["running"])
        try:
            # 幂等：已在跑不重建
            r2 = ctrl.start(hz=5.0)
            self.assertFalse(r2.get("started"))
            self.assertEqual(r2.get("reason"), "already_running")
            # 真在跑：等 0.6s（5Hz → ≥2 帧），心跳递增
            time.sleep(0.6)
            self.assertGreater(ctrl.state()["health"]["iter_count"], 0)
        finally:
            ctrl.stop()
        self.assertFalse(ctrl.state()["running"])
        self.assertEqual(svc.wheel_speeds, [0.0, 0.0, 0.0, 0.0])

    def test_pause_resume(self):
        ctrl, _svc = self._ctrl()
        ctrl.start(hz=5.0)
        try:
            r = ctrl.pause(timeout=1.0)
            self.assertTrue(r.get("paused"))
            self.assertTrue(ctrl.state()["paused"])
            r = ctrl.resume()
            self.assertTrue(r.get("resumed"))
            self.assertFalse(ctrl.state()["paused"])
        finally:
            ctrl.stop()

    def test_pause_not_running_noop(self):
        ctrl, _svc = self._ctrl()
        r = ctrl.pause(timeout=0.2)
        self.assertTrue(r.get("paused"))
        self.assertEqual(r.get("reason"), "not_running")

    def test_resume_never_started(self):
        ctrl, _svc = self._ctrl()
        r = ctrl.resume()
        self.assertFalse(r.get("resumed"))
        self.assertEqual(r.get("reason"), "never_started")


class TestBuildRunner(unittest.TestCase):
    def test_builds_double_loop_runner(self):
        from main.chassis.loops.closed_loop import DoubleLoopRunner
        from runtime.services.lane_nav import _build_runner
        svc = _FakeSvc()
        runner = _build_runner(svc, hz=5.0, controller_type="straight",
                               turn_cfg={"staircase": {"targets_deg": [45, 90, 120]},
                                         "detector": {"tol_deg": 20}},
                               watchdog_ms=500.0, lost_line_ms=None,
                               crossroad_turn=None)
        self.assertIsInstance(runner, DoubleLoopRunner)
        # 控制律/弯道装配生效（不是默认空）
        self.assertIsNotNone(runner.outer)
        self.assertIsNotNone(runner.turn)
        self.assertIsNotNone(runner.detector)
        self.assertEqual(list(runner.turn._targets), [45.0, 90.0, 120.0])


if __name__ == "__main__":
    unittest.main()
