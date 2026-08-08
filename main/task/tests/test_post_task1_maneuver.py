"""main/task/tests/test_post_task1_maneuver.py

验证 task1 结束后的编排段 (orchestrator._post_task1_maneuver):
  清零里程 → 切断视觉 → 直行 → 里程计 θ 转弯 → 恢复视觉 → 等 lane 新鲜。
离线, mock 掉 api, 不打硬件。
"""
import sys
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from main.chassis.state import LaneState  # noqa: E402
from main.start.orchestrator import Orchestrator  # noqa: E402

# 走 importlib 加载 _config.py, 绕开 main/task/__init__.py
import importlib.util as _il  # noqa: E402
_cfg_spec = _il.spec_from_file_location(
    "_main_task_config_under_test",
    _REPO_ROOT / "main/task/_config.py",
)
_config = _il.module_from_spec(_cfg_spec)
_cfg_spec.loader.exec_module(_config)
load_post_task1 = _config.load_post_task1  # noqa: E402
load_post_task6 = _config.load_post_task6  # noqa: E402


class _FakeApi:
    """模拟 ChassisClient: 里程计 θ 按 wz 积分 + 记录调用序列。"""

    def __init__(self):
        self.theta = 0.0
        self.calls = []          # ("stop_lane_feed",) / ("move_for", dx, dy) / ...
        self.feed_on = False     # start_lane_feed 后 lane_state 变新鲜

    def stop_lane_feed(self, timeout=5.0):
        self.feed_on = False
        self.calls.append(("stop_lane_feed",))

    def start_lane_feed(self, hz=50.0, timeout=10.0):
        self.feed_on = True
        self.calls.append(("start_lane_feed", hz))

    def move_for(self, dx_m=0.0, dy_m=0.0, timeout=30.0, max_velocity_ms=0.20):
        self.calls.append(("move_for", dx_m, dy_m))
        return {"status": "succeeded"}

    def get_odometry(self, timeout=5.0):
        return 0.0, 0.0, self.theta

    def set_chassis_velocity(self, vx, vy, wz=0.0, timeout=5.0):
        self.theta += wz * 0.02  # 模拟 50Hz 积分
        self.calls.append(("chassis_velocity", vx, vy, wz))

    def set_wheel_speeds(self, speeds, timeout=5.0):
        self.calls.append(("wheels", tuple(speeds)))

    def read_lane(self):
        return LaneState(age_ms=10 if self.feed_on else 5000)


class TestPostTask1Config(unittest.TestCase):
    def test_load_post_task1_present(self):
        seg = load_post_task1()
        self.assertIsNotNone(seg)
        self.assertTrue(seg.get("enabled"))
        self.assertAlmostEqual(float(seg["straight_m"]), 0.09)
        self.assertAlmostEqual(float(seg["turn_deg"]), -45.0)

    def test_load_post_task6_present(self):
        seg = load_post_task6()
        self.assertIsNotNone(seg)
        self.assertTrue(seg.get("enabled"))
        self.assertAlmostEqual(float(seg["straight_m"]), 0.2)
        self.assertAlmostEqual(float(seg["turn_deg"]), -120.0)


class TestTurnThetaDeg(unittest.TestCase):
    def test_rotates_45_deg_and_stops(self):
        api = _FakeApi()
        Orchestrator._turn_theta_deg(api, 45.0)
        # 转到 ~45° (2° 容差 + PID 收敛余量)
        self.assertAlmostEqual(api.theta, 0.7854, delta=0.06)
        # 结尾一定补了零速 (wheels), 转弯过程只走 realtime 不占 job
        self.assertEqual(api.calls[-1][0], "wheels")
        self.assertEqual(api.calls[-1][1], (0.0, 0.0, 0.0, 0.0))
        # 确实发过非零 ω
        wzs = [c[3] for c in api.calls if c[0] == "chassis_velocity"]
        self.assertTrue(any(abs(w) > 0 for w in wzs))

    def test_zero_turn_deg_no_motion(self):
        api = _FakeApi()
        Orchestrator._turn_theta_deg(api, 0.0)
        self.assertAlmostEqual(api.theta, 0.0)
        wzs = [c[3] for c in api.calls if c[0] == "chassis_velocity"]
        self.assertFalse(any(abs(w) > 0 for w in wzs))


class TestPostTask1Maneuver(unittest.TestCase):
    def test_full_sequence(self):
        api = _FakeApi()
        seg = {"enabled": True, "straight_m": 0.1, "turn_deg": 45.0}
        Orchestrator(waypoints=[])._post_task1_maneuver(api, seg)

        seq = [c[0] for c in api.calls]
        # 切断视觉 → 直行 → 转弯 → 恢复视觉 (wheels 零速是转弯收尾)
        self.assertEqual(seq[0], "stop_lane_feed")
        self.assertIn(("move_for", 0.1, 0.0), api.calls)
        self.assertTrue(any(c[0] == "chassis_velocity" and c[3] != 0 for c in api.calls))
        self.assertIn(("start_lane_feed", 50.0), api.calls)
        self.assertAlmostEqual(api.theta, 0.7854, delta=0.06)

    def test_zeroed_fields_skip_steps(self):
        api = _FakeApi()
        seg = {"enabled": True, "straight_m": 0.0, "turn_deg": 0.0}
        Orchestrator(waypoints=[])._post_task1_maneuver(api, seg)
        names = [c[0] for c in api.calls]
        self.assertNotIn("move_for", names)
        self.assertFalse(any(c[0] == "chassis_velocity" and c[3] != 0 for c in api.calls))
        # 视觉仍按需求 切断 → 恢复
        self.assertEqual(names[0], "stop_lane_feed")
        self.assertIn("start_lane_feed", names)


class TestWaitLaneFresh(unittest.TestCase):
    def test_fresh_after_feed_restart(self):
        api = _FakeApi()
        api.start_lane_feed()  # feed_on=True → read_lane 新鲜
        self.assertTrue(Orchestrator._wait_lane_fresh(api, timeout_s=1.0))

    def test_timeout_when_never_fresh(self):
        api = _FakeApi()  # feed_on=False → 恒不新鲜
        self.assertFalse(Orchestrator._wait_lane_fresh(api, timeout_s=0.5))


if __name__ == "__main__":
    unittest.main()
