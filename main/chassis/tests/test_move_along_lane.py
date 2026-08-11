"""main/chassis/tests/test_move_along_lane.py
``move_along_lane`` 单测 (stdlib unittest, 离线无硬件)。

2026-08-11 下沉后语义：循线闭环在 **runtime 进程内** 跑（``lane_dis_offset`` /
``lane_time`` 官方极简法），``move_along_lane`` 只 POST 一次 ``/v1/execute``
同步等结果。本测试验证委托正确性（距离模式 → lane_dis_offset、时间模式 →
lane_time、dry_run 不下发、失败抛错）。
"""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

# 路径: main/chassis/tests/ → repo_root（同 main/task/tests 的 bootstrap 写法,
# 让 `python3 main/chassis/tests/test_move_along_lane.py` 也能直接跑）
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from main.chassis.controllers.move_along_lane import move_along_lane


class TestMoveAlongLaneExposed(unittest.TestCase):
    def test_importable_from_controllers_package(self):
        # 方法在 controllers/ 目录下可经包直接 import（暴露入口）
        from main.chassis.controllers import move_along_lane
        self.assertTrue(callable(move_along_lane))


class _FakeHttp:
    """记录对 runtime 的 execute_car_action 调用。"""

    def __init__(self, result=None) -> None:
        self.calls = []
        self._result = result if result is not None else {"status": "succeeded", "result": None}

    def execute_car_action(self, name, *args, timeout=None, sync=False):
        self.calls.append({"name": name, "args": args, "timeout": timeout, "sync": sync})
        return self._result


class _FakeApi:
    def __init__(self, http=None) -> None:
        self.http = http if http is not None else _FakeHttp()


class TestMoveAlongLaneDelegates(unittest.TestCase):
    """move_along_lane 只委托 runtime 进程内 action，不建网络外环。"""

    def test_distance_mode_calls_lane_dis_offset(self):
        http = _FakeHttp()
        api = _FakeApi(http)
        with patch("main.chassis.controllers.move_along_lane.ChassisClient") as mc:
            mc.connect.return_value = api
            move_along_lane(vx=0.2, distance_m=1.5)
        self.assertEqual(len(http.calls), 1)
        self.assertEqual(http.calls[0]["name"], "lane_dis_offset")
        self.assertEqual(http.calls[0]["args"], (0.2, 1.5))
        self.assertTrue(http.calls[0]["sync"])

    def test_backward_vx_passes_signed_speed(self):
        http = _FakeHttp()
        api = _FakeApi(http)
        with patch("main.chassis.controllers.move_along_lane.ChassisClient") as mc:
            mc.connect.return_value = api
            move_along_lane(vx=-0.15, distance_m=2.0)
        self.assertEqual(http.calls[0]["args"], (-0.15, 2.0))

    def test_time_mode_calls_lane_time(self):
        http = _FakeHttp()
        api = _FakeApi(http)
        with patch("main.chassis.controllers.move_along_lane.ChassisClient") as mc:
            mc.connect.return_value = api
            move_along_lane(vx=0.2, max_seconds=3.0)
        self.assertEqual(len(http.calls), 1)
        self.assertEqual(http.calls[0]["name"], "lane_time")
        self.assertEqual(http.calls[0]["args"], (0.2, 3.0))

    def test_dry_run_does_not_issue_action(self):
        http = _FakeHttp()
        api = _FakeApi(http)
        with patch("main.chassis.controllers.move_along_lane.ChassisClient") as mc:
            mc.connect.return_value = api
            move_along_lane(vx=0.2, distance_m=1.0, dry_run=True)
        self.assertEqual(http.calls, [])

    def test_failed_status_raises(self):
        http = _FakeHttp(result={"status": "failed", "error": "lane lost"})
        api = _FakeApi(http)
        with patch("main.chassis.controllers.move_along_lane.ChassisClient") as mc:
            mc.connect.return_value = api
            with self.assertRaises(RuntimeError):
                move_along_lane(vx=0.2, distance_m=1.0)


if __name__ == "__main__":
    unittest.main()
