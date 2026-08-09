import unittest
from unittest.mock import MagicMock, patch

from main.start.orchestrator import Orchestrator, Waypoint


class TestTask4Task5Handoff(unittest.TestCase):
    def test_single_task4_does_not_defer_handoff(self):
        waypoints = [
            Waypoint("task4", task_id=4, task_module="task4"),
        ]
        self.assertFalse(Orchestrator._should_defer_task4_handoff(waypoints, 0))

    def test_adjacent_task4_task5_defers_handoff(self):
        waypoints = [
            Waypoint("task4", task_id=4, task_module="task4"),
            Waypoint("task5", task_id=5, task_module="task5"),
        ]
        self.assertTrue(Orchestrator._should_defer_task4_handoff(waypoints, 0))

    @patch("main.arm.ArmClient")
    def test_handoff_closes_storage_moves_phase1_and_restarts_feed(self, arm_cls):
        arm = arm_cls.return_value
        arm.set_storage_angle.return_value = {"ok": True}
        arm.composite_run.return_value = {
            "status": "succeeded", "result": {"ok": True},
        }
        client = MagicMock()
        client.execute.return_value = {
            "status": "succeeded", "result": {"started": True, "hz": 20.0},
        }

        thread = Orchestrator._start_task4_task5_handoff(client)
        self.assertTrue(Orchestrator._wait_task4_task5_handoff(thread))

        arm.set_storage_angle.assert_called_once_with(98, speed=5, timeout=10.0)
        arm.composite_run.assert_called_once_with(
            arm=90.0, x_mm=-28.0, y_mm=-121.0, hand=-58.0,
            speed=80, timeout=30.0,
        )
        client.execute.assert_called_once_with(
            "car", "start_arm_feed",
            kwargs={"hz": 20.0}, timeout=5.0, sync=True,
        )

    @patch("main.arm.ArmClient")
    def test_handoff_ok_false_when_arm_feed_restart_fails(self, arm_cls):
        """arm_feed 恢复失败 → ok=False, task5 不能误信 phase1_pose_ready 跳过摆臂。"""
        arm = arm_cls.return_value
        arm.set_storage_angle.return_value = {"ok": True}
        arm.composite_run.return_value = {
            "status": "succeeded", "result": {"ok": True},
        }
        client = MagicMock()
        client.execute.return_value = {"status": "failed", "error": "boom"}

        thread = Orchestrator._start_task4_task5_handoff(client)
        self.assertFalse(Orchestrator._wait_task4_task5_handoff(thread))


if __name__ == "__main__":
    unittest.main()
