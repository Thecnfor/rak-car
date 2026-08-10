import unittest

from runtime.services.fake_runtime import ActionRecorder, FakeCarRuntimeService


class FakeRuntimeTests(unittest.TestCase):
    def test_job_records_lifecycle_and_updates_relative_odom(self):
        service = FakeCarRuntimeService(action_delay=0.0)
        submitted = service.submit_job("car", "move_for", [[0.2, -0.1, 0.0]])
        result = service.wait_job(submitted["job_id"], timeout=1.0)
        self.assertEqual(result["status"], "succeeded")
        self.assertAlmostEqual(service.state["odom"]["x"], 0.2)
        phases = [event.phase for event in service.recorder.matching(action="move_for")]
        self.assertEqual(phases, ["queued", "started", "physical", "completed"])

    def test_realtime_does_not_create_job_and_estop_zeroes_outputs(self):
        service = FakeCarRuntimeService()
        service.set_arm_velocity(0.1, -0.2)
        service.emergency_stop()
        self.assertEqual(service.list_jobs(), [])
        self.assertEqual(service.state["wheels"], [0.0] * 4)
        self.assertTrue(service.recorder.matching(target="realtime", action="set_arm_velocity"))
        self.assertTrue(service.recorder.matching(action="set_wheel_speeds"))

    def test_cancelled_job_is_observable(self):
        service = FakeCarRuntimeService(action_delay=0.2)
        submitted = service.submit_job("arm", "grasp", [True])
        self.assertTrue(service.cancel_job(submitted["job_id"]))
        result = service.wait_job(submitted["job_id"], timeout=1.0)
        self.assertEqual(result["status"], "cancelled")

    def test_chassis_align_no_target_then_settles(self):
        service = FakeCarRuntimeService(action_delay=0.0)
        # 无目标 → no_target
        resp = service.chassis_align(target="h_tu_dou", setpoint_cxcy=[0.0, 0.0])
        self.assertEqual(resp["result"]["reason"], "no_target")
        self.assertFalse(resp["result"]["arrived"])
        # 注入目标 → 对齐到 setpoint，记录 physical_sample 命令包
        service.set_task_detections([{"label": "h_tu_dou", "cx": 0.02, "cy": -0.01,
                                      "score": 0.9}])
        resp2 = service.chassis_align(target="h_tu_dou", setpoint_cxcy=[0.0, 0.0])
        self.assertTrue(resp2["result"]["arrived"])
        self.assertTrue(resp2["result"]["final_frame"]["target_found"])
        samples = service.recorder.matching(action="chassis_align",
                                            phase="physical_sample")
        self.assertTrue(samples)
        self.assertEqual(samples[0].state["packets"][0]["name"],
                         "set_chassis_velocity")

    def test_replay_arm_trajectory_action_end_to_end(self):
        """一次 execute 传姿态 JSON，fake 进程内规划+回放，末端精确到 goal。"""
        from main.local_api_client import create_runtime_client
        from runtime.services.fake_runtime import get_fake_runtime

        client = create_runtime_client(transport="fake")
        route = [
            {"x_mm": -223.7, "y_mm": -150.1, "arm_deg": 90, "hand_deg": -10},
            {"x_mm": -98.2, "y_mm": -60.3, "arm_deg": 24, "hand_deg": -22},
            {"x_mm": -87.7, "y_mm": -158.6, "arm_deg": -82, "hand_deg": -22},
        ]
        job = client.execute_car_action("replay_arm_trajectory",
                                        route=route, sync=True, timeout=15.0)
        self.assertEqual(job["status"], "succeeded")
        res = job.get("result") or {}
        self.assertTrue(res.get("ok"))
        st = get_fake_runtime().get_arm_state()
        goal = route[-1]
        self.assertAlmostEqual(st["x_mm"], goal["x_mm"], delta=5.0)
        self.assertAlmostEqual(st["arm_angle"], goal["arm_deg"], delta=0.5)
        self.assertAlmostEqual(st["hand_angle"], goal["hand_deg"], delta=0.5)


if __name__ == "__main__":
    unittest.main()
