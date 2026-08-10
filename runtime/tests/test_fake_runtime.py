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


if __name__ == "__main__":
    unittest.main()
