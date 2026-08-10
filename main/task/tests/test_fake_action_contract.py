import unittest

from main.local_api_client import LocalRuntimeClient, create_runtime_client
from main.task import TASK_RUNNERS


class FakeActionContractTests(unittest.TestCase):
    def test_tasks_one_through_seven_are_registered(self):
        for task_id in range(1, 8):
            self.assertIn(task_id, TASK_RUNNERS)
            self.assertTrue(callable(TASK_RUNNERS[task_id]))

    def test_fake_factory_does_not_create_network_client(self):
        client = create_runtime_client(transport="fake")
        self.assertIsInstance(client, LocalRuntimeClient)
        self.assertTrue(client.get_runtime()["fake"])
        self.assertEqual(client.api_base, "local://runtime")
        self.assertIn("car", client.get_actions()["actions"])
        client.close_runtime()

    def test_sync_and_async_share_recorded_job(self):
        client = create_runtime_client(transport="fake")
        submitted = client.execute_car_action("move_for", [0.1, 0.0, 0.0])
        result = client.wait_job(submitted["job_id"], timeout=1.0)
        self.assertEqual(submitted["job_id"], result["job_id"])
        self.assertEqual(result["status"], "succeeded")
        client.close_runtime()


if __name__ == "__main__":
    unittest.main()
