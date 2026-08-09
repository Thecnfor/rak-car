import unittest
from unittest.mock import MagicMock, patch

from main.task.task5_sort import _phase1_detect_high_tower


class TestTask5Phase1Handoff(unittest.TestCase):
    def test_phase1_handoff_skips_duplicate_pose_and_still_detects_label(self):
        client = MagicMock()
        client.http.request_vision_task.return_value = {
            "detections": [{"label": "label_blue", "score": 0.8}]
        }
        runner = MagicMock()

        result = _phase1_detect_high_tower(client, runner, phase1_pose_ready=True)

        self.assertEqual(result["label"], "blue")
        client.composite_run.assert_not_called()
        client.http.request_vision_task.assert_called_once()


if __name__ == "__main__":
    unittest.main()
