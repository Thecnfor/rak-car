import unittest

from main.chassis.planning import PathWaypoint, Pose2D, plan_smooth_path


class SmoothPathTests(unittest.TestCase):
    def test_preserves_start_goal_and_waypoint_order(self):
        start = Pose2D(0.0, 0.0, 0.0)
        middle = PathWaypoint(1.0, 1.0, stop=True)
        goal = Pose2D(2.0, 0.0, 0.0)
        path = plan_smooth_path(start, [middle], goal, spacing_m=0.25)
        self.assertEqual(path.samples[0].pose, start)
        self.assertEqual(path.samples[-1].pose, goal)
        coords = [(s.pose.x_m, s.pose.y_m) for s in path.samples]
        self.assertIn((middle.x_m, middle.y_m), coords)
        self.assertEqual(sorted(s.arc_length_m for s in path.samples),
                         [s.arc_length_m for s in path.samples])

    def test_rejects_invalid_spacing_and_duplicate_middle(self):
        with self.assertRaises(ValueError):
            plan_smooth_path(Pose2D(0, 0), [], Pose2D(1, 0), spacing_m=0)
        with self.assertRaises(ValueError):
            plan_smooth_path(Pose2D(0, 0), [PathWaypoint(0, 0)], Pose2D(1, 0))

    def test_speed_is_bounded(self):
        path = plan_smooth_path(Pose2D(0, 0), [], Pose2D(1, 0),
                                 max_speed_mps=0.12)
        self.assertTrue(all(sample.speed_mps <= 0.12 for sample in path.samples))


if __name__ == "__main__":
    unittest.main()
