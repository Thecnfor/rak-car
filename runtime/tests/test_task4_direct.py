import unittest
from unittest.mock import Mock

from runtime.tasks.task4_direct import Task4Direct, run


class FakeArm:
    def __init__(self):
        self.calls = []

    def composite_run(self, **kwargs):
        self.calls.append(("composite", kwargs))
        return {"ok": True}

    def move_y_position(self, value):
        self.calls.append(("y", value))

    def move_x_position(self, value):
        self.calls.append(("x", value))

    def grasp(self, value):
        self.calls.append(("grasp", value))


class FakeCar:
    def __init__(self, detections=None, ir=None):
        self.detections = detections or []
        self.ir = ir or {"left": 0.4}
        self.x = 0.0
        self.moves = []
        self.arm = FakeArm()
        self.align_calls = []

    def get_odometry(self):
        return {"x": self.x}

    def get_all_ir_distance(self):
        return self.ir

    def get_detection_results(self, **kwargs):
        return self.detections

    def move_for(self, vector, stop=True):
        self.moves.append((vector, stop))
        self.x += vector[0]

    def move_to_detection_target(self, **kwargs):
        self.align_calls.append(kwargs)


class TestTask4Direct(unittest.TestCase):
    def test_no_target_exits_after_distance_budget(self):
        car = FakeCar()
        result = Task4Direct(car, creep_m=0.25).run()
        self.assertEqual(result["reason"], "distance_exhausted")
        self.assertFalse(result["ok"])
        self.assertEqual(car.moves[-1], ([0.0, 0.0, 0.0], True))

    def test_target_calls_direct_arm_and_alignment(self):
        target = [1, 0, "ball", 0.9, 0.0, 0.0, 0.2, 0.2]
        car = FakeCar(detections=[[2, 0, "person", 0.99, 0, 0, 1, 1], target])
        result = Task4Direct(car).run()
        self.assertTrue(result["ok"])
        self.assertEqual(result["picked"], 1)
        self.assertEqual(car.align_calls[0]["label"], "ball")
        self.assertIn(("grasp", True), car.arm.calls)
        self.assertIn(("grasp", False), car.arm.calls)

    def test_dry_run_does_not_construct_hardware(self):
        result = run(dry_run=True)
        self.assertEqual(result["reason"], "dry_run")
        self.assertEqual(result["picked"], 0)


if __name__ == "__main__":
    unittest.main()
