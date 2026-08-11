import sys
import threading
import types
import unittest
from unittest.mock import MagicMock


class _InferBackendService:
    pass


sys.modules.setdefault(
    "runtime.services.inference_service",
    types.SimpleNamespace(InferBackendService=_InferBackendService),
)

from runtime.services.car_runtime_service import CarRuntimeService


class TestRuntimeFourWheelTraction(unittest.TestCase):
    def test_set_chassis_velocity_forwards_all_four_sdk_wheels(self):
        service = CarRuntimeService.__new__(CarRuntimeService)
        service._realtime_gate = threading.Lock()
        service._chassis_cmd_lock = threading.Lock()
        service._chassis_cmd = {}
        service._chassis_cmd_history = []

        car = MagicMock()
        requested_wheels = [0.14, -0.02, -0.14, 0.02]
        car.chassis.calculate_wheel_velocities.return_value = requested_wheels
        service.car = car

        result = service.set_chassis_velocity(0.10, 0.08, 0.0)

        car.chassis.calculate_wheel_velocities.assert_called_once_with(
            0.10, 0.08, 0.0
        )
        car.wheels_chassis.set_linear.assert_called_once_with(requested_wheels)
        self.assertEqual(result["wheel_speeds"], requested_wheels)
        self.assertEqual(len(result["wheel_speeds"]), 4)


if __name__ == "__main__":
    unittest.main()
