import importlib.util
import pathlib
import sys
import types
import unittest

import numpy as np


_ROOT = pathlib.Path(__file__).resolve().parents[3]
_DRIVER_DIR = _ROOT / "smartcar" / "whalesbot" / "vehicle" / "driver"


def _package(name, path):
    module = types.ModuleType(name)
    module.__path__ = [str(path)]
    sys.modules[name] = module
    return module


_package("smartcar", _ROOT / "smartcar")
_package("smartcar.whalesbot", _ROOT / "smartcar" / "whalesbot")
_tools = _package("smartcar.whalesbot.tools", _ROOT / "smartcar" / "whalesbot" / "tools")
_tools.PID = object
_package("smartcar.whalesbot.vehicle", _ROOT / "smartcar" / "whalesbot" / "vehicle")
_package("smartcar.whalesbot.vehicle.driver", _DRIVER_DIR)
_base = _package("smartcar.whalesbot.vehicle.base", _ROOT / "smartcar" / "whalesbot" / "vehicle" / "base")
_controller = types.ModuleType("smartcar.whalesbot.vehicle.base.controller_wrap")
_controller.WheelWrap = object
sys.modules[_controller.__name__] = _controller
_log = types.ModuleType("smartcar.whalesbot.tools.log_wrap")
_log.logger = __import__("logging").getLogger("mecanum-test")
sys.modules[_log.__name__] = _log


def _load(name):
    path = _DRIVER_DIR / (name + ".py")
    full_name = "smartcar.whalesbot.vehicle.driver." + name
    spec = importlib.util.spec_from_file_location(full_name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[full_name] = module
    spec.loader.exec_module(module)
    return module


_mecanum = _load("mecanum")
MecanumChassis = _mecanum.MecanumChassis


class TestSdkMecanumTraction(unittest.TestCase):
    def setUp(self):
        self.chassis = MecanumChassis()

    def test_mixed_translation_keeps_four_wheels_loaded(self):
        base = self.chassis.inverse_kinematics(np.array([0.10, 0.10, 0.0]))
        actual = self.chassis.calculate_wheel_velocities(0.10, 0.10, 0.0)

        self.assertLess(min(abs(float(v)) for v in base), 0.02)
        self.assertGreater(min(abs(float(v)) for v in actual), 0.01)

    def test_traction_bias_preserves_body_velocity(self):
        requested = np.array([0.10, 0.08, 0.0])
        actual = self.chassis.calculate_wheel_velocities(*requested)
        recovered = actual @ self.chassis.wheel_to_vehicle_matrix
        np.testing.assert_allclose(recovered, requested, atol=1e-7)

    def test_pure_axes_and_stop_keep_existing_solution(self):
        for requested in ((0.10, 0.0, 0.0), (0.0, 0.10, 0.0), (0.0, 0.0, 0.5), (0.0, 0.0, 0.0)):
            expected = self.chassis.inverse_kinematics(np.array(requested))
            actual = self.chassis.calculate_wheel_velocities(*requested)
            np.testing.assert_allclose(actual, expected, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
