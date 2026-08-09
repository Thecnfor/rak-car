import unittest

from main.chassis.controllers.base import mecanum_inverse


class TestFourWheelTraction(unittest.TestCase):
    def test_diagonal_translation_has_four_nonzero_wheels(self):
        wheels = mecanum_inverse(0.10, 0.10, 0.0, r=0.30)
        self.assertEqual(len(wheels), 4)
        self.assertGreater(min(abs(v) for v in wheels), 0.01)

    def test_small_or_rotational_command_is_unchanged(self):
        self.assertEqual(
            mecanum_inverse(0.10, 0.0, 0.0, r=0.30),
            [0.10, -0.10, -0.10, 0.10],
        )
        self.assertEqual(
            mecanum_inverse(0.0, 0.0, 0.5, r=0.30),
            [0.15, 0.15, 0.15, 0.15],
        )


if __name__ == "__main__":
    unittest.main()
