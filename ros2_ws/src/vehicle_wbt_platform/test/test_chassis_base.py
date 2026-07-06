from __future__ import annotations

import math
from typing import Sequence

import pytest

from vehicle_wbt_platform.chassis_base import (
    BaseChassis,
    ChassisError,
    Pose2D,
    WheelSpeeds,
)


def test_base_chassis_cannot_be_instantiated_directly() -> None:
    with pytest.raises(TypeError):
        BaseChassis()  # type: ignore[abstract]


def test_pose2d_construction() -> None:
    p = Pose2D(x=1.0, y=2.0, theta=math.pi / 2)
    assert p.x == 1.0
    assert p.theta == pytest.approx(math.pi / 2)


def test_wheel_speeds_construction() -> None:
    ws = WheelSpeeds(values=(0.1, -0.1, 0.2, -0.2))
    assert len(ws.values) == 4
    assert ws.values[0] == 0.1


class DummyChassis(BaseChassis):
    """4-wheel abstract chassis for testing the contract only."""

    @property
    def num_wheels(self) -> int:
        return 4

    def set_velocity(self, vx: float, vy: float, omega: float) -> None:
        self._last_velocity = (vx, vy, omega)

    def get_pose(self) -> Pose2D:
        return self._pose

    def reset_odometry(self) -> None:
        self._pose = Pose2D(0.0, 0.0, 0.0)

    def forward_kinematics(self, wheel_speeds: WheelSpeeds) -> tuple[float, float, float]:
        return (0.0, 0.0, 0.0)

    def inverse_kinematics(self, vx: float, vy: float, omega: float) -> WheelSpeeds:
        return WheelSpeeds(values=(0.0,) * self.num_wheels)


def test_dummy_chassis_lifecycle() -> None:
    c = DummyChassis(chassis_id="dummy")
    assert c.chassis_id == "dummy"
    assert c.num_wheels == 4
    c.reset_odometry()
    p = c.get_pose()
    assert p.x == 0.0 and p.y == 0.0
    c.set_velocity(0.1, 0.0, 0.0)
    assert c._last_velocity == (0.1, 0.0, 0.0)
    ws = c.inverse_kinematics(0.1, 0.0, 0.0)
    assert len(ws.values) == 4


def test_dummy_chassis_id_required() -> None:
    with pytest.raises((TypeError, ChassisError)):
        DummyChassis()  # type: ignore[call-arg]
