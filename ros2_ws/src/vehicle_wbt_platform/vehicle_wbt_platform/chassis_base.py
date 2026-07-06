"""BaseChassis — chassis-agnostic abstract class.

Spec ref: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §Chassis 抽象

5 concrete subclasses planned for v1:
- MecanumChassis  (current car — 4 mecanum wheels, O layout)
- Diff2Chassis    (2-wheel differential)
- Diff4Chassis    (4-wheel differential, no sideways)
- TricycleChassis (2 drive + 1 steer)
- QuadricycleChassis (4 drive + 4 steer)

Only BaseChassis is implemented in Phase 1. Subclasses land in Phase 2.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple


class ChassisError(RuntimeError):
    """Raised when a chassis operation cannot be performed (e.g., bad kinematics input)."""


@dataclass(frozen=True)
class Pose2D:
    """2D pose in the odom frame — x/y in meters, theta in radians."""

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class WheelSpeeds:
    """Per-wheel angular velocity. Length == chassis.num_wheels.

    Order is chassis-specific:
    - Mecanum 4-wheel: (front_left, front_right, rear_left, rear_right)
    - Diff2: (left, right)
    - Diff4: (front_left, front_right, rear_left, rear_right)
    - Tricycle: (left, right, steer)
    - Quadricycle: (fl, fr, rl, rr)
    """

    values: Tuple[float, ...]

    def __post_init__(self) -> None:
        if not self.values or len(self.values) < 1:
            raise ChassisError(f"WheelSpeeds must have at least 1 wheel, got {len(self.values)}")


class BaseChassis(ABC):
    """Abstract base class for any chassis topology.

    Subclasses MUST:
    - implement the 5 abstract methods below
    - set num_wheels property
    - call super().__init__(chassis_id=...) in their __init__

    The base class is intentionally minimal — no kinematics assumptions baked in.
    """

    def __init__(self, *, chassis_id: str) -> None:
        if not chassis_id or not isinstance(chassis_id, str):
            raise ChassisError(f"chassis_id must be non-empty string, got {chassis_id!r}")
        self._chassis_id = chassis_id
        self._pose = Pose2D(0.0, 0.0, 0.0)

    @property
    def chassis_id(self) -> str:
        return self._chassis_id

    @property
    def pose(self) -> Pose2D:
        return self._pose

    @property
    @abstractmethod
    def num_wheels(self) -> int:
        """Number of driven wheels (excludes passive castors)."""

    @abstractmethod
    def set_velocity(self, vx: float, vy: float, omega: float) -> None:
        """Command body-frame velocity. m/s for vx/vy, rad/s for omega."""

    @abstractmethod
    def get_pose(self) -> Pose2D:
        """Return current odometry pose."""

    @abstractmethod
    def reset_odometry(self) -> None:
        """Reset pose to (0, 0, 0). Use after physical relocation."""

    @abstractmethod
    def forward_kinematics(self, wheel_speeds: WheelSpeeds) -> Tuple[float, float, float]:
        """Convert per-wheel speeds to body-frame (vx, vy, omega)."""

    @abstractmethod
    def inverse_kinematics(self, vx: float, vy: float, omega: float) -> WheelSpeeds:
        """Convert body-frame velocity to per-wheel speeds."""
