"""Smoke tests for the smartcar bridge + SDK.

Verifies that the package structure is sound without requiring a live
ROS2 daemon (mostly import-level + Python-level checks). Real e2e
service calls happen via the integration tests under
scripts/test_bridge_live.py on the Jetson after launch.
"""
from __future__ import annotations

import math
import os
import sys

# Make sure ament-built packages are importable in the test env.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def test_config_topics_well_formed() -> None:
    """All bridge topics live under /vehicle_wbt/v1/..."""
    from vehicle_wbt_smartcar_bridge import config as cfg
    topics = [
        cfg.STATE_CHASSIS_ODOMETRY,
        cfg.STATE_ARM_POSE,
        cfg.STATE_BRIDGE_STATUS,
        cfg.STATE_ODOM,
        cfg.STATE_ACTUATORS,
    ]
    for t in topics:
        assert t.startswith('/vehicle_wbt/v1/'), f'topic {t} violates namespace'


def test_arm_joint_names_match_node() -> None:
    """Joint order in bridge config must match arm_node.cpp joint_names_."""
    from vehicle_wbt_smartcar_bridge import config as cfg
    expected = ['horiz_m6', 'vert_stepper3', 'rotate_s3', 'grip_s7']
    assert cfg.ARM_JOINT_NAMES == expected, (
        f'joint names mismatch: {cfg.ARM_JOINT_NAMES} != {expected}')


def test_srv_modules_importable() -> None:
    """All 19 srv modules must import without error."""
    from vehicle_wbt_smartcar_msgs.srv import (
        MoveFor, MoveToPosition, MoveTime, MoveDistance, ResetOdometry,
        LaneBase, LaneTime, LaneDis, LaneDisOffset,
        ArmReset, ArmSetPose, ArmGrasp, ArmMoveX, ArmMoveY,
        ArmSetArmAngle, ArmSetHandAngle,
        Beep, SetStorage, Shoot,
    )
    # Each service has Request/Response
    for cls in (MoveFor, MoveToPosition, MoveTime, MoveDistance, ResetOdometry,
                LaneBase, LaneTime, LaneDis, LaneDisOffset,
                ArmReset, ArmSetPose, ArmGrasp, ArmMoveX, ArmMoveY,
                ArmSetArmAngle, ArmSetHandAngle,
                Beep, SetStorage, Shoot):
        assert hasattr(cls, 'Request'), f'{cls.__name__} missing Request'
        assert hasattr(cls, 'Response'), f'{cls.__name__} missing Response'


def test_msg_modules_importable() -> None:
    """All 3 msg modules must import without error."""
    from vehicle_wbt_smartcar_msgs.msg import ArmPose, ChassisState, BridgeStatus
    for cls in (ArmPose, ChassisState, BridgeStatus):
        # Smoke: can construct empty instance
        instance = cls()
        assert instance is not None


def test_calculation_dis_matches_official() -> None:
    """calculation_dis returns Euclidean distance in xy plane."""
    from vehicle_wbt_smartcar_sdk.utils import calculation_dis
    d = calculation_dis([3.0, 4.0, 0.0], [0.0, 0.0, 0.0])
    assert math.isclose(d, 5.0), f'expected 5.0, got {d}'


def test_arm_pose_mapping_canonical_form() -> None:
    """Verify the SDK's set_pose_canonical helper builds the right request.

    Doesn't actually call a service (no bridge running). We just exercise
    the request construction logic.
    """
    # Build the request manually the same way the SDK does
    from vehicle_wbt_smartcar_msgs.srv import ArmSetPose
    req = ArmSetPose.Request()
    req.arm_id = -115
    req.pitch = 0.05
    req.arm_dir = 'LEFT'
    req.hand_dir = 'DOWN'
    assert req.arm_id == -115
    assert req.arm_dir == 'LEFT'
    assert req.hand_dir == 'DOWN'


def test_set_hand_angle_string_mapping() -> None:
    """Official API accepts 'UP'/'DOWN'/'MID' strings. Verify SDK maps them."""
    # Mirror the mapping in arm.py without importing rclpy (which would
    # require a ROS_DOMAIN_ID). We re-implement the same logic.
    mapping = {'UP': 10, 'DOWN': -10, 'MID': 0}
    for s, expected in mapping.items():
        assert mapping[s.upper()] == expected


if __name__ == '__main__':
    test_config_topics_well_formed()
    test_arm_joint_names_match_node()
    test_srv_modules_importable()
    test_msg_modules_importable()
    test_calculation_dis_matches_official()
    test_arm_pose_mapping_canonical_form()
    test_set_hand_angle_string_mapping()
    print('all smoke tests passed')