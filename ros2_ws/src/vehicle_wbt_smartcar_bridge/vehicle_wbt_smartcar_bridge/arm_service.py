"""Arm service handlers.

Translates the official ArmController API (car_wrap_2026.py
smartcar/whalesbot/vehicle/arm/arm_base.py) into JointTrajectory
messages on /vehicle_wbt/v1/cmd/arm/main/trajectory.

Joint order (must match arm_node.cpp joint_names_):
  horiz_m6     — arm_id rotation about base (rad/servo)
  vert_stepper3 — y-axis vertical stepper
  rotate_s3    — gripper roll
  grip_s7      — gripper open/close

Mapping conventions (matches official):
  arm.set_arm_pose(arm_id, pitch, "LEFT"/"RIGHT", "UP"/"DOWN")
  arm.grasp(state)             — vacuum True=on, False=off
  arm.move_x_position(x)       — horizontal reach
  arm.move_y_position(y)       — vertical reach
  arm.x_get_position()         — current x
  arm.reset_position()         — home pose
  arm.set_arm_angle(angle)     — direct joint angle
  arm.set_hand_angle(angle)    — gripper angle
"""
from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from vehicle_wbt_smartcar_bridge import config as cfg
from vehicle_wbt_smartcar_msgs.msg import ArmPose
from vehicle_wbt_smartcar_msgs.srv import (
    ArmReset, ArmSetPose, ArmGrasp, ArmMoveX, ArmMoveY,
    ArmSetArmAngle, ArmSetHandAngle,
)


class ArmService:
    """All arm-related services for the smartcar bridge."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.traj_pub = node.create_publisher(
            JointTrajectory, cfg.ARM_TRAJECTORY_CMD, 10)
        self.state_sub = node.create_subscription(
            JointState, cfg.STATE_ACTUATORS, self._on_state, 10)
        self.pose_pub = node.create_publisher(
            ArmPose, cfg.STATE_ARM_POSE, 10)

        self._latest_state: Optional[JointState] = None
        # Local cache of arm pose (since JointState doesn't carry dir/grasping)
        self._arm_dir = 'LEFT'
        self._hand_dir = 'UP'
        self._grasping = False
        self._is_homed = False
        # Track desired joint positions for trajectory points
        self._x = 0.0
        self._y = 0.0
        self._arm_id = 0
        self._pitch = 0.0

        # Services
        node.create_service(ArmReset, cfg.SRV_ARM_PREFIX + 'reset_position', self._reset)
        node.create_service(ArmSetPose, cfg.SRV_ARM_PREFIX + 'set_pose', self._set_pose)
        node.create_service(ArmGrasp, cfg.SRV_ARM_PREFIX + 'grasp', self._grasp)
        node.create_service(ArmMoveX, cfg.SRV_ARM_PREFIX + 'move_x', self._move_x)
        node.create_service(ArmMoveY, cfg.SRV_ARM_PREFIX + 'move_y', self._move_y)
        node.create_service(
            ArmSetArmAngle, cfg.SRV_ARM_PREFIX + 'set_arm_angle', self._set_arm_angle)
        node.create_service(
            ArmSetHandAngle, cfg.SRV_ARM_PREFIX + 'set_hand_angle', self._set_hand_angle)

        node.get_logger().info(f'ArmService ready: {cfg.ARM_TRAJECTORY_CMD}')

    # ------------------------------------------------------------------
    # State tracking
    # ------------------------------------------------------------------

    def _on_state(self, msg: JointState) -> None:
        self._latest_state = msg
        # Mirror to ArmPose
        pose = ArmPose()
        pose.arm_id = int(self._arm_id)
        pose.x = self._x
        pose.y = self._y
        pose.pitch = self._pitch
        pose.arm_dir = self._arm_dir
        pose.hand_dir = self._hand_dir
        pose.grasping = self._grasping
        pose.is_homed = self._is_homed
        self.pose_pub.publish(pose)

    def _publish_traj(self, out_time: float = 2.0) -> None:
        """Send one JointTrajectory point with current desired positions."""
        traj = JointTrajectory()
        traj.joint_names = list(cfg.ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        # We send the same 4-tuple for all joints as a simplified contract:
        #   horiz_m6     -> arm_id (mapped -180..180 -> -pi..pi)
        #   vert_stepper3 -> y
        #   rotate_s3    -> pitch
        #   grip_s7      -> 0.05 if grasping else -0.05 (binary open/close)
        import math
        point.positions = [
            math.radians(self._arm_id),
            self._y,
            math.radians(self._pitch),
            0.05 if self._grasping else -0.05,
        ]
        point.time_from_start.sec = int(out_time)
        point.time_from_start.nanosec = int((out_time - int(out_time)) * 1e9)
        traj.points = [point]
        self.traj_pub.publish(traj)

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _reset(self, req: ArmReset.Request, resp: ArmReset.Response) -> ArmReset.Response:
        self._arm_id = 0
        self._pitch = 0.0
        self._x = 0.0
        self._y = 0.15
        self._arm_dir = 'LEFT'
        self._hand_dir = 'UP'
        self._grasping = False
        self._is_homed = True
        self._publish_traj(out_time=3.0)
        resp.success = True
        resp.message = 'arm reset to home pose'
        return resp

    def _set_pose(self, req: ArmSetPose.Request, resp: ArmSetPose.Response) -> ArmSetPose.Response:
        if req.arm_dir and req.arm_dir in ('LEFT', 'RIGHT'):
            self._arm_dir = req.arm_dir
        if req.hand_dir and req.hand_dir in ('UP', 'DOWN'):
            self._hand_dir = req.hand_dir
        self._arm_id = req.arm_id
        self._pitch = req.pitch
        self._publish_traj(out_time=2.0)
        resp.success = True
        resp.message = f'set_pose arm_id={req.arm_id} pitch={req.pitch}'
        return resp

    def _grasp(self, req: ArmGrasp.Request, resp: ArmGrasp.Response) -> ArmGrasp.Response:
        self._grasping = req.state
        self._publish_traj(out_time=0.5)
        resp.success = True
        resp.message = f'grasp={req.state}'
        return resp

    def _move_x(self, req: ArmMoveX.Request, resp: ArmMoveX.Response) -> ArmMoveX.Response:
        self._x = req.x
        out = req.out_time if req.out_time > 0 else 2.0
        self._publish_traj(out_time=out)
        resp.success = True
        resp.message = f'move_x={req.x}'
        return resp

    def _move_y(self, req: ArmMoveY.Request, resp: ArmMoveY.Response) -> ArmMoveY.Response:
        self._y = req.y
        out = req.out_time if req.out_time > 0 else 2.0
        self._publish_traj(out_time=out)
        resp.success = True
        resp.message = f'move_y={req.y}'
        return resp

    def _set_arm_angle(
        self, req: ArmSetArmAngle.Request, resp: ArmSetArmAngle.Response
    ) -> ArmSetArmAngle.Response:
        self._arm_id = req.angle
        self._publish_traj(out_time=2.0)
        resp.success = True
        resp.message = f'arm_angle={req.angle}'
        return resp

    def _set_hand_angle(
        self, req: ArmSetHandAngle.Request, resp: ArmSetHandAngle.Response
    ) -> ArmSetHandAngle.Response:
        # Heuristic: angle 10..70 = DOWN (open), -90..-10 = UP (closed).
        self._hand_dir = 'DOWN' if req.angle >= 0 else 'UP'
        self._publish_traj(out_time=1.0)
        resp.success = True
        resp.message = f'hand_angle={req.angle}'
        return resp