"""Chassis service handlers.

Translates the official MyCar motion API (move_for, move_to_position,
move_time, move_distance, lane_*, reset_position) into Twist commands
on /vehicle_wbt/v1/cmd/vel_safe, using the chassis odometry feedback
loop to determine completion.

This module does NOT run any perception. Lane-following here is a thin
wrapper that publishes a constant forward velocity — the actual PID
loop runs on the dev box (it has the GPU + camera image stream). The
bridge stops the chassis when (a) the requested distance/duration has
elapsed, or (b) the dev box publishes a stop signal via the
StopChassis service (not yet implemented).
"""
from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from vehicle_wbt_smartcar_bridge import config as cfg
from vehicle_wbt_smartcar_msgs.srv import (
    MoveFor, MoveToPosition, MoveTime, MoveDistance, ResetOdometry,
    LaneBase, LaneTime, LaneDis, LaneDisOffset,
)


class ChassisService:
    """All chassis-related services for the smartcar bridge."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, cfg.CMD_VEL_SAFE, 10)
        self.odom_sub = node.create_subscription(
            Odometry, cfg.STATE_ODOM, self._on_odom, 10)
        self._latest_odom: Optional[Odometry] = None

        # Services
        node.create_service(MoveFor, cfg.SRV_CHASSIS_PREFIX + 'move_for', self._move_for)
        node.create_service(
            MoveToPosition, cfg.SRV_CHASSIS_PREFIX + 'move_to_position',
            self._move_to_position)
        node.create_service(MoveTime, cfg.SRV_CHASSIS_PREFIX + 'move_time', self._move_time)
        node.create_service(
            MoveDistance, cfg.SRV_CHASSIS_PREFIX + 'move_distance', self._move_distance)
        node.create_service(
            ResetOdometry, cfg.SRV_CHASSIS_PREFIX + 'reset_odometry', self._reset_odometry)
        node.create_service(LaneBase, cfg.SRV_CHASSIS_PREFIX + 'lane_base', self._lane_base)
        node.create_service(LaneTime, cfg.SRV_CHASSIS_PREFIX + 'lane_time', self._lane_time)
        node.create_service(LaneDis, cfg.SRV_CHASSIS_PREFIX + 'lane_dis', self._lane_dis)
        node.create_service(
            LaneDisOffset, cfg.SRV_CHASSIS_PREFIX + 'lane_dis_offset', self._lane_dis_offset)

        node.get_logger().info(f'ChassisService ready: {cfg.CMD_VEL_SAFE}')

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_twist(self, vx: float, vy: float, wz: float) -> None:
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.angular.z = wz
        self.cmd_pub.publish(t)

    def _stop(self) -> None:
        self._publish_twist(0.0, 0.0, 0.0)

    def _wait_odom(self, timeout: float = 1.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and self._latest_odom is None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return self._latest_odom is not None

    def _odom_pose(self) -> tuple[float, float, float]:
        if self._latest_odom is None:
            return (0.0, 0.0, 0.0)
        p = self._latest_odom.pose.pose
        # theta from quaternion (yaw only)
        siny_cosp = 2.0 * (p.orientation.w * p.orientation.z)
        cosy_cosp = 1.0 - 2.0 * (p.orientation.z * p.orientation.z)
        return (p.position.x, p.position.y, math.atan2(siny_cosp, cosy_cosp))

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _move_for(self, req: MoveFor.Request, resp: MoveFor.Response) -> MoveFor.Response:
        """move_for([dx, dy, dz]) — relative motion in body frame."""
        if not self._wait_odom():
            resp.success = False
            resp.message = 'No odometry received yet'
            return resp

        x0, y0, z0 = self._odom_pose()
        # Body-frame relative motion: convert to odom-frame via current heading
        cos_t = math.cos(z0)
        sin_t = math.sin(z0)
        target_x = x0 + req.dx * cos_t - req.dy * sin_t
        target_y = y0 + req.dx * sin_t + req.dy * cos_t
        target_z = z0 + req.dz

        return self._drive_to(target_x, target_y, target_z)

    def _move_to_position(
        self, req: MoveToPosition.Request, resp: MoveToPosition.Response
    ) -> MoveToPosition.Response:
        """move_to_position(coords) — absolute waypoint with location_pid semantics."""
        if not self._wait_odom():
            resp.success = False
            resp.message = 'No odometry received yet'
            return resp
        return self._drive_to(req.x, req.y, req.z)

    def _move_time(self, req: MoveTime.Request, resp: MoveTime.Response) -> MoveTime.Response:
        """move_time(speed, duration) — open-loop motion for fixed duration."""
        self._publish_twist(req.vx, req.vy, req.wz)
        end = time.time() + req.duration
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self._stop()
        resp.success = True
        resp.message = f'move_time done ({req.duration}s)'
        return resp

    def _move_distance(
        self, req: MoveDistance.Request, resp: MoveDistance.Response
    ) -> MoveDistance.Response:
        """move_distance(speed, direction, distance) — open-loop straight line."""
        if not self._wait_odom():
            resp.success = False
            resp.message = 'No odometry received yet'
            return resp
        sign = -1.0 if req.direction == 1 else 1.0  # 0=forward, 1=backward
        vx = sign * abs(req.speed)
        x0, _, _ = self._odom_pose()
        self._publish_twist(vx, 0.0, 0.0)
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            x1, _, _ = self._odom_pose()
            if abs(x1 - x0) >= req.distance:
                break
        self._stop()
        resp.success = True
        resp.message = f'move_distance done ({req.distance}m)'
        return resp

    def _reset_odometry(
        self, req: ResetOdometry.Request, resp: ResetOdometry.Response
    ) -> ResetOdometry.Response:
        """reset_position() — the C++ chassis node does not expose reset; we
        log + return success. Dev box should subtract (x0,y0,z0) when needed.
        """
        self.node.get_logger().warn(
            'reset_odometry: C++ chassis does not expose reset; '
            'dev box should treat current pose as origin.')
        resp.success = True
        resp.message = 'no-op (see logger)'
        return resp

    # ------------------------------------------------------------------
    # Lane follow — thin wrappers. Real PID loop runs on dev box.
    # ------------------------------------------------------------------

    def _lane_base(self, req: LaneBase.Request, resp: LaneBase.Response) -> LaneBase.Response:
        self._publish_twist(req.speed, 0.0, 0.0)
        resp.success = True
        resp.message = 'lane_base: forward velocity published; dev box must close the loop'
        return resp

    def _lane_time(self, req: LaneTime.Request, resp: LaneTime.Response) -> LaneTime.Response:
        return self._move_time(
            MoveTime.Request(vx=req.speed, vy=0.0, wz=0.0, duration=req.duration),
            MoveTime.Response(),
        )

    def _lane_dis(self, req: LaneDis.Request, resp: LaneDis.Response) -> LaneDis.Response:
        return self._move_distance(
            MoveDistance.Request(speed=req.speed, direction=0, distance=req.distance),
            MoveDistance.Response(),
        )

    def _lane_dis_offset(
        self, req: LaneDisOffset.Request, resp: LaneDisOffset.Response
    ) -> LaneDisOffset.Response:
        """lane_dis_offset — most-used in car_task_function.py. Forward until
        dis_hold meters of x-axis travel has accumulated. Dev box is
        responsible for actually following the lane; the bridge just gates
        forward travel.
        """
        if not self._wait_odom():
            resp.success = False
            resp.message = 'No odometry received yet'
            return resp
        x0, _, _ = self._odom_pose()
        self._publish_twist(req.speed, 0.0, 0.0)
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            x1, _, _ = self._odom_pose()
            if abs(x1 - x0) >= req.dis_hold:
                break
        self._stop()
        resp.success = True
        resp.message = f'lane_dis_offset done ({req.dis_hold}m)'
        return resp

    # ------------------------------------------------------------------
    # Internal PD driver
    # ------------------------------------------------------------------

    def _drive_to(self, tx: float, ty: float, tz: float) -> MoveToPosition.Response:
        """Drive to (tx, ty, tz) using a simple position PD. Used by
        move_for and move_to_position.
        """
        resp = MoveToPosition.Response()
        speed = cfg.DEFAULT_LINEAR_SPEED
        ang_speed = cfg.DEFAULT_ANGULAR_SPEED

        # First rotate to face target
        for _ in range(2000):  # ~20s max
            x, y, z = self._odom_pose()
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            if dist < cfg.DEFAULT_POSITION_TOL:
                break
            target_heading = math.atan2(dy, dx)
            heading_err = self._wrap_angle(target_heading - z)
            if abs(heading_err) > cfg.DEFAULT_ANGLE_TOL:
                self._publish_twist(0.0, 0.0, ang_speed * self._sign(heading_err))
                rclpy.spin_once(self.node, timeout_sec=0.02)
                continue
            # Drive forward
            v = min(speed, dist * 2.0)
            self._publish_twist(v, 0.0, 0.0)
            rclpy.spin_once(self.node, timeout_sec=0.02)

        # Then rotate to target heading
        for _ in range(1000):
            _, _, z = self._odom_pose()
            heading_err = self._wrap_angle(tz - z)
            if abs(heading_err) < cfg.DEFAULT_ANGLE_TOL:
                break
            self._publish_twist(0.0, 0.0, ang_speed * self._sign(heading_err))
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self._stop()
        resp.success = True
        resp.message = f'reached ({tx:.3f}, {ty:.3f}, {tz:.3f})'
        return resp

    @staticmethod
    def _sign(x: float) -> float:
        return 1.0 if x > 0 else -1.0

    @staticmethod
    def _wrap_angle(a: float) -> float:
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a