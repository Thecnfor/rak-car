"""State publishers: re-publish existing /state/odom and /state/actuators
under the smartcar-bridge namespace, in a flatter shape suited to
task-level SDK consumers on the dev box.
"""
from __future__ import annotations

from typing import Optional

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from vehicle_wbt_smartcar_bridge import config as cfg
from vehicle_wbt_smartcar_msgs.msg import ChassisState, BridgeStatus


class StatePublisher:
    """Republish odom + actuator state in smartcar-bridge shape."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.chassis_pub = node.create_publisher(ChassisState, cfg.STATE_CHASSIS_ODOMETRY, 10)
        self.status_pub = node.create_publisher(BridgeStatus, cfg.STATE_BRIDGE_STATUS, 1)

        self._latest_odom: Optional[Odometry] = None
        self._latest_actuator: Optional[JointState] = None

        node.create_subscription(
            Odometry, cfg.STATE_ODOM, self._on_odom, 10)
        node.create_subscription(
            JointState, cfg.STATE_ACTUATORS, self._on_actuator, 10)

        # Status timer at 1 Hz
        node.create_timer(1.0, self._publish_status)
        # Chassis republisher at 50 Hz (matches mecanum_chassis_node rate)
        node.create_timer(1.0 / cfg.DEFAULT_CHASSIS_RATE_HZ, self._publish_chassis)

        node.get_logger().info(
            f'StatePublisher ready (chassis -> {cfg.STATE_CHASSIS_ODOMETRY} '
            f'@ {cfg.DEFAULT_CHASSIS_RATE_HZ:.0f} Hz, '
            f'status -> {cfg.STATE_BRIDGE_STATUS} @ 1 Hz)')

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_actuator(self, msg: JointState) -> None:
        self._latest_actuator = msg

    def _publish_chassis(self) -> None:
        if self._latest_odom is None:
            return
        o = self._latest_odom
        msg = ChassisState()
        msg.x = o.pose.pose.position.x
        msg.y = o.pose.pose.position.y
        msg.z = self._yaw_from_quat(o.pose.pose.orientation)
        msg.vx = o.twist.twist.linear.x
        msg.vy = o.twist.twist.linear.y
        msg.wz = o.twist.twist.angular.z
        self.chassis_pub.publish(msg)

    def _publish_status(self) -> None:
        msg = BridgeStatus()
        msg.component = 'smartcar_bridge_node'
        msg.state = 'ALIVE'
        msg.detail = (
            f'odom={"yes" if self._latest_odom else "no"} '
            f'actuator={"yes" if self._latest_actuator else "no"}'
        )
        self.status_pub.publish(msg)

    @staticmethod
    def _yaw_from_quat(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z)
        cosy_cosp = 1.0 - 2.0 * (q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)