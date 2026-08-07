"""Feed components — periodic sensor/actuator bridges from MC602 to ROS2 topics.

Spec ref: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §组件模型

Each feed is an rclpy Node that:
  1. Opens its hardware via the adapter at init()
  2. Starts a rclpy.Timer at the rate_hz from config
  3. On each tick: reads the MC602 port, publishes to the configured topic

Components are stateless wrt ROS2 — no singleton, no global state. The
orchestrator creates one instance per enabled config entry and drives it
through the BaseComponent lifecycle.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

from vehicle_wbt_platform.component_base import (
    BaseComponent,
    ComponentContext,
    ComponentError,
    HealthState,
    HealthStatus,
)
from vehicle_wbt_platform.mc602_adapter import MC602Adapter

if TYPE_CHECKING:
    # These are only used for static type checking — rclpy is optional at import time
    # so tests can run on dev boxes without ROS2 installed.
    from rclpy.node import Node
    from rclpy.publisher import Publisher
    from rclpy.timer import Timer

_logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# Lazily resolved + cached message types keyed by "pkg/MsgName" string.
_MSG_TYPE_CACHE: dict[str, type] = {}


def _resolve_msg_type(msg_type_str: str) -> type:
    """Import and return a ROS2 message class from a '<pkg>/<Msg>' string."""
    if msg_type_str in _MSG_TYPE_CACHE:
        return _MSG_TYPE_CACHE[msg_type_str]
    pkg, cls_name = msg_type_str.split("/", 1)
    module = __import__(f"{pkg}.msg", fromlist=[cls_name])
    msg_cls = getattr(module, cls_name)
    _MSG_TYPE_CACHE[msg_type_str] = msg_cls
    return msg_cls


# ---------------------------------------------------------------------------
# IRFeed — reads an IR sensor port, publishes Float32 (meters)
# ---------------------------------------------------------------------------

class IRFeed(BaseComponent):
    """Publishes IR distance readings from one MC602 P-port.

    Topic: /vehicle_wbt/v1/sensors/ir/<id>
    Type:   std_msgs/Float32  (meters)
    """

    COMPONENT_TYPE = "ir"

    def __init__(self, *, component_id: str, sensor_cfg: Any) -> None:
        super().__init__(component_id=component_id)
        self._cfg = sensor_cfg
        self._ctx: ComponentContext | None = None
        self._timer: Any = None  # Timer
        self._pub: Any = None    # Publisher

    # --- BaseComponent lifecycle ---

    def init(self, context: ComponentContext) -> None:
        self._ctx = context
        node = context.node
        msg_cls = _resolve_msg_type(self._cfg.msg_type)

        self._pub = node.create_publisher(msg_cls, self._cfg.topic, 10)
        node.get_logger().info(
            "IRFeed[%s] init: topic=%s port=%d rate=%.1f Hz",
            self._component_id, self._cfg.topic, self._cfg.port_id, self._cfg.rate_hz,
        )

    def start(self) -> None:
        if self._ctx is None or self._timer is not None:
            return
        period_s = 1.0 / max(self._cfg.rate_hz, 0.001)
        self._timer = self._ctx.node.create_timer(period_s, self._on_tick)
        self._ctx.node.get_logger().info("IRFeed[%s] started", self._component_id)

    def stop(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def cleanup(self) -> None:
        self.stop()
        self._pub = None
        self._ctx = None

    def health_status(self) -> HealthStatus:
        if self._ctx is None or not self._ctx.controller.is_open:
            return HealthStatus(HealthState.ERROR, {"reason": "adapter not open"})
        return HealthStatus(HealthState.RUNNING)

    # --- Internal ---

    def _on_tick(self) -> None:
        if self._ctx is None:
            return
        try:
            value = self._ctx.controller.read_sensor(
                port_id=self._cfg.port_id, sensor_type="ir",
            )
            if self._pub is not None:
                import std_msgs.msg as std_msgs_mod
                msg = std_msgs_mod.Float32()
                msg.data = float(value)
                self._pub.publish(msg)
        except Exception as e:
            self._ctx.node.get_logger().warning(
                "IRFeed[%s] read failed: %s", self._component_id, e
            )


# ---------------------------------------------------------------------------
# ChassisFeed — publishes Odometry from encoder readings
# ---------------------------------------------------------------------------

class ChassisFeed(BaseComponent):
    """Publishes chassis odometry from 4-wheel encoder readings.

    Topic: /vehicle_wbt/v1/state/odom
    TF:     /tf  (odom → base_link)
    """

    COMPONENT_TYPE = "chassis"

    def __init__(self, *, component_id: str = "mecanum_chassis") -> None:
        super().__init__(component_id=component_id)
        self._ctx: ComponentContext | None = None
        self._timer: Any = None
        self._odom_pub: Any = None
        self._tf_broadcaster: Any = None
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_counts: list[int] = [0, 0, 0, 0]

    def init(self, context: ComponentContext) -> None:
        self._ctx = context
        node = context.node

        nav_msgs_mod = __import__("nav_msgs.msg", fromlist=["Odometry"])
        self._odom_pub = node.create_publisher(
            nav_msgs_mod.Odometry, "/vehicle_wbt/v1/state/odom", 10
        )
        tf2_ros_mod = __import__("tf2_ros", fromlist=["TransformBroadcaster"])
        self._tf_broadcaster = tf2_ros_mod.TransformBroadcaster(node)
        node.get_logger().info("ChassisFeed init: publishing odom + tf")

    def start(self) -> None:
        if self._ctx is None or self._timer is not None:
            return
        period_s = 1.0 / 50.0  # Chassis at 50 Hz per spec
        self._timer = self._ctx.node.create_timer(period_s, self._on_tick)
        self._ctx.node.get_logger().info("ChassisFeed started @ 50 Hz")

    def stop(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def cleanup(self) -> None:
        self.stop()
        self._odom_pub = None
        self._tf_broadcaster = None
        self._ctx = None

    def health_status(self) -> HealthStatus:
        if self._ctx is None or not self._ctx.controller.is_open:
            return HealthStatus(HealthState.ERROR, {"reason": "adapter not open"})
        return HealthStatus(HealthState.RUNNING)

    # --- Internal ---

    def _on_tick(self) -> None:
        if self._ctx is None:
            return
        try:
            adapter: MC602Adapter = self._ctx.controller
            counts = adapter.read_encoder4()  # [FL(M2), FR(M1), RL(M3), RR(M4)]

            # Wheel radius from config (default 30 mm)
            cfg_any = getattr(self._ctx.registry, "actuators", {})
            wheel_radius = getattr(cfg_any, "type_specific", {}).get("wheel_radius", 0.03)

            # Delta counts → delta meters per wheel
            dt = 1.0 / 50.0
            d_counts = [c - l for c, l in zip(counts, self._last_counts)]
            d_meters = [MC602Adapter.ENCODER_2_RAD * wheel_radius * dc for dc in d_counts]
            self._last_counts = [int(c) for c in counts]

            # Mecanum forward kinematics
            Lx = 0.15  # half-track width
            Ly = 0.10  # half-wheel-base
            d_FR, d_FL, d_RL, d_RR = d_meters[1], d_meters[0], d_meters[2], d_meters[3]
            vx = (d_FR + d_FL + d_RL + d_RR) / (4.0 * dt)
            vy = (-d_FR + d_FL + d_RL - d_RR) / (4.0 * dt)
            vtheta = (-d_FR + d_FL - d_RL + d_RR) / (4.0 * (Lx + Ly) * dt)

            # Integrate position
            self._x += vx * dt
            self._y += vy * dt
            self._theta += vtheta * dt

            # Build Odometry message
            nav_msgs_mod = __import__("nav_msgs.msg", fromlist=["Odometry"])
            geometry_msgs_mod = __import__("geometry_msgs.msg", fromlist=["Quaternion"])
            std_msgs_mod = __import__("std_msgs.msg", fromlist=["Header"])

            import math
            odom = nav_msgs_mod.Odometry()
            odom.header = std_msgs_mod.Header()
            odom.header.stamp = self._ctx.node.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self._x
            odom.pose.pose.position.y = self._y
            odom.pose.pose.position.z = 0.0
            qz = math.sin(self._theta / 2.0)
            qw = math.cos(self._theta / 2.0)
            odom.pose.pose.orientation = geometry_msgs_mod.Quaternion(x=0.0, y=0.0, z=qz, w=qw)
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = vtheta

            if self._odom_pub is not None:
                self._odom_pub.publish(odom)

            # Publish TF (odom → base_link)
            if self._tf_broadcaster is not None:
                from geometry_msgs.msg import TransformStamped
                t = TransformStamped()
                t.header = odom.header
                t.child_frame_id = "base_link"
                t.transform.translation.x = self._x
                t.transform.translation.y = self._y
                t.transform.translation.z = 0.0
                t.transform.rotation = odom.pose.pose.orientation
                self._tf_broadcaster.sendTransform(t)

        except Exception as e:
            if self._ctx is not None:
                self._ctx.node.get_logger().warning("ChassisFeed tick failed: %s", e)


# ---------------------------------------------------------------------------
# ArmFeed — publishes JointState from servo bus readings
# ---------------------------------------------------------------------------

class ArmFeed(BaseComponent):
    """Publishes arm joint state from MC602 servo bus.

    Topic: /vehicle_wbt/v1/state/actuators/<id>
    Type:   sensor_msgs/JointState
    """

    COMPONENT_TYPE = "arm"

    def __init__(self, *, component_id: str, actuator_cfg: Any) -> None:
        super().__init__(component_id=component_id)
        self._cfg = actuator_cfg
        self._ctx: ComponentContext | None = None
        self._timer: Any = None
        self._pub: Any = None
        # Default joint names — overridden by init() from config type_specific
        self._joint_names: list[str] = ["joint1", "joint2", "joint3", "joint4"]

    def init(self, context: ComponentContext) -> None:
        self._ctx = context
        node = context.node

        sensor_msgs_mod = __import__("sensor_msgs.msg", fromlist=["JointState"])
        self._pub = node.create_publisher(
            sensor_msgs_mod.JointState, self._cfg.topic, 10,
        )
        ts = self._cfg.type_specific or {}
        self._joint_names = ts.get("joint_names", ["joint1", "joint2", "joint3", "joint4"])
        node.get_logger().info(
            "ArmFeed[%s] init: topic=%s joints=%s",
            self._component_id, self._cfg.topic, self._joint_names,
        )

    def start(self) -> None:
        if self._ctx is None or self._timer is not None:
            return
        period_s = 1.0 / max(self._cfg.rate_hz, 0.001)
        self._timer = self._ctx.node.create_timer(period_s, self._on_tick)
        self._ctx.node.get_logger().info("ArmFeed[%s] started", self._component_id)

    def stop(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def cleanup(self) -> None:
        self.stop()
        self._pub = None
        self._ctx = None

    def health_status(self) -> HealthStatus:
        if self._ctx is None or not self._ctx.controller.is_open:
            return HealthStatus(HealthState.ERROR, {"reason": "adapter not open"})
        return HealthStatus(HealthState.RUNNING)

    # --- Internal ---

    def _on_tick(self) -> None:
        if self._ctx is None:
            return
        try:
            adapter: MC602Adapter = self._ctx.controller
            positions = []
            for i in range(len(self._joint_names)):
                try:
                    angle = adapter.read_sensor(port_id=i + 1, sensor_type="analog_input")
                    positions.append(float(angle))
                except Exception:
                    positions.append(0.0)

            if self._pub is not None:
                sensor_msgs_mod = __import__("sensor_msgs.msg", fromlist=["JointState"])
                msg = sensor_msgs_mod.JointState()
                msg.header.stamp = self._ctx.node.get_clock().now().to_msg()
                msg.name = self._joint_names
                msg.position = positions
                msg.velocity = [0.0] * len(positions)
                self._pub.publish(msg)
        except Exception as e:
            self._ctx.node.get_logger().warning(
                "ArmFeed[%s] tick failed: %s", self._component_id, e
            )


# ---------------------------------------------------------------------------
# CmdVelWriteFeed — subscribes to cmd_vel_safe, writes motor speeds
# ---------------------------------------------------------------------------

class CmdVelWriteFeed(BaseComponent):
    """Subscribes to velocity commands and writes motor speeds to MC602.

    Topic: /vehicle_wbt/v1/cmd/vel_safe
    Type:   geometry_msgs/Twist

    Uses mecanum mixing: each wheel gets a share of vx/vy/vtheta.
    Motor ports: FL=M2(port 2), FR=M1(port 1), RL=M3(port 3), RR=M4(port 4)
    """

    COMPONENT_TYPE = "cmd_vel_write"

    def __init__(self, *, component_id: str = "cmd_vel_write") -> None:
        super().__init__(component_id=component_id)
        self._ctx: ComponentContext | None = None
        self._subscription: Any = None

    def init(self, context: ComponentContext) -> None:
        self._ctx = context
        node = context.node

        geometry_msgs_mod = __import__("geometry_msgs.msg", fromlist=["Twist"])
        self._subscription = node.create_subscription(
            geometry_msgs_mod.Twist,
            "/vehicle_wbt/v1/cmd/vel_safe",
            self._on_cmd,
            10,
        )
        node.get_logger().info("CmdVelWriteFeed init: subscribed to /vehicle_wbt/v1/cmd/vel_safe")

    def start(self) -> None:
        if self._ctx is None:
            return
        self._ctx.node.get_logger().info("CmdVelWriteFeed started")

    def stop(self) -> None:
        self._subscription = None

    def cleanup(self) -> None:
        self.stop()
        self._ctx = None

    def health_status(self) -> HealthStatus:
        if self._ctx is None or not self._ctx.controller.is_open:
            return HealthStatus(HealthState.ERROR, {"reason": "adapter not open"})
        return HealthStatus(HealthState.RUNNING)

    # --- Internal ---

    def _on_cmd(self, msg: Any) -> None:
        if self._ctx is None:
            return
        try:
            adapter: MC602Adapter = self._ctx.controller
            # Mecanum wheel mixing: each wheel speed = vx +/- vy +/- vtheta * track
            vx = msg.linear.x
            vy = msg.linear.y
            wz = msg.angular.z

            Lx = 0.15  # half-track
            Ly = 0.10  # half-wheel-base

            # FL(M2), FR(M1), RL(M3), RR(M4)
            fl = vx - vy - wz * (Lx + Ly)
            fr = vx + vy + wz * (Lx + Ly)
            rl = vx + vy - wz * (Lx + Ly)
            rr = vx - vy + wz * (Lx + Ly)

            wheel_speeds = [fr, fl, rl, rr]
            motor_ports = [1, 2, 3, 4]

            for port, speed in zip(motor_ports, wheel_speeds):
                try:
                    adapter.write_actuator(port_id=port, actuator_type="motor", value=float(speed))
                except Exception as e:
                    if self._ctx is not None:
                        self._ctx.node.get_logger().warning(
                            "CmdVelWriteFeed motor port %d write failed: %s", port, e
                        )

        except Exception as e:
            if self._ctx is not None:
                self._ctx.node.get_logger().warning("CmdVelWriteFeed callback error: %s", e)


# ---------------------------------------------------------------------------
# CmdArmWriteFeed — subscribes to arm trajectory, writes servo/stepper
# ---------------------------------------------------------------------------

class CmdArmWriteFeed(BaseComponent):
    """Subscribes to arm joint trajectory commands and writes to MC602.

    Topic: /vehicle_wbt/v1/cmd/arm/<id>/trajectory
    Type:   trajectory_msgs/JointTrajectory

    Maps trajectory positions to MC602 actuators:
      joint[0] → M6 horizontal motor (port 6, motor)
      joint[1] → Stepper3 vertical (port 3, stepper, angle_deg)
      joint[2] → S3 rotation servo (port 3, servo_bus, angle_deg)
      joint[3] → S7 grip servo (port 7, servo_pwm, angle_deg)
    """

    COMPONENT_TYPE = "cmd_arm_write"

    def __init__(self, *, component_id: str, arm_cfg: Any) -> None:
        super().__init__(component_id=component_id)
        self._cfg = arm_cfg
        self._ctx: ComponentContext | None = None
        self._subscription: Any = None

    def init(self, context: ComponentContext) -> None:
        self._ctx = context
        node = context.node

        topic = self._cfg.get("topic", f"/vehicle_wbt/v1/cmd/arm/main/trajectory")
        trajectory_msgs_mod = __import__("trajectory_msgs.msg", fromlist=["JointTrajectory"])
        self._subscription = node.create_subscription(
            trajectory_msgs_mod.JointTrajectory,
            topic,
            self._on_trajectory,
            10,
        )
        node.get_logger().info("CmdArmWriteFeed[%s] init: subscribed to %s", self._component_id, topic)

    def start(self) -> None:
        if self._ctx is None:
            return
        self._ctx.node.get_logger().info("CmdArmWriteFeed[%s] started", self._component_id)

    def stop(self) -> None:
        self._subscription = None

    def cleanup(self) -> None:
        self.stop()
        self._ctx = None

    def health_status(self) -> HealthStatus:
        if self._ctx is None or not self._ctx.controller.is_open:
            return HealthStatus(HealthState.ERROR, {"reason": "adapter not open"})
        return HealthStatus(HealthState.RUNNING)

    # --- Internal ---

    def _on_trajectory(self, msg: Any) -> None:
        if self._ctx is None:
            return
        try:
            adapter: MC602Adapter = self._ctx.controller
            positions = list(msg.positions) if msg.positions else []

            if not positions:
                return

            # Expand positions to 4 joints if trajectory has fewer
            while len(positions) < 4:
                positions.append(0.0)

            # Joint 0: M6 horizontal motor (port 6, motor type)
            try:
                horiz_speed = float(max(-1.0, min(1.0, positions[0])))
                adapter.write_actuator(port_id=6, actuator_type="motor", value=horiz_speed)
            except Exception as e:
                if self._ctx is not None:
                    self._ctx.node.get_logger().warning("CmdArmWriteFeed M6 write failed: %s", e)

            # Joint 1: Stepper3 vertical (port 3, stepper, angle_deg)
            try:
                vert_pos = max(0.0, min(0.3, float(positions[1])))
                vert_steps = vert_pos * 2000.0  # steps_per_meter approximation
                vert_angle_deg = vert_steps * 0.005 * 180.0 / math.pi
                adapter.write_actuator(port_id=3, actuator_type="stepper", value=vert_angle_deg)
            except Exception as e:
                if self._ctx is not None:
                    self._ctx.node.get_logger().warning("CmdArmWriteFeed stepper write failed: %s", e)

            # Joint 2: S3 rotation servo (port 3, servo_bus)
            try:
                side_idx = int(round(max(-1.0, min(1.0, positions[2]))))
                s3_map = [-93.0, 0.0, 93.0]
                s3_angle = s3_map[side_idx + 1] if -1 <= side_idx <= 1 else 0.0
                adapter.write_actuator(port_id=3, actuator_type="servo_bus", value=s3_angle)
            except Exception as e:
                if self._ctx is not None:
                    self._ctx.node.get_logger().warning("CmdArmWriteFeed S3 write failed: %s", e)

            # Joint 3: S7 grip servo (port 7, servo_pwm)
            try:
                grip_idx = int(round(max(-1.0, min(1.0, positions[3]))))
                s7_map = [-45.0, 0.0, 46.0]
                s7_angle = s7_map[grip_idx + 1] if -1 <= grip_idx <= 1 else 0.0
                adapter.write_actuator(port_id=7, actuator_type="servo_pwm", value=s7_angle)
            except Exception as e:
                if self._ctx is not None:
                    self._ctx.node.get_logger().warning("CmdArmWriteFeed S7 write failed: %s", e)

        except Exception as e:
            if self._ctx is not None:
                self._ctx.node.get_logger().warning("CmdArmWriteFeed callback error: %s", e)


# ---------------------------------------------------------------------------
# Component registry — maps config type strings to feed/write classes
# ---------------------------------------------------------------------------

COMPONENT_CLASSES: dict[str, type[BaseComponent]] = {
    "ir": IRFeed,
    "chassis": ChassisFeed,
    "arm": ArmFeed,
    "cmd_vel_write": CmdVelWriteFeed,
    "cmd_arm_write": CmdArmWriteFeed,
}
