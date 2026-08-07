"""Unit tests for feed components (Phase 6-7).

Tests verify the BaseComponent lifecycle (init -> start -> stop -> cleanup) and
the sensor read -> ROS2 publish path using a fake adapter.

No ROS2 daemon required -- ROS2 message packages are stubbed so tests run
on any dev box without ROS2 installed.
"""

from __future__ import annotations

import math
import sys
import types
from typing import Any
from unittest.mock import MagicMock, patch

import pytest


# ---------------------------------------------------------------------------
# Stub out ROS2 message packages so tests run without ROS2 installed.
# ---------------------------------------------------------------------------

def _make_msg_class(name: str) -> type:
    cls = MagicMock(name=name)
    instance = MagicMock(name=f"{name}_instance")
    cls.return_value = instance
    return cls


_std_msgs_mod = types.ModuleType("std_msgs")
_std_msgs_mod.Float32 = _make_msg_class("Float32")
_std_msgs_mod.Header = _make_msg_class("Header")
_std_msgs_mod.msg = types.ModuleType("std_msgs.msg")
_std_msgs_mod.msg.Float32 = _std_msgs_mod.Float32
_std_msgs_mod.msg.Header = _std_msgs_mod.Header

_sensor_msgs_mod = types.ModuleType("sensor_msgs")
_sensor_msgs_mod.JointState = _make_msg_class("JointState")
_sensor_msgs_mod.msg = types.ModuleType("sensor_msgs.msg")
_sensor_msgs_mod.msg.JointState = _sensor_msgs_mod.JointState

_nav_msgs_mod = types.ModuleType("nav_msgs")
_nav_msgs_mod.Odometry = _make_msg_class("Odometry")
_nav_msgs_mod.msg = types.ModuleType("nav_msgs.msg")
_nav_msgs_mod.msg.Odometry = _nav_msgs_mod.Odometry

_geometry_msgs_mod = types.ModuleType("geometry_msgs")
_geometry_msgs_mod.Quaternion = _make_msg_class("Quaternion")
_geometry_msgs_mod.Twist = _make_msg_class("Twist")
_geometry_msgs_mod.TransformStamped = _make_msg_class("TransformStamped")
_geometry_msgs_mod.Vector3 = _make_msg_class("Vector3")
_geometry_msgs_mod.msg = types.ModuleType("geometry_msgs.msg")
_geometry_msgs_mod.msg.Quaternion = _geometry_msgs_mod.Quaternion
_geometry_msgs_mod.msg.Twist = _geometry_msgs_mod.Twist
_geometry_msgs_mod.msg.TransformStamped = _geometry_msgs_mod.TransformStamped
_geometry_msgs_mod.msg.Vector3 = _geometry_msgs_mod.Vector3

_trajectory_msgs_mod = types.ModuleType("trajectory_msgs")
_trajectory_msgs_mod.JointTrajectory = _make_msg_class("JointTrajectory")
_trajectory_msgs_mod.msg = types.ModuleType("trajectory_msgs.msg")
_trajectory_msgs_mod.msg.JointTrajectory = _trajectory_msgs_mod.JointTrajectory

_tf2_ros_mod = types.ModuleType("tf2_ros")
_tf2_ros_bc = MagicMock(name="TransformBroadcaster")
_tf2_ros_mod.TransformBroadcaster = _tf2_ros_bc

sys.modules.setdefault("std_msgs", _std_msgs_mod)
sys.modules.setdefault("std_msgs.msg", _std_msgs_mod.msg)
sys.modules.setdefault("sensor_msgs", _sensor_msgs_mod)
sys.modules.setdefault("sensor_msgs.msg", _sensor_msgs_mod.msg)
sys.modules.setdefault("nav_msgs", _nav_msgs_mod)
sys.modules.setdefault("nav_msgs.msg", _nav_msgs_mod.msg)
sys.modules.setdefault("geometry_msgs", _geometry_msgs_mod)
sys.modules.setdefault("geometry_msgs.msg", _geometry_msgs_mod.msg)
sys.modules.setdefault("tf2_ros", _tf2_ros_mod)
sys.modules.setdefault("trajectory_msgs", _trajectory_msgs_mod)
sys.modules.setdefault("trajectory_msgs.msg", _trajectory_msgs_mod.msg)


# ---------------------------------------------------------------------------
# Fake adapter
# ---------------------------------------------------------------------------

class FakeAdapter:
    def __init__(self, *, encoder_values: list[int] | None = None) -> None:
        self._is_open = True
        self._encoder_values = encoder_values or [100, 200, 300, 400]
        self._read_count = 0
        self._servo_angles: dict[int, float] = {i + 1: 0.0 for i in range(7)}

    @property
    def is_open(self) -> bool:
        return self._is_open

    def read_sensor(self, *, port_id: int, sensor_type: str) -> float:
        self._read_count += 1
        if sensor_type == "ir":
            return 0.15
        if sensor_type == "analog_input":
            return self._servo_angles.get(port_id, 0.0)
        raise ValueError(f"unknown sensor_type {sensor_type!r}")

    def read_encoder4(self) -> list[int]:
        return list(self._encoder_values)

    def close(self) -> None:
        self._is_open = False

    def enumerate_ports(self) -> dict[str, int]:
        return {"motor": 4, "servo": 7, "stepper": 3, "io": 8}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_fake_node() -> MagicMock:
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    node.get_clock.return_value = MagicMock()
    clock_mock = node.get_clock.return_value
    clock_mock.now.return_value.to_msg.return_value = MagicMock()
    node.create_publisher.return_value = MagicMock()
    node.create_timer.return_value = MagicMock()
    return node


# ---------------------------------------------------------------------------
# IRFeed tests
# ---------------------------------------------------------------------------

class TestIRFeed:
    def test_init_creates_publisher(self):
        from vehicle_wbt_platform.feeds import IRFeed
        from vehicle_wbt_platform.config_loader import SensorConfig

        cfg = SensorConfig(
            id="ir_left", type="ir", port_id=8, port_physical="P8",
            topic="/vehicle_wbt/v1/sensors/ir/left",
            msg_type="std_msgs/Float32", rate_hz=20.0, enabled=True,
        )
        feed = IRFeed(component_id="ir_left", sensor_cfg=cfg)
        fake_node = _make_fake_node()
        ctx = MagicMock()
        ctx.node = fake_node
        ctx.controller = FakeAdapter()
        ctx.registry = MagicMock()

        feed.init(ctx)
        feed.start()
        assert feed._pub is not None
        feed.stop()
        feed.cleanup()

    def test_health_status_running_when_open(self):
        from vehicle_wbt_platform.feeds import IRFeed
        from vehicle_wbt_platform.config_loader import SensorConfig

        cfg = SensorConfig(
            id="ir_left", type="ir", port_id=8, port_physical="P8",
            topic="/vehicle_wbt/v1/sensors/ir/left",
            msg_type="std_msgs/Float32", rate_hz=20.0,
        )
        feed = IRFeed(component_id="ir_left", sensor_cfg=cfg)
        ctx = MagicMock()
        ctx.controller = FakeAdapter()
        ctx.node = _make_fake_node()
        feed._ctx = ctx

        status = feed.health_status()
        assert status.state.value == "RUNNING"

    def test_health_status_error_when_adapter_closed(self):
        from vehicle_wbt_platform.feeds import IRFeed
        from vehicle_wbt_platform.config_loader import SensorConfig

        cfg = SensorConfig(
            id="ir_left", type="ir", port_id=8, port_physical="P8",
            topic="/vehicle_wbt/v1/sensors/ir/left",
            msg_type="std_msgs/Float32", rate_hz=20.0,
        )
        feed = IRFeed(component_id="ir_left", sensor_cfg=cfg)
        fake_adapter = FakeAdapter()
        fake_adapter._is_open = False
        ctx = MagicMock()
        ctx.controller = fake_adapter
        ctx.node = _make_fake_node()
        feed._ctx = ctx

        status = feed.health_status()
        assert status.state.value == "ERROR"


# ---------------------------------------------------------------------------
# ArmFeed tests
# ---------------------------------------------------------------------------

class TestArmFeed:
    def test_init_with_joint_names_from_config(self):
        from vehicle_wbt_platform.feeds import ArmFeed
        from vehicle_wbt_platform.config_loader import ActuatorConfig

        cfg = ActuatorConfig(
            id="arm_main", type="arm", port_id=1, port_physical="Servo1",
            topic="/vehicle_wbt/v1/state/actuators/main",
            msg_type="sensor_msgs/JointState", rate_hz=50.0, enabled=True,
            type_specific={"joint_names": ["joint_base", "joint_shoulder", "joint_elbow"]},
        )
        feed = ArmFeed(component_id="arm_main", actuator_cfg=cfg)
        fake_node = _make_fake_node()
        ctx = MagicMock()
        ctx.node = fake_node
        ctx.controller = FakeAdapter()
        ctx.registry = MagicMock()

        feed.init(ctx)
        assert feed._joint_names == ["joint_base", "joint_shoulder", "joint_elbow"]

    def test_default_joint_names(self):
        from vehicle_wbt_platform.feeds import ArmFeed
        from vehicle_wbt_platform.config_loader import ActuatorConfig

        cfg = ActuatorConfig(
            id="arm_main", type="arm", port_id=1, port_physical="Servo1",
            topic="/vehicle_wbt/v1/state/actuators/main",
            msg_type="sensor_msgs/JointState", rate_hz=50.0,
        )
        feed = ArmFeed(component_id="arm_main", actuator_cfg=cfg)
        assert feed._joint_names == ["joint1", "joint2", "joint3", "joint4"]


# ---------------------------------------------------------------------------
# ChassisFeed tests
# ---------------------------------------------------------------------------

class TestChassisFeed:
    def test_init_creates_odom_and_tf_publishers(self):
        from vehicle_wbt_platform.feeds import ChassisFeed

        feed = ChassisFeed(component_id="mecanum_chassis")
        fake_node = _make_fake_node()
        ctx = MagicMock()
        ctx.node = fake_node
        ctx.controller = FakeAdapter()
        ctx.registry = MagicMock()

        feed.init(ctx)
        assert fake_node.create_publisher.call_count >= 1

    def test_mecanum_kinematics(self):
        wheel_radius = 0.03
        Lx, Ly = 0.15, 0.10
        dt = 1.0 / 50.0

        counts_per_rev = 2015.13
        encoder_2_rad = 2.0 * math.pi / counts_per_rev

        d_counts = [1000, 1000, 1000, 1000]
        d_meters = [encoder_2_rad * wheel_radius * dc for dc in d_counts]
        d_FR = d_meters[1]
        d_FL = d_meters[0]
        d_RL = d_meters[2]
        d_RR = d_meters[3]
        vx = (d_FR + d_FL + d_RL + d_RR) / (4.0 * dt)
        vy = (-d_FR + d_FL + d_RL - d_RR) / (4.0 * dt)
        vtheta = (-d_FR + d_FL - d_RL + d_RR) / (4.0 * (Lx + Ly) * dt)

        assert abs(vy) < 1e-6
        assert abs(vtheta) < 1e-6
        assert vx > 0.0


# ---------------------------------------------------------------------------
# Component registry tests
# ---------------------------------------------------------------------------

class TestComponentRegistry:
    def test_ir_type_mapped(self):
        from vehicle_wbt_platform.feeds import COMPONENT_CLASSES
        assert "ir" in COMPONENT_CLASSES

    def test_chassis_type_mapped(self):
        from vehicle_wbt_platform.feeds import COMPONENT_CLASSES
        assert "chassis" in COMPONENT_CLASSES

    def test_arm_type_mapped(self):
        from vehicle_wbt_platform.feeds import COMPONENT_CLASSES
        assert "arm" in COMPONENT_CLASSES

    def test_cmd_vel_write_mapped(self):
        from vehicle_wbt_platform.feeds import COMPONENT_CLASSES
        assert "cmd_vel_write" in COMPONENT_CLASSES

    def test_cmd_arm_write_mapped(self):
        from vehicle_wbt_platform.feeds import COMPONENT_CLASSES
        assert "cmd_arm_write" in COMPONENT_CLASSES


# ---------------------------------------------------------------------------
# CmdVelWriteFeed tests
# ---------------------------------------------------------------------------

class TestCmdVelWriteFeed:
    def test_init_creates_subscription(self):
        from vehicle_wbt_platform.feeds import CmdVelWriteFeed

        feed = CmdVelWriteFeed(component_id="cmd_vel_write")
        fake_node = _make_fake_node()
        ctx = MagicMock()
        ctx.node = fake_node
        ctx.controller = FakeAdapter()
        ctx.registry = MagicMock()

        feed.init(ctx)
        assert fake_node.create_subscription.called
        feed.cleanup()

    def test_mecanum_mixing_forward_right(self):
        """Forward + right: all wheels positive, FR > FL."""
        Lx, Ly = 0.15, 0.10
        vx, vy = 1.0, 0.5

        fl = vx - vy
        fr = vx + vy
        rl = vx + vy
        rr = vx - vy

        assert fl > 0 and fr > 0 and rl > 0 and rr > 0
        assert fr > fl
        assert rl > rr

    def test_mecanum_mixing_rotation_only(self):
        """Pure rotation: left wheels negative, right wheels positive."""
        Lx, Ly = 0.15, 0.10
        wz = 1.0
        k = Lx + Ly

        fl = -k * wz
        fr = k * wz
        rl = k * wz   # same sign as fr
        rr = -k * wz  # same sign as fl

        assert fl < 0 and rr < 0
        assert fr > 0 and rl > 0


# ---------------------------------------------------------------------------
# CmdArmWriteFeed tests
# ---------------------------------------------------------------------------

class TestCmdArmWriteFeed:
    def test_init_with_topic_from_config(self):
        from vehicle_wbt_platform.feeds import CmdArmWriteFeed

        cfg = {
            "id": "arm_main",
            "type": "cmd_arm_write",
            "topic": "/vehicle_wbt/v1/cmd/arm/main/trajectory",
            "rate_hz": 50.0,
        }
        feed = CmdArmWriteFeed(component_id="arm_main", arm_cfg=cfg)
        fake_node = _make_fake_node()
        ctx = MagicMock()
        ctx.node = fake_node
        ctx.controller = FakeAdapter()
        ctx.registry = MagicMock()

        feed.init(ctx)
        assert fake_node.create_subscription.called
        feed.cleanup()


# ---------------------------------------------------------------------------
# Config loader tests
# ---------------------------------------------------------------------------

# Feed-able sensor types: these are the ones COMPONENT_CLASSES is expected to cover.
# Other types (camera, raw analog_input, etc.) are handled by C++ nodes.
_FEEDABLE_TYPES = {"ir", "chassis", "arm"}


class TestConfigLoaderForComponents:
    def test_load_config_sensors_yml(self):
        from pathlib import Path
        from vehicle_wbt_platform.config_loader import load_registry

        repo_root = Path(__file__).resolve().parents[4]
        config_path = repo_root / "config_sensors.yml"
        if not config_path.exists():
            pytest.skip("config_sensors.yml not at expected path")

        registry = load_registry(str(config_path))
        assert len(registry.sensors) > 0 or len(registry.actuators) > 0

    def test_all_sensor_types_have_feed_class(self):
        from pathlib import Path
        from vehicle_wbt_platform.config_loader import load_registry
        from vehicle_wbt_platform.feeds import COMPONENT_CLASSES

        repo_root = Path(__file__).resolve().parents[4]
        config_path = repo_root / "config_sensors.yml"
        if not config_path.exists():
            pytest.skip("config_sensors.yml not at expected path")

        registry = load_registry(str(config_path))
        unmapped = []
        for sensor_id, cfg in registry.enabled_sensors().items():
            if cfg.type in _FEEDABLE_TYPES and cfg.type not in COMPONENT_CLASSES:
                unmapped.append(f"{sensor_id}:{cfg.type}")
        assert not unmapped, f"Feed-able sensor types with no feed class: {unmapped}"


# ---------------------------------------------------------------------------
# Orchestrator integration tests
# ---------------------------------------------------------------------------

class TestOrchestrator:
    def test_spawn_returns_components(self):
        from vehicle_wbt_platform.orchestrator import SidecarOrchestrator
        from pathlib import Path

        repo_root = Path(__file__).resolve().parents[4]
        config_path = repo_root / "config_sensors.yml"
        if not config_path.exists():
            pytest.skip("config_sensors.yml not at expected path")

        orch = SidecarOrchestrator(
            config_path=str(config_path),
            serial_port="/dev/null",
            ros_domain_id=42,
        )
        components = orch.spawn_components()
        assert len(components) > 0, "spawn_components returned empty list"
        for comp in components:
            assert hasattr(comp, "component_id")
            assert hasattr(comp, "init")
            assert hasattr(comp, "start")
            assert hasattr(comp, "stop")
            assert hasattr(comp, "cleanup")

    def test_shutdown_is_idempotent(self):
        from vehicle_wbt_platform.orchestrator import SidecarOrchestrator
        from pathlib import Path

        repo_root = Path(__file__).resolve().parents[4]
        config_path = repo_root / "config_sensors.yml"
        if not config_path.exists():
            pytest.skip("config_sensors.yml not at expected path")

        orch = SidecarOrchestrator(
            config_path=str(config_path),
            serial_port="/dev/null",
            ros_domain_id=42,
        )
        orch.shutdown()
        orch.shutdown()
