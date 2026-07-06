from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import pytest

from vehicle_wbt_platform.component_base import (
    BaseComponent,
    ComponentContext,
    ComponentError,
    HealthState,
    HealthStatus,
)


@dataclass
class FakeNode:
    """Minimal stand-in for rclpy.node.Node."""
    name: str = "fake_node"


@dataclass
class FakeRegistry:
    sensors: dict = None  # type: ignore[assignment]
    actuators: dict = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self.sensors is None:
            self.sensors = {}
        if self.actuators is None:
            self.actuators = {}


@dataclass
class FakeController:
    name: str = "fake_controller"


class DummyComponent(BaseComponent):
    """Concrete subclass that does nothing — for interface-shape tests."""

    def init(self, context: ComponentContext) -> None:
        self._init_called_with = context

    def start(self) -> None:
        self._start_count = getattr(self, "_start_count", 0) + 1

    def stop(self) -> None:
        self._stop_count = getattr(self, "_stop_count", 0) + 1

    def cleanup(self) -> None:
        self._cleanup_count = getattr(self, "_cleanup_count", 0) + 1

    def health_status(self) -> HealthStatus:
        return HealthStatus(state=HealthState.RUNNING, details={"name": "dummy"})


def test_base_component_cannot_be_instantiated_directly() -> None:
    with pytest.raises(TypeError):
        BaseComponent()  # type: ignore[abstract]


def test_context_holds_dependencies() -> None:
    node = FakeNode(name="n1")
    registry = FakeRegistry()
    controller = FakeController(name="mc602")
    ctx = ComponentContext(
        node=node,
        registry=registry,
        controller=controller,
        config_yaml_path="/tmp/cfg.yml",
        ros_domain_id=42,
    )
    assert ctx.node.name == "n1"
    assert ctx.ros_domain_id == 42


def test_dummy_component_lifecycle() -> None:
    node = FakeNode()
    ctx = ComponentContext(
        node=node,
        registry=FakeRegistry(),
        controller=FakeController(),
        config_yaml_path="x",
        ros_domain_id=42,
    )
    comp = DummyComponent(component_id="test_dummy")
    comp.init(ctx)
    assert comp._init_called_with is ctx
    comp.start()
    comp.start()
    assert comp._start_count == 2
    comp.stop()
    assert comp._stop_count == 1
    comp.cleanup()
    assert comp._cleanup_count == 1


def test_health_status_values() -> None:
    comp = DummyComponent(component_id="x")
    h = comp.health_status()
    assert h.state == HealthState.RUNNING
    assert h.details == {"name": "dummy"}


def test_health_state_enum_has_four_values() -> None:
    assert {s.name for s in HealthState} == {"BOOTING", "RUNNING", "ERROR", "STOPPED"}


def test_component_id_required() -> None:
    with pytest.raises((TypeError, ComponentError)):
        DummyComponent()  # type: ignore[call-arg]
