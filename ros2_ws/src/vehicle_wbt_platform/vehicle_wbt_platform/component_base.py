"""BaseComponent — abstract lifecycle contract for all platform components.

Spec ref: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §组件模型

Every concrete component (Camera, Infrared, Stepper, Chassis, etc.) must extend
BaseComponent and implement its 5 methods. The sidecar orchestrator drives each
component through the lifecycle: init() -> start() -> (run) -> stop() -> cleanup().
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Protocol


class ComponentError(RuntimeError):
    """Raised when a component cannot complete init/start/cleanup."""


class HealthState(str, Enum):
    """Component health states — also used as diagnostic_msgs/DiagnosticStatus level mapping."""

    BOOTING = "BOOTING"
    RUNNING = "RUNNING"
    ERROR = "ERROR"
    STOPPED = "STOPPED"


@dataclass(frozen=True)
class HealthStatus:
    """Returned by BaseComponent.health_status() — published to /v1/diagnostics/<component>."""

    state: HealthState
    details: dict[str, Any] = field(default_factory=dict)


class _NodeLike(Protocol):
    """Protocol matching rclpy.node.Node interface we use. Avoids hard import of rclpy."""

    def create_publisher(self, msg_type: Any, topic: str, qos: int) -> Any: ...
    def create_subscription(self, msg_type: Any, topic: str, callback: Any, qos: int) -> Any: ...
    def get_logger(self) -> Any: ...


class _RegistryLike(Protocol):
    """Protocol matching ConfigRegistry surface area used by components."""

    @property
    def sensors(self) -> dict: ...
    @property
    def actuators(self) -> dict: ...


class _ControllerLike(Protocol):
    """Protocol matching BaseControllerHardwareInterface surface area used by components."""

    def read(self, port_id: int) -> Any: ...
    def write(self, port_id: int, value: Any) -> None: ...


@dataclass(frozen=True)
class ComponentContext:
    """Dependencies injected into every component at init() time.

    - node: rclpy Node (or fake in tests)
    - registry: ConfigRegistry (sensors + actuators dicts)
    - controller: BaseControllerHardwareInterface adapter (e.g. MC602Adapter)
    - config_yaml_path: absolute path to config_sensors.yml (for re-load)
    - ros_domain_id: DDS domain id this component should use
    """

    node: _NodeLike
    registry: _RegistryLike
    controller: _ControllerLike
    config_yaml_path: str
    ros_domain_id: int


class BaseComponent(ABC):
    """Abstract base class — every platform component extends this.

    Lifecycle:
        init(ctx)  — create publishers/subscribers, validate config entry
        start()    — begin producing (called once after all components init'd)
        stop()     — halt production (called before cleanup)
        cleanup()  — destroy publishers/subscribers, release hardware handles

    Concrete subclasses MUST also call super().__init__(component_id=...) to set id.
    """

    def __init__(self, *, component_id: str) -> None:
        if not component_id or not isinstance(component_id, str):
            raise ComponentError(f"component_id must be non-empty string, got {component_id!r}")
        self._component_id = component_id

    @property
    def component_id(self) -> str:
        return self._component_id

    @abstractmethod
    def init(self, context: ComponentContext) -> None:
        """Create ROS2 publishers/subscribers and validate hardware config. Called once."""

    @abstractmethod
    def start(self) -> None:
        """Begin producing. Called once after init() and after all components have init'd."""

    @abstractmethod
    def stop(self) -> None:
        """Halt production. Idempotent — safe to call multiple times."""

    @abstractmethod
    def cleanup(self) -> None:
        """Release all resources. Called once before process exit."""

    @abstractmethod
    def health_status(self) -> HealthStatus:
        """Return current health. Published to /v1/diagnostics/<component_id>."""
