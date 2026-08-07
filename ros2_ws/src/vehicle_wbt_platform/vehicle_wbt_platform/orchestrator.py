"""Sidecar orchestrator — glue between config, controller, and feed components.

Spec ref: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §组件模型

Lifecycle:
    1. Load config + instantiate MC602Adapter
    2. Scan config_sensors.yml, instantiate one BaseComponent per enabled entry
    3. Drive each component: init() -> start()
    4. Spin rclpy executor (components self-publish via timers)
    5. On shutdown: stop() -> cleanup() each component in reverse order
"""

from __future__ import annotations

import logging
import signal
from typing import Any

from vehicle_wbt_platform.config_loader import ConfigRegistry, load_registry
from vehicle_wbt_platform.component_base import BaseComponent, ComponentContext, HealthState
from vehicle_wbt_platform.feeds import COMPONENT_CLASSES
from vehicle_wbt_platform.mc602_adapter import MC602Adapter

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False


_logger = logging.getLogger(__name__)


class SidecarOrchestrator:
    """Owns the config registry, controller adapter, and spawned feed components."""

    def __init__(self, *, config_path: str, serial_port: str, ros_domain_id: int) -> None:
        self._config_path = config_path
        self._ros_domain_id = ros_domain_id
        self.registry: ConfigRegistry = load_registry(config_path)
        self.adapter: MC602Adapter = MC602Adapter(serial_port=serial_port)
        self._components: list[BaseComponent] = []
        self._executor: MultiThreadedExecutor | None = None
        self._shutdown_called = False
        _logger.info(
            "SidecarOrchestrator ready: sensors=%d actuators=%d domain=%d",
            len(self.registry.sensors),
            len(self.registry.actuators),
            ros_domain_id,
        )

    # --- Component spawning (Phase 2) ---

    def spawn_components(self) -> list[BaseComponent]:
        """Instantiate one feed component per enabled config entry.

        Returns the list of spawned components (also stored in self._components).
        Component instantiation does not require ROS2 — only init()/start()
        (called later with a ROS2 node) do.
        """
        spawned: list[BaseComponent] = []

        # Sensors
        for sensor_id, cfg in self.registry.enabled_sensors().items():
            cls = COMPONENT_CLASSES.get(cfg.type)
            if cls is None:
                _logger.warning("No feed class for sensor type %r (id=%s); skipping", cfg.type, sensor_id)
                continue
            comp = cls(component_id=sensor_id, sensor_cfg=cfg)
            spawned.append(comp)
            _logger.debug("Spawned %s for sensor %s", cls.__name__, sensor_id)

        # Actuators
        for act_id, cfg in self.registry.enabled_actuators().items():
            cls = COMPONENT_CLASSES.get(cfg.type)
            if cls is None:
                _logger.warning("No feed class for actuator type %r (id=%s); skipping", cfg.type, act_id)
                continue
            comp = cls(component_id=act_id, actuator_cfg=cfg)
            spawned.append(comp)
            _logger.debug("Spawned %s for actuator %s", cls.__name__, act_id)

        self._components = spawned
        _logger.info("Spawned %d components", len(spawned))
        return spawned

    def init_components(self, node: Any) -> None:
        """Call init() on all spawned components with a ComponentContext."""
        ctx = ComponentContext(
            node=node,
            registry=self.registry,
            controller=self.adapter,
            config_yaml_path=self._config_path,
            ros_domain_id=self._ros_domain_id,
        )
        for comp in self._components:
            try:
                comp.init(ctx)
            except Exception as e:
                _logger.error("Component[%s] init failed: %s", comp.component_id, e)

    def start_components(self) -> None:
        """Call start() on all initialized components (starts their timers)."""
        for comp in self._components:
            try:
                comp.start()
            except Exception as e:
                _logger.error("Component[%s] start failed: %s", comp.component_id, e)

    def stop_components(self) -> None:
        """Call stop() on all components in reverse order."""
        for comp in reversed(self._components):
            try:
                comp.stop()
            except Exception as e:
                _logger.warning("Component[%s] stop raised: %s", comp.component_id, e)

    def cleanup_components(self) -> None:
        """Call cleanup() on all components in reverse order."""
        for comp in reversed(self._components):
            try:
                comp.cleanup()
            except Exception as e:
                _logger.warning("Component[%s] cleanup raised: %s", comp.component_id, e)

    # --- Hardware lifecycle ---

    def open_hardware(self) -> None:
        """Open the controller adapter."""
        self.adapter.open()

    # --- Spin / shutdown ---

    def spin(self, node: Any) -> None:
        """Run the rclpy executor until SIGINT/SIGTERM.

        Blocks until a signal arrives or the node is destroyed.
        """
        if not _ROS2_AVAILABLE:
            _logger.warning("rclpy not available — spin() is a no-op")
            return

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(node)

        # Wire signal handlers for clean shutdown
        self._install_signal_handlers()

        _logger.info("Spinning — %d components active", len(self._components))
        try:
            self._executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        """Idempotent shutdown sequence: stop → cleanup → close hardware."""
        if self._shutdown_called:
            return
        self._shutdown_called = True

        _logger.info("Shutting down sidecar orchestrator")
        self.stop_components()
        self.cleanup_components()
        try:
            self.adapter.close()
        except Exception as e:  # noqa: BLE001 — shutdown must never raise
            _logger.warning("adapter.close() raised %r, ignoring", e)

    # --- Summary ---

    def summary(self) -> str:
        comp_states = {
            c.component_id: c.health_status().state.value
            for c in self._components
        }
        return (
            f"sensors={len(self.registry.sensors)} "
            f"actuators={len(self.registry.actuators)} "
            f"enabled={len(self.registry.enabled_sensors())}/"
            f"{len(self.registry.enabled_actuators())} "
            f"domain={self._ros_domain_id} "
            f"hw={'open' if self.adapter.is_open else 'closed'} "
            f"components={comp_states}"
        )

    # --- Signal handling ---

    def _install_signal_handlers(self) -> None:
        """Install SIGINT/SIGTERM handlers that call shutdown()."""
        try:
            signal.signal(signal.SIGINT, self._on_signal)
            signal.signal(signal.SIGTERM, self._on_signal)
        except (ValueError, OSError):
            # Can't set signals in threads / non-main contexts
            pass

    def _on_signal(self, signum: int, _frame: Any) -> None:
        _logger.info("Received signal %d, shutting down", signum)
        self.shutdown()
        raise SystemExit(0)
