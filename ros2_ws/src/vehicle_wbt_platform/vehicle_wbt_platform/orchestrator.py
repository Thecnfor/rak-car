"""Sidecar orchestrator — glue between config, controller, and components.

Spec ref: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §组件模型

Phase 1: load config + instantiate MC602Adapter (no components yet).
Phase 2+: spawn components dynamically from config.
"""

from __future__ import annotations

import logging
from typing import Optional

from vehicle_wbt_platform.config_loader import ConfigRegistry, load_registry
from vehicle_wbt_platform.mc602_adapter import MC602Adapter


_logger = logging.getLogger(__name__)


class SidecarOrchestrator:
    """Owns the config registry and the controller adapter.

    Phase 1: just wires them together. Component spawning lands in Phase 2.
    """

    def __init__(self, *, config_path: str, serial_port: str, ros_domain_id: int) -> None:
        self._config_path = config_path
        self._ros_domain_id = ros_domain_id
        self.registry: ConfigRegistry = load_registry(config_path)
        self.adapter: MC602Adapter = MC602Adapter(serial_port=serial_port)
        self._shutdown_called = False
        _logger.info(
            "SidecarOrchestrator ready: sensors=%d actuators=%d enabled=%d/%d domain=%d",
            len(self.registry.sensors),
            len(self.registry.actuators),
            len(self.registry.enabled_sensors()),
            len(self.registry.enabled_actuators()),
            ros_domain_id,
        )

    def open_hardware(self) -> None:
        """Open the controller adapter. Components may begin reading/writing after this."""
        self.adapter.open()

    def shutdown(self) -> None:
        """Idempotent. Safe to call from finally / signal handler."""
        if self._shutdown_called:
            return
        self._shutdown_called = True
        try:
            self.adapter.close()
        except Exception as e:  # noqa: BLE001 — shutdown must never raise
            _logger.warning("adapter.close() raised %r, ignoring", e)

    def summary(self) -> str:
        return (
            f"sensors={len(self.registry.sensors)} "
            f"actuators={len(self.registry.actuators)} "
            f"enabled={len(self.registry.enabled_sensors())}/"
            f"{len(self.registry.enabled_actuators())} "
            f"domain={self._ros_domain_id} "
            f"hw={'open' if self.adapter.is_open else 'closed'}"
        )