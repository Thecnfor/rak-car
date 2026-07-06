"""config_sensors.yml schema + loader.

Spec ref: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §配置系统
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


# Topic namespace that every published topic MUST start with.
# Hardcoded per spec §预留 Topic Namespace; v1 协议内不可改.
V1_NAMESPACE_PREFIX = "/vehicle_wbt/v1/"


class ConfigSchemaError(ValueError):
    """Raised when config_sensors.yml violates the v1 schema."""


# Fields that every sensor/actuator entry MUST have.
_REQUIRED_FIELDS = ("id", "type", "port_id", "port_physical", "topic", "msg_type", "rate_hz")


@dataclass(frozen=True)
class SensorConfig:
    """One sensor entry from config_sensors.yml."""

    id: str
    type: str
    port_id: int
    port_physical: str
    topic: str
    msg_type: str
    rate_hz: float
    enabled: bool = True
    type_specific: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ActuatorConfig:
    """One actuator entry from config_sensors.yml."""

    id: str
    type: str
    port_id: int
    port_physical: str
    topic: str
    msg_type: str
    rate_hz: float
    enabled: bool = True
    type_specific: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ConfigRegistry:
    """Parsed content of config_sensors.yml."""

    sensors: dict[str, SensorConfig]
    actuators: dict[str, ActuatorConfig]

    def enabled_sensors(self) -> dict[str, SensorConfig]:
        return {k: v for k, v in self.sensors.items() if v.enabled}

    def enabled_actuators(self) -> dict[str, ActuatorConfig]:
        return {k: v for k, v in self.actuators.items() if v.enabled}


def _parse_entry(raw: dict[str, Any], kind: str) -> dict[str, Any]:
    """Validate one entry, return normalized dict separating common vs type_specific fields."""
    missing = [f for f in _REQUIRED_FIELDS if f not in raw]
    if missing:
        raise ConfigSchemaError(
            f"{kind} entry {raw.get('id', '?')!r}: missing required fields: {missing}"
        )

    if not str(raw["topic"]).startswith(V1_NAMESPACE_PREFIX):
        raise ConfigSchemaError(
            f"{kind} entry {raw['id']!r}: topic {raw['topic']!r} must start with {V1_NAMESPACE_PREFIX}"
        )

    # Type-specific fields are everything outside the required set + 'enabled'.
    common = set(_REQUIRED_FIELDS) | {"enabled"}
    type_specific = {k: v for k, v in raw.items() if k not in common}
    return {
        "id": str(raw["id"]),
        "type": str(raw["type"]),
        "port_id": int(raw["port_id"]),
        "port_physical": str(raw["port_physical"]),
        "topic": str(raw["topic"]),
        "msg_type": str(raw["msg_type"]),
        "rate_hz": float(raw["rate_hz"]),
        "enabled": bool(raw.get("enabled", True)),
        "type_specific": type_specific,
    }


def load_registry(path: str) -> ConfigRegistry:
    """Load and validate config_sensors.yml at `path`. Raises ConfigSchemaError on any issue."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ConfigSchemaError(f"config root must be a mapping, got {type(raw).__name__}")

    sensors_raw = raw.get("sensors", [])
    actuators_raw = raw.get("actuators", [])
    if not isinstance(sensors_raw, list):
        raise ConfigSchemaError("'sensors' must be a list")
    if not isinstance(actuators_raw, list):
        raise ConfigSchemaError("'actuators' must be a list")

    sensors: dict[str, SensorConfig] = {}
    for entry in sensors_raw:
        parsed = _parse_entry(entry, "sensor")
        if parsed["id"] in sensors:
            raise ConfigSchemaError(f"duplicate id {parsed['id']!r} in sensors")
        sensors[parsed["id"]] = SensorConfig(**parsed)

    actuators: dict[str, ActuatorConfig] = {}
    for entry in actuators_raw:
        parsed = _parse_entry(entry, "actuator")
        if parsed["id"] in actuators:
            raise ConfigSchemaError(f"duplicate id {parsed['id']!r} in actuators")
        actuators[parsed["id"]] = ActuatorConfig(**parsed)

    return ConfigRegistry(sensors=sensors, actuators=actuators)
