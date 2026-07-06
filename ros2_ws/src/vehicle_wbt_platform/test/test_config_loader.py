from pathlib import Path

import pytest
import yaml

from vehicle_wbt_platform.config_loader import (
    ActuatorConfig,
    ConfigRegistry,
    ConfigSchemaError,
    SensorConfig,
    load_registry,
)


SAMPLE_YAML = """
sensors:
  - id: front_ir
    type: ir
    port_id: 7
    port_physical: P7
    topic: /vehicle_wbt/v1/sensors/ir/front
    msg_type: std_msgs/Float32
    rate_hz: 20
  - id: vert_limit
    type: analog_input
    port_id: 6
    port_physical: P6
    topic: /vehicle_wbt/v1/sensors/analog/vert_limit
    msg_type: std_msgs/Int32
    rate_hz: 50
    invert: false
    threshold: 1000
actuators:
  - id: motor_ejection
    type: motor
    port_id: 5
    port_physical: M5
    topic: /vehicle_wbt/v1/actuators/motor/m5/state
    msg_type: std_msgs/Float32
    rate_hz: 50
    perimeter: 0.032
    reverse: -1
"""


def test_load_registry_returns_registry(tmp_path: Path) -> None:
    cfg_path = tmp_path / "config_sensors.yml"
    cfg_path.write_text(SAMPLE_YAML)
    registry = load_registry(str(cfg_path))
    assert isinstance(registry, ConfigRegistry)
    assert len(registry.sensors) == 2
    assert len(registry.actuators) == 1


def test_sensor_config_typed() -> None:
    cfg_path = Path("/tmp/test_config_sensors.yml")
    cfg_path.write_text(SAMPLE_YAML)
    registry = load_registry(str(cfg_path))
    ir = registry.sensors["front_ir"]
    assert isinstance(ir, SensorConfig)
    assert ir.type == "ir"
    assert ir.port_id == 7
    assert ir.port_physical == "P7"
    assert ir.topic == "/vehicle_wbt/v1/sensors/ir/front"
    assert ir.msg_type == "std_msgs/Float32"
    assert ir.rate_hz == 20
    assert ir.enabled is True  # default


def test_actuator_config_typed() -> None:
    cfg_path = Path("/tmp/test_config_sensors.yml")
    cfg_path.write_text(SAMPLE_YAML)
    registry = load_registry(str(cfg_path))
    motor = registry.actuators["motor_ejection"]
    assert isinstance(motor, ActuatorConfig)
    assert motor.type == "motor"
    assert motor.port_id == 5
    assert motor.type_specific["perimeter"] == 0.032


def test_enabled_false_default_works() -> None:
    yaml_with_disabled = """
sensors:
  - id: ultrasonic_front
    type: ultrasonic
    port_id: 5
    port_physical: P5
    topic: /vehicle_wbt/v1/sensors/ultrasonic/front
    msg_type: std_msgs/Float32
    rate_hz: 10
    enabled: false
"""
    cfg_path = Path("/tmp/test_config_sensors_disabled.yml")
    cfg_path.write_text(yaml_with_disabled)
    registry = load_registry(str(cfg_path))
    assert registry.sensors["ultrasonic_front"].enabled is False


def test_missing_required_field_raises(tmp_path: Path) -> None:
    bad = """
sensors:
  - id: bad_sensor
    type: ir
    # missing: port_id, topic, msg_type, rate_hz
"""
    cfg_path = tmp_path / "bad.yml"
    cfg_path.write_text(bad)
    with pytest.raises(ConfigSchemaError):
        load_registry(str(cfg_path))


def test_duplicate_id_raises(tmp_path: Path) -> None:
    dup = """
sensors:
  - {id: dup, type: ir, port_id: 1, port_physical: P1, topic: /vehicle_wbt/v1/sensors/ir/a, msg_type: std_msgs/Int32, rate_hz: 10}
  - {id: dup, type: ir, port_id: 2, port_physical: P2, topic: /vehicle_wbt/v1/sensors/ir/b, msg_type: std_msgs/Int32, rate_hz: 10}
"""
    cfg_path = tmp_path / "dup.yml"
    cfg_path.write_text(dup)
    with pytest.raises(ConfigSchemaError, match="duplicate id"):
        load_registry(str(cfg_path))


def test_topic_must_be_under_v1_namespace(tmp_path: Path) -> None:
    bad_topic = """
sensors:
  - id: bad
    type: ir
    port_id: 1
    port_physical: P1
    topic: /wrong/namespace/ir
    msg_type: std_msgs/Float32
    rate_hz: 10
"""
    cfg_path = tmp_path / "bad_topic.yml"
    cfg_path.write_text(bad_topic)
    with pytest.raises(ConfigSchemaError, match="must start with /vehicle_wbt/v1/"):
        load_registry(str(cfg_path))
