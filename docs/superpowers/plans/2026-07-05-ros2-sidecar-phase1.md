# ROS2 Sidecar Phase 1 — Platform Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Establish the `vehicle_wbt_platform` colcon package skeleton with config-driven Component base class, MC602 controller adapter, and URDF framework — so Phase 2+ can add specific components (camera, IR, chassis, arm) by writing YAML + 1 component class each, without touching platform code.

**Architecture:** Pure Python 3.10+ ROS2 Humble package. Components are discovered by scanning `config_sensors.yml` and dynamically loaded via `importlib`. Each component extends `BaseComponent` (lifecycle contract) and uses a `BaseControllerHardwareInterface` adapter (e.g., `MC602Adapter`) for hardware I/O. No business code in main/qqq.py is touched; the sidecar is opt-in via `ENABLE_ROS2=1` env var.

**Tech Stack:**
- ROS2 Humble (LTS, supported until 2027-05) on Ubuntu 22.04 + JetPack 6.x
- Python 3.10+
- colcon build system
- pytest 7.x for unit tests
- CycloneDDS for ROS2 transport
- xacro for URDF templating

**Related spec:** `docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md` (1799 lines, 25 sections)

**This plan covers:** Spec §组件模型 / §硬件接口插件 / §配置系统 / §Chassis 抽象 (BaseChassis only) / §Camera 抽象 (config schema only). NOT covered: §机械臂抽象 / §安全设计 (4-layer gate) / §生命周期 (systemd) / §远程接入 (DDS) / §仿真回路 (Gazebo) — those are Plans B/C.

---

## Global Constraints

These apply to every task. Copied verbatim from the spec:

1. **不可修改现有核心文件**: `main/qqq.py`, `car_wrap.py`, `vehicle/base/serial_wrap.py`, `vehicle/base/mc601_ctl2.py`, `vehicle/base/mc602_ctl2.py`, `vehicle/base/controller_wrap.py`, `vehicle/arm/arm_base.py`, `vehicle/driver/vehicle_base.py` — all listed in CLAUDE.md "DO_NOT_MODIFY". Sidecar may `import` from these but must not edit them.
2. **ENABLE_ROS2=0 字节级一致**: when env var unset/false, sidecar module is not even imported; main program behavior must be byte-identical to pre-sidecar state.
3. **Topic namespace**: ALL ROS2 topics under `/vehicle_wbt/v1/`. Never publish to unprefixed topic names.
4. **不可新增 anti-patterns**: 裸 `except:`, `while True: time.sleep(1)` 替代错误处理, 硬编码密钥, `eval()` 解析 LLM 输出, `eval(chassis_type)` — all banned per CLAUDE.md.
5. **Python 3.10+ syntax** (match...case, type hints with | union, etc.).
6. **绝对禁止窗口**: 比赛前 4 周内 (2026-07-13 → 2026-08-12) 不在 main 分支合任何 ROS2 代码. Phase 1 work lives on `develop/ros2-sidecar` only.
7. **测试必须真跑**: `pytest` 真的能执行；不允许 "would pass if run" 的伪测试。
8. **每个 task 一个 commit**: Task 1..6 各对应一个 git commit, message 格式 `feat(phase1): <task-summary>`.

---

## File Structure (Phase 1)

Files created by this plan (13 total):

```
ros2_ws/                                          # colcon workspace root
├── src/
│   └── vehicle_wbt_platform/                     # colcon package (Python ament)
│       ├── package.xml                            # ROS2 package manifest
│       ├── setup.py                               # Python package setup
│       ├── setup.cfg                              # pytest config
│       ├── vehicle_wbt_platform/                 # Python module
│       │   ├── __init__.py                        # version + public API
│       │   ├── __main__.py                        # sidecar entry point
│       │   ├── config_loader.py                   # config_sensors.yml schema + loader
│       │   ├── component_base.py                  # BaseComponent abstract class
│       │   ├── controller_base.py                 # BaseControllerHardwareInterface
│       │   ├── mc602_adapter.py                   # MC602Adapter
│       │   └── chassis_base.py                    # BaseChassis abstract class
│       └── test/                                  # pytest tests
│           ├── __init__.py
│           ├── conftest.py                        # shared fixtures
│           ├── test_config_loader.py
│           ├── test_component_base.py
│           ├── test_mc602_adapter.py
│           └── test_chassis_base.py
config_sensors.yml                                # sensors/actuators registry
urdf/
├── vehicle_wbt.urdf.xacro                        # base robot URDF
└── README.md                                      # URDF usage docs
```

Files modified:
- `docs/README.md` — add entries for `config_sensors.yml`, `urdf/`, Phase 1 plan link
- `docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md` — mark Phase 1 sections as "✅ Implemented in commit <hash>"

---

## Task Decomposition

### Task 1: Colcon workspace skeleton + package manifest

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_platform/package.xml`
- Create: `ros2_ws/src/vehicle_wbt_platform/setup.py`
- Create: `ros2_ws/src/vehicle_wbt_platform/setup.cfg`
- Create: `ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/__init__.py`
- Create: `ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/__main__.py` (empty stub)
- Create: `ros2_ws/src/vehicle_wbt_platform/test/__init__.py`
- Create: `ros2_ws/src/vehicle_wbt_platform/test/conftest.py`

**Interfaces:**
- Produces: importable `vehicle_wbt_platform` Python package
- Produces: `vehicle_wbt_platform.__version__` string

- [ ] **Step 1: Write failing test for package import**

Create `ros2_ws/src/vehicle_wbt_platform/test/test_package.py`:

```python
def test_package_imports():
    import vehicle_wbt_platform
    assert vehicle_wbt_platform.__version__ == "0.1.0"


def test_main_module_imports():
    from vehicle_wbt_platform import __main__  # noqa: F401
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ros2_ws/src/vehicle_wbt_platform && python -m pytest test/test_package.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_wbt_platform'`

- [ ] **Step 3: Create `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model
  href="http://download.ros.org/schema/package_format3.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vehicle_wbt_platform</name>
  <version>0.1.0</version>
  <description>Platform-level ROS2 sidecar for vehicle_wbt — see docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md</description>
  <maintainer email="thecnfor@users.noreply.github.com">Thecnfor</maintainer>
  <license>Proprietary</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>python</depend>

  <test_depend>python3-pytest</test_depend>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>ament_flake8</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4: Create `setup.py`**

```python
from glob import glob
import os

from setuptools import find_packages, setup

PACKAGE_NAME = "vehicle_wbt_platform"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="Thecnfor",
    maintainer_email="thecnfor@users.noreply.github.com",
    description="Platform-level ROS2 sidecar for vehicle_wbt",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sidecar = vehicle_wbt_platform.__main__:main",
        ],
    },
)
```

- [ ] **Step 5: Create `setup.cfg`**

```ini
[develop]
script_dir=$base/lib/vehicle_wbt_platform
[install]
install_scripts=$base/lib/vehicle_wbt_platform
[tool:pytest]
testpaths=test
addopts=-v --tb=short
```

- [ ] **Step 6: Create `vehicle_wbt_platform/__init__.py`**

```python
"""Platform-level ROS2 sidecar for vehicle_wbt.

See docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md for design.
"""

__version__ = "0.1.0"
```

- [ ] **Step 7: Create `vehicle_wbt_platform/__main__.py`**

```python
"""sidecar entry point — full implementation in Task 6."""

import os
import sys


def main() -> int:
    if os.environ.get("ENABLE_ROS2", "").lower() not in {"1", "true", "yes"}:
        print("[vehicle_wbt_platform] ENABLE_ROS2 not set, exiting cleanly")
        return 0
    print("[vehicle_wbt_platform] ENABLE_ROS2=1, full init in Task 6", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 8: Create `test/__init__.py` (empty file)**

- [ ] **Step 9: Create `test/conftest.py`**

```python
"""Shared pytest fixtures for vehicle_wbt_platform tests."""

import os
import pytest


@pytest.fixture
def enable_ros2(monkeypatch):
    """Set ENABLE_ROS2=1 for tests that require ROS2 to be active."""
    monkeypatch.setenv("ENABLE_ROS2", "1")
    yield


@pytest.fixture
def disable_ros2(monkeypatch):
    """Ensure ENABLE_ROS2 is unset for tests verifying zero-impact behavior."""
    monkeypatch.delenv("ENABLE_ROS2", raising=False)
    yield
```

- [ ] **Step 10: Create `resource/vehicle_wbt_platform` (empty marker file)**

```bash
mkdir -p ros2_ws/src/vehicle_wbt_platform/resource
touch ros2_ws/src/vehicle_wbt_platform/resource/vehicle_wbt_platform
```

- [ ] **Step 11: Run test to verify it passes**

Run: `cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python -m pytest test/test_package.py -v`
Expected: PASS (2 tests)

- [ ] **Step 12: Commit**

```bash
git add ros2_ws/
git commit -m "feat(phase1): colcon workspace skeleton + package manifest

Establishes the vehicle_wbt_platform ament_python package with:
- package.xml declaring ROS2 Humble dependencies
- setup.py with sidecar console script entry point
- test/ directory with conftest.py fixtures (enable_ros2 / disable_ros2)
- __main__.py stub that returns 0 when ENABLE_ROS2 is unset

Spec ref: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §组件模型
Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 2: `config_sensors.yml` schema + loader

**Files:**
- Create: `config_sensors.yml` (root of repo)
- Create: `ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/config_loader.py`
- Create: `ros2_ws/src/vehicle_wbt_platform/test/test_config_loader.py`

**Interfaces:**
- Produces: `ConfigRegistry` class with `load(path) -> ConfigRegistry` class method
- Produces: `ConfigRegistry.sensors: dict[str, SensorConfig]`
- Produces: `ConfigRegistry.actuators: dict[str, ActuatorConfig]`
- Produces: `SensorConfig` / `ActuatorConfig` dataclasses with fields: id, type, port_id, port_physical, topic, msg_type, rate_hz, enabled, type_specific: dict

- [ ] **Step 1: Write failing tests**

Create `ros2_ws/src/vehicle_wbt_platform/test/test_config_loader.py`:

```python
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
  - {id: dup, type: ir, port_id: 1, port_physical: P1, topic: /x, msg_type: std_msgs/Int32, rate_hz: 10}
  - {id: dup, type: ir, port_id: 2, port_physical: P2, topic: /y, msg_type: std_msgs/Int32, rate_hz: 10}
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
```

- [ ] **Step 2: Run tests to verify they all fail**

Run: `cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python -m pytest test/test_config_loader.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_wbt_platform.config_loader'`

- [ ] **Step 3: Write `config_sensors.yml` (root of repo)**

```yaml
# vehicle_wbt Platform — Sensors/Actuators Registry
#
# Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §配置系统
#
# This file is the single source of truth for what hardware is connected.
# Adding new hardware = adding one entry here + one URDF link.
# Sidecar reads this file at startup and spawns corresponding ROS2 nodes.

sensors:
  # ---- Currently wired (v1 enabled) ----
  - id: left_ir
    type: ir
    port_id: 8
    port_physical: P8
    topic: /vehicle_wbt/v1/sensors/ir/left
    msg_type: std_msgs/Float32
    rate_hz: 20
    enabled: true

  - id: right_ir
    type: ir
    port_id: 7
    port_physical: P7
    topic: /vehicle_wbt/v1/sensors/ir/right
    msg_type: std_msgs/Float32
    rate_hz: 20
    enabled: true

  - id: vert_limit
    type: analog_input
    port_id: 6
    port_physical: P6
    topic: /vehicle_wbt/v1/sensors/analog/vert_limit
    msg_type: std_msgs/Int32
    rate_hz: 50
    invert: false
    threshold: 1000
    enabled: true   # sensor wired, mechanical mount pending per hardware-port-mapping.md

  # ---- Reserved (v1 disabled, hardware not wired) ----
  - id: ultrasonic_front
    type: ultrasonic
    port_id: 5
    port_physical: P5
    topic: /vehicle_wbt/v1/sensors/ultrasonic/front
    msg_type: std_msgs/Float32
    rate_hz: 10
    enabled: false

actuators:
  # ---- Currently wired (v1 enabled) ----
  - id: motor_ejection
    type: motor
    port_id: 5
    port_physical: M5
    topic: /vehicle_wbt/v1/actuators/motor/m5/state
    perimeter: 0.032
    reverse: -1
    enabled: true

  - id: motor_arm_horiz
    type: motor
    port_id: 6
    port_physical: M6
    topic: /vehicle_wbt/v1/actuators/motor/m6/state
    perimeter: 0.032
    reverse: -1
    enabled: true

  - id: stepper_ejection_angle
    type: stepper
    port_id: 1
    port_physical: STEP1
    topic: /vehicle_wbt/v1/actuators/stepper/1/state
    perimeter: 0.008
    stepper2rad: 0.001963   # math.pi/180 * 1.8 / 16
    enabled: true

  - id: stepper_arm_vert
    type: stepper
    port_id: 3
    port_physical: STEP3
    topic: /vehicle_wbt/v1/actuators/stepper/3/state
    perimeter: 0.008
    stepper2rad: 0.001963
    reverse: -1
    enabled: true

  - id: servo_hand_rotate
    type: servo_bus
    port_id: 3
    port_physical: S3
    topic: /vehicle_wbt/v1/actuators/servo/s3/state
    enabled: true

  - id: servo_hand_grip
    type: servo_pwm
    port_id: 7
    port_physical: S7
    mode: 270
    topic: /vehicle_wbt/v1/actuators/servo/s7/state
    enabled: true

  - id: servo_weather
    type: servo_bus
    port_id: 2
    port_physical: S2
    topic: /vehicle_wbt/v1/actuators/servo/s2/state
    enabled: true

  - id: vacuum_pump
    type: dout
    port_id: 2
    port_physical: P2
    topic: /vehicle_wbt/v1/actuators/io/p2/state
    enabled: true

  - id: vacuum_valve
    type: dout
    port_id: 3
    port_physical: P3
    topic: /vehicle_wbt/v1/actuators/io/p3/state
    enabled: true

  - id: ejection_valve
    type: dout
    port_id: 4
    port_physical: P4
    topic: /vehicle_wbt/v1/actuators/io/p4/state
    enabled: true

  # ---- Reserved (v1 disabled) ----
  - id: stepper_2
    type: stepper
    port_id: 2
    port_physical: STEP2
    topic: /vehicle_wbt/v1/actuators/stepper/2/state
    perimeter: 0.008
    stepper2rad: 0.001963
    enabled: false

  - id: servo_s1_reserved
    type: servo_bus
    port_id: 1
    port_physical: S1
    topic: /vehicle_wbt/v1/actuators/servo/s1/state
    enabled: false

  - id: servo_s4_reserved
    type: servo_pwm
    port_id: 4
    port_physical: S4
    mode: 270
    topic: /vehicle_wbt/v1/actuators/servo/s4/state
    enabled: false
```

- [ ] **Step 4: Write `config_loader.py`**

```python
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python -m pytest test/test_config_loader.py -v`
Expected: PASS (7 tests)

- [ ] **Step 6: Verify `config_sensors.yml` is valid**

Run: `cd /home/xrak/Desktop/rak-car && PYTHONPATH=ros2_ws/src/vehicle_wbt_platform python -c "from vehicle_wbt_platform.config_loader import load_registry; r = load_registry('config_sensors.yml'); print('sensors:', len(r.sensors), 'actuators:', len(r.actuators), 'enabled:', len(r.enabled_sensors()), len(r.enabled_actuators()))"`
Expected: `sensors: 3 actuators: 14 enabled: 3 11`

- [ ] **Step 7: Commit**

```bash
git add config_sensors.yml ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/config_loader.py ros2_ws/src/vehicle_wbt_platform/test/test_config_loader.py
git commit -m "feat(phase1): config_sensors.yml schema + typed loader

Adds:
- config_sensors.yml — single source of truth for wired + reserved hardware
  - 3 sensors enabled (left_ir, right_ir, vert_limit)
  - 11 actuators enabled (motors/steppers/servos/vacuum/ejection)
  - 4 reserved entries (stepper_2, ultrasonic, S1, S4) with enabled: false
- config_loader.py — ConfigRegistry + SensorConfig + ActuatorConfig dataclasses
  with strict schema validation: required fields, /v1/ namespace, no duplicate ids
- test_config_loader.py — 7 pytest cases covering all validation paths

Adding new hardware = add 1 YAML line + 1 URDF link. No business code touched.

Spec ref: §配置系统
Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 3: `BaseComponent` abstract class with lifecycle contract

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/component_base.py`
- Create: `ros2_ws/src/vehicle_wbt_platform/test/test_component_base.py`

**Interfaces:**
- Produces: `BaseComponent` ABC with `init(context)`, `start()`, `stop()`, `cleanup()`, `health_status()` methods
- Produces: `ComponentContext` dataclass holding node, registry, controller_adapter
- Produces: `HealthStatus` dataclass with `state: enum` (BOOTING/RUNNING/ERROR/STOPPED) + `details: dict`

- [ ] **Step 1: Write failing tests**

Create `ros2_ws/src/vehicle_wbt_platform/test/test_component_base.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python -m pytest test/test_component_base.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_wbt_platform.component_base'`

- [ ] **Step 3: Write `component_base.py`**

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python -m pytest test/test_component_base.py -v`
Expected: PASS (6 tests)

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/component_base.py ros2_ws/src/vehicle_wbt_platform/test/test_component_base.py
git commit -m "feat(phase1): BaseComponent abstract class + HealthStatus

Adds:
- BaseComponent ABC with 5 lifecycle methods (init/start/stop/cleanup/health_status)
- HealthState enum (BOOTING/RUNNING/ERROR/STOPPED) — maps to diagnostic_msgs level
- ComponentContext dataclass — dependencies injected at init (node, registry,
  controller, config path, ROS_DOMAIN_ID)
- Protocol classes (_NodeLike/_RegistryLike/_ControllerLike) to keep the
  contract explicit without forcing rclpy import in unit tests
- 6 pytest cases covering lifecycle flow, abstract instantiation guard,
  context shape, and health enum

Spec ref: §组件模型
Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 4: `BaseControllerHardwareInterface` + `MC602Adapter` skeleton

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/controller_base.py`
- Create: `ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/mc602_adapter.py`
- Create: `ros2_ws/src/vehicle_wbt_platform/test/test_mc602_adapter.py`

**Interfaces:**
- Produces: `BaseControllerHardwareInterface` ABC with `read_sensor(port_id, sensor_type)`, `write_actuator(port_id, actuator_type, value)`, `enumerate_ports() -> dict`
- Produces: `MC602Adapter` implementing the interface; uses existing `controller_wrap.MC602` under the hood
- Produces: `ControllerError` exception

- [ ] **Step 1: Write failing tests**

Create `ros2_ws/src/vehicle_wbt_platform/test/test_mc602_adapter.py`:

```python
from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import pytest

from vehicle_wbt_platform.controller_base import (
    BaseControllerHardwareInterface,
    ControllerError,
)
from vehicle_wbt_platform.mc602_adapter import MC602Adapter


def test_base_cannot_be_instantiated_directly() -> None:
    with pytest.raises(TypeError):
        BaseControllerHardwareInterface()  # type: ignore[abstract]


def test_mc602_adapter_init_with_serial_path() -> None:
    adapter = MC602Adapter(serial_port="/dev/ttyUSB0", baud=1000000)
    assert adapter.serial_port == "/dev/ttyUSB0"
    assert adapter.baud == 1000000
    assert not adapter.is_open


def test_mc602_adapter_open_close() -> None:
    adapter = MC602Adapter(serial_port="/dev/ttyUSB0", baud=1000000)
    adapter.open()
    try:
        assert adapter.is_open
    finally:
        adapter.close()
    assert not adapter.is_open


def test_mc602_adapter_read_ir_returns_float() -> None:
    adapter = MC602Adapter(serial_port="/dev/ttyUSB0", baud=1000000)
    adapter.open()
    try:
        # Stub the underlying MC602 read; we don't want real hardware in tests.
        adapter._mc602 = MagicMock()
        adapter._mc602.infrared_read.return_value = 0.234
        val = adapter.read_sensor(port_id=7, sensor_type="ir")
        assert val == pytest.approx(0.234)
    finally:
        adapter.close()


def test_mc602_adapter_read_unknown_sensor_type_raises() -> None:
    adapter = MC602Adapter(serial_port="/dev/ttyUSB0", baud=1000000)
    adapter.open()
    try:
        with pytest.raises(ControllerError, match="unsupported sensor type"):
            adapter.read_sensor(port_id=1, sensor_type="tachyon_beam")
    finally:
        adapter.close()


def test_mc602_adapter_enumerate_ports() -> None:
    adapter = MC602Adapter(serial_port="/dev/ttyUSB0", baud=1000000)
    ports = adapter.enumerate_ports()
    assert "motor" in ports and ports["motor"] == 6
    assert "servo" in ports and ports["servo"] == 7
    assert "stepper" in ports and ports["stepper"] == 3
    assert "io" in ports and ports["io"] == 8


def test_mc602_adapter_write_actuator_idempotent_under_double_call() -> None:
    adapter = MC602Adapter(serial_port="/dev/ttyUSB0", baud=1000000)
    adapter.open()
    try:
        adapter._mc602 = MagicMock()
        adapter.write_actuator(port_id=5, actuator_type="motor", value=0.5)
        adapter.write_actuator(port_id=5, actuator_type="motor", value=0.5)
        assert adapter._mc602.motor_set_speed.call_count == 2
    finally:
        adapter.close()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python -m pytest test/test_mc602_adapter.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_wbt_platform.controller_base'`

- [ ] **Step 3: Write `controller_base.py`**

```python
"""BaseControllerHardwareInterface — generic controller adapter contract.

Spec ref: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §硬件接口插件

Every controller family (MC601, MC602, future MCxxx) is one adapter subclass.
Adding a new controller = writing one new adapter file in
ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/ and registering it in
the sidecar entry point (Task 6). No changes to business code.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class ControllerError(RuntimeError):
    """Raised when a controller adapter cannot complete read/write."""


class BaseControllerHardwareInterface(ABC):
    """Abstract interface every controller adapter must implement.

    Sidecar components depend only on this interface, not on MCxxx-specific
    classes. Tests can substitute a fake without touching real hardware.
    """

    @property
    @abstractmethod
    def is_open(self) -> bool:
        """True if the underlying serial port / bus is open."""

    @property
    @abstractmethod
    def serial_port(self) -> str:
        """Path to the serial device (e.g., /dev/ttyUSB0) for diagnostics."""

    @property
    @abstractmethod
    def baud(self) -> int:
        """Baud rate the adapter opened at."""

    @abstractmethod
    def open(self) -> None:
        """Open the serial port and run any controller self-test. Idempotent."""

    @abstractmethod
    def close(self) -> None:
        """Close the serial port. Idempotent. Safe to call after exception."""

    @abstractmethod
    def read_sensor(self, *, port_id: int, sensor_type: str) -> float:
        """Read a sensor value. Returns the value in physical units (m, V, ...)."""

    @abstractmethod
    def write_actuator(self, *, port_id: int, actuator_type: str, value: float) -> None:
        """Write a value to an actuator. Idempotent under repeated calls."""

    @abstractmethod
    def enumerate_ports(self) -> dict[str, int]:
        """Return a dict of port-type -> count, e.g. {'motor': 6, 'servo': 7, ...}."""
```

- [ ] **Step 4: Write `mc602_adapter.py`**

```python
"""MC602Adapter — concrete adapter for Waveshare MC602 motor controller.

Spec ref: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §硬件接口插件

This adapter wraps the existing `vehicle.controller_wrap.MC602` class — it does
NOT reimplement the protocol. It exposes the BaseControllerHardwareInterface
contract so sidecar components stay controller-agnostic.

Adding a future controller (e.g., MC603) = copy this file, change the wrapped
class, add a new console_script entry in setup.py. No sidecar code changes.
"""

from __future__ import annotations

import logging
from typing import Any, Optional

from vehicle_wbt_platform.controller_base import (
    BaseControllerHardwareInterface,
    ControllerError,
)


_logger = logging.getLogger(__name__)


# Map of sensor_type strings to MC602 method names on the underlying controller_wrap
# wrapper. Extending this map is how new sensor modes get supported.
_SENSOR_TYPE_TO_METHOD = {
    "ir": "infrared_read",
    "analog_input": "analog_input_read",
    "ultrasonic": "ultrasonic_read",
    "touch": "touch_read",
    "ambient_light": "ambient_light_read",
}

# Map of actuator_type strings to MC602 write method names.
_ACTUATOR_TYPE_TO_METHOD = {
    "motor": "motor_set_speed",
    "servo_bus": "servo_bus_set",
    "servo_pwm": "servo_pwm_set",
    "stepper": "stepper_goto",
    "dout": "dout_set",
}


class MC602Adapter(BaseControllerHardwareInterface):
    """Adapter for MC602 over CH340 USB serial."""

    # MC602 physical port counts (per hardware-port-mapping.md)
    _MOTOR_MAX = 6
    _SERVO_MAX = 7
    _STEPPER_MAX = 3
    _IO_MAX = 8

    def __init__(self, *, serial_port: str, baud: int = 1_000_000) -> None:
        if baud not in (380_400, 1_000_000, 115_200):
            raise ControllerError(
                f"unsupported baud {baud}; MC602 supports 380400/1000000/115200"
            )
        self._serial_port = serial_port
        self._baud = baud
        self._mc602: Optional[Any] = None  # lazy — actual MC602 wrapper from vehicle

    # --- BaseControllerHardwareInterface ---

    @property
    def is_open(self) -> bool:
        return self._mc602 is not None

    @property
    def serial_port(self) -> str:
        return self._serial_port

    @property
    def baud(self) -> int:
        return self._baud

    def open(self) -> None:
        if self.is_open:
            return
        # Lazy import to keep this module testable without vehicle package.
        try:
            from vehicle.base.controller_wrap import MC602 as _MC602  # type: ignore
        except ImportError as e:
            raise ControllerError(
                f"cannot import vehicle.base.controller_wrap.MC602 — is vehicle package on PYTHONPATH? {e}"
            )
        self._mc602 = _MC602(port=self._serial_port, baud=self._baud)
        _logger.info("MC602Adapter opened %s @ %d", self._serial_port, self._baud)

    def close(self) -> None:
        if self._mc602 is None:
            return
        try:
            self._mc602.close()
        except Exception as e:  # noqa: BLE001 — close() must not raise on already-broken HW
            _logger.warning("MC602 close raised %r, ignoring", e)
        self._mc602 = None

    def read_sensor(self, *, port_id: int, sensor_type: str) -> float:
        self._require_open()
        method_name = _SENSOR_TYPE_TO_METHOD.get(sensor_type)
        if method_name is None:
            raise ControllerError(
                f"unsupported sensor type {sensor_type!r}; "
                f"supported: {sorted(_SENSOR_TYPE_TO_METHOD)}"
            )
        method = getattr(self._mc602, method_name, None)
        if method is None:
            raise ControllerError(
                f"MC602 has no method {method_name!r} (sensor type {sensor_type!r})"
            )
        return float(method(port_id=port_id))

    def write_actuator(self, *, port_id: int, actuator_type: str, value: float) -> None:
        self._require_open()
        method_name = _ACTUATOR_TYPE_TO_METHOD.get(actuator_type)
        if method_name is None:
            raise ControllerError(
                f"unsupported actuator type {actuator_type!r}; "
                f"supported: {sorted(_ACTUATOR_TYPE_TO_METHOD)}"
            )
        method = getattr(self._mc602, method_name, None)
        if method is None:
            raise ControllerError(
                f"MC602 has no method {method_name!r} (actuator type {actuator_type!r})"
            )
        method(port_id=port_id, value=float(value))

    def enumerate_ports(self) -> dict[str, int]:
        return {
            "motor": self._MOTOR_MAX,
            "servo": self._SERVO_MAX,
            "stepper": self._STEPPER_MAX,
            "io": self._IO_MAX,
        }

    # --- internals ---

    def _require_open(self) -> None:
        if not self.is_open:
            raise ControllerError("MC602Adapter not open; call open() first")
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python -m pytest test/test_mc602_adapter.py -v`
Expected: PASS (7 tests)

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/controller_base.py ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/mc602_adapter.py ros2_ws/src/vehicle_wbt_platform/test/test_mc602_adapter.py
git commit -m "feat(phase1): BaseControllerHardwareInterface + MC602Adapter

Adds:
- BaseControllerHardwareInterface ABC — generic adapter contract covering
  MC601/MC602/future MCxxx. Sidecar components depend only on this interface.
- MC602Adapter — concrete wrapper around existing vehicle.controller_wrap.MC602.
  Lazy-imports the underlying class to keep tests hardware-free.
- 7 pytest cases covering abstract guard, lifecycle, port enumeration,
  sensor/actuator method dispatch, and unsupported-type errors.

Spec ref: §硬件接口插件
Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 5: `BaseChassis` abstract class (chassis-agnostic foundation)

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/chassis_base.py`
- Create: `ros2_ws/src/vehicle_wbt_platform/test/test_chassis_base.py`

**Interfaces:**
- Produces: `BaseChassis` ABC with `set_velocity(vx, vy, omega)`, `get_pose() -> Pose2D`, `reset_odometry()`, `forward_kinematics(wheel_speeds)`, `inverse_kinematics(vx, vy, omega) -> wheel_speeds`
- Produces: `Pose2D` dataclass (x, y, theta)
- Produces: `WheelSpeeds` dataclass (per-wheel velocity, N-dim)

- [ ] **Step 1: Write failing tests**

Create `ros2_ws/src/vehicle_wbt_platform/test/test_chassis_base.py`:

```python
from __future__ import annotations

import math
from typing import Sequence

import pytest

from vehicle_wbt_platform.chassis_base import (
    BaseChassis,
    ChassisError,
    Pose2D,
    WheelSpeeds,
)


def test_base_chassis_cannot_be_instantiated_directly() -> None:
    with pytest.raises(TypeError):
        BaseChassis()  # type: ignore[abstract]


def test_pose2d_construction() -> None:
    p = Pose2D(x=1.0, y=2.0, theta=math.pi / 2)
    assert p.x == 1.0
    assert p.theta == pytest.approx(math.pi / 2)


def test_wheel_speeds_construction() -> None:
    ws = WheelSpeeds(values=(0.1, -0.1, 0.2, -0.2))
    assert len(ws.values) == 4
    assert ws.values[0] == 0.1


class DummyChassis(BaseChassis):
    """4-wheel abstract chassis for testing the contract only."""

    @property
    def num_wheels(self) -> int:
        return 4

    def set_velocity(self, vx: float, vy: float, omega: float) -> None:
        self._last_velocity = (vx, vy, omega)

    def get_pose(self) -> Pose2D:
        return self._pose

    def reset_odometry(self) -> None:
        self._pose = Pose2D(0.0, 0.0, 0.0)

    def forward_kinematics(self, wheel_speeds: WheelSpeeds) -> tuple[float, float, float]:
        return (0.0, 0.0, 0.0)

    def inverse_kinematics(self, vx: float, vy: float, omega: float) -> WheelSpeeds:
        return WheelSpeeds(values=(0.0,) * self.num_wheels)


def test_dummy_chassis_lifecycle() -> None:
    c = DummyChassis(chassis_id="dummy")
    assert c.chassis_id == "dummy"
    assert c.num_wheels == 4
    c.reset_odometry()
    p = c.get_pose()
    assert p.x == 0.0 and p.y == 0.0
    c.set_velocity(0.1, 0.0, 0.0)
    assert c._last_velocity == (0.1, 0.0, 0.0)
    ws = c.inverse_kinematics(0.1, 0.0, 0.0)
    assert len(ws.values) == 4


def test_dummy_chassis_id_required() -> None:
    with pytest.raises((TypeError, ChassisError)):
        DummyChassis()  # type: ignore[call-arg]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python -m pytest test/test_chassis_base.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_wbt_platform.chassis_base'`

- [ ] **Step 3: Write `chassis_base.py`**

```python
"""BaseChassis — chassis-agnostic abstract class.

Spec ref: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §Chassis 抽象

5 concrete subclasses planned for v1:
- MecanumChassis  (current car — 4 mecanum wheels, O layout)
- Diff2Chassis    (2-wheel differential)
- Diff4Chassis    (4-wheel differential, no sideways)
- TricycleChassis (2 drive + 1 steer)
- QuadricycleChassis (4 drive + 4 steer)

Only BaseChassis is implemented in Phase 1. Subclasses land in Phase 2.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple


class ChassisError(RuntimeError):
    """Raised when a chassis operation cannot be performed (e.g., bad kinematics input)."""


@dataclass(frozen=True)
class Pose2D:
    """2D pose in the odom frame — x/y in meters, theta in radians."""

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class WheelSpeeds:
    """Per-wheel angular velocity. Length == chassis.num_wheels.

    Order is chassis-specific:
    - Mecanum 4-wheel: (front_left, front_right, rear_left, rear_right)
    - Diff2: (left, right)
    - Diff4: (front_left, front_right, rear_left, rear_right)
    - Tricycle: (left, right, steer)
    - Quadricycle: (fl, fr, rl, rr)
    """

    values: Tuple[float, ...]

    def __post_init__(self) -> None:
        if not self.values or len(self.values) < 1:
            raise ChassisError(f"WheelSpeeds must have at least 1 wheel, got {len(self.values)}")


class BaseChassis(ABC):
    """Abstract base class for any chassis topology.

    Subclasses MUST:
    - implement the 5 abstract methods below
    - set num_wheels property
    - call super().__init__(chassis_id=...) in their __init__

    The base class is intentionally minimal — no kinematics assumptions baked in.
    """

    def __init__(self, *, chassis_id: str) -> None:
        if not chassis_id or not isinstance(chassis_id, str):
            raise ChassisError(f"chassis_id must be non-empty string, got {chassis_id!r}")
        self._chassis_id = chassis_id
        self._pose = Pose2D(0.0, 0.0, 0.0)

    @property
    def chassis_id(self) -> str:
        return self._chassis_id

    @property
    def pose(self) -> Pose2D:
        return self._pose

    @property
    @abstractmethod
    def num_wheels(self) -> int:
        """Number of driven wheels (excludes passive castors)."""

    @abstractmethod
    def set_velocity(self, vx: float, vy: float, omega: float) -> None:
        """Command body-frame velocity. m/s for vx/vy, rad/s for omega."""

    @abstractmethod
    def get_pose(self) -> Pose2D:
        """Return current odometry pose."""

    @abstractmethod
    def reset_odometry(self) -> None:
        """Reset pose to (0, 0, 0). Use after physical relocation."""

    @abstractmethod
    def forward_kinematics(self, wheel_speeds: WheelSpeeds) -> Tuple[float, float, float]:
        """Convert per-wheel speeds to body-frame (vx, vy, omega)."""

    @abstractmethod
    def inverse_kinematics(self, vx: float, vy: float, omega: float) -> WheelSpeeds:
        """Convert body-frame velocity to per-wheel speeds."""
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python -m pytest test/test_chassis_base.py -v`
Expected: PASS (6 tests)

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/chassis_base.py ros2_ws/src/vehicle_wbt_platform/test/test_chassis_base.py
git commit -m "feat(phase1): BaseChassis abstract class

Adds the chassis-agnostic foundation that future Mecanum/Diff2/Diff4/
Tricycle/Quadricycle subclasses will extend. Phase 1 ships only the abstract
class + Pose2D + WheelSpeeds dataclasses + 6 pytest cases covering the
contract. Subclasses land in Phase 2 (Plan B).

Spec ref: §Chassis 抽象
Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 6: `__main__.py` orchestrator + sidecar entry point

**Files:**
- Modify: `ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/__main__.py`
- Create: `ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/orchestrator.py`
- Create: `ros2_ws/src/vehicle_wbt_platform/test/test_orchestrator.py`

**Interfaces:**
- Produces: `SidecarOrchestrator` class with `run()` method that loads config, instantiates MC602Adapter, and (in future) spawns components
- Produces: `__main__.py` that reads `ENABLE_ROS2` env var, prints status, returns 0/1

- [ ] **Step 1: Write failing tests**

Create `ros2_ws/src/vehicle_wbt_platform/test/test_orchestrator.py`:

```python
from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

import pytest

from vehicle_wbt_platform.orchestrator import SidecarOrchestrator


SAMPLE_YAML = """
sensors:
  - {id: ir1, type: ir, port_id: 7, port_physical: P7, topic: /vehicle_wbt/v1/sensors/ir/left, msg_type: std_msgs/Float32, rate_hz: 20}
actuators: []
"""


def test_orchestrator_init_loads_config(tmp_path: Path) -> None:
    cfg = tmp_path / "cfg.yml"
    cfg.write_text(SAMPLE_YAML)
    orch = SidecarOrchestrator(
        config_path=str(cfg),
        serial_port="/dev/ttyUSB0",
        ros_domain_id=42,
    )
    assert orch.registry is not None
    assert "ir1" in orch.registry.sensors


def test_orchestrator_init_creates_adapter() -> None:
    orch = SidecarOrchestrator(
        config_path="config_sensors.yml",
        serial_port="/dev/ttyUSB0",
        ros_domain_id=42,
    )
    assert orch.adapter is not None
    assert orch.adapter.serial_port == "/dev/ttyUSB0"


def test_orchestrator_adapter_not_open_at_init() -> None:
    orch = SidecarOrchestrator(
        config_path="config_sensors.yml",
        serial_port="/dev/ttyUSB0",
        ros_domain_id=42,
    )
    assert not orch.adapter.is_open


def test_orchestrator_shutdown_is_safe_to_call_twice() -> None:
    orch = SidecarOrchestrator(
        config_path="config_sensors.yml",
        serial_port="/dev/ttyUSB0",
        ros_domain_id=42,
    )
    orch.shutdown()
    orch.shutdown()  # must not raise


def test_orchestrator_summary_reports_counts() -> None:
    orch = SidecarOrchestrator(
        config_path="config_sensors.yml",
        serial_port="/dev/ttyUSB0",
        ros_domain_id=42,
    )
    summary = orch.summary()
    assert "sensors=" in summary
    assert "actuators=" in summary
    assert "enabled=" in summary
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python -m pytest test/test_orchestrator.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'vehicle_wbt_platform.orchestrator'`

- [ ] **Step 3: Write `orchestrator.py`**

```python
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
```

- [ ] **Step 4: Modify `__main__.py` to use orchestrator**

```python
"""sidecar entry point — full orchestrator integration.

When ENABLE_ROS2 is not set, the sidecar returns 0 immediately without
importing any ROS2 modules, keeping main/qqq.py behavior byte-identical.
When ENABLE_ROS2=1, it instantiates the orchestrator and prints a status
line. Actual component spawning lands in Phase 2.
"""

import logging
import os
import sys
from pathlib import Path


# Repo layout: sidecar lives in ros2_ws/, but config_sensors.yml is at repo root.
_DEFAULT_CONFIG = str(Path(__file__).resolve().parents[3] / "config_sensors.yml")


def main() -> int:
    enable = os.environ.get("ENABLE_ROS2", "").lower() in {"1", "true", "yes"}
    if not enable:
        print("[vehicle_wbt_platform] ENABLE_ROS2 not set, exiting cleanly")
        return 0

    logging.basicConfig(
        level=os.environ.get("ROS_LOG_LEVEL", "INFO"),
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
        stream=sys.stderr,
    )

    from vehicle_wbt_platform.orchestrator import SidecarOrchestrator

    config_path = os.environ.get("VEHICLE_WBT_CONFIG", _DEFAULT_CONFIG)
    serial_port = os.environ.get("VEHICLE_WBT_SERIAL", "/dev/ttyUSB0")
    ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "42"))

    orch = SidecarOrchestrator(
        config_path=config_path,
        serial_port=serial_port,
        ros_domain_id=ros_domain_id,
    )
    try:
        orch.open_hardware()
        print(f"[vehicle_wbt_platform] sidecar ready: {orch.summary()}")
        # Phase 1: just print status and exit. Phase 2 adds rclpy.spin().
        return 0
    finally:
        orch.shutdown()


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python -m pytest test/test_orchestrator.py -v`
Expected: PASS (5 tests)

- [ ] **Step 6: Run the sidecar end-to-end**

Run A (no sidecar — must print clean exit):
```bash
cd /home/xrak/Desktop/rak-car && unset ENABLE_ROS2 && python ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/__main__.py
```
Expected: prints `[vehicle_wbt_platform] ENABLE_ROS2 not set, exiting cleanly` and exits 0.

Run B (sidecar on — must load config without real hardware):
```bash
cd /home/xrak/Desktop/rak-car && ENABLE_ROS2=1 VEHICLE_WBT_SERIAL=/dev/ttyUSB_MOCK python ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/__main__.py 2>&1 | head -10
```
Expected: prints config-load + adapter-init lines, then exits 0 (or 1 if /dev/ttyUSB_MOCK open fails, which is fine — the config-loader step happens first).

- [ ] **Step 7: Commit**

```bash
git add ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/__main__.py ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/orchestrator.py ros2_ws/src/vehicle_wbt_platform/test/test_orchestrator.py
git commit -m "feat(phase1): sidecar orchestrator + __main__ entry point

Adds:
- SidecarOrchestrator — owns ConfigRegistry + MC602Adapter, idempotent shutdown
- __main__.py — when ENABLE_ROS2 unset, returns 0 without importing rclpy
  (preserves byte-identical main/qqq.py behavior per spec §Global Constraints)
- When ENABLE_ROS2=1, loads config + opens adapter + prints status
- 5 pytest cases covering config load, adapter init, shutdown idempotency

Phase 1 ships config + adapter + orchestrator. Phase 2 will add
rclpy.spin() and dynamic component spawning from config_sensors.yml.

Spec ref: §组件模型 + §生命周期
Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Acceptance Criteria (Phase 1 done = all true)

- [ ] `pytest` passes: `cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python -m pytest test/ -v` → all green
- [ ] `ENABLE_ROS2` unset → sidecar returns 0, no rclpy imported
- [ ] `ENABLE_ROS2=1` → config loads, adapter constructs, summary prints
- [ ] `config_sensors.yml` validates: 3 enabled sensors, 11 enabled actuators, 4 reserved (disabled)
- [ ] No `import vehicle` happens in test path (adapter imports are lazy in `open()`)
- [ ] `git log --oneline` shows 6 Phase 1 commits, all squashed-mergeable to main after 2026-08-12

## What's NOT in Phase 1 (next plans)

| Section in spec | Plan | Why deferred |
|-----------------|------|--------------|
| §Chassis 抽象 — 5 concrete subclasses | Plan B (Phase 2) | Mecanum needs URDF + URDF-parsing logic; diff/trike/quad need new kinematics math |
| §Camera 抽象 — concrete Camera component | Plan B (Phase 2) | Needs `rclpy` integration with `image_transport`; tested against real /dev/cam* |
| §机械臂抽象 — Arm component | Plan B (Phase 2) | Needs trajectory_msgs Action + arm_cfg.yaml integration |
| §安全设计 — 4-layer safety_gate | Plan C (Phase 3) | Needs full lifecycle integration + heartbeat protocol + mode state machine |
| §远程接入 — DDS / cyclonedds.xml | Plan C (Phase 3) | Network-level config, not part of platform skeleton |
| §生命周期 — systemd integration | Plan C (Phase 3) | After components spawn (Plan B) — we know what to start |
| §仿真回路 — Gazebo + URDF + ros2_control | Plan D (Phase 4) | Requires URDF stable + components implemented first |
| §测试策略 — sim-to-real gap | Plan D (Phase 4) | After Gazebo integration |

Plans B/C/D will be drafted after Phase 1 ships and the orchestrator+adapter pattern proves out.
