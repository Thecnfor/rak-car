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
_DEFAULT_CONFIG = str(Path(__file__).resolve().parents[4] / "config_sensors.yml")
_DEFAULT_CONFIG_DIR = str(Path(_DEFAULT_CONFIG).resolve().parent)
_DEFAULT_SERIAL = "/dev/ttyUSB0"
_DEFAULT_DOMAIN_ID = 42
# ROS2 DDS domain id range is 0..232 (per ROS2 docs).
_MIN_DOMAIN_ID = 0
_MAX_DOMAIN_ID = 232


def _env_int(name: str, default: int, *, lo: int, hi: int) -> int:
    """Read an integer env var, validating range. Fail-CLOSED: bad value -> SystemExit."""
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        val = int(raw)
    except ValueError:
        raise SystemExit(f"{name} must be an integer, got {raw!r}")
    if not (lo <= val <= hi):
        raise SystemExit(f"{name} must be in [{lo}, {hi}], got {val}")
    return val


def _validate_config_path(p: str) -> str:
    """Resolve the config path and reject anything outside the repo config dir.

    Prevents VEHICLE_WBT_CONFIG=/etc/passwd style escapes.
    """
    resolved = Path(p).resolve()
    if not str(resolved).startswith(_DEFAULT_CONFIG_DIR + os.sep):
        raise SystemExit(
            f"VEHICLE_WBT_CONFIG must be inside {_DEFAULT_CONFIG_DIR!r}, got {p!r}"
        )
    if not resolved.is_file():
        raise SystemExit(f"VEHICLE_WBT_CONFIG file does not exist: {p!r}")
    return str(resolved)


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

    config_path = _validate_config_path(
        os.environ.get("VEHICLE_WBT_CONFIG", _DEFAULT_CONFIG)
    )
    serial_port = os.environ.get("VEHICLE_WBT_SERIAL", _DEFAULT_SERIAL)
    ros_domain_id = _env_int("ROS_DOMAIN_ID", _DEFAULT_DOMAIN_ID,
                             lo=_MIN_DOMAIN_ID, hi=_MAX_DOMAIN_ID)

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