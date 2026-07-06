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