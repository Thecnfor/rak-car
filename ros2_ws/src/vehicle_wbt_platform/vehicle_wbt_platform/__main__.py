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
