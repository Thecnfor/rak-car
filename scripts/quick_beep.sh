#!/usr/bin/env bash
# scripts/quick_beep.sh — Quick-verify: trigger a beep on Jetson's MC602.
#
# Run this on YOUR dev box (not on Jetson). No SSH needed — DDS auto-discovers
# Jetson's mc602_node on the LAN. This is the trust signal: if you hear a beep,
# the bridge is working.
#
# Usage:
#   ./scripts/quick_beep.sh                # 440 Hz, 200 ms (default)
#   ./scripts/quick_beep.sh 880 100        # 880 Hz, 100 ms
#   FREQ=200 DURATION=500 ./scripts/quick_beep.sh   # via env vars
#
# Prereqs: ros2 CLI + vehicle_wbt_smartcar_msgs built on this dev box.
# See docs/integration/DEV_QUICKSTART.md for setup.

set -euo pipefail

FREQ="${1:-${FREQ:-440}}"
DURATION_MS="${2:-${DURATION:-200}}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "❌ ros2 CLI not found. Install ROS2 Humble or source /opt/ros/humble/setup.bash" >&2
    exit 1
fi

if ! ros2 service list 2>/dev/null | grep -q '/vehicle_wbt/v1/mc602/buzzer'; then
    echo "❌ Jetson's /vehicle_wbt/v1/mc602/buzzer service is NOT visible." >&2
    echo "   Troubleshooting:" >&2
    echo "   1. Is Jetson's mc602_node running? (ssh jetson@192.168.3.69 systemctl status vehicle-wbt-mc602)" >&2
    echo "   2. Same ROS_DOMAIN_ID? (echo \$ROS_DOMAIN_ID should be 42)" >&2
    echo "   3. Same LAN? (ping 192.168.3.69)" >&2
    echo "   4. Run ./scripts/check_link.sh for a full diagnostic." >&2
    exit 2
fi

echo "🔔 Triggering ${FREQ}Hz for ${DURATION_MS}ms on Jetson..."
if ros2 service call \
    /vehicle_wbt/v1/mc602/buzzer \
    vehicle_wbt_smartcar_msgs/srv/Buzzer \
    "{freq_hz: ${FREQ}, duration_ms: ${DURATION_MS}}" \
    --timeout 5 2>&1 | grep -E 'success=|response='; then
    echo "✅ Service call succeeded. You should hear a beep on Jetson."
else
    echo "❌ Service call failed. See output above." >&2
    exit 3
fi
