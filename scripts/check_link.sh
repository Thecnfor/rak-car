#!/usr/bin/env bash
# scripts/check_link.sh — Verify dev box can reach Jetson's MC602 bridge.
#
# Run this on YOUR dev box. Checks (in order):
#   1. ros2 CLI installed
#   2. ROS_DOMAIN_ID set to 42
#   3. CycloneDDS configured
#   4. Jetson reachable on LAN
#   5. Jetson's /vehicle_wbt/v1/mc602/* services visible via DDS
#
# Exits 0 if all checks pass, non-zero otherwise. Use this BEFORE quick_beep.sh
# to diagnose connectivity issues.

set -uo pipefail

OK=0
FAIL=1

pass() { echo "  ✅ $1"; }
warn() { echo "  ⚠️  $1"; }
fail() { echo "  ❌ $1"; echo; echo "FAILED at: $1"; exit $FAIL; }
section() { echo; echo "── $1 ──"; }

section "1. ros2 CLI"
if command -v ros2 >/dev/null 2>&1; then
    pass "ros2 CLI found: $(ros2 --version 2>&1 | head -1)"
else
    fail "ros2 CLI not found. source /opt/ros/humble/setup.bash or install ROS2 Humble"
fi

section "2. ROS_DOMAIN_ID"
CURRENT_DOMAIN="${ROS_DOMAIN_ID:-unset}"
if [ "$CURRENT_DOMAIN" = "42" ]; then
    pass "ROS_DOMAIN_ID=42 (matches Jetson)"
else
    fail "ROS_DOMAIN_ID=$CURRENT_DOMAIN, expected 42. Run: export ROS_DOMAIN_ID=42"
fi

section "3. CycloneDDS"
if [ "${RMW_IMPLEMENTATION:-}" = "rmw_cyclonedds_cpp" ]; then
    pass "RMW=rmw_cyclonedds_cpp (matches Jetson)"
else
    warn "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}; Jetson uses CycloneDDS. Recommended: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
fi

section "4. LAN reachability"
JETSON_IP="${JETSON_IP:-192.168.3.69}"
if ping -c 1 -W 2 "$JETSON_IP" >/dev/null 2>&1; then
    pass "Jetson $JETSON_IP reachable"
else
    fail "Cannot ping $JETSON_IP. Check LAN/Wi-Fi connection."
fi

section "5. DDS service discovery"
SERVICES=$(timeout 3 ros2 service list 2>&1 || true)
if echo "$SERVICES" | grep -q '/vehicle_wbt/v1/mc602/'; then
    pass "Jetson's /vehicle_wbt/v1/mc602/* services visible:"
    echo "$SERVICES" | grep '/vehicle_wbt/v1/mc602/' | sort | sed 's/^/      /'
else
    fail "No /vehicle_wbt/v1/mc602/* services visible. Jetson's bridge is not advertising."
fi

echo
echo "✅ All checks passed. Try ./scripts/quick_beep.sh"
exit $OK