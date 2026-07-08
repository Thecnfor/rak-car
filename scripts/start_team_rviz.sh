#!/usr/bin/env bash
# start_team_rviz.sh — one-click RViz viewing of the Jetson's cameras.
#
# Run from any teammate's laptop on the same LAN as the Jetson:
#
#   ./scripts/start_team_rviz.sh
#
# Does:
#   1. Sets ROS_DOMAIN_ID=42 (team-wide convention; matches what
#      full_system.launch.py enforces on the Jetson side).
#   2. Detects ROS2 distro (humble/jazzy/iron) and sources its setup.bash.
#   3. Installs CycloneDDS config on first run (sudo), so DDS discovery
#      uses the same XML across all machines.
#   4. Sources this repo's install/setup.bash if already built.
#   5. Sanity-checks that Jetson's camera nodes are visible.
#   6. Launches RViz2 with a pre-built layout pointing at the live
#      image_compressed topics.
#
# The pre-built layout is at:
#   ros2_ws/src/vehicle_wbt_platform_cpp/config/team_cameras.rviz
# Customise by opening RViz2, editing the layout, File → Save Config As.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RVIZ_CONFIG="${REPO_ROOT}/ros2_ws/src/vehicle_wbt_platform_cpp/config/team_cameras.rviz"
DDS_CONFIG_SRC="${REPO_ROOT}/ros2_ws/src/vehicle_wbt_platform_cpp/config/cyclonedds.xml"

# 1. Domain
export ROS_DOMAIN_ID=42

# 2. ROS2 distro auto-detect
# Order matters: prefer humble (Jetson default) → jazzy → lyrical (dev box) → older
# See docs/development/dev-target-architecture.md for distro choice rationale.
ros_active=0
for distro in humble jazzy lyrical iron foxy galactic; do
  if [ -f "/opt/ros/${distro}/setup.bash" ]; then
    # Temporarily disable `set -u`: Lyrical's setup.bash checks
    # $AMENT_TRACE_SETUP_FILES without an explicit default, which trips
    # `set -u` (unbound variable) even when the variable is intentionally empty.
    set +u
    source "/opt/ros/${distro}/setup.bash"
    set -u
    echo "✅ ROS2 ${distro} loaded"
    ros_active=1
    break
  fi
done
if [ "$ros_active" = 0 ]; then
  echo "❌ No /opt/ros/{humble,jazzy,lyrical,iron,foxy,galactic}/setup.bash found." >&2
  echo "   Install ROS2 first (https://docs.ros.org/en/${distro:-humble}/Installation.html)" >&2
  exit 1
fi

# 3. CycloneDDS config — install once per laptop
DDS_USER_CFG=~/.ros/cyclonedds.xml
DDS_SYSTEM_CFG=/etc/cyclonedds/cyclonedds.xml
if [ ! -f "$DDS_USER_CFG" ] && [ ! -f "$DDS_SYSTEM_CFG" ]; then
  if [ -f "$DDS_CONFIG_SRC" ]; then
    echo "📥 First-time setup: installing CycloneDDS config (one-time)"
    mkdir -p ~/.ros
    cp "$DDS_CONFIG_SRC" "$DDS_USER_CFG"
    echo "   → ${DDS_USER_CFG}"
  else
    echo "⚠️  CycloneDDS config not found at $DDS_CONFIG_SRC"
    echo "   (likely a fresh checkout — using DDS defaults)"
  fi
fi

# 4. Project workspace (if installed)
WORKSPACE="${REPO_ROOT}/ros2_ws"
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source "${WORKSPACE}/install/setup.bash"
fi

# 5. Sanity-check discovery
echo ""
echo "🔎 Hunting for Jetson camera nodes on ROS_DOMAIN_ID=$ROS_DOMAIN_ID ..."
if timeout 6 ros2 node list 2>/dev/null | grep -E "camera_(arm|front)" >/dev/null; then
  echo "✅ Found:"
  timeout 4 ros2 node list 2>/dev/null | grep -E "camera|mecanum|arm_main" | sed 's/^/   /'
else
  echo "⚠️  Couldn't see camera nodes within 6s. Check:"
  echo "   - Same WiFi/LAN as the Jetson (e.g. ping orin)"
  echo "   - Jetson is running:   ssh xrak@orin 'ros2 node list'"
  echo ""
  read -rp "Open RViz2 anyway? [y/N] " yn
  case "$yn" in
    [yY]*) ;;
    *) echo "Aborted."; exit 1 ;;
  esac
fi

echo ""
echo "🚀 Launching RViz2 with team_cameras.rviz ..."
echo "   Topics visible:"
echo "     /vehicle_wbt/v1/sensors/camera/{front,arm}/image_compressed"
echo "     /vehicle_wbt/v1/sensors/camera/{front,arm}/camera_meta"
echo "     /vehicle_wbt/v1/sensors/camera/{front,arm}/camera_status"
echo "     /tf_static (base_link → *_camera_optical_frame)"
echo ""
exec rviz2 -d "$RVIZ_CONFIG"
