#!/usr/bin/env bash
# start_team_rviz.sh — one-click RViz VIEWING of the Jetson's cameras.
#
# This script is for **viewing only**. Teammates never SSH from here.
# Control (if you ever need it for debugging) is done separately with
# `ros2 topic pub` over DDS — same channel RViz uses.
#
# Run from any teammate's laptop on the same LAN as the Jetson
# (Jetson is hard-coded at 192.168.3.69 — see docs/team-constants.md):
#
#   ./scripts/start_team_rviz.sh
#
# Does:
#   1. Sets ROS_DOMAIN_ID=42 (team-wide convention; matches what
#      full_system.launch.py enforces on the Jetson side).
#   2. Detects ROS2 distro (humble/jazzy/iron) and sources its setup.bash.
#   3. Copies CycloneDDS config to ~/.ros/ on first run (no sudo),
#      so DDS discovery uses the same XML across all machines.
#   4. Sources this repo's install/setup.bash if already built.
#   5. Sanity-checks that Jetson's camera nodes are visible (DDS only).
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
# Order matters: prefer humble (matches Jetson, safest for ABI) → jazzy → lyrical → iron
# Falls back to scanning /opt/ros/ for any distro not in the preferred list.
# See docs/team-constants.md.
ros_active=0
for distro in humble jazzy lyrical iron foxy galactic; do
  if [ -f "/opt/ros/${distro}/setup.bash" ]; then
    # Temporarily disable `set -u`: Lyrical's setup.bash checks
    # $AMENT_TRACE_SETUP_FILES without an explicit default, which trips
    # `set -u` (unbound variable) even when the variable is intentionally empty.
    set +u
    source "/opt/ros/${distro}/setup.bash"
    set -u
    ROS_DISTRO="$distro"
    echo "✅ ROS2 ${distro} loaded"
    ros_active=1
    break
  fi
done
# Fallback: scan /opt/ros/ for any distro not in the preferred list
if [ "$ros_active" = 0 ]; then
  for setup_bash in /opt/ros/*/setup.bash; do
    [ -f "$setup_bash" ] || continue
    distro=$(basename "$(dirname "$setup_bash")")
    set +u
    source "$setup_bash"
    set -u
    ROS_DISTRO="$distro"
    echo "⚠️  ROS2 ${distro} loaded (auto-detected from /opt/ros/, not in preferred list)"
    ros_active=1
    break
  done
fi
if [ "$ros_active" = 0 ]; then
  echo "❌ No ROS2 installation found in /opt/ros/." >&2
  echo "   Install ROS2 first (https://docs.ros.org/en/humble/Installation.html)" >&2
  exit 1
fi
# Warn if dev distro != Jetson's humble (still works, just FYI)
if [ "$ROS_DISTRO" != "humble" ]; then
  echo "ℹ️  dev ROS2 is '$ROS_DISTRO' (Jetson is humble). Fine for source-only dev;"
  echo "    Jetson builds its own install under Humble. See docs/team-constants.md."
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
# Same `set +u` workaround as the distro source above: colcon-generated
# install/setup.bash references $COLCON_TRACE without a default, which
# trips `set -u` on distros (e.g. Lyrical) that don't pre-set that var.
WORKSPACE="${REPO_ROOT}/ros2_ws"
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "${WORKSPACE}/install/setup.bash"
  set -u
fi

# 5. Sanity-check discovery (DDS only — no SSH from this script)
echo ""
echo "🔎 Hunting for Jetson camera nodes on ROS_DOMAIN_ID=$ROS_DOMAIN_ID ..."
if timeout 6 ros2 node list 2>/dev/null | grep -E "camera_(arm|front)" >/dev/null; then
  echo "✅ Found:"
  timeout 4 ros2 node list 2>/dev/null | grep -E "camera|mecanum|arm_main" | sed 's/^/   /'
else
  echo "⚠️  No camera nodes seen via DDS within 6s."
  echo "   (This script never SSHes — diagnosing further is *your* call,"
  echo "    not required to view cameras.)"
  echo ""
  echo "   Common causes — none of them need SSH from your laptop:"
  echo "   - Not on the team LAN / wrong subnet:"
  echo "       ping 192.168.3.69          # 0% loss = OK"
  echo "   - CycloneDDS config not in ~/.ros/cyclonedds.xml"
  echo "   - Firewall blocking UDP 7400-7500 on the Jetson"
  echo "   - Jetson sidecar may not be running — ask whoever is at the"
  echo "     car to verify. (Starting sidecar is the operator's job,"
  echo "     not the viewer's. Viewers never SSH — they just look.)"
  echo ""
  echo "   For the full 15-item health check (does SSH internally), run:"
  echo "       bash scripts/diagnose.sh"
  echo ""
  echo "   Opening RViz2 anyway — empty topic list is itself a useful clue."
fi

echo ""
echo "🚀 Launching RViz2 with team_cameras.rviz ..."
echo "   Topics visible:"
echo "     /vehicle_wbt/v1/sensors/camera/{front,arm}/image_compressed"
echo "     /vehicle_wbt/v1/sensors/camera/{front,arm}/camera_meta"
echo "     /vehicle_wbt/v1/sensors/camera/{front,arm}/camera_status"
echo "     /tf_static (base_link → *_camera_optical_frame)"
echo ""
# Workaround for XWayland bug: hardware OpenGL context creation fails
# with "Failed to create an OpenGL context - BadWindow" on many modern
# Linux desktops (GNOME Wayland + XWayland). Software OpenGL works
# everywhere but uses more CPU. Override with HW_OPENGL=1 to force
# hardware (only do this on a real X11 session, not XWayland).
HW_OPENGL="${HW_OPENGL:-0}"
if [[ "$HW_OPENGL" != "1" ]]; then
  echo "   (using software OpenGL — set HW_OPENGL=1 to force hardware)"
  export LIBGL_ALWAYS_SOFTWARE=1
fi
exec rviz2 -d "$RVIZ_CONFIG"
