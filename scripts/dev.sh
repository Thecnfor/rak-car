#!/usr/bin/env bash
# vehicle_wbt — one-click "no-hardware development" launcher.
#
# Usage:
#   bash scripts/dev.sh                    # start dev-sidecar stubs + URDF
#   bash scripts/dev.sh --with-mission '[seeding]'
#   bash scripts/dev.sh --with-rviz         # also try to open RViz2
#   bash scripts/dev.sh --help              # show all options
#
# What it does:
#   1. Sources ROS2 (Humble or Lyrical — whichever is on $PATH)
#   2. Sources the colcon install/setup.bash (if built)
#   3. Sets ROS_DOMAIN_ID=42
#   4. Launches dev_all.launch.py in foreground
#      - robot_state_publisher (URDF → /tf + /robot_description)
#      - 5 dev-sidecar stub nodes (camera/IR/chassis/arm/safety_gate)
#      - optionally MissionRunnerNode with a task list
#      - optionally RViz2 (if $DISPLAY is set)
#
# Note: stub nodes are dev-sidecar only — see CLAUDE.md "no mocks in
# production code" rule and docs/development/no-hw-dev.md.
#
# Press Ctrl-C to stop all nodes cleanly.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# -------- 1. Source ROS2 --------
# Try Lyrical first (Ubuntu 26.04), then Humble (Ubuntu 22.04 / Jetson)
if [ -f /opt/ros/lyrical/setup.bash ]; then
  ROS_DISTRO=lyrical
elif [ -f /opt/ros/humble/setup.bash ]; then
  ROS_DISTRO=humble
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  ROS_DISTRO=jazzy
else
  echo "[dev.sh] ERROR: no ROS2 install found in /opt/ros/" >&2
  echo "[dev.sh]        install ros-humble-desktop or ros-lyrical-desktop" >&2
  exit 1
fi
echo "[dev.sh] using ROS2: $ROS_DISTRO"
source /opt/ros/$ROS_DISTRO/setup.bash

# -------- 2. Source the colcon install (if built) --------
INSTALL_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
if [ -f "$INSTALL_SETUP" ]; then
  source "$INSTALL_SETUP"
  echo "[dev.sh] sourced ${INSTALL_SETUP}"
else
  echo "[dev.sh] WARNING: ${INSTALL_SETUP} not found; run colcon build first" >&2
  echo "[dev.sh]          cd ros2_ws && colcon build" >&2
  exit 1
fi

# -------- 3. Set ROS domain ID --------
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# -------- 4. Forward args to ros2 launch --------
ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-mission)  ARGS+=("with_mission:=$2"); shift 2 ;;
    --with-rviz)     ARGS+=("with_rviz:=true"); shift ;;
    --no-rviz)       ARGS+=("with_rviz:=false"); shift ;;
    --help|-h)
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --with-mission 'TASK_LIST'   Run MissionRunnerNode with this task list"
      echo "                                e.g. --with-mission '[seeding, pest_scout]'"
      echo "  --with-rviz                  Try to open RViz2 (needs \$DISPLAY)"
      echo "  --no-rviz                    Don't open RViz2 (default)"
      echo "  --help                       Show this help"
      echo ""
      echo "Environment:"
      echo "  ROS_DOMAIN_ID                Set DDS domain (default: 42)"
      exit 0
      ;;
    *)              ARGS+=("$1"); shift ;;
  esac
done

cd "$REPO_ROOT"

echo "[dev.sh] launching: ros2 launch vehicle_wbt_platform_cpp dev_all.launch.py ${ARGS[*]}"
exec ros2 launch vehicle_wbt_platform_cpp dev_all.launch.py "${ARGS[@]}"
