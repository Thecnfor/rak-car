#!/bin/bash
# Post-create setup for dev container. Idempotent.

set -e

# Install colcon
apt-get update
apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pytest \
    python3-yaml \
    ros-humble-xacro \
    ros-humble-ros2control

# Build the workspace
cd /workspaces/rak-car/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_wbt_platform vehicle_wbt_platform_cpp \
    --event-handlers console_direct+

echo "============================================"
echo "vehicle_wbt dev container ready!"
echo "  source /opt/ros/humble/setup.bash"
echo "  source /workspaces/rak-car/ros2_ws/install/setup.bash"
echo "  ros2 launch vehicle_wbt_platform_cpp mock_system.launch.py"
echo "============================================"
