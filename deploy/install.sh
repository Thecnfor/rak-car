#!/usr/bin/env bash
# deploy/install.sh — Phase 1 Jetson 一键安装脚本
#
# 在 Jetson 上以 sudo 跑这个脚本,它会:
# 1. 复制 systemd unit + env file + CycloneDDS config + udev rules 到 /etc/...
# 2. 重新加载 udev + systemd
# 3. 启用并启动 vehicle-wbt-mc602 服务
#
# Run:
#   sudo bash deploy/install.sh
#
# 前提:
# - Jetson 上 `~/workspace/rak-car/ros2_ws/install/` 已经有 colcon build
# - /dev/ttyUSB1 (CH340) 已插上连接到 MC602
# - /dev/ttyUSB* 不被其他进程占用(mc602_node 是唯一拥有者)

set -euo pipefail

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DEPLOY_DIR="$REPO_DIR/deploy"

echo "=== Phase 1 install from $DEPLOY_DIR ==="

# 1. 复制 systemd unit
install -m 0644 "$DEPLOY_DIR/systemd/vehicle-wbt-mc602.service" /etc/systemd/system/
echo "✓ /etc/systemd/system/vehicle-wbt-mc602.service"

# 2. 复制 env file
mkdir -p /etc/vehicle-wbt
install -m 0644 "$DEPLOY_DIR/ros_env.sh" /etc/vehicle-wbt/ros.env
echo "✓ /etc/vehicle-wbt/ros.env"

# 3. 复制 CycloneDDS 配置
install -m 0644 "$DEPLOY_DIR/cyclonedds/cyclonedds.xml" /etc/cyclonedds.xml
echo "✓ /etc/cyclonedds.xml"

# 4. 复制 udev rules
install -m 0644 "$DEPLOY_DIR/udev/99-usbvideo.rules" /etc/udev/rules.d/
echo "✓ /etc/udev/rules.d/99-usbvideo.rules"

# 5. 重新加载
udevadm control --reload-rules
udevadm trigger
echo "✓ udev reloaded"

systemctl daemon-reload
echo "✓ systemd reloaded"

# 6. 启用并启动
systemctl enable --now vehicle-wbt-mc602
echo "✓ vehicle-wbt-mc602 enabled + started"

# 7. 状态检查
sleep 2
if systemctl is-active --quiet vehicle-wbt-mc602; then
    echo
    echo "✅ vehicle-wbt-mc602 is active"
    echo "   同事现在可以在 dev box 上跑:"
    echo "     cd ~/workspace/rak-car && ./scripts/quick_beep.sh"
    echo "   应该听到 0.2 秒 440Hz 蜂鸣"
else
    echo
    echo "❌ vehicle-wbt-mc602 FAILED to start"
    echo "   Check: journalctl -u vehicle-wbt-mc602 -n 50"
    exit 1
fi
