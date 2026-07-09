#!/usr/bin/env bash
# deploy/install.sh — Phase 1 Jetson 一键安装脚本
#
# 在 Jetson 上以 sudo 跑这个脚本,它会:
# 1. 复制 systemd unit(2 个)+ env file + CycloneDDS config + udev rules 到 /etc/...
# 2. 重新加载 udev + systemd
# 3. 启用并启动 2 个服务(mc602 + 7-node)
#
# Run:
#   sudo bash deploy/install.sh
#
# 前提:
# - Jetson 上 `~/workspace/rak-car/ros2_ws/install/` 已经有 colcon build
# - /dev/ttyUSB0 (CH340) 已插上连接到 MC602
# - /dev/cam3 + /dev/cam4 已就位
# - /dev/ttyUSB* 不被其他进程占用(mc602_node 是唯一拥有者)

set -euo pipefail

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DEPLOY_DIR="$REPO_DIR/deploy"

echo "=== Phase 1 install from $DEPLOY_DIR ==="

# 0. 校验当前 user(避免 User=jetson 装在 xrak 机器上跑不动)
if ! id xrak >/dev/null 2>&1; then
    echo "⚠  no 'xrak' user — service unit 写死了 User=xrak,需要手动改 deploy/systemd/*.service"
fi

# 1. 复制 systemd unit(2 个)
install -m 0644 "$DEPLOY_DIR/systemd/vehicle-wbt-mc602.service" /etc/systemd/system/
echo "✓ /etc/systemd/system/vehicle-wbt-mc602.service"

install -m 0644 "$DEPLOY_DIR/systemd/vehicle-wbt-platform-cpp.service" /etc/systemd/system/
echo "✓ /etc/systemd/system/vehicle-wbt-platform-cpp.service"

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

# 6. 启用并启动 2 个服务
systemctl enable --now vehicle-wbt-mc602
echo "✓ vehicle-wbt-mc602 enabled + started"

systemctl enable --now vehicle-wbt-platform-cpp
echo "✓ vehicle-wbt-platform-cpp enabled + started"

# 7. 状态检查
sleep 3
echo
if systemctl is-active --quiet vehicle-wbt-mc602; then
    echo "✅ vehicle-wbt-mc602 is active"
else
    echo "❌ vehicle-wbt-mc602 FAILED to start"
    echo "   Check: journalctl -u vehicle-wbt-mc602 -n 50"
fi

if systemctl is-active --quiet vehicle-wbt-platform-cpp; then
    echo "✅ vehicle-wbt-platform-cpp is active"
else
    echo "❌ vehicle-wbt-platform-cpp FAILED to start"
    echo "   Check: journalctl -u vehicle-wbt-platform-cpp -n 50"
fi

# 8. 提示
echo
echo "=== Jetson 守护进程就绪 ==="
echo "  systemctl status vehicle-wbt-mc602 vehicle-wbt-platform-cpp"
echo "  journalctl -u vehicle-wbt-mc602 -f"
echo "  systemctl restart vehicle-wbt-mc602  # 改了代码后重启"
echo
echo "同事 dev box 上:"
echo "  export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo "  ros2 topic list | wc -l    # 应该 25+"
echo "  ./scripts/quick_beep.sh    # 听到 beep = 全链路通"
