#!/usr/bin/env bash
# watch_board_key.sh — 持续监测板载按钮 + Key4Btn,每次状态变化打印一次
#
# 用法:
#   ./scripts/watch_board_key.sh [duration_sec]   # default 30s
#
# 会在 Jetson 端启动 mc602_node (background) + 在本 shell 监听 /state/raw。
# 你按 Key4Btn 上的键 (board on P1) 或板载按钮,会看到:
#   - /state/raw board_key: True (pressed) / False (released)
#   - /board/button_events pressed: True (press edge) / False (release edge)
#
# 退出 Ctrl+C 自动 kill node。

set -eo pipefail

DURATION="${1:-30}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="$REPO_DIR/ros2_ws"

cd "$ROS_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_OVERRIDE=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Clean singleton state
python3 -c "from vehicle_wbt_smartcar_hw.serial import serial_mc602; serial_mc602.close()" 2>/dev/null || true

# Start node in background
python3 install/vehicle_wbt_smartcar_bridge/lib/vehicle_wbt_smartcar_bridge/mc602_node \
    --ros-args -r __node:=mc602_io -p serial_port:=/dev/ttyUSB0 -p baud:=1000000 \
    > /tmp/mc602_node.log 2>&1 &
NODE_PID=$!
trap "echo; echo 'Stopping node (PID '$NODE_PID')...'; kill $NODE_PID 2>/dev/null || true; wait $NODE_PID 2>/dev/null || true; exit" INT TERM EXIT

# Wait for node to come up
echo "Waiting for mc602_node to start..."
for i in {1..20}; do
    if ros2 service list 2>/dev/null | grep -q '/vehicle_wbt/v1/mc602/buzzer'; then
        echo "Node is up."
        break
    fi
    sleep 0.5
done

echo
echo "=== Listening for ${DURATION}s ==="
echo ">>> PRESS any Key4Btn key (1/2/3/4) on P1 NOW <<<"
echo ">>> OR press the on-board button <<<"
echo

# Use a simple Python ROS2 node to print board_key state changes
timeout "$DURATION" python3 -c "
import rclpy
from rclpy.node import Node
from vehicle_wbt_smartcar_msgs.msg import RawState
from std_msgs.msg import Header
import time

rclpy.init()
class W(Node):
    def __init__(self):
        super().__init__('watcher')
        self.sub = self.create_subscription(RawState, '/vehicle_wbt/v1/mc602/state/raw', self.cb, 10)
        self.last_key = None
        self.start_t = time.time()
    def cb(self, m):
        if m.board_key != self.last_key:
            t = time.time() - self.start_t
            state = 'PRESSED' if m.board_key else 'released'
            print(f'  [t=+{t:5.1f}s] board_key: {self.last_key} -> {m.board_key}  ({state})', flush=True)
            self.last_key = m.board_key

n = W()
rclpy.spin(n)
rclpy.shutdown()
" 2>&1 | head -40
