# Jetson Target Setup — Orin Nano 4GB (JetPack 6, Humble base)

> **本机是生产目标机**：只装 ROS2 Humble BASE (rclcpp + rclpy，无 GUI/Gazebo/RViz)。所有开发、调试、可视化都在 dev 桌面机完成；Jetson 只发布真实传感器数据 + 接收控制指令。

## 当前 Jetson 状态（2026-07-05）

```
Hostname:        orin  (ssh xrak@orin)
OS:              Ubuntu 22.04.5 LTS (jammy)
Jetson:          R36 (JetPack 6.x), aarch64
RAM:             3.5 GB usable (4 GB with ~500 MB reserved)
Disk:            184 GB free on /
ROS2:            ✅ Humble installed (ros-humble-action-msgs, ros-humble-ament-cmake, ...)
colcon:          ❌ NOT installed (需要装)
RViz2:           ❌ NOT installed (内存不够)
Gazebo:          ❌ NOT installed (内存不够)
User:            xrak
SSH:             ✅ works from dev (ssh xrak@orin or ssh orin)
```

**结论**：Jetson **已经是目标状态**（JetPack 6 + Ubuntu 22.04 + Humble），**不需要刷机**。下一步只需要装 colcon 就能 build。

---

## Step 1: 装 colcon（一次性）

```bash
ssh xrak@orin
sudo apt update
sudo apt install -y python3-colcon-common-extensions
# 这一步会拉 ~50MB 依赖，包含 colcon-ros / colcon-cmake / colcon-python
```

验证：
```bash
colcon --version
# 期望: colcon-common-extensions X.Y.Z
```

---

## Step 2: 创建 ros2_ws（一次性）

```bash
ssh xrak@orin
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
# Clone 或 rsync 代码（见 ssh-workflow.md）
```

---

## Step 3: 拉代码 + build

```bash
ssh xrak@orin
cd ~/ros2_ws
# Option A: 从 dev 拉 (rsync) — 详见 ssh-workflow.md
rsync -avz --exclude='build/' --exclude='install/' --exclude='.git/' \
  dev-host:~/work/rak-car/ ~/ros2_ws/   # 然后单独 rsync .git

# Option B: 从 git 拉
git clone https://github.com/Thecnfor/rak-car.git src/rak-car
cd src/rak-car
git checkout develop/ros2-sidecar

# Build (target 只 build 我们自己的包,不 build 第三方)
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_wbt_platform_cpp vehicle_wbt_platform
```

**注意**: target 内存只有 3.5GB，**colcon build 时不可同时跑其他大程序**。如果 build OOM，加 `--executor sequential` 或 `--parallel-workers 1`。

---

## Step 4: 跑 sidecar 节点

```bash
ssh xrak@orin
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ENABLE_ROS2=1
ros2 launch vehicle_wbt_platform_cpp vehicle_wbt_platform.launch.py
# → sidecar 节点在 Jetson 上跑，publish 真实传感器数据到 /vehicle_wbt/v1/...
```

---

## Step 5: 验证 dev 端能订阅

另开 dev 桌面终端：

```bash
# Dev (jazzy container)
dev-ros2 bash
export ROS_DOMAIN_ID=42
ros2 topic list
# → 看到 /vehicle_wbt/v1/... topics
ros2 topic echo /vehicle_wbt/v1/sensors/ir/left
# → 看到 Jetson 发布的红外数据
rviz2
# → Add Topic → /vehicle_wbt/v1/... → 实时可视化
```

**成功标志**：dev 端 RViz 看到 Jetson 真实传感器数据。**这就是"thick client, thin server"**。

---

## 不要在 Jetson 上做的事

| ❌ 不要 | 原因 |
|---------|------|
| 装 `ros-humble-desktop` / `rviz2` / `gazebo` | 4GB 内存会 OOM |
| 跑 `colcon test`（所有单元测试） | dev 上跑更快，不占 Jetson cycles |
| 跑仿真 | Gazebo 需要 2-4GB 内存 + GPU，Jetson 4GB 跑不动 |
| 装完整 VSCode / IDE | dev 桌面编辑，scp/rsync 过来 |
| 跑 `ros2 bag record` 长任务 | SSD 写满影响后续真机部署 |

## Jetson 应该做的事

| ✅ 应该 | 原因 |
|---------|------|
| 跑 sidecar rclcpp 节点（真实 HW） | 这是 Jetson 的本职 |
| 接收控制指令（MANUAL 模式） | 通过 DDS 接受 dev 上的 remote control |
| 发布真实传感器数据 | dev 上的 RViz 才能看到真实数据 |
| 短时 ros2 bag record（1-2 分钟） | 调试时录一段，dev 上回放 |
| systemd 拉起 main + sidecar | production 部署时 `py_boot.service` |

---

## 环境健康检查清单

```bash
# 1. ROS 环境
ssh xrak@orin "source /opt/ros/humble/setup.bash && ros2 --help | head -3"
# 期望: usage: ros2 ...

# 2. colcon
ssh xrak@orin "colcon --version"
# 期望: colcon-common-extensions X.Y.Z

# 3. 网络：dev 能 ping orin
ping orin  # 期望: 0% packet loss

# 4. DDS 发现: dev 端能看到 Jetson 节点
# Dev 端:
ros2 node list
# → 应看到 orin 上的 sidecar_xxx 节点

# 5. 内存
ssh xrak@orin "free -h | head -2"
# 期望: available > 1GB (build/run 时留余量)
```

---

## 故障排查

| 症状 | 可能原因 | 解决 |
|------|---------|------|
| dev 上 `ros2 topic list` 看不到 Jetson topics | ROS_DOMAIN_ID 不同 | 两个机器都 `export ROS_DOMAIN_ID=42` |
| Jetson 上 `colcon build` OOM | 内存不够 | 加 `--executor sequential --parallel-workers 1` |
| `ros2 node list` 看不到 Jetson 节点 | DDS multicast 被防火墙拦 | `sudo ufw allow 7400-7500/udp` |
| Jetson 时间不同步 | 无 NTP | `sudo apt install -y chrony && sudo systemctl enable --now chrony` |
| 找不到 `vehicle_wbt_platform_cpp` 包 | 没 source install/setup.bash | `source ~/ros2_ws/install/setup.bash` |

---

## 下一步

- SSH + 文件同步细节: [ssh-workflow.md](ssh-workflow.md)
- 哪些测试在 dev / target 跑: [test-matrix.md](test-matrix.md)
- 刷机历史 (本 Jetson 不需要刷): [../migration/jetpack6-ros2-humble.md](../migration/jetpack6-ros2-humble.md)
