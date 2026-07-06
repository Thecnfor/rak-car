# Dev Machine Setup — Desktop / Laptop (any Linux)

> **本机是开发机**：装完整 ROS2 desktop (RViz2 + Gazebo + colcon + 工具链)。SSH 到 Jetson 只是为了部署 + 跑真实硬件集成测试，**所有"日常开发"在 dev 上完成**。

## 当前 dev 机状态（2026-07-05）

```
OS:           Ubuntu 26.04 LTS (x86_64)
ROS2:         ❌ NOT installed yet
colcon:       ❌ NOT installed yet
RViz2:        ❌ NOT installed
Gazebo:       ❌ NOT installed
Git:          ✅ installed
Python:       3.12+ ✅
Docker:       check with `docker --version`
```

**当前 dev 机 (Ubuntu 26.04) 没有官方 ROS2 release**（Jazzy/Kilted 配 Ubuntu 24.04，Humble 配 22.04）。**推荐方案：Docker 容器跑 ROS2 Jazzy**。下面给出 3 种安装方案。

---

## 方案 A：Docker 容器跑 ROS2 Jazzy（推荐 — 当前 dev Ubuntu 26.04）

**优点**：和 host OS 隔离；多个 ROS2 distro 可同时跑；和 host 网络共享 DDS；删除容器=干净卸载。

### Step 1: 安装 Docker（如未装）

```bash
sudo apt update
sudo apt install -y docker.io
sudo usermod -aG docker $USER
newgrp docker  # 或重新登录
docker --version
```

### Step 2: 拉 ROS2 Jazzy 镜像

```bash
docker pull osrf/ros:jazzy-desktop
```

### Step 3: 创建 helper 脚本 `~/bin/dev-ros2`

```bash
mkdir -p ~/bin
cat > ~/bin/dev-ros2 <<'EOF'
#!/bin/bash
# Run a command inside the ROS2 Jazzy dev container.
# Mounts ~/work so source code lives on host, ROS2 env inside container.
# Shares host network so DDS multicast reaches Jetson on same LAN.
docker run --rm -it \
  --network=host \
  --env=DISPLAY=$DISPLAY \
  --env=ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} \
  --volume=$HOME/work:/work \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:jazzy-desktop \
  bash -c "source /opt/ros/jazzy/setup.bash && $*"
EOF
chmod +x ~/bin/dev-ros2
```

### Step 4: 验证

```bash
# Terminal 1: source the helper
dev-ros2 bash

# Inside container:
ros2 --help
rviz2 --help  # GUI tools available

# Terminal 2 (on dev host): run a quick test
dev-ros2 ros2 topic list
```

### Step 5: 克隆代码 + build

```bash
mkdir -p ~/work && cd ~/work
git clone git@github.com:Thecnfor/rak-car.git
cd rak-car
dev-ros2 bash  # enter container
# Inside container:
apt update && apt install -y python3-colcon-common-extensions  # 一次性
cd ros2_ws && colcon build
source install/setup.bash
ros2 pkg list | grep vehicle_wbt
ros2 launch vehicle_wbt_platform_cpp vehicle_wbt_platform.launch.py
```

---

## 方案 B：裸机安装 ROS2 Humble（如果 dev 是 Ubuntu 22.04）

```bash
# 1. Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Add ROS2 apt repo (Humble Hawksbill for Ubuntu 22.04 jammy)
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install Humble desktop
sudo apt update
sudo apt install -y ros-humble-desktop  # full: rclcpp + rclpy + RViz + ros2 bag

# 4. Install colcon
sudo apt install -y python3-colcon-common-extensions

# 5. Source setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 6. Verify
ros2 --help
rviz2 --help
```

---

## 方案 C：裸机安装 ROS2 Jazzy（如果 dev 是 Ubuntu 24.04）

把方案 B 的 `ros-humble-desktop` 替换为 `ros-jazzy-desktop`，repo URL 同样换成 jazzy。

---

## dev 必装工具（不管哪个 ROS2 版本）

```bash
sudo apt install -y \
  python3-pip python3-colcon-common-extensions \
  git build-essential cmake g++ \
  libpcl-dev libeigen3-dev libboost-all-dev \
  ros-$ROS_DISTRO-ros-gz  # 仿真 (Harmonic + bridge)
```

**Dev 桌面建议配置**：
- 8GB+ RAM（Docker 跑 Jazzy 至少 4GB）
- 50GB 磁盘
- 任何 Linux（Ubuntu 22.04/24.04/26.04 都行，Docker 解决版本问题）

---

## dev 上能跑哪些 ROS2 distro？

| Dev 主机 OS | 裸机推荐 | Docker 推荐 |
|-------------|----------|-------------|
| Ubuntu 22.04 | Humble (apt) | 任意 |
| Ubuntu 24.04 | Jazzy (apt) | 任意 |
| Ubuntu 26.04 | (暂无官方) | **Jazzy 容器** |
| macOS | 不可 | Jazzy 容器 |
| Windows | 不可 | WSL2 + Jazzy 容器 |

> **重点**：dev 装什么 distro 不重要（同一份代码兼容 Jazzy / Humble / Kilted），**只要 ROS_DOMAIN_ID 相同**就能和 target 通信。

---

## Dev 上的"日常开发循环"

```bash
# 1. 编辑代码 (host 上)
$EDITOR ~/work/rak-car/ros2_ws/src/vehicle_wbt_platform/...

# 2. 跑测试 (dev-ros2 容器内)
dev-ros2 bash
cd /work/rak-car/ros2_ws && colcon test
# → 单元测试秒回

# 3. 跑仿真 (dev 容器内,带 RViz GUI)
dev-ros2 bash
cd /work/rak-car/ros2_ws && colcon build
source install/setup.bash
ros2 launch vehicle_wbt_platform_cpp vehicle_wbt_platform.launch.py
# → 另开一个终端跑 RViz 看 topic
dev-ros2 rviz2

# 4. 录包/回放 (dev 容器)
dev-ros2 ros2 bag record -a -o my_run
# → 拷贝到 Jetson 上回放测试
```

## 下一步

- SSH 到 Jetson: [ssh-workflow.md](ssh-workflow.md)
- 跑哪些测试在 dev: [test-matrix.md](test-matrix.md)
- 部署到 Jetson: [jetson-target-setup.md](jetson-target-setup.md)
