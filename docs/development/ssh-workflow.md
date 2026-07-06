# SSH Workflow — Dev ↔ Jetson

> **核心模式**：dev 写代码 → rsync 到 Jetson → SSH 上去 build → 在 dev 上 RViz 订阅 Jetson topic → 调试。**全程不需要在 Jetson 上用编辑器或装 IDE**。

## SSH 接入

```bash
# 标准方式
ssh xrak@orin
# 或（如果 ~/.ssh/config 配了)
ssh orin

# 配置 SSH 免密 (一次性, dev 机执行)
ssh-copy-id xrak@orin
# 之后 ssh orin 即可免密

# 测试连通性
ssh orin "echo connected; uname -m"
# 期望: connected; aarch64
```

## ~/.ssh/config 模板（推荐配置在 dev 机）

```sshconfig
# ~/.ssh/config on dev
Host orin
    HostName orin
    User xrak
    ForwardX11 yes        # X11 forward for RViz over SSH
    ServerAliveInterval 60
    IdentityFile ~/.ssh/id_ed25519
```

`ForwardX11 yes` 让 Jetson 上的 GUI 程序（虽然我们不用）能 forward 回 dev 显示。

---

## 文件同步 3 种方式

### 方式 A：rsync 单向（推荐 — 频繁改代码时）

```bash
# Dev 机: 推送代码到 Jetson
rsync -avz --exclude='build/' --exclude='install/' --exclude='.git/' \
  --exclude='__pycache__/' --exclude='*.pyc' --exclude='.pytest_cache/' \
  ~/work/rak-car/ xrak@orin:~/work/rak-car/

# 注意: 单独 rsync .git 目录
rsync -avz ~/work/rak-car/.git xrak@orin:~/work/rak-car/
```

**alias 化**（dev ~/.bashrc）：

```bash
alias push2orin='rsync -avz --exclude="build/" --exclude="install/" --exclude=".git/" --exclude="__pycache__/" --exclude="*.pyc" --exclude=".pytest_cache/" --exclude="logs/" ~/work/rak-car/ xrak@orin:~/work/rak-car/ && rsync -avz ~/work/rak-car/.git xrak@orin:~/work/rak-car/'
```

### 方式 B：git push + pull（推荐 — 团队协作时）

```bash
# Dev 机
git push origin develop/ros2-sidecar

# Jetson 机
ssh orin
cd ~/work/rak-car  # 或 src/rak-car 在 ros2_ws/src/
git pull
```

### 方式 C：共享 NFS / sshfs（推荐 — 大项目，频繁 sync）

```bash
# Dev 机: mount Jetson 工作目录到本地
sshfs xrak@orin:~/work/rak-car ~/work/rak-car-orin

# 编辑 ~/work/rak-car-orin/... 实际在 Jetson 上
# 缺点: sshfs 慢,IDE 大项目会卡
```

**推荐: 方式 A (rsync) 用于日常 + 方式 B (git) 用于大版本切换**。

---

## 远程 build

```bash
# Dev 触发 Jetson build（无需登录)
ssh orin "cd ~/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-up-to vehicle_wbt_platform_cpp"

# 看 build 输出
ssh orin "cd ~/ros2_ws && colcon build --packages-up-to vehicle_wbt_platform_cpp --event-handlers console_direct+"

# 长时间 build（30+ 分钟）— 用 tmux 防止断连
ssh orin
tmux new -s build
cd ~/ros2_ws && colcon build ...
# Ctrl-b d  detach
# ssh orin -t "tmux attach -t build"  # 重新连上看进度
```

---

## 远程测试

```bash
# 在 Jetson 上跑 Python 单元测试
ssh orin "cd ~/ros2_ws && PYTHONPATH=src/vehicle_wbt_platform python3 -m pytest src/vehicle_wbt_platform/test/ -v"
# 注意: Jetson 上跑测试是浪费 cycles,通常 dev 跑就够了
# 但 gtest 必须在 Jetson 上跑 (因为 Jetson 才有 rclcpp)

# 远程 gtest (Phase 1.5+)
ssh orin "cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select vehicle_wbt_platform_cpp"
```

---

## 跨机器调试：Dev 跑 RViz，Jetson 跑节点

### Step 1: Jetson 跑 sidecar

```bash
ssh orin
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch vehicle_wbt_platform_cpp vehicle_wbt_platform.launch.py
# 终端留在前台,持续跑
```

### Step 2: Dev 端 RViz 订阅

```bash
# Dev 端
dev-ros2 bash
export ROS_DOMAIN_ID=42
rviz2
# Add → By topic → /vehicle_wbt/v1/sensors/ir/left → PointCloud2 (或 LaserScan)
# 应该看到 Jetson 发布的真实数据
```

### Step 3: Dev 端手动发控制指令

```bash
# Dev 端 (MANUAL mode 已通过物理按钮 3 启用)
dev-ros2 bash
export ROS_DOMAIN_ID=42
ros2 topic pub /vehicle_wbt/v1/cmd/vel_safe geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
# Jetson 端会接收并控制电机
```

---

## 跨机器录包/回放

```bash
# Dev 录 Jetson 发布的 topic (代替 Jetson 录)
dev-ros2 ros2 bag record -a -o debug_session
# → 在 dev 端录,不用占 Jetson 资源

# 之后在 dev 上回放 (脱离 Jetson)
dev-ros2 ros2 bag play debug_session
# → 触发 RViz 重放,定位 bug
```

---

## DDS 网络配置

**dev 和 Jetson 默认用 UDP multicast（7400-7500 端口）发现**。如果在同一 LAN（最常见），**开箱即用**。

如果跨 VLAN / VPN / 防火墙，配置 unicast：

```bash
# 双方都设置 (dev + Jetson)
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI='<CycloneDDS><Network><Interfaces><NetworkInterface autodetect="true"/></Interfaces><Peers><Peer address="192.168.1.100"/></Peers></Network></CycloneDDS>'
# 192.168.1.100 = 对方的 IP
```

详细配置见 `docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §远程接入`。

---

## 故障排查

| 症状 | 原因 | 解决 |
|------|------|------|
| `ssh: Could not resolve hostname orin` | dev 没配 hosts | `echo "192.168.x.x orin" \| sudo tee -a /etc/hosts` |
| `Permission denied (publickey)` | 没配 SSH key | `ssh-copy-id xrak@orin` |
| rsync 极慢 | 在 Jetson 上 build 后又 rsync `build/` | 加 `--exclude='build/'`（默认已加）|
| dev 上看不到 Jetson topic | ROS_DOMAIN_ID 不一致 | 都 `export ROS_DOMAIN_ID=42` |
| RViz 看到 topic 但无数据 | 防火墙拦 UDP | `sudo ufw allow 7400-7500/udp` |
| Jetson build OOM | 4GB 内存不够 colcon 并行 | `colcon build --executor sequential --parallel-workers 1` |
| SSH 断连后 build 中断 | 长 build 没 tmux 保护 | 用 `tmux new -s build` 包住 |

---

## 推荐 Dev 桌面工作流（一次设置后永久受益）

```bash
# ~/.bashrc (dev 机) 推荐加
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0  # 0=接受跨机器, 1=仅本机 (调试时用 1)
alias orin='ssh xrak@orin'
alias push2orin='rsync -avz --exclude="build/" --exclude="install/" --exclude=".git/" --exclude="__pycache__/" --exclude="*.pyc" --exclude=".pytest_cache/" --exclude="logs/" ~/work/rak-car/ xrak@orin:~/work/rak-car/ && rsync -avz ~/work/rak-car/.git xrak@orin:~/work/rak-car/'
alias build2orin='ssh orin "cd ~/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-up-to vehicle_wbt_platform_cpp"'
alias rviz='dev-ros2 rviz2'
```

之后工作循环：

```bash
# 1. 编辑
$EDITOR ~/work/rak-car/...

# 2. 推 Jetson
push2orin

# 3. 远程 build
build2orin

# 4. dev 上跑测试 (秒回)
dev-ros2 pytest ~/work/rak-car/ros2_ws/src/vehicle_wbt_platform/test/

# 5. 启 RViz 看 Jetson 数据
rviz

# 6. SSH 上启动 sidecar
ssh orin
cd ~/ros2_ws && source install/setup.bash
ros2 launch vehicle_wbt_platform_cpp vehicle_wbt_platform.launch.py
```

**5 步日常循环，零 GUI 在 Jetson 上**。
