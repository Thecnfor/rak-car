# Day-One Setup — 第一次连 dev 机器

> 新成员 / 换 dev 机器 / 重装系统后必读。
> 跑完本文档后你能：连 dev 机器 + build 项目 + 看到 Jetson 实时 cameras。

## 前置

- [ ] **dev 机器**：Ubuntu 22.04 / 24.04 / 26.04 之一，已装好系统
- [ ] **GitHub 访问**：被加入 `Thecnfor/rak-car` collaborators
- [ ] **Jetson 信息**：Jetson IP **硬编码 `192.168.3.69`**（团队约定，不能改），用户名 `xrak`
- [ ] **同 LAN**：dev 机跟 Jetson 在同一路由器下（`ping 192.168.3.69`）

> 如果你的 dev 机器没装 Ubuntu，看 [`dev-machine-setup.md`](../development/dev-machine-setup.md)。

---

## Step 1: 装 OS 工具（5 min）

```bash
sudo apt update
sudo apt install -y git openssh-client curl wget
```

## Step 2: 装 ROS2（10-30 min）

| Ubuntu | ROS2 distro | 安装方式 |
|--------|-------------|----------|
| 22.04 | Humble | `sudo apt install ros-humble-desktop` |
| 24.04 | Jazzy | `sudo apt install ros-jazzy-desktop` |
| 26.04 | Lyrical | `sudo apt install ros-lyrical-desktop` |

> 完整步骤（含 source bashrc、Docker 方案等）见 [`docs/development/dev-machine-setup.md`](../development/dev-machine-setup.md)。

**验证**：

```bash
ls /opt/ros/*/setup.bash | head -1
# 应该看到 /opt/ros/humble/setup.bash 或类似
```

## Step 3: 克隆仓库（1 min）

```bash
# 配置 Git（如果第一次用）
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# 克隆
git clone git@github.com:Thecnfor/rak-car.git
cd rak-car
```

> **遇到问题**：`Permission denied (publickey)` → 你的 GitHub SSH key 没配。`ssh-keygen -t ed25519` 然后去 GitHub Settings → SSH keys 添加 `~/.ssh/id_ed25519.pub`。

## Step 4: 跑 onboard 脚本（10-30 min）

```bash
bash scripts/onboard.sh
```

这个脚本会自动：
1. **Phase 1**：探测环境（OS / ROS2 / 磁盘 / 网络）
2. **Phase 2**：装 colcon-common-extensions + 部署 CycloneDDS 配置 + 加 `ROS_DOMAIN_ID=42` 到 `~/.bashrc`
3. **Phase 3**：`colcon build` 编译项目 + 跑 pytest smoke test（45/45 应通过）

**可选 flag**：
- `--dry-run`：只检查不修改
- `--phase=1|2|3`：单独跑某一阶段
- `--skip-ros-install`：已装 ROS2 时跳过 apt install

## Step 5: 验证（30s）

```bash
bash scripts/diagnose.sh
```

期望看到 **12+ pass**，包括：
- `✅ [04] ssh:passwordless` —— SSH 免密成功
- `✅ [05] jetson:user` —— Jetson 用户可访问
- `✅ [13] dds:image_topic` —— camera 在 publish（~30Hz）
- `✅ [15] dds:ros2_daemon` —— ROS2 daemon 在跑

**如果有 FAIL**：看输出的 🚨 Action items，照着修复。

## Step 6: 第一次看 cameras（启动 RViz2）

```bash
bash scripts/start_team_rviz.sh
```

会弹出一个 RViz2 窗口，左边 panel 的 `front_camera` 和 `arm_camera` ImageDisplay 显示 Jetson 实时画面。

**用完关掉**：直接关 RViz2 窗口，或 Ctrl-C 终端。

## 常见坑

| 坑 | 症状 | 解决 |
|------|------|------|
| ROS2 没装 | `ros2: command not found` | 重做 Step 2 |
| SSH 连不上 Jetson | `Permission denied (publickey)` 或 `Connection refused` | 找 Thecnfor 配 / 检查网线 |
| ROS_DOMAIN_ID 不一致 | 看不到 Jetson topic | `export ROS_DOMAIN_ID=42` + 重启 shell |
| CycloneDDS 配置没拷 | 看不到 Jetson topic | `cp ros2_ws/src/vehicle_wbt_platform_cpp/config/cyclonedds.xml ~/.ros/` |
| Ping orin 不通 | 找不到 Jetson | `ping 192.168.3.69` — 检查网线 / 路由器 / 你在不在内网 |
| colcon build 失败 | 缺依赖 / 缺包 | 看错误信息，缺什么装什么；不要忘了 `source /opt/ros/<distro>/setup.bash` |

完整排错见 [`../operations/troubleshooting.md`](../operations/troubleshooting.md)。

## 完成 ✅

现在你应该：
- ✅ 能在 dev 机 build 项目
- ✅ SSH 免密到 Jetson
- ✅ RViz2 看到 2 个 camera 实时画面
- ✅ `diagnose.sh` 大部分 ✅

**下一步**：
- 想看 3D robot model：跑 `bash scripts/dev.sh --with-rviz`
- 想改代码：看 [SSH workflow](../development/ssh-workflow.md) + [branch strategy](../contributing/branch-strategy.md)
- 想标定相机：看 [`scripts/calibrate_camera.py`](../../scripts/calibrate_camera.py) + [`scripts/README.md#calibration`](../../scripts/README.md)
