# JetPack 6 / Ubuntu 22.04 / ROS2 Humble 迁移计划

> **负责人**: Thecnfor（亲自刷机）
> **目标读者**: 项目硬件负责人，需要逐条照做的实操 runbook
> **当前文档状态**: 计划草案（ADR-001 明确表示当前不迁移 ROS2，本文档仅作"赛后迁移"预案，不在 8.10 比赛前执行）

---

## 当前状态与迁移动机

Jetson 上位机当前运行的软件栈：

- **L4T**: R35.3.1（JetPack 5.x 系列）
- **Ubuntu**: 20.04 LTS（focal，2025-04 已 EOL，标准安全更新停止）
- **ROS**: Noetic（2025-05 EOL，预装但项目代码 0 引用 — 见 `docs/ros-analysis.md`）
- **Python**: 3.8（focal 默认）
- **内核**: 5.10

**问题清单**:

1. Ubuntu 20.04 已进入扩展安全维护（ESM），第三方 apt 源逐步迁移到 jammy，PaddlePaddle / 一些 Jetson 社区 wheels 优先发布 22.04 构建
2. ROS Noetic 已 EOL，新传感器驱动（RealSense D400 系列更新固件、ZED X 等）只发布 ROS2 humble/iron 包
3. JetPack 5 不支持新发布的 Jetson Orin Nano Super / Orin NX 16GB 等硬件，若未来升级载板会被迫升级
4. 当前推理后端（PaddlePaddle 2.5+）在 Python 3.10+ 上性能提升 8-12%，但项目仍跑在 3.8

**为什么 ROS2 Humble**:

- Humble 是 Ubuntu 22.04 官方 LTS 对应 ROS2 发行版，支持到 2027-05（与本项目 2027 赛季窗口对齐）
- Iron / Jazzy 需要更新的 Ubuntu（24.04），风险更大
- Humble 的 `ros-humble-ros-gz`（Gazebo Garden 桥接）成熟稳定，仿真调试收益明显

---

## 目标环境

| 组件 | 版本 |
|------|------|
| JetPack | 6.1（最新稳定，L4T R36.3 + CUDA 12.6 + TensorRT 10.3） |
| Ubuntu | 22.04 LTS（jammy） |
| ROS2 | Humble Hawksbill（rqt、rviz2、ros2cli、nav2 可选） |
| Python | 3.10（jammy 默认） |
| OpenCV | 4.5.4（apt） + 4.8（pip，启用 CUDA） |
| PyTorch | 2.3+（仅 CPU 推理，CUDA 路径按需） |
| PaddlePaddle | 2.6+（jammy wheels 需从官网下载） |

**GPU 与硬件支持**: JetPack 6 保留对 Orin 全系 / Xavier NX / Nano 的支持。本项目使用的 Orin 模组继续兼容，但旧 Nano A02（4GB）需注意 JetPack 6 不再官方支持，若载板是旧 Nano 需降回 JetPack 5。

---

## 迁移前置条件

**绝对禁止窗口**: 比赛前 4 周内（即 2026-07-13 至 2026-08-12）不得执行刷机或 ROS2 迁移。本计划最早开始日 = **2026-08-13（比赛后第一天）**。

**硬件清单**:

- 1× Jetson 载板（当前部署件）+ 1× 备用载板（如有）
- USB-C 数据线（连接 host PC 与 Jetson，**必须支持数据**，纯充电线不行）
- 跳线 / 杜邦线（强制恢复模式时短接 FC REC 与 GND）
- HDMI 显示器 + USB 键鼠（用于 Jetson 本地调试，可选但强烈推荐）
- host PC：Ubuntu 20.04/22.04，**至少 50GB 可用磁盘**（SDK Manager + 镜像 + 缓存）
- 16GB+ U 盘（备份当前 Jetson 关键文件）
- 备用 SD 卡或 NVMe（如当前 Jetson 是从 SD 卡启动，需备份 SD 卡镜像）

**备份清单（必须执行）**:

```bash
# 在 Jetson 上执行（或 host 通过 ssh jetson@<ip>）
ssh jetson@<jetson-ip>

# 1. 备份 systemd 服务定义
sudo cp /etc/systemd/system/py_boot.service /tmp/backup/
sudo cp /etc/systemd/system/multi-user.target.wants/py_boot.service /tmp/backup/ 2>/dev/null || true

# 2. 备份 main/boot_py.sh 引用的整个 vehicle_wbt 工作目录
tar czf /tmp/backup/vehicle_wbt_$(date +%Y%m%d).tar.gz \
    -C /home/jetson/workspace vehicle_wbt/

# 3. 备份所有 yml 配置（含硬件校准参数）
tar czf /tmp/backup/configs_$(date +%Y%m%d).tar.gz \
    /home/jetson/workspace/vehicle_wbt/config_car.yml \
    /home/jetson/workspace/vehicle_wbt/vehicle/driver/cfg_vehicle.yaml \
    /home/jetson/workspace/vehicle_wbt/vehicle/arm/arm_cfg.yaml \
    /home/jetson/workspace/vehicle_wbt/infer_cs/base/infer.yaml \
    /home/jetson/workspace/vehicle_wbt/vehicle/base/mc602_cfg.yaml

# 4. 备份 CH340 串口 udev 规则（若有）
sudo cp /etc/udev/rules.d/*.rules /tmp/backup/ 2>/dev/null || true

# 5. 完整 SD 卡 / eMMC 镜像（dd，慢但完整）
#    对 eMMC：sudo dd if=/dev/mmcblk0 bs=4M status=progress | gzip > /tmp/backup/jetson_full_emmc.img.gz
#    对 SD： sudo dd if=/dev/mmcblk1 bs=4M status=progress | gzip > /tmp/backup/jetson_full_sd.img.gz

# 6. 把 /tmp/backup/* scp 回 host
scp -r jetson@<jetson-ip>:/tmp/backup ./jetson_pre_migration_backup_$(date +%Y%m%d)/
```

**比赛前保护**: 备份完成后，本文档附录"备份完整性校验"小节提供一个 md5sum 校验脚本，刷机前/后各跑一次确认无遗漏。

---

## 刷机步骤

1. **下载 NVIDIA SDK Manager**
   - 官网: <https://developer.nvidia.com/sdk-manager>
   - 当前最新版本: 2.1.0（支持 JetPack 6.x）
   - host 上执行：`sudo apt install sdkmanager_2.1.0-12002_amd64.deb` 或直接 `./sdkmanager_2.1.0-12002_amd64.deb`

2. **将 Jetson 置于强制恢复模式**
   - 断开电源
   - 用跳线短接 FC REC 引脚到 GND（Orin: 9-10 针脚；Xavier: 7-8 针脚 — 查载板丝印）
   - 接 USB-C 到 host
   - 接通电源
   - host 验证：`lsusb | grep -i nvidia` 应看到 `NVIDIA Corp. APX`

3. **SDK Manager 烧录**
   - 启动 `sdkmanager`
   - 登录 NVIDIA 开发者账号（免费注册）
   - Target Hardware 选对应载板型号
   - Target Operating System 选 **JetPack 6.1 (rev. 1)**
   - **关键**: 勾选 "Jetson OS" + "Jetson SDK Components"
   - 取消勾选 "DeepStream" / "Isaac"（本项目不用，避免冗余）
   - Storage Device 选 NVMe（而非 SD），若载板支持
   - Pre-Config 页面：用户名/密码与原 Jetson 一致（避免 systemd service 路径错位）

4. **首次启动配置**
   - Jetson 上电后接 HDMI，进入 oem-config
   - 时区设 Asia/Shanghai，键盘 us，语言 en_US.UTF-8
   - 完成后 `sudo apt update && sudo apt upgrade -y`

5. **安装 ROS2 Humble**
   ```bash
   # 官方源（参考 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html）
   sudo apt install software-properties-common -y
   sudo add-apt-repository universe -y
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
       | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-humble-desktop ros-humble-ros-gz -y
   sudo apt install ros-dev-tools -y

   # bash 注入
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

6. **安装项目 Python 依赖**
   ```bash
   sudo apt install python3-colcon-common-extensions python3-pip python3-venv -y
   pip install --upgrade pip setuptools
   pip install opencv-python pyserial simple_pid paddlepaddle==2.6.1 pyzmq \
               PySide2 psutil PyYAML jsonschema
   # erniebot 用 venv 单独装（API 库版本敏感）
   ```

7. **还原 systemd 服务**
   ```bash
   scp -r host:./jetson_pre_migration_backup_*/backup /tmp/
   sudo cp /tmp/backup/py_boot.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable py_boot.service
   ```

8. **验证 ZMQ 推理后端能起**
   - 把备份的 `infer_cs/` 恢复到 `/home/jetson/workspace/vehicle_wbt/`
   - 运行 `python infer_cs/base/infer_back_end.py`，确认 5001-5004 端口监听

---

## 风险与回滚

| 风险 | 概率 | 影响 | 回滚方案 |
|------|------|------|----------|
| SDK Manager 烧录中断（线缆松动/断电） | 中 | Jetson 变砖 | 重进强制恢复模式重烧，镜像可重下 |
| PaddlePaddle 在 22.04 wheel 缺失 | 中 | 推理服务起不来 | 改用 conda-forge 或回退到源码编译（2-4 小时） |
| CH340 驱动在 22.04 内核 5.15 不兼容 | 低 | 串口打不开 | 装 `brltty` 卸载 + 手动加载 `ch341` 模块（参考 GitHub issue） |
| systemd service 路径变更 | 低 | 自启失败 | 备份里的 `py_boot.service` 直接还原即可 |
| `pyqt5` / `PySide2` 在 22.04 库依赖冲突 | 中 | HRI 启动失败 | `pip install --no-deps PySide2` 绕过，或换 PyQt6 |
| ROS2 humble 与 PaddlePaddle CUDA 库冲突 | 中 | 推理报 `libcudart.so` 找不到 | 设置 `LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH` |

**整体回滚方案**:

- **首选**: 用备份的完整 eMMC/SD 镜像 `dd` 写回 → 5-10 分钟恢复 JetPack 5.1.2 + Noetic 环境
- **次选**: 重烧 JetPack 5.1.2 SDK（同 SDK Manager 步骤，目标 OS 选 5.x），然后 `scp` 回备份的 vehicle_wbt 工作目录
- **应急**: 跑备用载板（旧 Jetson）顶上比赛，主载板在赛后慢慢排查

---

## 迁移后第一周要做的验证

按顺序逐项验证，**任何一项失败不要继续**：

1. **基础 ROS2 冒烟测试**
   ```bash
   ros2 --version                            # 期望 ros2cli 1.x
   ros2 topic list                          # 期望空列表（无错误）
   ros2 run demo_nodes_cpp talker &         # 后台跑
   ros2 topic echo /chatter                 # 应看到 "Hello World"
   ros2 run demo_nodes_py listener          # 应打印 Hello World
   ```

2. **rviz2 启动**
   ```bash
   ros2 run rviz2 rviz2                     # 应弹出 GUI 无段错误
   ```

3. **原项目入口能跑（关键回归）**
   ```bash
   cd /home/jetson/workspace/vehicle_wbt
   python main/qqq.py                       # 手动启动 5 分钟，确认 systemd path 仍正确
   ```

4. **ZMQ 推理后端连通性**
   ```bash
   python infer_cs/base/infer_back_end.py &
   nc -zv localhost 5001 && nc -zv localhost 5002 \
    && nc -zv localhost 5003 && nc -zv localhost 5004   # 全部应 succeed
   ```

5. **串口与硬件通信**
   - 确认 `/dev/ttyUSB0` / `/dev/ttyUSB1` 出现
   - 跑 `python -c "import vehicle"` 看是否探测到 MC601/MC602

6. **摄像头**
   - `ls /dev/video*` 确认有设备节点
   - 启动 `python camera/base/camera.py` 抓 10 帧，OpenCV `imshow` 应能显示

7. **机械臂 / 底盘运动**
   - 慢速模式下（速度 PID 设 0.1）跑一次完整 Hanoi 流程
   - 记录所有报错到 `logs/migration_smoke_$(date +%Y%m%d).log`

---

## 时间表

| 日期 | 里程碑 |
|------|--------|
| 2026-08-12 | 比赛结束 |
| 2026-08-13 | 备份当前 Jetson（dd + scp，约 30 分钟） |
| 2026-08-13 | SDK Manager 烧 JetPack 6.1（约 45-60 分钟） |
| 2026-08-13/14 | oem-config + apt update/upgrade |
| 2026-08-14 | 装 ROS2 Humble + 项目 pip 依赖（约 2 小时） |
| 2026-08-15 | 还原 systemd service + vehicle_wbt 源码 |
| 2026-08-15/16 | 跑"迁移后第一周验证"7 项 |
| 2026-08-17 | 排查任何冒烟失败项 |
| 2026-08-20 | 缓冲窗口（处理意外问题） |
| 2026-08-25 | 里程碑: 完整回归（跑一次完整比赛流程，无人工干预） |
| 2026-09-01 | 新赛季开发可在 ROS2 Humble 环境下继续 |

**预留缓冲**: 总共 2 周（08-13 → 08-25），其中 5 天是缓冲期。期间若硬件相关问题卡住超过 2 天，**立刻回滚到 JetPack 5** 不要硬撑 — 比赛已经结束，下赛季再迁移。

---

## 附录：备份完整性校验

```bash
#!/bin/bash
# /tmp/backup/verify.sh — 刷机前后各跑一次
set -e
echo "[$(date)] Verifying backup integrity..."
md5sum /tmp/backup/vehicle_wbt_*.tar.gz \
       /tmp/backup/configs_*.tar.gz \
       /tmp/backup/jetson_full_*.img.gz 2>/dev/null
test -f /tmp/backup/py_boot.service && md5sum /tmp/backup/py_boot.service
ls -la /tmp/backup/
```

把刷机前的 md5 输出存到 `jetson_pre_migration_backup_*/md5_before.txt`，刷机后用同一脚本比对 — 如有 hash 漂移说明备份过程出错，立即重新备份。

---

**变更记录**:
- 2026-07-05: 初稿，由 Thecnfor 起草
- 2026-07-05: 填入 Orin 模组信息 — Jetson Orin NX 16GB（项目当前使用，见 `docs/system-environment.md`）；备用载板 availability 待 Thecnfor 现场确认（建议刷机前向 NVIDIA 经销商备货 1 块备用模组，预算约 ¥3500）