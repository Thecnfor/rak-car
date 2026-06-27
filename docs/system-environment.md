# 系统环境与刷机恢复指南

> 本文档记录了项目对 Jetson 系统环境的所有依赖，以及刷机后的完整恢复步骤。

## 当前系统环境快照

| 项目 | 版本 | 备注 |
|------|------|------|
| JetPack / L4T | R35.3.1 (2023-03-19) | JetPack 5.1.2 |
| CUDA | 11.4.315 | |
| TensorRT | 8.5.2 | |
| cuDNN | 8.6.0 | |
| Python | 3.8.10 | `/usr/bin/python → python3` |
| PaddlePaddle | paddlepaddle-gpu 2.5.2 | Jetson 专用 aarch64 wheel |
| OpenCV | 系统预装 | JetPack 自带 |
| ROS | Noetic (1.17.4) | 项目不使用，系统预装 |
| 用户 | `jetson` | 在 dialout 组 |

## 系统依赖分类

### 🔴 硬依赖 — 不装就不能跑

| 依赖 | 说明 | 来源 |
|------|------|------|
| Python 3.8+ | 代码用 f-string，需要 3.6+，当前 3.8 | JetPack 预装 |
| `/usr/bin/python → python3` | systemd 服务用 `/usr/bin/python` 启动 | 需手动创建软链接 |
| paddlepaddle-gpu 2.5.2 | 推理核心，必须 Jetson aarch64 版 | pip 安装专用 wheel |
| pyserial | 串口通信 | pip |
| pyzmq | ZMQ 推理服务通信 | pip |
| numpy | 数值计算 | pip |
| opencv-python | 图像处理 | JetPack 预装 |
| PySide2 | HRI 机器人表情显示 | apt |
| `jetson` 用户 + dialout 组 | 串口权限 | 系统配置 |

### 🟠 软依赖 — 不装部分功能异常

| 依赖 | 影响范围 | 来源 |
|------|---------|------|
| simple_pid | PID 控制器 | pip |
| erniebot | 百度文心大模型 | pip |
| openai | DeepSeek/通义大模型 | pip |
| requests | 天气 API | pip |
| psutil | 推理服务进程检测 | pip |
| PyYAML | 配置文件读取 | pip |
| jsonschema | JSON 验证 | pip |
| Xvfb | HRI 虚拟显示 | apt |
| Qt5 EGLFS 插件 | HRI 无头显示 | apt |

### 🟢 不需要 — 系统有但项目不用

| 依赖 | 说明 |
|------|------|
| ROS Noetic | 系统预装了完整 ROS，但项目**零使用** |
| catkin / colcon | ROS 构建工具，项目不用 |
| Gazebo | 仿真器，项目不用 |
| actionlib / tf2 | ROS 通信库，项目用 ZMQ 替代 |

---

## 摄像头设备映射

摄像头不直接用 `/dev/video*`，而是通过 udev 规则创建符号链接：

```
/dev/cam1 → /dev/video0  (USB 端口 2.1)
/dev/cam2 → /dev/video2  (USB 端口 2.2)
/dev/cam3 → /dev/video?  (USB 端口 2.3)
/dev/cam4 → /dev/video?  (USB 端口 2.4)
```

**映射依据是 USB 物理端口号（devpath），不是 video 编号。** 这保证了无论摄像头插入顺序如何，cam1 永远对应特定的 USB 口。

当前 udev 规则文件：`/etc/udev/rules.d/99-usbvideo.rules`

```
KERNEL=="video*" , SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{devpath}=="2.1", MODE:="0777", SYMLINK+="cam1"
KERNEL=="video*" , SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{devpath}=="2.2", MODE:="0777", SYMLINK+="cam2"
KERNEL=="video*" , SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{devpath}=="2.3", MODE:="0777", SYMLINK+="cam3"
KERNEL=="video*" , SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{devpath}=="2.4", MODE:="0777", SYMLINK+="cam4"
```

串口规则文件：`/etc/udev/rules.d/99-usb-serial.rules`

```
KERNEL=="ttyUSB[0-9]*",MODE="0666"
```

**⚠️ 这两个 udev 规则文件不在 git 仓库中，刷机后丢失，必须手动重建。**

---

## 硬编码路径清单

以下文件硬编码了 `/home/jetson/workspace/vehicle_wbt/`：

| 文件 | 行号 | 内容 | 影响 |
|------|:---:|------|------|
| `main/qqq.py` | 5 | `sys.path.append(...)` | import 路径 |
| `main/main.py` | 3 | `sys.path.append(...)` | import 路径 |
| `main/scripy1~5.py` | 3 | `sys.path.append(...)` | import 路径 |
| `main/aaa.py`, `bbb.py`, `111.py` | 3 | `sys.path.append(...)` | import 路径 |
| `main/test.py`, `grab.py`, `angle.py` | 3-4 | `sys.path.append(...)` | import 路径 |
| `main/5.py` | 3 | `sys.path.append(...)` | import 路径 |
| `test2.py` | 3 | `sys.path.append(...)` | import 路径 |
| **`car_wrap.py`** | **207** | **推理服务自动启动路径** | **核心功能** |
| `main/boot_py.sh` | 15-16 | systemd 服务路径 | 开机自启 |
| `main/hri/hri-autostart.service` | 10, 21 | systemd 服务路径 | HRI 自启 |

**结论：** 只要用户名是 `jetson`、项目在 `/home/jetson/workspace/vehicle_wbt/`，就不用改任何路径。

---

## systemd 服务

### py_boot.service — 主服务

安装方式：`sudo bash main/boot_py.sh`

```ini
[Unit]
Description=Python Boot Service
After=network.target

[Service]
User=jetson
ExecStart=/usr/bin/python -u /home/jetson/workspace/vehicle_wbt/main/qqq.py
WorkingDirectory=/home/jetson/workspace/vehicle_wbt/main/
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### hri-autostart.service — HRI 表情服务

```ini
[Unit]
Description=HRI Robot Face Display
After=network.target graphical.target

[Service]
User=jetson
Group=jetson
WorkingDirectory=/home/jetson/workspace/vehicle_wbt/main/hri
ExecStart=/home/jetson/workspace/vehicle_wbt/main/hri/hri_startup.sh
Restart=always
RestartSec=15
Environment="DISPLAY=:0"
Environment="QT_QPA_PLATFORM=xcb"
Environment="QT_XCB_GL_INTEGRATION=none"
Environment="QT_OPENGL=software"
Environment="QT_QUICK_BACKEND=software"
Environment="QT_SCALE_FACTOR=1"
Environment="QT_AUTO_SCREEN_SCALE_FACTOR=0"
Environment="QT_SCREEN_SCALE_FACTORS=1"
Environment="QT_FONT_DPI=96"
Environment="QT_LOGGING_RULES=*=false"

[Install]
WantedBy=graphical.target
```

---

## HRI 显示配置

HRI 服务期望的显示环境：

| 配置项 | 值 | 说明 |
|--------|---|------|
| 分辨率 | 1024×600 | DP-1 输出 |
| Qt 平台 | xcb / eglfs | 根据 `/dev/dri/card0` 自动选择 |
| 渲染 | software | 不用 GPU 渲染 QML |

`hri_startup.sh` 的显示检测逻辑：
1. 如果 `/dev/dri/card0` 存在 → 用 `eglfs`（直接 DRM 输出）
2. 否则 → 用 `xcb`（X11）或 `fb`（framebuffer，`/dev/fb0`）

---

## 网络依赖

### 本地通信（不需要外网）

| 服务 | 地址 | 端口 |
|------|------|:---:|
| 推理服务 lane | `tcp://127.0.0.1` | 5001 |
| 推理服务 task | `tcp://127.0.0.1` | 5002 |
| 推理服务 front | `tcp://127.0.0.1` | 5003 |
| 推理服务 ocr | `tcp://127.0.0.1` | 5004 |

### 外部 API（需要外网）

| 服务 | 用途 | 是否必须 |
|------|------|:---:|
| 百度 AI Studio | 文心大模型 | 竞赛答题用 |
| DeepSeek API | 大模型 | 替代文心 |
| 阿里 DashScope | 通义大模型 | 替代文心 |
| 高德天气 API | 天气显示任务 | 竞赛天气任务用 |

---

## 刷机后完整恢复步骤

```bash
# ============================================================
# 第一步：基础系统配置
# ============================================================

# 确保用户名是 jetson，项目在 /home/jetson/workspace/vehicle_wbt/
# （如果用其他用户名，需要改 17+ 个文件的硬编码路径）

# Python 软链接
sudo ln -sf /usr/bin/python3 /usr/bin/python

# 用户组权限
sudo usermod -aG dialout,input jetson
# 重新登录使生效

# ============================================================
# 第二步：udev 规则（摄像头 + 串口）
# ============================================================

sudo tee /etc/udev/rules.d/99-usbvideo.rules << 'EOF'
KERNEL=="video*" , SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{devpath}=="2.1", MODE:="0777", SYMLINK+="cam1"
KERNEL=="video*" , SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{devpath}=="2.2", MODE:="0777", SYMLINK+="cam2"
KERNEL=="video*" , SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{devpath}=="2.3", MODE:="0777", SYMLINK+="cam3"
KERNEL=="video*" , SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{devpath}=="2.4", MODE:="0777", SYMLINK+="cam4"
EOF

sudo tee /etc/udev/rules.d/99-usb-serial.rules << 'EOF'
KERNEL=="ttyUSB[0-9]*",MODE="0666"
EOF

sudo udevadm control --reload-rules && sudo udevadm trigger

# ============================================================
# 第三步：Python 包
# ============================================================

# 基础包
pip3 install numpy pyserial simple_pid pyzmq psutil PyYAML jsonschema requests openai erniebot

# PaddlePaddle GPU — 必须用 Jetson 专用 wheel！
# 当前版本: paddlepaddle-gpu 2.5.2
# 下载地址: https://www.paddlepaddle.org.cn/inference/user_guides/download_lib.html
# 选择: CUDA 11.4 + JetPack 5.x + aarch64 + Python 3.8
# pip3 install paddlepaddle_gpu-2.5.2-cp38-cp38-linux_aarch64.whl

# PySide2（HRI 机器人表情）
sudo apt install python3-pyside2.qt*

# HRI 依赖
sudo apt install xvfb x11-utils x11-xserver-utils

# ============================================================
# 第四步：模型文件（gitignored，需要从备份恢复）
# ============================================================

# 需要恢复的目录：
# paddle_jetson/base/lane_model/
# paddle_jetson/base/task_wbt2025/
# paddle_jetson/base/front_model2/
# paddle_jetson/base/ch_PP-OCRv3_det_infer/
# paddle_jetson/base/ch_PP-OCRv3_rec_infer/
# paddle_jetson/base/mot_ppyoloe_s_36e_pipeline/
# paddle_jetson/base/PPLCNet_x1_0_person_attribute_945_infer/

# ============================================================
# 第五步：systemd 服务
# ============================================================

cd /home/jetson/workspace/vehicle_wbt
sudo bash main/boot_py.sh
sudo cp main/hri/hri-autostart.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable py_boot.service hri-autostart.service

# ============================================================
# 第六步：验证
# ============================================================

# 验证 Python 环境
python -c "import cv2, numpy, zmq, yaml, serial; print('基础包 OK')"

# 验证摄像头
ls -la /dev/cam*

# 验证串口
ls -la /dev/ttyUSB*

# 验证推理服务
python infer_cs/base/infer_back_end.py &
sleep 10
python -c "from infer_cs import ClintInterface; c = ClintInterface('lane'); print('推理服务 OK')"

# 验证硬件
python -c "from vehicle import *; print('硬件通信 OK')"
```

---

## 刷机风险评估

| 风险 | 概率 | 影响 | 预防措施 |
|------|:---:|------|---------|
| 用户名不是 jetson | 低 | 🔴 全部路径失效 | 刷机时保持用户名 |
| 项目路径变了 | 低 | 🔴 全部路径失效 | 保持原路径 |
| PaddlePaddle 版本不兼容 | 中 | 🔴 推理全部失败 | 用同版本 wheel |
| 摄像头 USB 口变了 | 中 | 🟠 cam1/cam2 映射错 | 检查物理接线 |
| JetPack 版本不同 | 中 | 🟠 CUDA/TensorRT 不匹配 | 用同版本 JetPack |
| API 密钥过期 | 低 | 🟡 大模型功能失效 | 定期检查 |
| 模型文件丢失 | 中 | 🔴 推理全部失败 | 提前备份 |
