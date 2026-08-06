# XRAK Autonomous Robot Platform

## 系统概览

XRAK 是一个基于 NVIDIA Jetson 与 WhalesBot 硬件平台打造的全栈自主移动机器人系统。它集成了实时感知、自主导航、精密操作与物流自动化能力，能够在复杂动态环境中完成端到端的自主任务。

本系统采用分布式客户端-服务端架构：车端运行时（Runtime）独占硬件资源并对外暴露标准 API，上层业务逻辑（Mission Client）通过 HTTP/WebSocket 远程调用，实现了解耦的软硬件分离设计。

---

## 快速启动

### 局域网联调入口

如果你正在局域网内联调，优先查阅以下文档：

- [API_USAGE.md](file:///home/jetson/workspace/rak-car/API_USAGE.md)
- [runtime/README.md](file:///home/jetson/workspace/rak-car/runtime/README.md)
- [main/README.md](file:///home/jetson/workspace/rak-car/main/README.md)

当前默认局域网地址：

- API 服务：`http://192.168.6.231:5050`
- FastAPI 交互文档：`http://192.168.6.231:5050/docs`
- 实时视频流：`http://192.168.6.231:5050/stream/`

如需快速修改监听地址或端口，仅需调整：

```python
/home/jetson/workspace/rak-car/runtime/core/settings.py
```

推荐生产启动方式：

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pip install -r runtime/requirements.txt
pm2 start ecosystem.config.js
pm2 save
```

---

## 硬件架构

| 组件 | 规格 |
|------|------|
| 上位机 | NVIDIA Jetson Orin Nano / Nano（JetPack/L4T） |
| 下位机 | MC602 运动控制器（USB 串口，1M baud） |
| 底盘 | WhalesBot 麦克纳姆轮全向移动平台 |
| 感知层 | 前置摄像头（车道检测）+ 侧向摄像头（目标检测） |
| 操作臂 | 4-DOF 机械臂（夹取 / 放置 / 气泵吸取） |
| 执行器 | 蜂鸣器、储物架伺服、射击机构 |
| 遥测 | 双摄像头 MJPEG 实时流、轮速里程计 |

---

## 软件架构

```
├── runtime/                 # 车端运行时（FastAPI 服务）
│   ├── api/                 # 路由层：/v1/execute、/v1/vision、/v1/realtime、/v1/ws
│   ├── core/                # 配置管理（settings.py）与动作注册表（actions.py）
│   ├── services/            # MyCar 单例聚合 + 运行时服务（队列、锁、守护线程）
│   └── hardware/            # MC602 USB 会话状态机 + 自动恢复
│
├── main/                    # 业务客户端（仅通过 HTTP/WS 与 runtime 通信）
│   ├── start/               # 任务编排器：50Hz 循迹外环 + 航点状态机
│   ├── task/                # 任务注册表（TASK_RUNNERS 1–7）+ 任务逻辑
│   ├── arm/                 # 机械臂业务：视觉伺服、PID 闭环、S 曲线、软限位
│   ├── chassis/             # 底盘控制：50Hz 主循环、P/Stanley/curvature 控制器
│   └── misc/                # 独立脚本（射击、边走边打等）
│
├── smartcar/                # 硬件 SDK + 深度学习推理封装
│   ├── whalesbot/           # 车辆/机械臂驱动、SerialEngine、PID、摄像头工具
│   └── paddlebaidu/         # YOLO / OCR / Lane 推理 + ZMQ REQ/REP 通信
│
├── config_car.yml           # 摄像头通道、PID 增益、推理服务配置
├── task_config.yml          # 任务级校准：机械臂姿态、仓储位姿、航点列表
├── ecosystem.config.js      # PM2 生产进程定义 + 环境变量
├── run.py                   # 任务入口：完整任务流 / 单任务调试
└── test/                    # MC602 协议实验室（独立包，真实硬件在 --dangerous 模式下运行）
```

---

## 技术栈

- **运行时**：Python 3 / FastAPI / uvicorn / ZMQ
- **深度学习推理**：百度 PaddlePaddle Inference（TensorRT FP16/FP32）
- **视觉模型**：YOLOE（目标检测）、LaneBlend（车道线）、PP-OCRv3（文字识别）
- **自然语言处理**：ERNIE Bot（订单解析 / 语义理解）
- **控制算法**：麦克纳姆轮运动解算、PID 位置/速度闭环、S 曲线加减速
- **硬件接口**：WhalesBot SDK（MC602 串口协议、4-DOF 机械臂、舵机 PWM）
- **遥测与调试**：MJPEG 视频流、WebSocket 实时状态订阅、PM2 进程守护

---

## 核心能力

### 1. 自主导航

- 50 Hz 车道线外环循迹（双环架构：主循环 + 里程计线程）
- Stanley / Pure Pursuit / 自适应曲率控制器可切换
- 航点列表驱动，支持 IR 传感器 + 里程计双重触发
- 相对位移导航（`move_for`）规避麦克纳姆轮 odometry theta 漂移

### 2. 视觉感知

- 实时目标检测（侧向摄像头，YOLOE，30 Hz 缓存）
- 车道线检测（前置摄像头，128×128 推理，50 Hz 缓存）
- OCR 文字识别（订单 / 住户标签）
- 所有推理后端独立运行于子进程（ZMQ 通信），支持 LRU 自动卸载与 OOM 治理

### 3. 精密操作

- 4-DOF 机械臂：位置模式（goto_position）与速度模式（velocity mode）双回路
- 视觉伺服闭环：bbox → TargetSelector → depth-aware PID → S 曲线 → 复合动作
- 复合动作并行执行（ThreadPoolExecutor），支持夹取、放置、真空吸取
- 软限位 + 磁 safety gate + 目标丢失自动降速

### 4. 任务编排

- 完整任务流由 `Orchestrator` 驱动：循迹到触发点 → 暂停主循环 → 执行任务 → 恢复
- 单任务调试：`python run.py --task N`
- 循环频率可调：`python run.py --lane-hz 30`

---

## 配置说明

### 1. 环境要求

- Python 3.8+（Jetson 系统 Python）
- PaddlePaddle Inference（配合 JetPack CUDA 版本）
- PM2 进程管理器（生产环境）

### 2. 关键参数

编辑 `config_car.yml` 校准场地参数：

```yaml
# 摄像头通道（OpenCV 索引）
camera:
  front: 1    # 前视 / 车道
  side: 2     # 侧视 / 目标

# 速度限制（m/s 或 rad/s）
speed:
  x:
    limit: 0.7    # 横向
  y:
    limit: 0.7    # 纵向
  angle:
    limit: 3      # 角速度

# 推理后端端口
infer_cfg:
  lane: 5001
  task: 5002
  ocr: 5004
```

编辑 `task_config.yml` 校准任务级参数：

```yaml
task_cfg:
  task1:
    arm_pose: { arm_angle: 45, pitch: 90, side: "LEFT" }
    slot_map: [ ... ]
  waypoints:
    - { x: 0.0, y: 0.0, task: 1 }
    - { x: 1.2, y: 0.5, task: 2 }
```

### 3. 大模型 API

ERNIE 访问令牌配置于 `config_car.yml` 的 `ernie_access_token` 字段，或通过环境变量 `ERNIE_ACCESS_TOKEN` 注入。

---

## 核心 API

### MyCar 控制接口（通过 `/v1/execute` 调用）

| 动作 | 参数 | 说明 |
|------|------|------|
| `reset_position` | — | 里程计清零 |
| `get_odometry` | — | 获取当前坐标 (x, y, theta) |
| `move_for` | `[dx, dy, dz]` | 相对移动（米 / 弧度） |
| `lane_dis_offset` | `speed, dis_hold` | 巡线行驶指定距离 |
| `move_to_detection_target` | label | 视觉伺服对准目标 |
| `get_detection_results` | — | 获取最新检测框列表 |
| `get_ocr` | label, time_out | OCR 文字识别 |
| `set_storage` | state | 储物架抬升 / 下降 |
| `start_lane_feed` | — | 启动 50 Hz 车道缓存守护线程 |
| `stop_arm_feed` | force | 停止机械臂缓存（释放串口） |
| `start_arm_feed` | — | 启动 20 Hz 机械臂状态缓存 |

### 机械臂动作

| 动作 | 参数 | 说明 |
|------|------|------|
| `reset_position` | — | 机械臂复位（零点标定） |
| `set_arm_pose` | arm_id, pitch, side | 设置大臂姿态 |
| `grasp` | state | 气泵吸取（True）/ 释放（False） |
| `move_x_position` | mm | X 轴绝对位置移动 |
| `move_y_position` | mm | Y 轴绝对位置移动 |

**实时控制（bypass 队列，µs 级响应）**：

```bash
POST /v1/realtime/arm-velocity
{"x_vel": 0.0, "y_vel": 0.0}   # m/s，视觉伺服推荐模式
```

---

## 遥测与数据采集

### 实时视频流

车端启动后自动提供双路 MJPEG 流：

- cam1（前置 / 车道）：`/video_feed/cam1`
- cam2（侧向 / 目标）：`/video_feed/cam2`

浏览器访问 `http://<JETSON_IP>:5050/stream/` 可同时查看双路画面。

### WebSocket 实时状态

订阅 `/v1/ws` 可接收：

- `lane_state`（车道线偏移、曲率）
- `arm_state`（x/y 位置、编码器值）
- `task_state`（检测结果缓存）
- `ir_state`（红外传感器原始值）
- `odom_state`（轮速里程计）

### 数据采集模式

运行以下命令启动双摄像头数据采集与网页遥控：

```bash
python -m smartcar.whalesbot.tools.collect_control
```

浏览器访问 `http://<JETSON_IP>:5000/`，通过网页虚拟手柄或实体游戏手柄实时遥控机械臂，并保存图像数据集到 `./dataset/`。

---

## 并发模型（关键设计）

Runtime 采用双层引用锁 + 双队列 worker 架构，避免旧式单锁导致的资源争抢：

- **`_ref_lock`**：仅保护 `self.car` 引用替换（init / recover / close），微秒级
- **`_realtime_gate`**：`/v1/realtime/*` 端点入口直接取引用，绕过队列
- **`arm_queue` + `car_queue`**：机械臂长动作（1–3s PID 闭环）与底盘短动作隔离
- **SerialEngine**：单 IO 线程 + 帧队列，写合并（coalesce）+ 读共享（share）+ URGENT 插队

急停 / 取消后，必须 `POST /v1/control/reset-stop` + 重启 `start_lane_feed`，守护线程才会恢复。

---

## 故障排查

| 现象 | 可能原因 | 处置建议 |
|------|----------|----------|
| 摄像头读取失败 | 通道配置错误 / 连接不良 | 检查 `config_car.yml` 与物理连接 |
| 循迹偏离车道 | 光照突变 / 场地纹理干扰 | 降低巡线速度或调整 PID 增益 |
| 目标检测失败 | 摄像头角度偏移 / 目标超出视场 | 重新校准姿态，检查 `task_config.yml` |
| 机械臂抓取失败 | 气泵管路漏气 / 位置偏差 | 检查气密性，微调 arm_pose |
| 大模型解析超时 | 网络波动 / API 限流 | 检查 `ERNIE_ACCESS_TOKEN` 有效性 |
| MC602 掉线 | USB 重枚举 / 串口冲突 | Runtime 会自动恢复；检查 `RAK_CAR_AUTO_INIT` |

---

## 开发与测试

### 单元测试（离线，无需硬件）

```bash
# 机械臂业务（141 项）
/usr/bin/python3 -m unittest discover -s main/arm/tests -p 'test_*.py'

# 任务编排（14 项）
/usr/bin/python3 -m unittest discover -s main/task/tests -p 'test_*.py'

# 串口引擎（16 项，模拟模式）
RAK_CAR_SERIAL_AUTO_CONNECT=0 /usr/bin/python3 -m unittest smartcar.test.test_serial_engine -v
```

### 健康检查

```bash
curl -s http://192.168.6.231:5050/health           # 服务存活
curl -s http://192.168.6.231:5050/v1/infer/state   # 推理后端状态
curl -s http://192.168.6.231:5050/stream/health    # 视频流状态
```

---

## 贡献

欢迎提交 Issue 或 Pull Request。所有代码变更需通过单元测试，并在真车上完成物理验证。

## 许可证

MIT License
