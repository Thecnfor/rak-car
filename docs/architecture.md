# 整体架构

## 六层架构

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 5: 入口脚本层                                          │
│  main/qqq.py, main/main.py, car_start.py, important_car.py  │
│  竞赛方案编排、任务序列、菜单系统                                  │
├─────────────────────────────────────────────────────────────┤
│  Layer 4: 应用编排层                                          │
│  car_wrap.py (MyCar) — 中央调度器                              │
│  车道线跟随 + 目标定位 + OCR + 菜单UI + 调试                     │
├─────────────────────────────────────────────────────────────┤
│  Layer 3: 任务原语层                                          │
│  task_func.py (MyTask, Ejection)                             │
│  抓取、放置、弹射、种植、天气显示等原子操作                         │
├─────────────────────────────────────────────────────────────┤
│  Layer 2: 感知推理层                                          │
│  infer_cs/ (ZMQ 客户端/服务器)                                │
│  paddle_jetson/ (PaddlePaddle 模型封装)                       │
│  ernie_bot/ (大模型集成)                                      │
│  camera/ (摄像头采集)                                         │
├─────────────────────────────────────────────────────────────┤
│  Layer 1: 运动控制层                                          │
│  vehicle/driver/vehicle_base.py (CarBase, 运动学)             │
│  vehicle/arm/arm_base.py (ArmBase, 机械臂)                   │
├─────────────────────────────────────────────────────────────┤
│  Layer 0: 硬件驱动层                                          │
│  vehicle/base/controller_wrap.py (统一硬件抽象)                │
│  vehicle/base/serial_wrap.py (串口通信)                       │
│  vehicle/base/mc601_ctl2.py (MC601 协议)                     │
│  vehicle/base/mc602_ctl2.py (MC602 协议)                     │
└─────────────────────────────────────────────────────────────┘
```

## 依赖关系图

```
入口脚本 (main/qqq.py 等)
    │
    ├── import car_wrap.MyCar
    │       │
    │       ├── import vehicle (CarBase → Motors, WheelWrap, MotorConvert)
    │       ├── import task_func (MyTask → ArmBase, ServoBus, Ejection)
    │       ├── import camera (Camera)
    │       ├── import infer_cs (ClintInterface × 4)
    │       ├── import ernie_bot (ErnieBotWrap / OpenAiWrap)
    │       ├── import tools (PID, CountRecord, get_yaml)
    │       └── import log_info (logger)
    │
    └── import ernie_bot.base.answer_wenxin (或 answer)
```

## 核心数据流

### 1. 车道线跟随

```
Camera(前方) → frame
    │
    ▼
ClintInterface("lane").get_infer(frame)  →  ZMQ  →  LaneInfer
    │
    ▼
返回 [偏离距离, 偏离角度]
    │
    ▼
LanePidCal → PID(偏离距离) + PID(偏离角度)
    │
    ▼
CarBase.set_velocity(vx, vy, omega)  →  Motors → 单片机 → 电机
```

### 2. 目标检测定位

```
Camera(侧方) → frame
    │
    ▼
ClintInterface("task").get_infer(frame)  →  ZMQ  →  YoloeInfer
    │
    ▼
返回 [class_id, class_name, confidence, x, y, w, h]
    │
    ▼
lane_det_location_v4():
    过滤目标 → 计算像素误差 → PID(x_error) + PID(width_error)
    │
    ▼
CarBase.set_velocity() → 逐步逼近目标
    │
    ▼
CountRecord 连续 N 帧在阈值内 → 停车 → 返回 (成功, 距离)
```

### 3. OCR 文字识别

```
Camera(侧方) → frame
    │
    ▼
ClintInterface("task").get_infer(frame)  → 检测文字区域
    │
    ▼
裁剪 ROI → ClintInterface("ocr").get_infer(roi)  → 文字识别
    │
    ▼
difflib.SequenceMatcher 过滤相似结果 → 返回文本列表
```

### 4. AI 辅助决策

```
OCR 文本 → ErnieBotWrap / OpenAiWrap
    │
    ▼
PromptJson 构造结构化 Prompt (含 JSON Schema + Few-shot)
    │
    ▼
LLM 返回 JSON → get_json_str() 提取 → json.loads()
    │
    ▼
结构化结果 → 驱动任务逻辑 (选食物、算BMI、回答问题)
```

## 线程模型

```
主线程: 入口脚本 + 任务编排 + 运动控制循环
    │
    ├── Camera 后台线程 (daemon) — 持续抓帧，存入 self.frame
    │
    ├── Key4Btn 后台线程 (daemon) — 轮询按键状态
    │
    ├── InferServer 进程 (独立进程) — 4 个 ZMQ REP 线程
    │       ├── lane 线程 (port 5001)
    │       ├── task 线程 (port 5002)
    │       ├── front 线程 (port 5003)
    │       └── ocr 线程 (port 5004)
    │
    └── HRI 进程 (独立进程) — PySide2/QML 机器人表情
```

## 全局状态

| 变量 | 位置 | 作用 | 问题 |
|------|------|------|------|
| `serial_wrap` | `serial_wrap.py:352` | 全局串口实例 | import 时初始化硬件 |
| `ctl_id` | `controller_wrap.py:37` | 控制器类型 (0=MC601, 1=MC602) | import 时确定，不可变 |
| `serial_mc601` | `mc601_ctl2.py:17` | MC601 串口引用 | 模块级全局变量 |
| `serial_mc602` | `mc602_ctl2.py:21` | MC602 串口引用 | 模块级全局变量 |
| `logger` | `log_wrap.py:92` | 日志实例 | import 时创建 |
| `encoder_motor_all_sim1` | `mc601_ctl2.py:291` | MC601 模拟编码器 | 全局单例 |

## 模块职责表

| 模块 | 文件数 | 核心类 | 职责 |
|------|:---:|--------|------|
| `vehicle/base/` | 5 | `SerialWrap`, `Motors`, `ServoBus` | 硬件驱动、串口通信 |
| `vehicle/driver/` | 3 | `CarBase`, `OdometryBase` | 底盘运动学、里程计 |
| `vehicle/arm/` | 1 | `ArmBase` | 机械臂控制 |
| `camera/` | 1 | `Camera` | 摄像头采集 |
| `infer_cs/` | 2 | `InferServer`, `ClintInterface` | ZMQ 推理服务 |
| `paddle_jetson/` | 1 | `YoloeInfer`, `LaneInfer`, `OCRReco` | 模型封装 |
| `ernie_bot/` | 7 | `ErnieBotWrap`, `OpenAiWrap` | 大模型集成 |
| `task_func.py` | 1 | `MyTask`, `Ejection` | 任务原语 |
| `car_wrap.py` | 1 | `MyCar` | 中央调度（God Object） |
| `tools/` | 1 | `PID`, `CountRecord`, `get_yaml` | 工具函数 |
| `log_info/` | 1 | `logger` | 日志 |
| `main/` | 18 | 各入口脚本 | 竞赛编排 |
