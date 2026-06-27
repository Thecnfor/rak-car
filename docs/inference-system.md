# 推理系统

> ZMQ 客户端-服务器架构，PaddlePaddle 模型部署。

## 整体架构

```
┌─────────────────┐    ZMQ REQ     ┌─────────────────────────┐
│  主进程          │ ──────────────→ │  InferServer (独立进程)    │
│  ClintInterface  │ ←────────────── │                         │
│  (客户端)        │    ZMQ REP     │  4 个推理线程             │
└─────────────────┘                 │  ├── lane   (port 5001) │
                                    │  ├── task   (port 5002) │
                                    │  ├── front  (port 5003) │
                                    │  └── ocr    (port 5004) │
                                    └─────────────────────────┘
```

## 启动流程

```python
# ClintInterface.__init__() 自动启动流程：
1. 读取 infer.yaml 配置
2. 检查 infer_back_end.py 是否已在运行 (psutil.process_iter)
3. 如果没有 → 启动子进程: python3 infer_back_end.py &
4. 循环发送 ATATA 健康检查，等待服务就绪
5. 建立 ZMQ REQ 连接
```

## ZMQ 通信协议

### 健康检查

```
客户端 → 服务端:  b"ATATA"  (5字节)
服务端 → 客户端:  JSON(True) 或 JSON(False)
```

`False` 表示模型还在加载中。

### 推理请求

```
客户端 → 服务端:  b"image" + JPEG字节  (5字节头 + 图像数据)
服务端 → 客户端:  JSON(推理结果)
```

- 图像以 JPEG 编码传输（可选客户端预缩放）
- 响应以 UTF-8 编码的 JSON 字节返回

## 推理服务配置

`infer_cs/base/infer.yaml`：

```yaml
infer_list:
  - name: lane
    type: LaneInfer
    args:
      model_dir: "paddle_jetson/base/lane_model"
    port: 5001
    img_size: [128, 128]

  - name: task
    type: YoloeInfer
    args:
      model_dir: "paddle_jetson/base/task_wbt2025"
      run_mode: "paddle"
    port: 5002
    img_size: [416, 416]

  - name: front
    type: YoloeInfer
    args:
      model_dir: "paddle_jetson/base/front_model2"
      run_mode: "paddle"
    port: 5003
    img_size: [416, 416]

  - name: ocr
    type: OCRReco
    args:
      det_model_dir: "paddle_jetson/base/ch_PP-OCRv3_det_infer"
      rec_model_dir: "paddle_jetson/base/ch_PP-OCRv3_rec_infer"
    port: 5004
    img_size: null
```

## 各推理服务详解

### LaneInfer — 车道线分割 (port 5001)

- **输入：** 128×128 RGB 图像
- **输出：** 车道线偏移距离和角度
- **模型：** 自训练 CNN 车道线模型
- **GPU 内存：** 限制 100MB
- **预处理：** resize → normalize [-1,1] → BGR→RGB → HWC→CHW

### YoloeInfer — 目标检测 (port 5002, 5003)

- **输入：** 416×416 RGB 图像
- **输出：** `List[Bbox]` — 检测框列表
- **模型：** PaddleDetection YOLOE
- **推理后端：** 支持 `paddle` / `trt_fp16` / `trt_fp32`

**Bbox 数据结构：**
```python
class Bbox:
    cls_id: int        # 类别 ID
    cls_name: str      # 类别名称
    confidence: float  # 置信度 (0~1)
    x: float           # 中心 x (归一化 -1~1)
    y: float           # 中心 y (归一化 -1~1)
    w: float           # 宽度 (归一化)
    h: float           # 高度 (归一化)
```

**坐标系：** 归一化到 [-1, 1]，原点在图像中心。

### OCRReco — 文字识别 (port 5004)

- **输入：** 任意尺寸 RGB 图像
- **输出：** `List[(text, confidence)]` — 识别文本列表
- **流程：** 文字检测(PP-OCRv3_det) → 透视裁剪 → 文字识别(PP-OCRv3_rec)
- **置信度阈值：** 0.5

### 模型标签

#### task_wbt2025 (port 5002)

| cls_id | 类别 | 用途 |
|:------:|------|------|
| 0 | cauliflower | 食材-花菜 |
| 1 | chili | 食材-辣椒 |
| 2 | tofu | 食材-豆腐 |
| 3 | tomato | 食材-番茄 |
| 4 | meat | 食材-肉 |
| 5 | egg | 食材-鸡蛋 |
| 6 | mushroom | 食材-蘑菇 |
| 7 | turn_right | 指令卡-右转 |
| 8 | turn_left | 指令卡-左转 |
| 9 | text_det | 文字区域 |
| 10 | cylinder1 | 圆柱体-小 |
| 11 | cylinder2 | 圆柱体-中 |
| 12 | cylinder3 | 圆柱体-大 |
| ... | ... | ... |

> **⚠️ 具体 cls_id 以模型配置文件 `infer_cfg.yml` 为准，不同版本可能不同！**

#### front_model2 (port 5003)

| cls_id | 类别 |
|:------:|------|
| 0 | turn_left |
| 1 | turn_right |

## 客户端使用方法

```python
from infer_cs import ClintInterface

# 创建客户端（自动启动服务端）
lane_client = ClintInterface("lane")
task_client = ClintInterface("task")
ocr_client = ClintInterface("ocr")

# 推理
frame = camera.read()
results = task_client.get_infer(frame)       # → List[Bbox]
lane_result = lane_client.get_infer(frame)   # → [distance, angle]
ocr_result = ocr_client.get_infer(frame)     # → List[(text, conf)]
```

## 模型热加载

`InferServer.__init__()` 启动时加载所有模型，并做 3 次预热：

```python
for name in infer_list:
    model = create_model(name)       # 加载模型
    for _ in range(3):               # 预热 3 次
        model.infer(blank_image)
```

预热完成前，健康检查返回 `False`。

## 已知问题

| 问题 | 严重性 | 说明 |
|------|:---:|------|
| 服务端线程无 try/except | 🔴 | 推理异常直接杀死线程 |
| `close()` 调用不存在方法 | 🟠 | `thread.close()` 在 Python 中不存在 |
| ZMQ Context 未清理 | 🟡 | 每个 socket 独立创建 context |
| 无重连机制 | 🟡 | 连接断开后无法恢复 |
| 健康检查无超时 | 🟡 | 客户端无限等待 |
| `DetectResult` 硬编码 640×480 | 🟡 | 裁剪框不适用于其他分辨率 |
