# 推理结果 API 文档

这份文档只讲 runtime 已经暴露出来的推理结果接口。

目标：

- 让前端直接拿结构化模型结果
- 不用再走通用 `/v1/execute`
- 一边拿 JSON，一边复用 `/stream/` 做预览

## 地址约定

- API 根地址：`http://192.168.6.231:5050`
- 预览页：`http://192.168.6.231:5050/stream/`

摄像头别名：

- `cam1` = `front`
- `cam2` = `side`

## 当前模型状态

- `lane`：已启用，输入 `cam1`
- `task`：已启用，输入 `cam2`
- `ocr`：已启用，输入 `cam2`
- `front`：未启用，不建议接入前端

说明：

- `front` 虽然还保留在 `config_car.yml` 里，但当前业务没用，`MyCar` 也没有接入 `front_det`
- 所以 runtime 接口会明确把它标记为 `enabled=false`

## 推理托管说明

从现在开始，`infer_back_end.py` 由 `runtime` 统一托管。

这意味着：

- 不再依赖 legacy `ClintInterface` 自动拉起或重启推理后端
- `vision/*` 接口只消费 runtime 当前的推理状态
- 如果推理仍在冷启动，接口会返回“未就绪”语义，而不是无限等待

建议配套观察：

- `GET /v1/health`
- `GET /v1/infer/state`
- `GET /stream/health`

## 推理状态接口

### `GET /v1/infer/state`

用途：

- 看 runtime 当前托管的推理后端状态
- 区分“推理冷启动中”和“推理真正异常”
- 看各模型端口是否 ready

示例：

```bash
curl -sS http://127.0.0.1:5050/v1/infer/state
```

返回示例：

```json
{
  "ok": true,
  "infer": {
    "status": "ready",
    "managed": true,
    "process_running": true,
    "pid": 12345,
    "last_error": null,
    "models": [
      {"name": "lane", "port": 5001, "ready": true},
      {"name": "task", "port": 5002, "ready": true},
      {"name": "ocr", "port": 5004, "ready": true}
    ]
  }
}
```

字段说明：

- `status`
  - `starting` / `ready` / `stopped`
- `process_running`
  - runtime 当前托管的后端进程是否仍在
- `models[*].ready`
  - 各推理端口是否通过了 runtime 的健康探测

## 1. 模型总览

### `GET /v1/vision/models`

用途：

- 返回当前可用模型
- 明确哪些模型已启用，哪些只是配置残留

示例：

```bash
curl -sS http://127.0.0.1:5050/v1/vision/models
```

返回示例：

```json
{
  "ok": true,
  "models": [
    {
      "name": "lane",
      "enabled": true,
      "camera": "cam1",
      "camera_alias": "front",
      "return_schema": {
        "error": "float",
        "angle": "float"
      },
      "preview_frame_url": "/stream/frame/cam1.jpg"
    },
    {
      "name": "task",
      "enabled": true,
      "camera": "cam2",
      "camera_alias": "side",
      "return_schema": {
        "detections": "list"
      },
      "preview_frame_url": "/stream/frame/cam2.jpg"
    },
    {
      "name": "ocr",
      "enabled": true,
      "camera": "cam2",
      "camera_alias": "side",
      "return_schema": {
        "text": "string|null",
        "matched_detection": "object|null"
      },
      "preview_frame_url": "/stream/frame/cam1.jpg"
    },
    {
      "name": "front",
      "enabled": false,
      "reason": "当前业务未使用，MyCar 未接入 front_det"
    }
  ]
}
```

失败语义：

- 若推理未 ready，接口可能返回 `503`
- 此时优先看 `/v1/infer/state`，不要直接手工杀 `infer_back_end.py`

## 2. 车道结果

### `POST /v1/vision/lane`

用途：

- 触发一次前视巡线推理
- 返回结构化的 `error` 和 `angle`
- 同时把预览图更新到 `cam1`

请求体：

```json
{
  "timeout": 20
}
```

示例：

```bash
curl -sS -X POST http://127.0.0.1:5050/v1/vision/lane \
  -H 'Content-Type: application/json' \
  -d '{}'
```

返回示例：

```json
{
  "ok": true,
  "model": "lane",
  "camera": "cam1",
  "frame_url": "/stream/frame/cam1.jpg",
  "preview_url": "/stream/",
  "result": {
    "error": 0.00008048475137911737,
    "angle": -0.43544939160346985
  },
  "frame_shape": [480, 640, 3]
}
```

失败语义：

- 若控制器未就绪，会在健康接口中体现为 `state.components.controller.ready=false`
- 若推理未 ready，会返回 `503` 或 job 失败信息里包含“推理服务未就绪”

字段说明：

- `result.error`
  - 车道横向偏差
- `result.angle`
  - 车道角度偏差
- `frame_url`
  - 取当前推理后单帧

### `GET /v1/vision/lane/state`

用途：

- 读取 runtime 当前缓存的实时 lane 状态
- 最适合前端轮询
- 当你在执行 `task.auto_lane_tracing`、`car.lane_time`、`car.lane_dis`、`car.lane_dis_offset` 时，这个接口会持续更新

示例：

```bash
curl -sS http://127.0.0.1:5050/v1/vision/lane/state
```

返回示例：

```json
{
  "ok": true,
  "active": true,
  "mode": "tracking",
  "error_y": 0.0012,
  "error_angle": -0.4354,
  "forward_speed": 0.3,
  "lateral_speed": -0.012,
  "angular_speed": 0.086,
  "distance": 1.284,
  "frame_shape": [480, 640, 3],
  "updated_at": 1783863600.123,
  "frame_url": "/stream/frame/cam1.jpg",
  "preview_url": "/stream/"
}
```

失败语义：

- 如果只想确认相机是否还在出实时帧，优先看 `/stream/health`
- 如果 `detections` 正常但截图仍是占位图，说明是流服务链路问题，不一定是推理问题

字段说明：

- `active`
  - 当前是否处于巡线闭环中
- `mode`
  - `tracking` / `idle` / `stopped`
- `error_y`
  - 当前横向偏差
- `error_angle`
  - 当前角度偏差
- `forward_speed`
  - 当前前进速度
- `lateral_speed`
  - 当前横移修正速度
- `angular_speed`
  - 当前转向修正角速度
- `distance`
  - 当前累计行驶距离

### 外环专用：WebSocket `subscribe_lane`

> **外环 = 不在车端、但要稳定拿到 `(error_y, error_angle)` 做控制**。
> lane_feed 守护线程（20Hz 刷 lane_state）由 `runtime/services/runtime_service.py` 在 init 时**默认启动**，
> 不需要任何手动开关。直接连 WS 订阅 `lane_state` push 或 HTTP 轮询 `/v1/vision/lane/state` 即可。

#### WebSocket `subscribe_lane`（**推荐外环用这个**）

通过 `/v1/ws`（WebSocket）订阅服务端主动 push。

用法：

```python
import json, websocket
ws = websocket.create_connection('ws://192.168.6.231:5050/v1/ws')
ws.recv()  # welcome

ws.send(json.dumps({"op": "subscribe_lane"}))
print(json.loads(ws.recv()))  # {ok, subscribed, hz}

while True:
    msg = json.loads(ws.recv())
    if msg.get("op") == "lane_state":
        d = msg["data"]
        # d 跟 GET /v1/vision/lane/state 返回完全一致
        error_y, error_angle = d["error_y"], d["error_angle"]
        # ... 你的外环控制 ...
```

服务端行为：
- 后台起 asyncio 任务，按 20Hz 轮询 `get_lane_state()`，**只在 `updated_at` 变化时 push**
- 5 秒实测 ~80 次 push（受 lane_feed 频率上限约束）
- `op: unsubscribe_lane` 或连接断开时自动 cancel 任务

下发轮速用同一个 WS（不走 job_queue，50Hz 友好；走 car_lock 同步路径）：

**推荐：`op: realtime/chassis_velocity` 直接发 (vx, vy, wz)**，服务端 IK 反算 4 轮速、绕开 set_velocity 的里程计耦合：

```python
ws.send(json.dumps({
    "op": "realtime/chassis_velocity",
    "vx": 0.3, "vy": 0.0, "wz": 0.0    # m/s 和 rad/s
}))
```

**备选：`op: realtime/wheel_speeds` 直接发 4 轮速**（你自己算好 IK）：

```python
ws.send(json.dumps({
    "op": "realtime/wheel_speeds",
    "speeds": [v1, v2, v3, v4]   # 4 个轮速（m/s）
}))
```

HTTP 也行（同步路径一样）：

```bash
curl -X POST http://192.168.6.231:5050/v1/realtime/chassis-velocity \
     -H 'Content-Type: application/json' \
     -d '{"vx":0.3,"vy":0,"wz":0}'
```

#### 完整外环骨架（Python）

```python
import json, websocket

ws = websocket.create_connection('ws://192.168.6.231:5050/v1/ws')
ws.recv()  # welcome

ws.send(json.dumps({"op": "subscribe_lane"}))
ws.recv()  # subscribe ack

while True:
    msg = json.loads(ws.recv())
    if msg.get("op") != "lane_state":
        continue
    d = msg["data"]
    ey, ea = d["error_y"], d["error_angle"]
    if ey is None: continue
    # === 你的外环控制律 ===
    vx = 0.3                          # 前向速度
    vy = -0.5 * ey                    # 横移修正
    wz = -0.8 * ea                    # 转向修正
    # 直发 (vx, vy, wz)，服务端 IK 反算 4 轮速
    ws.send(json.dumps({
        "op": "realtime/chassis_velocity",
        "vx": vx, "vy": vy, "wz": wz,
    }))
```

## 3. 目标检测结果

### `POST /v1/vision/task`

用途：

- 触发一次侧视目标检测
- 返回结构化检测框
- 同时把画框结果更新到 `cam2`

请求体：

```json
{
  "sort_pos": [0, 0],
  "limit_x": 1,
  "limit_y": 1,
  "timeout": 20
}
```

字段说明：

- `sort_pos`
  - 用于按目标距离某个归一化点排序
- `limit_x`
  - 只保留 `abs(x_center) <= limit_x` 的框
- `limit_y`
  - 只保留 `abs(y_center) <= limit_y` 的框
- `timeout`
  - 本次等待上限

示例：

```bash
curl -sS -X POST http://127.0.0.1:5050/v1/vision/task \
  -H 'Content-Type: application/json' \
  -d '{
    "sort_pos":[0,0],
    "limit_x":1,
    "limit_y":1,
    "timeout":20
  }'
```

返回示例：

```json
{
  "ok": true,
  "model": "task",
  "camera": "cam2",
  "frame_url": "/stream/frame/cam2.jpg",
  "preview_url": "/stream/",
  "filters": {
    "sort_pos": [0, 0],
    "limit_x": 1,
    "limit_y": 1
  },
  "count": 1,
  "detections": [
    {
      "index": 0,
      "class_id": 3,
      "track_id": 0,
      "label": "order",
      "score": 0.98,
      "bbox_norm": {
        "x_center": 0.11,
        "y_center": -0.03,
        "width": 0.22,
        "height": 0.15
      },
      "bbox_pixels": {
        "x1": 289,
        "y1": 201,
        "x2": 359,
        "y2": 237,
        "width": 70,
        "height": 36
      }
    }
  ],
  "frame_shape": [480, 640, 3]
}
```

最关键的数据：

- `label`
- `score`
- `bbox_norm`
- `bbox_pixels`

如果你要做业务处理，优先用：

- `bbox_norm`
  - 方便跨分辨率
- `bbox_pixels`
  - 方便直接裁图、绘制、前端定位

## 4. OCR 结果

### `POST /v1/vision/ocr`

用途：

- 先做一次 `task` 检测
- 找到匹配 `label` 的文本区域
- 再做 OCR
- 返回识别文本和对应检测框

请求体：

```json
{
  "label": "order",
  "sort_pos": [0, 0],
  "limit_x": 1,
  "limit_y": 1,
  "timeout": 20
}
```

字段说明：

- `label`
  - 常用是 `order` 或 `name`
  - 不传时，默认接受 `order/name`

示例：

```bash
curl -sS -X POST http://127.0.0.1:5050/v1/vision/ocr \
  -H 'Content-Type: application/json' \
  -d '{
    "label":"order",
    "sort_pos":[0,0],
    "limit_x":1,
    "limit_y":1,
    "timeout":20
  }'
```

命中时返回结构：

```json
{
  "ok": true,
  "model": "ocr",
  "camera": "cam2",
  "frame_url": "/stream/frame/cam1.jpg",
  "source_frame_url": "/stream/frame/cam2.jpg",
  "preview_url": "/stream/",
  "label": "order",
  "text": "苹果两个",
  "matched_detection": {
    "index": 0,
    "class_id": 3,
    "track_id": 0,
    "label": "order",
    "score": 0.98,
    "bbox_norm": {
      "x_center": 0.11,
      "y_center": -0.03,
      "width": 0.22,
      "height": 0.15
    },
    "bbox_pixels": {
      "x1": 289,
      "y1": 201,
      "x2": 359,
      "y2": 237,
      "width": 70,
      "height": 36
    }
  },
  "detections": []
}
```

未命中时返回结构：

```json
{
  "ok": true,
  "model": "ocr",
  "camera": "cam2",
  "frame_url": "/stream/frame/cam1.jpg",
  "preview_url": "/stream/",
  "label": "order",
  "text": null,
  "matched_detection": null,
  "detections": [],
  "message": "当前画面未找到匹配的 OCR 检测框"
}
```

注意：

- `frame_url` 指向 `cam1`
  - 这里展示的是 OCR 裁剪预览
- `source_frame_url` 指向 `cam2`
  - 这里是侧视原图来源

## 5. 前端推荐接法

推荐流程：

1. 首次进入页面，先请求 `/v1/vision/models`
2. 选择模型后，调用对应 `POST` 接口
3. 结果 JSON 直接做业务处理
4. 预览图直接读 `frame_url`
5. 如果要实时看两路画面，同时挂 `/stream/` 或 `/video_feed/{cam_id}`

最常见的三个按钮：

- `检测一次` -> `POST /v1/vision/task`
- `OCR 一次` -> `POST /v1/vision/ocr`
- `巡线状态` -> `POST /v1/vision/lane`

## 6. 什么时候还要用 `/v1/execute`

只有在这几种情况下才建议继续用通用执行接口：

- 你要调用底层运动动作
- 你要执行任务函数
- 你要临时调试一个还没做成专用接口的方法

如果只是拿模型结果：

- 优先用 `/v1/vision/*`
