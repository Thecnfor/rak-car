# 推理结果 API 文档

这份文档只讲 runtime 已经暴露出来的推理结果接口。

目标：

- 让前端直接拿结构化模型结果
- 不用再走通用 `/v1/execute`
- 一边拿 JSON，一边复用 `/stream/` 做预览

## 地址约定

- API 根地址：`http://192.168.3.60:5050`
- 预览页：`http://192.168.3.60:5050/stream/`

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

字段说明：

- `result.error`
  - 车道横向偏差
- `result.angle`
  - 车道角度偏差
- `frame_url`
  - 取当前推理后单帧

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
