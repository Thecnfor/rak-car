# 双摄流 API 文档

这份文档只讲一件事：

- runtime 里的双摄实时画面怎么给前端用
- 怎么截当前帧
- 怎么一键保存并下载，方便采样训练新模型

## 地址约定

默认服务地址：

- API 根地址：`http://192.168.6.231:5050`
- 监控页：`http://192.168.6.231:5050/stream/`

摄像头别名：

- `cam1` = `front`
- `cam2` = `side`

默认截图保存目录：

- `cam1` -> `dataset/image_set_lane/runtime_capture/`
- `cam2` -> `dataset/image_set_object/runtime_capture/`

## 1. 什么时候用哪个接口

- 只想看实时画面：用 `/stream/` 或 `/video_feed/{cam_id}`
- 前端想拿可控的单帧 JPEG：用 `/stream/frame/{cam_id}.jpg`
- 想保存一张训练图：用 `POST /stream/capture`
- 想保存后立刻下载：用 `POST /stream/capture/{cam_id}/download`
- 想让前端先发现所有入口：用 `/stream/info`

## 2. 现有 MJPEG 接口到底合不合适

结论：

- 对“网页监控”是合适的
- 对“业务前端”和“训练采样”不够顺手

原因：

- `MJPEG` 适合直接挂 `<img src="...">` 看连续画面
- 但它不适合前端做细粒度状态管理
- 也不方便做“当前帧截图并保存”
- 也不方便拿结构化 JSON 描述各个入口

所以现在保留了原有能力，同时新增了：

- `stream info` JSON
- 单帧 JPEG
- 保存截图
- 保存并直接下载

## 3. 接口总览

### 3.1 `GET /stream/info`

用途：

- 返回当前流服务所有入口
- 前端不需要自己拼 URL

示例：

```bash
curl -sS http://127.0.0.1:5050/stream/info
```

返回示例：

```json
{
  "ok": true,
  "page_url": "http://127.0.0.1:5050/stream/",
  "health_url": "http://127.0.0.1:5050/stream/health",
  "keypress_url": "http://127.0.0.1:5050/keypress",
  "capture_url": "http://127.0.0.1:5050/stream/capture",
  "cameras": {
    "cam1": {
      "aliases": ["cam1", "front"],
      "mjpeg_url": "http://127.0.0.1:5050/video_feed/cam1",
      "frame_url": "http://127.0.0.1:5050/stream/frame/cam1.jpg",
      "capture_url": "http://127.0.0.1:5050/stream/capture",
      "capture_download_url": "http://127.0.0.1:5050/stream/capture/cam1/download",
      "default_capture_dir": "dataset/image_set_lane/runtime_capture"
    }
  }
}
```

### 3.2 `GET /stream/`

用途：

- 打开内置双摄监控页

示例：

```bash
xdg-open http://127.0.0.1:5050/stream/
```

### 3.3 `GET /stream/health`

用途：

- 查看流服务状态
- 看当前是否已经拿到 `cam1` / `cam2`

示例：

```bash
curl -sS http://127.0.0.1:5050/stream/health
```

返回示例：

```json
{
  "status": "running",
  "active_cams": ["cam1", "cam2"],
  "fps": 20.0,
  "quality": 80
}
```

### 3.4 `GET /video_feed/{cam_id}`

用途：

- 输出 MJPEG 连续视频流
- 最适合直接给 `<img>` 标签使用

支持：

- `cam1`
- `front`
- `cam2`
- `side`

示例：

```bash
curl -I http://127.0.0.1:5050/video_feed/cam1
```

前端示例：

```html
<img src="http://127.0.0.1:5050/video_feed/cam1" />
```

### 3.5 `GET /stream/frame/{cam_id}.jpg`

用途：

- 获取当前最新单帧 JPEG
- 比 MJPEG 更适合做前端按钮式刷新、缩略图、截图预览

参数：

- `download=1`
  - 让浏览器按附件下载

示例：

```bash
curl -o cam1.jpg http://127.0.0.1:5050/stream/frame/cam1.jpg
```

直接下载：

```bash
curl -OJ "http://127.0.0.1:5050/stream/frame/cam2.jpg?download=1"
```

前端示例：

```js
const imgUrl = "http://127.0.0.1:5050/stream/frame/cam1.jpg?t=" + Date.now();
document.querySelector("#preview").src = imgUrl;
```

### 3.6 `POST /stream/capture`

用途：

- 把当前帧保存到数据集目录
- 返回保存结果和下载地址

请求体：

```json
{
  "cam_id": "cam2",
  "prefix": "weed",
  "subdir": "session_01"
}
```

字段说明：

- `cam_id`
  - `cam1` / `front` / `cam2` / `side`
- `prefix`
  - 文件名前缀，默认 `capture`
- `subdir`
  - 额外子目录
  - 例如 `session_01`
  - 最终会保存到 `runtime_capture/session_01/`

示例：

```bash
curl -sS -X POST http://127.0.0.1:5050/stream/capture \
  -H 'Content-Type: application/json' \
  -d '{
    "cam_id":"cam2",
    "prefix":"weed",
    "subdir":"session_01"
  }'
```

返回示例：

```json
{
  "ok": true,
  "capture": {
    "cam_id": "cam2",
    "filename": "weed_cam2_20260712_201530_123.jpg",
    "file_path": "/home/jetson/workspace/rak-car/dataset/image_set_object/runtime_capture/session_01/weed_cam2_20260712_201530_123.jpg",
    "relative_path": "dataset/image_set_object/runtime_capture/session_01/weed_cam2_20260712_201530_123.jpg",
    "download_name": "weed_cam2_20260712_201530_123.jpg",
    "shape": [480, 640, 3],
    "saved_at": 1783858530.123,
    "subdir": "dataset/image_set_object/runtime_capture/session_01",
    "download_url": "/stream/captures/cam2/weed_cam2_20260712_201530_123.jpg?subdir=session_01",
    "frame_url": "/stream/frame/cam2.jpg"
  }
}
```

### 3.7 `POST /stream/capture/{cam_id}/download`

用途：

- 保存当前帧
- 立即以附件形式返回给前端下载

这是最适合“快捷保存 + 快速下载”的接口。

示例：

```bash
curl -X POST "http://127.0.0.1:5050/stream/capture/cam1/download" \
  -H 'Content-Type: application/json' \
  -d '{"prefix":"lane","subdir":"batch_a"}' \
  -o lane_latest.jpg
```

前端示例：

```js
async function captureAndDownload(camId) {
  const resp = await fetch(`/stream/capture/${camId}/download`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ prefix: "sample", subdir: "web" })
  });
  const blob = await resp.blob();
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = `${camId}.jpg`;
  a.click();
  URL.revokeObjectURL(url);
}
```

### 3.8 `GET /stream/captures/{cam_id}/{filename}`

用途：

- 下载已保存的截图

参数：

- `subdir`
  - 如果保存时用了子目录，这里也要传同一个值
- `download=1`
  - 默认按附件下载

示例：

```bash
curl -OJ "http://127.0.0.1:5050/stream/captures/cam2/weed_cam2_20260712_201530_123.jpg?subdir=session_01"
```

### 3.9 `POST /keypress`

用途：

- 给内置监控页回传键盘按键
- 一般只有监控页本身会用

## 4. 最推荐的前端接法

如果你是普通 Web 页，推荐这样分层：

- 实时大画面：`/video_feed/cam1`、`/video_feed/cam2`
- 缩略图或点击刷新：`/stream/frame/{cam_id}.jpg`
- 截图入库：`POST /stream/capture`
- 一键保存并下载：`POST /stream/capture/{cam_id}/download`
- 首次发现所有地址：`GET /stream/info`

## 5. 训练采样建议

推荐做法：

- 车道类数据从 `cam1` 保存
- 目标类数据从 `cam2` 保存
- 每轮采样用不同 `subdir`
- 每类样本用不同 `prefix`

例如：

- `cam1 + prefix=lane_left + subdir=day_01`
- `cam2 + prefix=weed + subdir=day_01`
- `cam2 + prefix=crop + subdir=day_01`

这样后续整理数据最轻松。

## 6. 常见问题

### 6.1 返回 `Waiting`

说明：

- runtime 还没初始化
- 或者当前 `MyCar` 还没拿到摄像头帧

先查：

```bash
curl -sS http://127.0.0.1:5050/v1/health?snapshot=1
curl -sS http://127.0.0.1:5050/stream/health
```

### 6.2 `capture` 返回 409

说明：

- 当前还没有最新帧
- 一般是摄像头未初始化或刚启动尚未取到画面

### 6.3 想直接让浏览器“点一下就保存+下载”

最直接：

- 前端按钮调用 `POST /stream/capture/{cam_id}/download`

这样一次请求就完成：

- 落盘
- 返回图片
- 浏览器下载
