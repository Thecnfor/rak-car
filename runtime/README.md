# runtime 工程说明

这里是常驻式 API 服务的工程目录。

## 目录结构

- `server.py`: `uvicorn` 启动入口
- `requirements.txt`: runtime 服务依赖
- `STREAM_API.md`: 双摄流、单帧、截图保存下载接口文档
- `VISION_API.md`: 推理结果结构化接口文档
- `api/`: FastAPI 装配层
- `api/app.py`: 应用创建与启动钩子
- `api/routes.py`: `v1` 与 legacy 路由
- `core/`: 配置与动作注册
- `core/settings.py`: 环境变量与运行时配置
- `core/actions.py`: task/car/arm/system 动作注册表
- `services/`: 服务编排层
- `services/runtime_service.py`: `MyCar()` 生命周期、任务队列、动作分发
- `services/camera_stream_service.py`: runtime 内置双摄画面缓存与 MJPEG 输出
- `hardware/`: 硬件探测层
- `hardware/controller_probe.py`: 下位机探测与自动恢复

## 先安装依赖

```bash
/usr/bin/python3 -m pip install -r /home/jetson/workspace/rak-car/runtime/requirements.txt
```

## 本地启动

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m runtime.server
```

## PM2 启动

```bash
pm2 start ecosystem.config.js
pm2 logs rak-car-api
```

## 默认访问地址

- API: `http://192.168.6.231:5050`
- FastAPI 文档: `http://192.168.6.231:5050/docs`
- 视频流页面: `http://192.168.6.231:5050/stream/`
- cam1 MJPEG: `http://192.168.6.231:5050/video_feed/cam1`
- cam2 MJPEG: `http://192.168.6.231:5050/video_feed/cam2`

更完整的流媒体接口说明见：

- [STREAM_API.md](file:///home/jetson/workspace/rak-car/runtime/STREAM_API.md)
- [VISION_API.md](file:///home/jetson/workspace/rak-car/runtime/VISION_API.md)

## 快速修改 IP

多人协作时，优先改:

```python
/home/jetson/workspace/rak-car/runtime/core/settings.py
```

优先关注:

- `BIND_HOST`
- `BIND_PORT`
- `PUBLIC_HOST`
- `PUBLIC_STREAM_PORT`
- `PUBLIC_STREAM_PATH`

## 当前推荐的环境变量

- `RAK_CAR_BIND_HOST`: 服务监听地址，默认 `0.0.0.0`
- `RAK_CAR_BIND_PORT`: API 监听端口，默认 `5050`
- `RAK_CAR_PUBLIC_HOST`: 返回给同事访问的统一主机地址，API 和视频流共用
- `RAK_CAR_PUBLIC_STREAM_PORT`: 视频流对外端口，默认复用 `BIND_PORT`
- `RAK_CAR_PUBLIC_STREAM_PATH`: 视频流页面路径，默认 `/stream/`
- `RAK_CAR_INFER_AUTO_START`: runtime 启动后是否统一托管推理后端，默认 `1`
- `RAK_CAR_INFER_POLL_INTERVAL`: 推理后端健康轮询间隔，默认 `1.0`
- `RAK_CAR_INFER_READY_TIMEOUT`: runtime 等待推理后端 ready 的超时时间，默认 `45`
- `RAK_CAR_INFER_HEALTH_TIMEOUT`: 单次推理健康探测超时，默认 `2.0`

其中 API 对外端口默认复用 `BIND_PORT`，视频流也默认挂在同一个 runtime 服务里，不再单独维护独立 Flask 端口。

## 推理托管边界

`runtime` 现在不仅托管 `MyCar()`，也统一托管 `infer_back_end.py`。

这意味着：

- `runtime` 是唯一允许拉起/停止推理后端的服务
- legacy `ClintInterface` 只保留 ZMQ 客户端职责，不再负责自启、自杀、重启 `infer_back_end.py`
- 业务层和前端只通过 API 看推理状态，不再依赖隐式脚本行为

推荐排障顺序：

1. `GET /v1/health`
2. `GET /v1/infer/state`
3. `GET /stream/health`
4. 再看具体 `vision/*` 或 `execute` 接口

## 组件隔离

runtime 按三条链路分别管理：

- 控制器链路：`controller_session` / `MyCar()` 初始化
- 推理链路：`infer_back_end.py` + ZMQ 健康状态
- 摄像头链路：`camera_stream_service.py` 双摄缓存与 MJPEG 输出

设计要求：

- 推理未 ready，不应把摄像头流线程带死
- 控制器暂时掉线，不应把流服务和推理状态接口带死
- 摄像头无新帧时，仍然保留健康接口、占位图与最近错误元信息

## 失败语义

看 `GET /v1/health` 时重点关注：

- `state.components.controller`
  - 下位机是否在 `PROGRAM_READY`
- `state.components.infer`
  - 推理是否 `ready`
- `state.components.camera`
  - 当前是否存在活跃相机帧

新增只读接口：

- `GET /v1/infer/state`
  - 查看 runtime 托管的推理进程状态、最近错误、各模型端口是否 ready

常见判断：

- `controller.ready=false`
  - 优先排查下位机 / 串口 / program 模式
- `infer.ready=false`
  - 优先排查推理后端冷启动、模型初始化、端口探测
- `camera.ready=false`
  - 优先排查摄像头节点、帧是否 stale、相机重开是否持续失败

## 运维建议

- 修改 `ecosystem.config.js` 里的 runtime 入口或推理相关环境变量后，用 `pm2 delete rak-car-api && pm2 start ecosystem.config.js` 刷新注册信息
- 冷启动阶段优先看 `v1/infer/state`，不要再根据旧的 `infer_front.py` 输出判断推理是否需要重启
- 如果 `vision/*` 返回 503，优先确认推理后端是否仍在冷启动，而不是直接 kill `infer_back_end.py`

## 实时硬件控制

`runtime` 服务除了常规的 `execute` 作业队列外，还提供 `/v1/realtime/*` 一组 HTTP 端点以及 `realtime/*` 一组 WebSocket op。它们走 `car_lock` 同步路径、不进 `job_queue`，50Hz 高频场景下延迟可控。

涵盖：4 轮线速度/编码器、单电机、步进电机、总线舵机（含读角度）、模拟量。

详细端点表参见 `main/API_REFERENCE.md` 中「实时硬件控制（50Hz 直达路径）」章节。
