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

- API: `http://192.168.3.60:5050`
- FastAPI 文档: `http://192.168.3.60:5050/docs`
- 视频流页面: `http://192.168.3.60:5050/stream/`
- cam1 MJPEG: `http://192.168.3.60:5050/video_feed/cam1`
- cam2 MJPEG: `http://192.168.3.60:5050/video_feed/cam2`

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

其中 API 对外端口默认复用 `BIND_PORT`，视频流也默认挂在同一个 runtime 服务里，不再单独维护独立 Flask 端口。
