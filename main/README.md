# main 最小业务层

`main/` 现在只保留最小可用集，目标就是一件事：

- 不关心底层源码细节，只通过 HTTP API 做真实业务开发
- 业务代码只允许放在 `main/`，并且只能通过接口控制
- 不允许直接依赖 `main/` 以外的任何 Python 源码

## 文件分工

- [settings.py](file:///home/jetson/workspace/rak-car/main/settings.py)
  - 统一配置入口，优先改这里或对应环境变量
- [api_client.py](file:///home/jetson/workspace/rak-car/main/api_client.py)
  - HTTP 同步调用客户端
- [ws_client.py](file:///home/jetson/workspace/rak-car/main/ws_client.py)
  - WebSocket 长连接客户端
- [quick_start.py](file:///home/jetson/workspace/rak-car/main/quick_start.py)
  - 开发自检脚本，先看配置，再测连通性
- [car_start_api.py](file:///home/jetson/workspace/rak-car/main/car_start_api.py)
  - 类似官方 `car_start_2026.py` 的 API 编排模板
- [BUSINESS_API_GUIDE.md](file:///home/jetson/workspace/rak-car/main/BUSINESS_API_GUIDE.md)
  - 业务层调用手册，只讲怎么控制和怎么闭环读取
- [CAPABILITY_LIST.md](file:///home/jetson/workspace/rak-car/main/CAPABILITY_LIST.md)
  - 能力总表
- [API_REFERENCE.md](file:///home/jetson/workspace/rak-car/main/API_REFERENCE.md)
  - 主文档，所有接口总表速查

## 配置

默认配置在 [settings.py](file:///home/jetson/workspace/rak-car/main/settings.py)：

- `RAK_CAR_SERVER_ORIGIN` 默认 `http://127.0.0.1`
- `RAK_CAR_API_PORT` 默认 `5050`
- `RAK_CAR_STREAM_PORT` 默认复用 `5050`
- `RAK_CAR_STREAM_PATH` 默认 `/stream/`
- `RAK_CAR_API_PREFIX` 默认 `/v1`
- `RAK_CAR_REQUEST_TIMEOUT` 默认 `10`
- `RAK_CAR_WAIT_TIMEOUT` 默认 `300`
- `RAK_CAR_POLL_INTERVAL` 默认 `0.5`
- `RAK_CAR_API_BASE`
  - 兼容旧写法，会覆盖自动拼出来的 API 地址
- `RAK_CAR_STREAMER_URL`
  - 兼容旧写法，会覆盖自动拼出来的推流地址

最常用的只改一个：

```bash
export RAK_CAR_SERVER_ORIGIN=http://192.168.3.60
```

这样会同时影响：

- API: `http://192.168.3.60:5050`
- Streamer: `http://192.168.3.60:5050/stream/`

安装依赖：

```bash
python3 -m pip install -r /home/jetson/workspace/rak-car/main/requirements.txt
```

## 建议开发顺序

```bash
export RAK_CAR_SERVER_ORIGIN=http://192.168.3.60
python3 /home/jetson/workspace/rak-car/main/quick_start.py
```

先确认联通和状态，再直接看 [API_REFERENCE.md](file:///home/jetson/workspace/rak-car/main/API_REFERENCE.md)。

## Python 调用

推荐只记一个类：[RuntimeApiClient](file:///home/jetson/workspace/rak-car/main/api_client.py)

最简单：

```python
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()
result = client.call("car", "beep", timeout=40)
print(result)
```

前进 5cm：

```python
client.call("car", "move_for", [0.05, 0.0, 0.0], timeout=60)
```

巡线前进 20cm：

```python
client.call("car", "lane_dis_offset", timeout=80, speed=0.3, dis_hold=0.2)
```

机械臂横向移动：

```python
client.call("arm", "move_x_position", 0.20, timeout=20)
```

接口总表和用途，直接看 [API_REFERENCE.md](file:///home/jetson/workspace/rak-car/main/API_REFERENCE.md)。

## 现成脚本

官方流程风格的 API 模板：

```bash
python3 /home/jetson/workspace/rak-car/main/car_start_api.py
```

这个脚本默认不直接动小车，只保留和 `car_start_2026.py` 一样的任务顺序模板。
你只要把需要的那几行取消注释，就能开始业务编排。

## curl 调用

最常用的就是 `POST /v1/execute`：

```bash
curl -sS -X POST http://127.0.0.1:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"car","name":"beep","timeout":40}'
```

所有接口总表，见 [API_REFERENCE.md](file:///home/jetson/workspace/rak-car/main/API_REFERENCE.md)。

如果你想先看“这台车现在到底会什么”，直接看 [CAPABILITY_LIST.md](file:///home/jetson/workspace/rak-car/main/CAPABILITY_LIST.md)。

## 你该怎么理解几类移动

- 纯底盘直控：
  - `move_for`
  - `move_to_position`
  - `move_time`
  - `move_distance`
- 智能巡线导航：
  - `lane_time`
  - `lane_dis`
  - `lane_dis_offset`
- 视觉对齐：
  - `move_to_detection_target`
- 机械臂控制：
  - `move_x_position`
  - `move_y_position`
  - `set_arm_angle`
  - `set_hand_angle`
  - `set_arm_pose`
  - `grasp`

区别和参数细节都写在 [API_REFERENCE.md](file:///home/jetson/workspace/rak-car/main/API_REFERENCE.md)。
