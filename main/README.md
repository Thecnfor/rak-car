# main 最小业务层

`main/` 现在只保留最小可用集，目标就是一件事：

- 不关心底层源码细节，只通过 HTTP API 做真实业务开发
## 配置

默认配置在 [settings.py](file:///home/jetson/workspace/rak-car/main/settings.py)：

- `RAK_CAR_API_BASE` 默认 `http://192.168.3.60:5050`
- `RAK_CAR_API_PREFIX` 默认 `/v1`
- `RAK_CAR_REQUEST_TIMEOUT` 默认 `10`
- `RAK_CAR_WAIT_TIMEOUT` 默认 `300`
- `RAK_CAR_POLL_INTERVAL` 默认 `0.5`
- `RAK_CAR_STREAMER_URL` 默认 `http://192.168.3.60:5000/`

最常用的只有一个：

```bash
export RAK_CAR_API_BASE=http://192.168.3.60:5050
```

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

## curl 调用

最常用的就是 `POST /v1/execute`：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"car","name":"beep","timeout":40}'
```

更多可直接抄的请求体，见 [API_REFERENCE.md](file:///home/jetson/workspace/rak-car/main/API_REFERENCE.md)。

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
