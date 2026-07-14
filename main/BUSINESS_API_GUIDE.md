# main 业务 API 编排手册

这份文档只面向 `main/` 的使用者。

默认前提：

- 你不用了解驱动层、串口层、推理层内部实现
- 你只需要知道怎么在 `main/` 里发动作、读状态、做闭环
- 你写的是业务脚本，不是底层驱动

如果你要看完整参数速查，请看 [API_REFERENCE.md](file:///home/jetson/workspace/rak-car/main/API_REFERENCE.md)。
如果你要看当前到底暴露了哪些能力，请看 [CAPABILITY_LIST.md](file:///home/jetson/workspace/rak-car/main/CAPABILITY_LIST.md)。

## 1. 先只记住这 4 个文件

- [settings.py](file:///home/jetson/workspace/rak-car/main/settings.py)
  - 配 API 地址、超时、轮询间隔
- [api_client.py](file:///home/jetson/workspace/rak-car/main/api_client.py)
  - 同步 HTTP 调用入口，绝大多数业务只用它
- [ws_client.py](file:///home/jetson/workspace/rak-car/main/ws_client.py)
  - WebSocket 长连接入口，适合高频控制或持续状态拉取
- [car_start_api.py](file:///home/jetson/workspace/rak-car/main/car_start_api.py)
  - 现成业务编排模板，适合从比赛任务流改成自己的流程

最常用环境变量只改一个：

```bash
export RAK_CAR_SERVER_ORIGIN=http://192.168.6.231
```

安装依赖：

```bash
python3 -m pip install -r /home/jetson/workspace/rak-car/main/requirements.txt
```

联通性自检：

```bash
python3 /home/jetson/workspace/rak-car/main/quick_start.py
```

## 2. 业务层怎么理解这套 API

从业务视角，只分三类：

- `car`
  - 底盘动作、传感器读取、视觉读取、系统外围控制
- `arm`
  - 机械臂位置、姿态、抓取
- `task`
  - 已封装好的比赛级流程

统一调用入口只有一个：

- `POST /v1/execute`

请求体固定格式：

```json
{
  "target": "car",
  "name": "move_for",
  "args": [[0.05, 0.0, 0.0]],
  "kwargs": {},
  "timeout": 60
}
```

字段语义：

- `target`
  - 调哪一层，`car` / `arm` / `task`
- `name`
  - 动作名
- `args`
  - 位置参数
- `kwargs`
  - 关键字参数
- `timeout`
  - 这次动作最多等多久

服务端已经帮你做了这几件事：

- 动作前自动等小车初始化完成
- 控制器掉线后自动尝试恢复
- `execute` 模式下直接等待动作执行结束

所以业务层最推荐的写法不是自己管底层状态，而是：

1. 读一次健康状态
2. 发动作
3. 再读一次闭环结果确认是否到位

## 3. Python 最推荐写法

只用 [RuntimeApiClient](file:///home/jetson/workspace/rak-car/main/api_client.py)：

```python
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()
client.wait_until_ready()

client.call("car", "beep", timeout=20)
client.call("car", "move_for", [0.05, 0.0, 0.0], timeout=60)
print(client.call("car", "get_odometry", timeout=20))
```

你可以这样理解几个常用方法：

- `client.call(target, name, *args, timeout=..., **kwargs)`
  - 最顺手，适合 90% 业务脚本
- `client.execute(target, name, args=None, kwargs=None, timeout=None)`
  - 显式传 `args/kwargs`，适合动态组包
- `client.create_job(...) + client.wait_job(...)`
  - 适合你想先创建任务、后面再轮询
- `client.get_health()`
  - 看 runtime 是否就绪
- `client.get_runtime()`
  - 看里程计、累计距离、停机标记
- `client.get_actions()`
  - 动态查看当前允许调哪些动作

## 4. 先学会怎么选接口

如果你想做这些事，优先选这些 API：

- 纯前进 5cm / 横移 3cm / 转 90 度
  - `car.move_for`
- 回到某个已经记录过的位置
  - `car.move_to_position`
- 跟着赛道继续往前走
  - `car.lane_dis_offset`
- 粗略按速度跑一小段时间
  - `car.move_time`
- 视觉对齐到侧摄目标
  - `car.move_to_detection_target`
- 机械臂横轴/竖轴定位
  - `arm.move_x_position` / `arm.move_y_position`
- 一次设机械臂整体姿态
  - `arm.set_arm_pose`
- 跑完整比赛任务
  - `task.*`

一句话判断：

- 想要里程计闭环位移，用 `move_for` / `move_to_position`
- 想要巡线闭环前进，用 `lane_dis_offset`
- 想要视觉闭环对齐，用 `move_to_detection_target`
- 想要机械臂闭环读取，就配合 `get_arm_state`、`x_get_position`、`y_get_position`

## 5. 业务层最常用的闭环读取接口

你真正写业务时，不是只会“发动作”，还要“读反馈”。

最常用的读取如下。

### 5.1 runtime 级状态

- `client.get_health()`
  - 看服务在线、是否初始化完成、控制器是否健康
- `client.get_runtime()`
  - 看当前 runtime 快照

`GET /v1/runtime` 返回重点字段：

```json
{
  "ok": true,
  "runtime": {
    "odometry": [0.0, 0.0, 0.0],
    "distance": 0.0,
    "stop_after_action": false,
    "stop_flag": false
  }
}
```

适合：

- 动作完成后二次确认位姿
- 巡线后确认累计距离
- 确认当前是否处于 stop 状态

### 5.2 底盘闭环读取

- `car.get_odometry`
  - 返回 `[x, y, theta]`
- `car.get_distance`
  - 返回累计行驶距离
- `car.get_lane_results`
  - 返回 `(error_y, error_angle)`

适合：

- `move_for` / `move_to_position` 后复核位置
- 巡线过程中做人自己的上层判定
- 做额外的状态显示或日志记录

### 5.3 视觉闭环读取

- `car.get_detection_results`
  - 返回检测列表，每项是 `[cls_id, det_id, label, score, x_c, y_c, w, h]`
- `car.get_ocr`
  - 返回 OCR 文本或 `None`

适合：

- 先看当前看到了什么，再决定要不要发 `move_to_detection_target`
- 对齐前做标签筛选
- 取到订单、姓名、类别之后再继续编排

### 5.4 机械臂闭环读取

- `car.get_arm_state`
  - 返回：

```json
{
  "x": 0.0,
  "y": 0.0,
  "side": "RIGHT",
  "arm_angle": 94,
  "hand_angle": 120,
  "y_limit": false
}
```

- `arm.x_get_position`
  - 返回当前横轴位置
- `arm.y_get_position`
  - 返回当前竖轴位置

适合：

- 动作前后确认机械臂是否真的到位
- 判断当前是左侧抓还是右侧抓
- 记录抓取流程中的臂姿状态

### 5.5 传感器与外围读取

- `car.get_all_ir_distance`
  - 返回 `{"left": ..., "right": ...}`
- `car.get_battery_voltage`
  - 返回当前电池电压
- `car.get_key_event`
  - 返回按键事件
- `car.get_key_state`
  - 返回按键状态
- `car.get_bluetooth_pad`
  - 返回手柄状态

适合：

- 业务流程前做环境检测
- 低电压保护
- 人工接管或调试交互

## 6. 最常见 5 种业务闭环模板

### 6.1 模板一：位移后复核里程计

```python
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()
client.wait_until_ready()

before = client.call("car", "get_odometry", timeout=20)["result"]
client.call("car", "move_for", [0.05, 0.0, 0.0], timeout=60)
after = client.call("car", "get_odometry", timeout=20)["result"]

print("before:", before)
print("after :", after)
```

适合：

- 精确前进/横移/旋转
- 不依赖赛道线

### 6.2 模板二：巡线后复核距离

```python
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()
client.wait_until_ready()

start_distance = client.call("car", "get_distance", timeout=20)["result"]
client.call("car", "lane_dis_offset", timeout=80, speed=0.3, dis_hold=0.2)
end_distance = client.call("car", "get_distance", timeout=20)["result"]

print("delta_distance:", end_distance - start_distance)
```

适合：

- 沿赛道自动推进
- 业务层想确认到底走了多远

### 6.3 模板三：先读检测，再做视觉对齐

```python
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()
client.wait_until_ready()

dets_job = client.call(
    "car",
    "get_detection_results",
    timeout=20,
    sort_pos=[0, 0],
    limit_x=0.6,
    limit_y=1.0,
)
dets = dets_job["result"]
print("dets:", dets)

if dets:
    align_job = client.call(
        "car",
        "move_to_detection_target",
        timeout=50,
        delta_x=0.0,
        delta_y=None,
        label=dets[0][2],
        time_out=3.0,
    )
    print("align:", align_job["result"])
```

适合：

- 业务先看是不是目标标签
- 不想盲调 `move_to_detection_target`

### 6.4 模板四：机械臂动作后复核位置

```python
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()
client.wait_until_ready()

client.call("arm", "move_x_position", 0.20, timeout=20)
state = client.call("car", "get_arm_state", timeout=20)["result"]
x_now = client.call("arm", "x_get_position", timeout=20)["result"]

print("arm_state:", state)
print("x_now:", x_now)
```

适合：

- 抓取、放置、扫描前确认臂姿
- 调节 `x/y` 行程时做闭环记录

### 6.5 模板五：任务返回值继续编排

```python
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()
client.wait_until_ready()

animal_job = client.execute_task("target_shooting_detection", timeout=240)
animal_list = animal_job["result"]

if animal_list:
    client.execute_task(
        "target_shooting",
        timeout=240,
        animal_list=animal_list,
    )

order_job = client.execute_task("get_order", timeout=300)
order_list = order_job["result"]

if order_list:
    client.execute_task(
        "order_delivery",
        timeout=300,
        order_list=order_list,
    )
```

适合：

- 上层只做任务编排，不关心底层具体实现
- 把比赛流程拆成“先识别，再执行”

## 7. 什么时候该用 HTTP，什么时候该用 WebSocket

优先用 HTTP：

- 单步动作
- 顺序业务编排
- 大部分脚本化调用

优先用 WebSocket：

- 高频底盘速度控制
- 持续轮询状态
- 需要一条长连接反复发命令

WebSocket 地址：

- `ws://<ip>:5050/v1/ws`

最常用消息：

```json
{"op":"health","request_id":"h1"}
{"op":"runtime","request_id":"r1"}
{"op":"execute","request_id":"e1","target":"car","name":"set_chassis_velocity","kwargs":{"x":0.1,"y":0.0,"z":0.0,"duration":0.2}}
```

## 8. 给业务同学的最小规则

只记下面这些就够了：

- 不要直接碰驱动层文件，业务代码只写在 `main/`
- 优先用 `client.call(...)` 和 `client.execute_task(...)`
- 位移动作后，优先读 `get_odometry` / `get_distance`
- 视觉动作前，优先读 `get_detection_results`
- 机械臂动作后，优先读 `get_arm_state` / `x_get_position` / `y_get_position`
- 需要确认服务状态时，先看 `get_health()` 和 `get_runtime()`

## 9. 最后怎么落地

如果你现在要开始写自己的业务，最推荐顺序是：

1. 先改 `RAK_CAR_SERVER_ORIGIN`
2. 跑 [quick_start.py](file:///home/jetson/workspace/rak-car/main/quick_start.py)
3. 参考 [car_start_api.py](file:///home/jetson/workspace/rak-car/main/car_start_api.py) 改成自己的业务脚本
4. 遇到参数不确定时去翻 [API_REFERENCE.md](file:///home/jetson/workspace/rak-car/main/API_REFERENCE.md)
5. 需要确认能力边界时去翻 [CAPABILITY_LIST.md](file:///home/jetson/workspace/rak-car/main/CAPABILITY_LIST.md)

一句话总结：

- `main/` 只负责业务编排
- `car/arm/task` 负责动作执行
- `get_*` 接口负责闭环读取
- 你只需要用这些现成 API 就能完成控制和反馈
