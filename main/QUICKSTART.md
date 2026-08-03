# main/ 5 分钟起步

> 把 runtime 起来 → 连一次 → 跑一个机械臂动作 → 跑一个底盘动作。
> 详细 API 见 [API_INDEX.md](./API_INDEX.md)。

---

## 1. 装依赖（一次性）

```bash
python3 -m pip install -r /home/jetson/workspace/rak-car/main/requirements.txt
export RAK_CAR_SERVER_ORIGIN=http://192.168.5.230:5050
```

不连硬件：装 `main/requirements.txt` 就够；连硬件用 `runtime/requirements.txt`（在车端跑）。

## 2. 起 runtime（在车端 / Jetson）

```bash
# dev
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m runtime.server

# production（比赛用）
pm2 start ecosystem.config.js
pm2 logs rak-car-api
```

健康检查：

```bash
curl http://192.168.5.230:5050/v1/health
```

应该看到 `state.initialized=true`。

## 3. 连一次

```bash
python3 /home/jetson/workspace/rak-car/main/quick_start.py
```

输出示例：

```text
health: {'state': {'initialized': True, ...}, 'inference': {...}}
actions: count=60+   # CAR_ACTIONS + ARM_ACTIONS + SYSTEM_ACTIONS，见 API_INDEX.md §6
client ready
```

## 4. 跑一个机械臂动作（业务层 10 行）

```python
from main.arm import ArmClient, ArmRunner

client = ArmClient.connect()
runner = ArmRunner(client)

runner.move_xy(100.0, -80.0)            # y 负 = 向上
runner.set_arm_angle(-90.0)             # 大臂角度
runner.move_xy(120.0, -40.0)
runner.grasp(True)                      # 吸
runner.move_xy(0.0, -30.0)
runner.grasp(False)                     # 放
runner.go_home()
```

真机模板：[`main/arm/examples/11_grasp.py`](./arm/examples/11_grasp.py) / [`12_vision_pick_water.py`](./arm/examples/12_vision_pick_water.py)。

机械臂首次上电：

- 默认 `RAK_CAR_RESET_ARM=1`（见 `ecosystem.config.js:23`），runtime init 时自动跑 `arm.reset_position`。
- 只有 `RAK_CAR_RESET_ARM=0` 且从未手 reset 时坐标系才是未标定状态——软限位用默认值。

## 5. 跑一个底盘动作

```python
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()
client.wait_until_ready()

# 同步：阻塞等结果
client.execute_car_action("move_for", speeds=[0.1, 0.1, 0.1, 0.1], time=2.0, sync=True, timeout=10)

# 异步：拿到 job_id 后轮询
job = client.execute_car_action("move_to_position", x=1.0, y=0.5, timeout=30)
client.wait_job(job["id"], timeout=60)
```

外环 50Hz：

```python
client = RuntimeApiClient()
while True:
    state = client.realtime_lane_state()
    ey = state["error_y"]
    ea = state["error_angle"]
    # ... P / Stanley / curvature 算 vx, wz ...
    client.realtime_wheel_speeds([v0, v1, v2, v3])
```

## 6. 跑一个目标检测

```python
# 单次精确识别（带 sort/limit 过滤）
result = client.execute_car_action(
    "get_detection_results", sync=True, timeout=20,
    sort_pos=[0, 0], limit_x=1, limit_y=1,
)

# 边走边看（task_feed 守护线程缓存，不打 socket）
task_state = client.get_task_state()
for det in task_state["detections"]:
    print(det["label"], det["score"], det["bbox_norm"])
```

## 7. 常见错误

| 报错 | 原因 | 处理 |
| --- | --- | --- |
| `RuntimeError: 等待小车初始化超时` | runtime 没起 / MC602 没插 | `pm2 status`；`pm2 restart rak-car-api` |
| `EFSM` / `Operation cannot be accomplished in current state` | ZMQ REQ socket 多线程并发 | 2026-07-31 已修复（`ClintInterface` 加锁）；出现即代码绕开了 runtime 直打硬件 |
| `execute 超时` | 硬件堵转 / 编码器漂移 | arm 调 `arm.reset_origin("left")`；底盘查串口 |
| 急停后 lane 失灵 | `_stop_flag=True` 未清 | `client.reset_stop_flag()`（必须重启 `lane_feed`） |
| `soft_y_max_mm` 报错 | y 超出 `arm_origin.yaml` 软限位 | 改 `soft_y_max_m` 后重 calibrate |

## 8. 下一步

- 机械臂：[`main/arm/README.md`](./arm/README.md)（10 行起步 + examples 05–12 都在里面）
- 写 task 先查：[`main/README.md` 现成方法速查](./README.md#写-task-先查这张表现成方法速查)
- 底盘外环：[`main/chassis/README.md`](./chassis/README.md)
- 完整 API：[`main/API_INDEX.md`](./API_INDEX.md)
- mini 任务模板：[`main/misc/`](./misc/)