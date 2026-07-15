# 机械臂 10 行起步

## 1. 装依赖（一次性）

```bash
python3 -m pip install -r /home/jetson/workspace/rak-car/main/requirements.txt
export RAK_CAR_SERVER_ORIGIN=http://192.168.3.60
```

## 2. 首次上电：4 键手动定原点

```bash
python3 /home/jetson/workspace/rak-car/main/arm/examples/01_calibrate_origin.py left
```

按车端 4 键（**连续按住**）：

| 键 | 行为 |
| --- | --- |
| 1 | y 下降（朝触底方向） |
| 3 | y 上升 |
| 2 | x 左移 |
| 4 | x 右移 |

把机械臂按到：

- y 触底（磁感触发）
- x 撞左侧墙（编码器堵转）

然后**同时按住 1 + 3 持续 1 秒**，原点就写到 `main/arm/arm_origin.yaml` 了，程序退出。

## 3. 10 行起步：移动到目标点

```python
from main.arm import ArmClient, ArmRunner

client = ArmClient.connect()           # 1. 连 HTTP（自动加载 arm_origin.yaml）
runner = ArmRunner(client)             # 2. 业务编排入口

runner.move_xy(100.0, 80.0)            # 3. 双轴同步 (dry-run + 走车端 PID)
runner.set_side("LEFT")                # 4. 大臂向左
runner.move_xy(120.0, 40.0)            # 5. 移到抓取点
runner.grasp(True)                     # 6. 吸盘抓
runner.set_hand("DOWN")                # 7. 手爪放下
runner.move_xy(0.0, 30.0)              # 8. 移到放料点
runner.grasp(False)                    # 9. 释放
runner.go_home()                       # 10. 回原点
```

完整 pick-and-place 见 [examples/04_grasp_template.py](./examples/04_grasp_template.py)。

## 4. 不连硬件：只算 S 曲线

```bash
python3 /home/jetson/workspace/rak-car/main/arm/examples/02_trajectory_preview.py
```

输出示例：

```
TrajectoryPlan((0.0,0.0) -> (100.0,80.0) mm, T=2.30s, peak_vx=120.0 peak_vy=80.0 mm/s)

   t(s)      x(mm)      y(mm)   vx(mm/s)   vy(mm/s)
--------------------------------------------------------
   0.000       0.00       0.00         0.00         0.00
   0.200      22.85      12.46        91.39        82.30
   ...
```

## 5. 常见错误

| 报错 | 原因 | 处理 |
| --- | --- | --- |
| `ValueError: y_mm=200 超出软上限 180mm` | y 超出 `arm_origin.yaml` 软限位 | 改 `soft_y_max_m` 后重 calibrate |
| `RuntimeError: 等待小车初始化超时` | runtime 没起来 | `pm2 restart rak-car-api` |
| `执行超时: arm.goto_position` | 硬件堵转 / 编码器漂移 | `arm.reset_origin("left")` 重新定原点 |
| 动作全部从 0 开始（坐标系没标定） | 没跑 `01_calibrate_origin.py` | 先跑一次 |

## 6. 下一步

- 看完整业务 API：[ARM_API.md](./ARM_API.md)
- 看机械臂子包总览：[README.md](./README.md)
- 看底盘组对应工作区：[../chassis/README.md](../chassis/README.md)
- 看 main 整体架构：[../README.md](../README.md)
