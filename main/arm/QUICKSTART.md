# 机械臂 10 行起步

## 1. 装依赖（一次性）

```bash
python3 -m pip install -r /home/jetson/workspace/rak-car/main/requirements.txt
export RAK_CAR_SERVER_ORIGIN=http://192.168.6.231
```

## 2. 首次上电：runtime 自动定原点

不需要任何手动操作。runtime 启动时（pm2 拉起 `rak-car-api`），`ecosystem.config.js` 默认把 `RAK_CAR_RESET_ARM=1` 注入环境变量；`runtime/core/settings.py:111-112` 看到后会在自动初始化阶段调一次 `arm.reset_position`，让 y 触底，并把当前编码器值落到 `main/arm/arm_origin.yaml`。

> 注（2026-07-16）：reset_x 已删除，`reset_position` 只归 y；x 位置由视觉闭环控制。

正常情况下你接好车、连上 `RAK_CAR_SERVER_ORIGIN`、跑业务就行；`arm_origin.yaml` 不存在也不会卡死 —— runtime 会用默认软限位（仅 y 软上限），下次 reset 后被覆盖。

只有当机械臂"漂移严重 / PID 范围卡死 / 编码器读数明显不对"时，才手动跑一次 reset：

```bash
python3 /home/jetson/workspace/rak-car/main/arm/examples/01_calibrate_origin.py left   # 或 right
```

底层会调车端 `arm.reset_position` + 读 `y_get_position` / `x_get_position`，最后写 `arm_origin.yaml`。和旧版"按 4 键手动 jog"的区别：当前版本不再监听按键，直接走车端闭环 reset。

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
| `ValueError: y_mm=200 超出软上限 200mm` | y 超出 `arm_origin.yaml` 软限位 | 改 `soft_y_max_m` 后重 calibrate |
| `RuntimeError: 等待小车初始化超时` | runtime 没起来 | `pm2 restart rak-car-api` |
| `执行超时: arm.goto_position` | 硬件堵转 / 编码器漂移 | `arm.reset_origin("left")` 重新定原点 |
| 动作全部从 0 开始（坐标系没标定） | `RAK_CAR_RESET_ARM=0` 且从未手调用 `arm.reset_position` | 在 [ecosystem.config.js:23] 把它设回 `1`，pm2 重启 runtime；或手跑 `examples/01_calibrate_origin.py left` |

## 6. 下一步

- 看完整业务 API：[ARM_API.md](./ARM_API.md)
- 看机械臂子包总览：[README.md](./README.md)
- 看底盘组对应工作区：[../chassis/README.md](../chassis/README.md)
- 看 main 整体架构：[../README.md](../README.md)
