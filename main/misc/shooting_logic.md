# 枪射击逻辑研究结论

调研日期：基于当前 `main` 分支，源码引用全部带 commit-时间戳无关的文件位置。

## 1. 硬件层

[`smartcar/whalesbot/vehicle/base/controller_wrap.py`](file:///home/jetson/workspace/rak-car/smartcar/whalesbot/vehicle/base/controller_wrap.py#L479-L485)：

```python
class PoutD():
    def __init__(self, port):
        self.pout_1 = PortOut_1(port)
        self.pout_2 = PoutD_2(port)
    def set(self, val):
        func = [self.pout_1.out, self.pout_2.set]
        return func[ctl_id](val)
```

要点：

- 双控制器适配类：根据全局 `ctl_id`（0=mc601 / 1=mc602）分别走 `PortOut_1` 或 `PoutD_2`
- 本质都是数字输出口 `set(0/1)`
- 这台车用的是 **`PoutD(4)`**（见 [car_wrap_2026.py:414](file:///home/jetson/workspace/rak-car/car_wrap_2026.py#L414)），对应数字口第 4 路

## 2. MyCar 暴露的两个 API

[car_wrap_2026.py:440-453](file:///home/jetson/workspace/rak-car/car_wrap_2026.py#L440-L453)：

| 方法 | 行为 | 时序 |
| --- | --- | --- |
| `shooting()` | 单次触发脉冲 | `set(0)` → sleep 50ms → `set(1)` → sleep 250ms → `finally: set(0)` → sleep 200ms（总 ~500ms） |
| `set_shoot_state(value)` | 直接写数字口 | 立刻 `set(1/0)`，不睡 |

`car.shooting()` 关键点：

- 用 `try/finally` 保证异常也能拉低，避免继电器常吸把枪烧穿
- 250ms 高电平脉冲是经验值：长了过烧、短了不触发
- 总耗时约 500ms

`car.set_shoot_state(value)` 关键点：

- **不带收尾逻辑**，纯写电平
- **不要拿来连发**，只适合"常开/常闭"场景（例如调试时手动维持）
- 连发场景必须多次调 `shooting()`，每次都走完完整 50ms 预拉低 + 250ms 高 + 200ms 后置

注册到 runtime：[runtime/core/actions.py:26-27](file:///home/jetson/workspace/rak-car/runtime/core/actions.py#L26-L27)。

## 3. 任务层怎么用

[car_task_function.py:268-309](file:///home/jetson/workspace/rak-car/car_task_function.py#L268-L309) 的 `target_shooting(animal_list)` 流程：

1. 解析 `animal_list`（4 个位置，0=需要打），算相邻打击点距离
2. 大臂摆 `LEFT`、抬 `UP`、`arm.set_arm_pose(x=0.3, y=0.02)` 准备姿态
3. `lane_dis_offset(speed=0.3, dis_hold=3.0)` 巡线到目标区域
4. `move_for([-0.2, 0, 0])` 后退微调
5. `move_to_detection_target(delta_x=d_x, delta_y=None, sort_pos=(d_x, 0))` 视觉对齐
6. 循环每个打击点：`my_car.shooting()` + `time.sleep(5)` 等待弹丸装填
7. 最后一发完再补 `lane_dis_offset(speed=0.3, dis_hold=0.48 - sum(relative_loc))` 距离补偿

`animal_list` 由 `task.target_shooting_detection` 返回，business 层可以这样串：

```python
animal_list = client.execute_task("target_shooting_detection", timeout=240)["result"]
client.execute_task("target_shooting", timeout=240, animal_list=animal_list)
```

## 4. 业务层调用

走 `POST /v1/execute`，[main/API.md 第 6.4 节](file:///home/jetson/workspace/rak-car/main/API.md)：

| 接口 | 关键参数 | 关键返回 |
| --- | --- | --- |
| `car.shooting` | 无 | 动作完成 |
| `car.set_shoot_state` | `value` | `true/false` |
| `car.set_digital_output` | `port` `value` | `{"port","value"}` |

```python
client.call("car", "shooting", timeout=5)   # 单发
client.call("car", "set_shoot_state", True, timeout=5)   # 拉高
client.call("car", "set_shoot_state", False, timeout=5)  # 拉低
```

**timeout 提示**：`shooting()` 内置 500ms sleep，正常给 `timeout=5` 即可；连发 3 次给 `timeout=15`（每次还要加手动 sleep）。

## 5. 三条红线

1. **不要用 `set_shoot_state(True)` 持续触发**——没有收尾，硬件会常吸。
2. **`shooting()` 调用间隔不要 < 500ms**——内部 sleep 没结束就被打断会乱。
3. **数字口占用**：当前只有 `PoutD(4)` 一个口在用枪；如果以后接多个继电器，注意端口号不要冲突。
