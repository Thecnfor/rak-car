# 机械臂调试脚本

这个目录用来放临时的手动调试脚本，不依赖任何测试框架。

## 使用方式

从项目根目录运行：

```bash
# 跑某个任务（先 init 复位机械臂，再执行任务）
python -m smartcar.test.tasks.<任务名>

# 跑某个底层机械臂方法
python -m smartcar.test.functions.<类别名>
```

## 目录结构

```
smartcar/test/
├── README.md
├── __init__.py
├── tasks/                      # 按任务拆分（每个任务一个文件）
│   ├── __init__.py
│   ├── init.py                 # 初始化（含可选机械臂复位）
│   ├── auto_seeding.py         # 任务1: 自动播种
│   ├── target_shooting_detection.py  # 任务2: 虫害侦察
│   ├── water_tower_task.py     # 任务3: 水塔灌溉
│   ├── target_shooting.py      # 任务4: 射击除害
│   ├── crop_harvesting.py      # 任务5: 作物采收
│   ├── sort_and_store.py       # 任务6: 分类储存
│   ├── get_order.py            # 任务7: 获取订单
│   └── order_delivery.py       # 任务8: 订单配送
└── functions/                  # 按 ArmController 方法类别拆分
    ├── __init__.py
    ├── y_axis.py               # Y 轴（竖直）：move_y / y_pid / y_reset / y_get
    ├── x_axis.py               # X 轴（水平）：move_x / x_pid / x_stop / x_get
    ├── grasp.py                # 气泵：grasp()
    ├── pose.py                 # 姿态：set_arm_pose / set_arm_angle / set_hand_angle / goto / go_for
    ├── reset.py                # 复位：reset_position / reset_y / reset_x / switch_side
    └── speed.py                # 速度：y_speed / x_speed
```

## 说明

- `tasks/` 里的脚本从 `car_task_function.py` 移植，机械臂相关代码保持一致
- `functions/` 里的脚本是 `ArmController` 各方法的独立调用示例
- 直接 `python -m` 跑任何一个文件即可单独调试该部分