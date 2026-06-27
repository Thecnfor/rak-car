# 底盘运动学系统

> **核心代码不可修改。** `vehicle/driver/vehicle_base.py` 是运动学核心。

## 支持的底盘类型

| 类型 | 类名 | 轮数 | 自由度 | 配置名 |
|------|------|:---:|:---:|--------|
| 麦克纳姆轮 | `Mecanum` | 4 | 3 (x, y, yaw) | `mecanum` |
| 两轮差速 | `Diff2` | 2 | 2 (x, yaw) | `diff2` |
| 四轮差速 | `Diff4` | 4 | 2 (x, yaw) | `diff4` |
| 三轮全向 | `Tricycle` | 3 | 3 (x, y, yaw) | `tricycle` |

配置在 `vehicle/driver/cfg_vehicle.yaml` 的 `chassis.type` 字段。

## 麦克纳姆轮运动学 (Mecanum)

### 逆运动学（速度 → 轮速）

给定车身速度 `(vx, vy, ω)`，计算四个轮子的线速度：

```
v_fl = vx - vy - ω × (a + b)    # 前左
v_fr = vx + vy + ω × (a + b)    # 前右
v_rl = vx + vy - ω × (a + b)    # 后左
v_rr = vx - vy + ω × (a + b)    # 后右
```

其中：
- `a` = `track / 2`（左右轮距的一半）
- `b` = `wheel_base / 2`（前后轴距的一半）

**配置值（cfg_vehicle.yaml）：**
- `track = 0.30` m（轮距）
- `wheel_base = 0.28` m（轴距）
- `wheel_radius = 0.03` m（轮子半径）

### 正运动学（轮速 → 车身速度）

```
vx  = (v_fl + v_fr + v_rl + v_rr) / 4
vy  = (-v_fl + v_fr + v_rl - v_rr) / 4
ω   = (-v_fl + v_fr - v_rl + v_rr) / (4 × (a + b))
```

### 里程计更新

```python
# 每次控制循环调用
d_left = (d_fl + d_rl) / 2   # 左侧平均位移
d_right = (d_fr + d_rr) / 2  # 右侧平均位移
d_center = (d_left + d_right) / 2
d_theta = (d_right - d_left) / track

# 全局坐标更新
x += d_center × cos(theta + d_theta/2)
y += d_center × sin(theta + d_theta/2)
theta += d_theta
```

## 两轮差速运动学 (Diff2)

### 逆运动学

```
v_left  = vx - ω × track / 2
v_right = vx + ω × track / 2
```

### 正运动学

```
vx = (v_left + v_right) / 2
ω  = (v_right - v_left) / track
```

## CarBase 核心方法

### set_velocity(vx, vy=0, omega=0)

```python
def set_velocity(self, vx, vy=0, omega=0):
    # 1. 限速
    vx = limit_val(vx, -self.speed_x_limit, self.speed_x_limit)
    vy = limit_val(vy, -self.speed_y_limit, self.speed_y_limit)
    omega = limit_val(omega, -self.angle_limit, self.angle_limit)

    # 2. 逆运动学: (vx, vy, omega) → 各轮线速度
    wheel_speeds = self.chassis.inverse_kinematics(vx, vy, omega)

    # 3. 线速度 → 角速度 → 虚拟速度 → 单片机
    self.wheel.set_linear(wheel_speeds)
```

### 里程计更新线程

```python
def updata_odom(self):  # ⚠️ 拼写错误: 应为 update_odom
    while True:
        # 1. 读取编码器
        wheel_d = self.chassis.get_distance(self.motors.get_encoder())

        # 2. 正运动学更新位姿
        self.chassis.forward_kinematics(wheel_d)

        # 3. 更新全局位姿
        self.x, self.y, self.theta = self.chassis.get_pose()

        time.sleep(0.01)  # 100Hz 更新
```

### 位姿操作

| 方法 | 功能 |
|------|------|
| `get_pose()` | 返回 `(x, y, theta)` 全局位姿 |
| `set_pose(x, y, theta)` | 设置当前位姿（校准用） |
| `set_pose_offset(dx, dy, dtheta)` | 相对偏移当前位姿 |
| `reset_odom()` | 里程计归零 |

### move_base(sp, end_function, stop=True)

核心运动循环：

```python
def move_base(self, sp, end_function, stop=True):
    """
    sp: 速度向量 (vx, vy, omega) 或可调用对象
    end_function: 结束条件函数，返回 True 时停止
    stop: 结束后是否停车
    """
    self._stop_flag = False
    while not self._stop_flag:
        # 计算速度（sp 可以是函数，动态计算）
        vel = sp() if callable(sp) else sp
        self.set_velocity(*vel)

        # 检查结束条件
        if end_function():
            break

        time.sleep(0.01)

    if stop:
        self.set_velocity(0, 0, 0)
```

## 速度限制

配置在 `config_car.yml`：

```yaml
speed:
  x: {limit: 0.7}      # 最大前进速度 0.7 m/s
  y: {limit: 0.7}      # 最大横移速度 0.7 m/s
  angle: {limit: 3}     # 最大角速度 3 rad/s
```

## 底盘配置文件

`vehicle/driver/cfg_vehicle.yaml`：

```yaml
chassis:
  type: mecanum              # 底盘类型
  track: 0.30                # 左右轮距 (m)
  wheel_base: 0.28           # 前后轴距 (m)
  wheel_radius: 0.03         # 轮子半径 (m)

motors:
  type: motor_280            # 电机型号
  ports: [1, 2, 3, 4]       # 电机端口号
  reverse: [1, -1, -1, 1]   # 旋转方向修正

pid_vel_params:              # 速度环 PID
  Kp: 0.18
  Ki: 0.01
  Kd: 0.0018
```

**⚠️ 注意：** `vehicle/test/cfg_vehicle.yaml` 有不同参数（更小的底盘），是另一台机器人的配置，不要混用。
