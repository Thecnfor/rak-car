# 配置文件参考

## 配置文件分布

```
vehicle_wbt/
├── config_car.yml                    # 主配置（摄像头、IO、PID）
├── infer_cs/base/infer.yaml          # 推理服务配置
├── vehicle/arm/arm_cfg.yaml          # 机械臂配置
├── vehicle/base/mc602_cfg.yaml       # MC602 校准数据
├── vehicle/driver/cfg_vehicle.yaml   # 底盘运动学配置
└── vehicle/test/cfg_vehicle.yaml     # ⚠️ 另一台机器人的配置，不要混用
```

---

## config_car.yml — 主配置

```yaml
wbt_car: 0.1              # 版本号

camera:
  front: 2                 # 前方摄像头 /dev/cam2
  side: 1                  # 侧方摄像头 /dev/cam1

io:
  left_sensor: 8           # 左红外传感器端口
  right_sensor: 7          # 右红外传感器端口
  key: 1                   # 四按键端口
  light: 2                 # LED 灯端口

speed:
  x: {limit: 0.7}          # 最大前进速度 (m/s)
  y: {limit: 0.7}          # 最大横移速度 (m/s)
  angle: {limit: 3}        # 最大角速度 (rad/s)

lane_pid:                  # 车道线跟随 PID
  cfg_pid_y:
    Kp: 4, Ki: 0, Kd: 0
    setpoint: 0
    output_limits: [-0.7, 0.7]
  cfg_pid_angle:
    Kp: 3, Ki: 0, Kd: 0
    setpoint: 0
    output_limits: [-1.5, 1.5]

det_pid:                   # 检测跟随 PID
  cfg_pid_y:
    Kp: 1, Ki: 0, Kd: 0
    setpoint: 0
    output_limits: [-0.7, 0.7]
  cfg_pid_angle:
    Kp: 1, Ki: 0, Kd: 0
    setpoint: 0
    output_limits: [-1.5, 1.5]

location_pid:              # 定位 PID（当前未使用）
  pid_x: {Kp: 2, Ki: 0, Kd: 0, output_limits: [-0.7, 0.7]}
  pid_y: {Kp: 2, Ki: 0, Kd: 0, output_limits: [-0.7, 0.7]}
```

**加载方式：** `car_wrap.py` 中 `get_yaml("config_car.yml")`

---

## cfg_vehicle.yaml — 底盘配置

```yaml
chassis:
  type: mecanum            # mecanum / diff2 / diff4 / tricycle
  track: 0.30              # 左右轮距 (m)
  wheel_base: 0.28         # 前后轴距 (m)
  wheel_radius: 0.03       # 轮子半径 (m)

motors:
  type: motor_280          # motor_280 / motor_280_0
  ports: [1, 2, 3, 4]     # 电机端口列表
  reverse: [1, -1, -1, 1] # 旋转方向修正

pid_vel_params:            # 速度环 PID（未使用 simple_pid）
  Kp: 0.18
  Ki: 0.01
  Kd: 0.0018
  setpoint: 0.0
  output_limits: [-100, 100]
```

**加载方式：** `vehicle_base.py` 中 `yaml.load(open("cfg_vehicle.yaml"))`

**⚠️ `vehicle/test/cfg_vehicle.yaml` 有不同的参数（track=0.205, wheel_base=0.14），是另一台机器人。**

---

## arm_cfg.yaml — 机械臂配置

```yaml
arm:
  stepper1:
    id: 1
    reverse: 1
    perimeter: 0.008       # 步进电机导程 (m/圈)
  stepper2:
    id: 2
    reverse: 1
    perimeter: 0.008
  servo_bus:
    id: 2                  # 总线舵机端口
  motor:
    id: 5
    reverse: -1
    perimeter: 0.06        # 电机周长 (m)
  vacuum_pump:
    port: 4                # 真空泵数字输出端口
  limits:                  # 关节角度限制 (rad)
    joint1: [-3.14, 3.14]
    joint2: [-3.14, 3.14]
```

---

## infer.yaml — 推理服务配置

详见 [inference-system.md](inference-system.md)。

---

## mc602_cfg.yaml — MC602 校准

MC602 控制器的电位器中点校准数据。一般不需要修改。

---

## 配置加载方式对比

| 文件 | 加载函数 | 位置 |
|------|---------|------|
| `config_car.yml` | `get_yaml()` | `tools/base/tools_class.py` |
| `cfg_vehicle.yaml` | `yaml.load()` 直接调用 | `vehicle_base.py` |
| `arm_cfg.yaml` | `yaml.load()` 直接调用 | `arm_base.py` |
| `infer.yaml` | `get_yaml()` | `infer_front.py` |
| `mc602_cfg.yaml` | `yaml.load()` 直接调用 | `mc602_ctl2.py` |

**⚠️ 加载方式不统一。** `get_yaml` 和 `yaml.load()` 混用。
