# Chassis Topics — 同事在 dev box 上控车指南

> **TL;DR**:同事在 dev box 上 **不需要 SSH Jetson**,也 **不需要自己跑节点**。
> Jetson 上的 `mc602_node` + `chassis_kinematics_node` 通过 DDS auto-discovery 暴露
> 4 个 topic 给 dev box,你只需要 `ros2 topic pub / ros2 topic echo`。

## 前置条件(dev box 一次设置)

```bash
# 1. 同 ROS 域,Jetson 和 dev box 必须一致
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc

# 2. 同 DDS 实现(对齐 Jetson 默认)
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# 3. 重启 shell 或 source ~/.bashrc
```

如果网络不通,先 ping Jetson IP(`192.168.3.69`)。`ros2 topic list` 应该能看到 `/vehicle_wbt/v1/*`。

## 4 个 chassis topic

### `/vehicle_wbt/v1/cmd_vel` (订阅 — 你发)
类型:`geometry_msgs/Twist` · 期望车体速度 · 默认 50Hz 控制循环响应

| 字段 | 单位 | 含义 |
|------|------|------|
| `linear.x` | m/s | 前后(正值 = 前进) |
| `linear.y` | m/s | 左右(正值 = 左移,麦纳姆特色) |
| `linear.z` | m/s | (忽略,2D 底盘) |
| `angular.x`, `angular.y` | rad/s | (忽略) |
| `angular.z` | rad/s | 旋转(正值 = 逆时针) |

⚠ **0.5s 超时保护**:Jetson 上 `chassis_kinematics_node` 0.5s 没收到 `cmd_vel` 就停轮,
所以**必须保持持续发送**,不能只发一次就期望车一直走。推荐做法:

- 启动时起一个 10-50Hz 的 timer 持续 publish
- 或者业务结束时就主动发 `linear.x=0 linear.y=0 angular.z=0`(默认会停轮)

### `/vehicle_wbt/v1/odom` (订阅 — 你读)
类型:`nav_msgs/Odometry` · 50Hz · 麦纳姆轮累计 odom(基于 encoder)

字段:`pose.pose.position.{x,y}` 世界坐标位移,`pose.pose.orientation` 四元数 yaw。
**这是相对起点**,不是 GPS。每次启动从 (0,0,0) 开始。

⚠ 这是 **odometry**,不是 SLAM。没有回环检测、不能漂移修正。
长时间运行后会累积误差 — 短任务(< 5 分钟)够用,长任务需要外部校正。

### `/tf` (订阅 — 你读)
类型:`tf2_msgs/TFMessage` · 50Hz · `odom → base_link` 变换

如果你想可视化(RViz / Foxglove),订阅 `/tf` + `/vehicle_wbt/v1/odom` 就够。

### `/vehicle_wbt/v1/mc602/state/raw` (订阅 — 你读)
类型:`vehicle_wbt_smartcar_msgs/msg/RawState` · 20Hz · 完整传感器快照

包含 `encoders[4]` (M1~M4 原始脉冲)、IR、battery 等。**底盘业务通常不需要订阅这个**,
已经有 `/odom` 算好了。如果你需要原始 encoder 数据调 PID,这个 topic 是 source of truth。

## 5 行命令测试(Jetson 已 launch 的情况下)

```bash
# 1. 确认能看到 Jetson 的 topics
ros2 topic list | grep vehicle_wbt/v1

# 2. 订阅 odom,看车当前位置
ros2 topic echo /vehicle_wbt/v1/odom --once

# 3. 订阅 tf,看 odom→base_link 变换
ros2 topic echo /tf --once

# 4. 直线前进 0.5s(单次脉冲,timeout 0.5s 后会自动停)
ros2 topic pub --once /vehicle_wbt/v1/cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 5. 持续前进(10Hz,车会一直走直到 Ctrl-C)
ros2 topic pub --rate 10 /vehicle_wbt/v1/cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## Python rclpy 客户端模板

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ChassisDriver(Node):
    def __init__(self):
        super().__init__('chassis_driver')
        self.cmd_pub = self.create_publisher(
            Twist, '/vehicle_wbt/v1/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/vehicle_wbt/v1/odom', self._on_odom, 10)
        # 10Hz 持续发布,避免 timeout 停轮
        self.create_timer(0.1, self._tick_cmd)
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_omega = 0.0
        self.latest_odom = None

    def _tick_cmd(self):
        msg = Twist()
        msg.linear.x = float(self.target_vx)
        msg.linear.y = float(self.target_vy)
        msg.angular.z = float(self.target_omega)
        self.cmd_pub.publish(msg)

    def _on_odom(self, msg):
        self.latest_odom = msg

    def drive_forward(self, speed=0.2):
        self.target_vx = speed

    def stop(self):
        self.target_vx = self.target_vy = self.target_omega = 0.0


def main():
    rclpy.init()
    driver = ChassisDriver()
    driver.drive_forward(0.2)
    # 业务循环
    rclpy.spin(driver)
    driver.stop()
    rclpy.shutdown()
```

## 真机校准提示(踩过的坑)

### 1. 速度符号(默认已 flip 过)
这台车的物理轮位/接线跟 SDK 矩阵假设相反,`chassis_kinematics_node` launch
时**默认 vx/vy/omega 全部乘 -1**(即 `sign_flip_vx/vy/omega = -1`)。
如果你换了车/重新接线,在 launch 时改:

```bash
ros2 launch vehicle_wbt_smartcar_bridge mc602.launch.py \
    serial_port:=/dev/ttyUSB1 \
    sign_flip_vx:=1.0 sign_flip_vy:=1.0 sign_flip_omega:=1.0
```

验证方法:`ros2 topic pub /vehicle_wbt/v1/cmd_vel ... "{linear: {x: 0.1}}"`
然后 `ros2 topic echo /vehicle_wbt/v1/odom --once` 看 x 是 +0.05 还是 -0.05。

### 2. 实际速度 ≈ cmd_vel × 0.5
电机从 0 加速到 100% PWM 有响应延迟,加上 50Hz set_wheels service 的离散化,
**实际线速度大约是 cmd_vel 的 50-70%**。需要精准距离时,要么:
- 用 `/odom` 反馈做闭环(参考位置而不是速度)
- 用 `cmd_vel_timeout_s` 让车多走一段时间再校正

### 3. 几何参数
`track=0.30, wheel_base=0.28, wheel_radius=0.03` 是 `cfg_vehicle.yaml` 里的值。
如果换轮子或改了底盘尺寸,launch 时覆盖:

```bash
ros2 launch vehicle_wbt_smartcar_bridge mc602.launch.py \
    chassis_track:=0.32 chassis_wheel_base:=0.28 chassis_wheel_radius:=0.035
```

### 4. encoder_resolution
`2015.13` 是 `motor_280` 的标准值。如果换成其他电机(motor_310 / motor_350 等),
在 launch 时覆盖:`encoder_resolution:=...`。

## 当前限制(不在 Phase 2 范围)

- ❌ 无安全包络(没限制最大加速度 / 急停)
- ❌ 无 collision check(同事要自己写)
- ❌ 无 action interface(只有 cmd_vel topic + 隐式 odom)
- ❌ odom 漂移(长时间任务需要外部校正)
- ❌ 不支持非麦纳姆(Diff / 三轮 / 四轮普通底盘)

这些是 Phase 2 的范围,目前先把 P/M/S/PWM/IO 接口打通。
