"""chassis_kinematics_node — 麦纳姆轮底盘运动学子节点。

设计:
  - 独立节点,不污染 mc602_node 的原子 IO 职责
  - SDK `MecanumChassis` + `Odometry` 数学 1:1 抄过来(无重写)
  - 同事在 dev box 上只需 `ros2 topic pub /cmd_vel geometry_msgs/Twist "{...}"`,
    不用自己起 50Hz 调 service / 自己订阅 state/raw 算 odom / 自己广播 tf

ROS 接口(自动通过 ROS_DOMAIN_ID=42 + CycloneDDS 在 LAN 上被 dev box 发现):
  Sub  /vehicle_wbt/v1/cmd_vel              geometry_msgs/Twist        → 期望车体速度
  Sub  /vehicle_wbt/v1/mc602/state/raw      RawState                   → encoder 数据
  Pub  /vehicle_wbt/v1/odom                 nav_msgs/Odometry          → 位姿 + 速度
  Pub  /tf                                  tf2_msgs/TFMessage         → odom → base_link
  Cli  /vehicle_wbt/v1/mc602/set_wheels     SetWheels                  → 实际 4 路电机

参数:
  track              0.30    轮距 (m)
  wheel_base         0.28    轴距 (m)
  wheel_radius       0.03    轮半径 (m)
  encoder_resolution 2015.13 motor_280 编码器一圈脉冲数
  cmd_vel_timeout_s  0.5     超时没收到 cmd_vel 就停轮
  publish_tf         True    是否广播 /tf
  frame_id_odom      odom
  frame_id_base      base_link
  control_rate_hz    50.0
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

try:
    # ROS Iron+ / Humble 推荐
    from tf_transformations import quaternion_from_euler
except ImportError:
    # 退路:自己算 yaw→quaternion,只用 sin/cos 一半项
    def quaternion_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,   # x
            cr * sp * cy + sr * cp * sy,   # y
            cr * cp * sy - sr * sp * cy,   # z
            cr * cp * cy + sr * sp * sy,   # w
        )

from vehicle_wbt_smartcar_msgs.msg import RawState
from vehicle_wbt_smartcar_msgs.srv import SetWheels


# 编码器一圈 = 12 栅格 × 4 倍频 × 减速比 (28/11)^4 ≈ 2015.13 (motor_280)
DEFAULT_ENCODER_RES = 2015.1279
# 虚拟速度 100 = 编码器 1 圈/秒(MC602 内部约定)
SPEED_RATE = 100.0


class ChassisKinematicsNode(Node):
    """麦纳姆轮:Twist → 4 轮 + encoder → odom + tf"""

    def __init__(self):
        super().__init__('chassis_kinematics')

        # ---- 参数 ----
        self.declare_parameter('track', 0.30)
        self.declare_parameter('wheel_base', 0.28)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('encoder_resolution', DEFAULT_ENCODER_RES)
        self.declare_parameter('cmd_vel_timeout_s', 0.5)
        self.declare_parameter('frame_id_odom', 'odom')
        self.declare_parameter('frame_id_base', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('control_rate_hz', 50.0)
        # 符号校准:这台车物理 forward = odom -x 方向 → vx 翻号
        # 真机验证后改 1.0 即可。
        self.declare_parameter('sign_flip_vx', +1.0)   # cmd_vel.vx=+0.1 → 车往前
        self.declare_parameter('sign_flip_vy', -1.0)   # 待验证
        self.declare_parameter('sign_flip_omega', -1.0)  # 待验证

        self.track = self.get_parameter('track').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.timeout_s = self.get_parameter('cmd_vel_timeout_s').value
        self.frame_odom = self.get_parameter('frame_id_odom').value
        self.frame_base = self.get_parameter('frame_id_base').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.control_hz = float(self.get_parameter('control_rate_hz').value)
        self.sign_flip_vx = float(self.get_parameter('sign_flip_vx').value)
        self.sign_flip_vy = float(self.get_parameter('sign_flip_vy').value)
        self.sign_flip_omega = float(self.get_parameter('sign_flip_omega').value)

        self.get_logger().info(
            f'chassis_kinematics: track={self.track}, wheel_base={self.wheel_base}, '
            f'wheel_radius={self.wheel_radius}, enc_res={self.encoder_resolution:.2f}, '
            f'rate={self.control_hz}Hz'
        )

        # ---- 运动学矩阵(SDK `MecanumChassis.init_parameters` 1:1)----
        self._init_matrices()

        # ---- 状态 ----
        self.pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)  # x, y, theta
        self.last_encoders: np.ndarray | None = None
        self.last_state_time = None
        self.last_cmd_vel: Twist | None = None
        self.last_cmd_vel_time = None
        self._svc_ready = False

        # ---- ROS 接口 ----
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/vehicle_wbt/v1/cmd_vel', self._on_cmd_vel, 10)
        self.state_sub = self.create_subscription(
            RawState, '/vehicle_wbt/v1/mc602/state/raw', self._on_state_raw, 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/vehicle_wbt/v1/odom', 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )
        self.set_wheels_cli = self.create_client(
            SetWheels, '/vehicle_wbt/v1/mc602/set_wheels')

        # ---- Timer(50Hz 控制 + 50Hz odom)----
        period = 1.0 / self.control_hz
        self.control_timer = self.create_timer(period, self._tick_control)
        self.odom_timer = self.create_timer(period, self._tick_odom)

        self.get_logger().info('chassis_kinematics node ready')

    # ------------------------------------------------------------------ #
    # 运动学(SDK `MecanumChassis.init_parameters` 1:1)
    # ------------------------------------------------------------------ #
    def _init_matrices(self):
        roller_angle = math.pi / 4 * 1.052
        tan_roller = math.tan(roller_angle)
        half_track = self.track / 2
        half_wb = self.wheel_base / 2
        wheel_constant = half_track * tan_roller + half_wb

        # 正解:wheel_velocity[4] -> car_velocity[3]
        self.wheel_to_vehicle = np.array([
            [1 / 4,  1 / 4 / tan_roller, 1 / wheel_constant / 4],
            [-1 / 4, 1 / 4 / tan_roller, 1 / wheel_constant / 4],
            [-1 / 4, -1 / 4 / tan_roller, 1 / wheel_constant / 4],
            [1 / 4, -1 / 4 / tan_roller, 1 / wheel_constant / 4],
        ], dtype=np.float64)

        # 逆解:car_velocity[3] -> wheel_velocity[4]
        self.vehicle_to_wheel = np.array([
            [1,             -1,             -1,             1],
            [tan_roller,    tan_roller,    -tan_roller,    -tan_roller],
            [wheel_constant, wheel_constant, wheel_constant, wheel_constant],
        ], dtype=np.float64)

    # ------------------------------------------------------------------ #
    # 回调
    # ------------------------------------------------------------------ #
    def _on_cmd_vel(self, msg: Twist):
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    def _on_state_raw(self, msg: RawState):
        encs = np.array(msg.encoders, dtype=np.float64)
        now = self.get_clock().now()
        if self.last_encoders is None or self.last_state_time is None:
            self.last_encoders = encs
            self.last_state_time = now
            return

        dt = (now - self.last_state_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # 先算 delta,再更新 last_encoders(顺序很关键,反了每次都 0)
        delta_enc = encs - self.last_encoders
        self.last_encoders = encs
        self.last_state_time = now

        # 编码器脉冲 → 轮子线位移 (m)
        disp_per_pulse = 2 * math.pi * self.wheel_radius / self.encoder_resolution
        wheel_displacements = delta_enc * disp_per_pulse
        # 正解:轮子位移 → 车体位移 (车体坐标系)
        # SDK `MecanumChassis.forward_kinematics`:`wheel_velocity @ wheel_to_vehicle_matrix`
        # matrix 是 (4, 3),wheel 是 (4,),所以是 `wheel @ matrix`
        car_disp = wheel_displacements @ self.wheel_to_vehicle
        # SDK `Odometry.update` 1:1:旋转 z_angle 把车体位移转到世界坐标系
        self._update_pose(car_disp)

    def _update_pose(self, d_vector: np.ndarray):
        """SDK `Odometry.update` 1:1。d_vector = [dx, dy, dtheta] (车体系)"""
        z_angle = self.pose[2]
        cos_a = math.cos(z_angle)
        sin_a = math.sin(z_angle)
        # SDK 矩阵(注意符号约定:与 REP-103 一致,正 yaw 是逆时针)
        d_pose_transform = np.array([
            [cos_a,  sin_a],
            [-sin_a, cos_a],
        ], dtype=np.float64)
        d_pose_xy = d_vector[:2] @ d_pose_transform
        d_pose = np.array([d_pose_xy[0], d_pose_xy[1], d_vector[2]], dtype=np.float64)
        self.pose += d_pose

    # ------------------------------------------------------------------ #
    # Timer
    # ------------------------------------------------------------------ #
    def _tick_control(self):
        # 等 service 就绪(不阻塞构造)
        if not self._svc_ready:
            if self.set_wheels_cli.wait_for_service(timeout_sec=0.05):
                self._svc_ready = True
                self.get_logger().info('set_wheels service ready')
            else:
                return

        # 超时保护:没收到 cmd_vel 就停
        vx = vy = omega = 0.0
        if (self.last_cmd_vel is not None
                and self.last_cmd_vel_time is not None):
            age = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
            if age < self.timeout_s:
                vx = self.last_cmd_vel.linear.x
                vy = self.last_cmd_vel.linear.y
                omega = self.last_cmd_vel.angular.z

        # 逆解:车体速度 [vx, vy, omega] → 轮子线速度 (m/s)
        # SDK `MecanumChassis.inverse_kinematics`:`car_velocity @ vehicle_to_wheel_matrix`
        # matrix 是 (3, 4),car 是 (3,),所以是 `car @ matrix` → (4,)
        # 符号校准:vy / omega 根据真机物理轮位 flip
        car_vel = np.array([
            vx * self.sign_flip_vx,
            vy * self.sign_flip_vy,
            omega * self.sign_flip_omega,
        ], dtype=np.float64)
        wheel_vel_mps = car_vel @ self.vehicle_to_wheel  # 4 个轮子 (m/s)

        # m/s → virtual speed (-100..100)
        # virtual 100 = 编码器 1 rev/s = (encoder_resolution pulses/rev) / 1 sec
        # encoder_speed = wheel_vel_mps / disp_per_pulse
        # virtual_speed = encoder_speed / SPEED_RATE
        disp_per_pulse = 2 * math.pi * self.wheel_radius / self.encoder_resolution
        virtual = wheel_vel_mps / (disp_per_pulse * SPEED_RATE)

        # 限幅到 int8
        v0 = int(np.clip(round(float(virtual[0])), -100, 100))
        v1 = int(np.clip(round(float(virtual[1])), -100, 100))
        v2 = int(np.clip(round(float(virtual[2])), -100, 100))
        v3 = int(np.clip(round(float(virtual[3])), -100, 100))

        req = SetWheels.Request()
        req.v0, req.v1, req.v2, req.v3 = v0, v1, v2, v3
        self.set_wheels_cli.call_async(req)

    def _tick_odom(self):
        x, y, theta = float(self.pose[0]), float(self.pose[1]), float(self.pose[2])
        q = quaternion_from_euler(0.0, 0.0, theta)

        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_base
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.frame_odom
            t.child_frame_id = self.frame_base
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 安全停轮
        try:
            cli = node.create_client(SetWheels, '/vehicle_wbt/v1/mc602/set_wheels')
            if cli.wait_for_service(timeout_sec=0.5):
                req = SetWheels.Request()
                req.v0 = req.v1 = req.v2 = req.v3 = 0
                fut = cli.call_async(req)
                rclpy.spin_until_future_complete(node, fut, timeout_sec=1.0)
        except Exception as e:
            node.get_logger().warn(f'stop-on-exit failed: {e}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
